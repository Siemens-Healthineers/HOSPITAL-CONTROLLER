// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_CONTROLLER__CONTROLLER_UTILS_HPP_
#define HOSPITAL_CONTROLLER__CONTROLLER_UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <new>
#include <stdexcept>
#include <stop_token>  // NOLINT(build/include_order): C++20 header unknown to cpplint
#include <string_view>
#include <type_traits>
#include <utility>

#include "hospital_interfaces/msg/stopwatch.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"

namespace hospital_controller {

template <typename OnExit>
class ScopeGuard {
  static_assert(std::is_nothrow_invocable_v<OnExit&>);

 public:
  explicit ScopeGuard(OnExit on_exit) : on_exit_(std::move(on_exit)) {}
  ScopeGuard(const ScopeGuard&) = delete;
  ScopeGuard& operator=(const ScopeGuard&) = delete;
  ~ScopeGuard() noexcept { on_exit_(); }

 private:
  OnExit on_exit_;
};

// Release an already-acquired reservation only after the terminal result has
// been delivered, under the same lock used by goal admission.
class RunReservation {
 public:
  RunReservation(std::mutex& mutex, bool& running)
      : mutex_(mutex), running_(running) {}
  RunReservation(const RunReservation&) = delete;
  RunReservation& operator=(const RunReservation&) = delete;

  template <typename Completion>
  void complete(Completion&& completion) {
    std::scoped_lock lock(mutex_);
    std::forward<Completion>(completion)();
    running_ = false;
    armed_ = false;
  }

  ~RunReservation() {
    if (armed_) {
      std::scoped_lock lock(mutex_);
      running_ = false;
    }
  }

 private:
  std::mutex& mutex_;
  bool& running_;
  bool armed_{true};
};

class Stopwatch {
 public:
  using Message = hospital_interfaces::msg::Stopwatch;
  using Clock = std::chrono::steady_clock;

  Stopwatch(rclcpp::Publisher<Message>::SharedPtr publisher,
            const rclcpp::Logger& logger, std::string_view device_id,
            uint8_t kind, const rclcpp::Time& start_stamp,
            double nominal_seconds = 0.0)
      : publisher_(std::move(publisher)),
        logger_(logger),
        start_(Clock::now()) {
    message_.device_id = device_id;
    message_.kind = kind;
    message_.start_stamp = start_stamp;
    message_.nominal_seconds = static_cast<float>(nominal_seconds);
  }

  Stopwatch(const Stopwatch&) = delete;
  Stopwatch& operator=(const Stopwatch&) = delete;

  Clock::time_point start_time() const { return start_; }

  void tick() { publish(true, false); }

  void finish(bool interrupted) {
    if (!terminal_published_) {
      publish(false, interrupted);
      terminal_published_ = true;
    }
  }

  // Safe for a ScopeGuard during unwinding; publisher failures are still
  // logged.
  void interrupt_if_running() noexcept {
    try {
      finish(true);
    } catch (const rclcpp::exceptions::RCLInvalidArgument& error) {
      RCLCPP_ERROR(logger_, "Failed to finalize stopwatch for '%s': %s",
                   message_.device_id.c_str(), error.what());
    } catch (const std::runtime_error& error) {
      RCLCPP_ERROR(logger_, "Failed to finalize stopwatch for '%s': %s",
                   message_.device_id.c_str(), error.what());
    } catch (const std::bad_alloc& error) {
      RCLCPP_ERROR(logger_, "Failed to finalize stopwatch for '%s': %s",
                   message_.device_id.c_str(), error.what());
    }
  }

 private:
  void publish(bool running, bool interrupted) {
    message_.elapsed_seconds = static_cast<float>(
        std::chrono::duration<double>(Clock::now() - start_).count());
    message_.running = running;
    message_.interrupted = interrupted;
    message_.final_seconds = running ? 0.0f : message_.elapsed_seconds;
    publisher_->publish(message_);
  }

  rclcpp::Publisher<Message>::SharedPtr publisher_;
  rclcpp::Logger logger_;
  Clock::time_point start_;
  Message message_;
  bool terminal_published_{false};
};

template <typename Update>
bool animate_steps(std::stop_token stop_token, double duration_seconds,
                   double publish_rate, Update&& update,
                   std::chrono::steady_clock::time_point start =
                       std::chrono::steady_clock::now()) {
  using Clock = std::chrono::steady_clock;
  const int steps = std::max(
      1, static_cast<int>(std::lround(duration_seconds * publish_rate)));
  std::mutex wait_mutex;
  std::condition_variable_any wait_cv;

  for (int step = 1; step <= steps; ++step) {
    const double fraction = static_cast<double>(step) / steps;
    // Each deadline is anchored to the start, never to the preceding update.
    const auto deadline =
        start + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(duration_seconds * fraction));
    {
      std::unique_lock lock(wait_mutex);
      wait_cv.wait_until(lock, stop_token, deadline, [] { return false; });
    }
    if (stop_token.stop_requested()) {
      return false;
    }
    update(fraction);
  }
  return true;
}

}  // namespace hospital_controller

#endif  // HOSPITAL_CONTROLLER__CONTROLLER_UTILS_HPP_

// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_CONTROLLER__WORKER_THREAD_HPP_
#define HOSPITAL_CONTROLLER__WORKER_THREAD_HPP_

#include <exception>
#include <stop_token>  // NOLINT(build/include_order): C++20 header unknown to cpplint
#include <thread>
#include <utility>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace hospital_controller {

// Start a std::jthread running `work(stop_token)`. An exception escaping a
// thread function calls std::terminate and would take down the whole component
// container, so any exception is caught and logged instead.
template <typename Work>
[[nodiscard]] std::jthread make_worker_thread(const rclcpp::Logger& logger,
                                              Work&& work) {
  return std::jthread(
      [logger, work = std::forward<Work>(work)](std::stop_token stop_token) {
        try {
          work(stop_token);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(logger, "Worker thread terminated by exception: %s",
                       e.what());
        } catch (...) {
          RCLCPP_ERROR(logger, "Worker thread terminated by unknown exception");
        }
      });
}

}  // namespace hospital_controller

#endif  // HOSPITAL_CONTROLLER__WORKER_THREAD_HPP_

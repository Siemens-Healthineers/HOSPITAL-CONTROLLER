// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

// Room-dwell + task activity logger.
//
// A robot-agnostic telemetry recorder. It does NOT rely on any robot's
// internal state: it subscribes only to the public pose of each robot
// (/hospital/<id>/pose, geometry_msgs/PoseStamped in the world frame) and
// decides which room the robot is in purely from geometry (axis-aligned room
// rectangles + one L-shaped clinical corridor polygon). This works identically
// for the bundled simulation and for a third-party robot as long as it
// publishes that pose. Room transitions are debounced so a pose wobbling on a
// boundary does not spam enter/exit events, and each completed visit is logged
// with its dwell time.
//
// Task durations come from the ExecuteTask dispatch contract, not from
// geometry: the RViz panel is the authoritative timer for a ward task and
// forwards the exact elapsed value it displays via
// hospital_interfaces/TaskEvent on /hospital/task_events, so the logged task
// time matches the on-screen Mission dispatch table.
//
// Recording is toggled with a std_srvs/SetBool service
// (/hospital/logging/set_recording), which the panel's Start/Stop Recording
// buttons call. Each recording creates run_<timestamp>/ in the configured log
// directory with one <robot_id>/rooms.csv and <robot_id>/tasks.csv per robot;
// rows are appended (and flushed) while recording, and stopping writes a
// run.json summary and closes the files.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hospital_interfaces/msg/task_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace hospital_robot {

// An axis-aligned room region (rectangles) or an arbitrary polygon (clinical
// corridor).
struct RoomRegion {
  std::string name;
  bool is_polygon{false};
  // Rectangle bounds (valid when !is_polygon).
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
  // Polygon vertices as (x, y) pairs (valid when is_polygon).
  std::vector<std::pair<double, double>> polygon;

  bool contains(double x, double y) const {
    if (!is_polygon) {
      return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
    }
    // Ray-casting point-in-polygon test.
    bool inside = false;
    const std::size_t n = polygon.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
      const double xi = polygon[i].first;
      const double yi = polygon[i].second;
      const double xj = polygon[j].first;
      const double yj = polygon[j].second;
      const bool intersects =
          ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
      if (intersects) {
        inside = !inside;
      }
    }
    return inside;
  }
};

class ActivityLoggerNode : public rclcpp::Node {
 public:
  explicit ActivityLoggerNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("activity_logger_node", options) {
    declareParameters();
    loadRegions();

    // Subscribe to every robot's public pose. Match the robot's latched
    // (transient_local) QoS so a freshly started logger sees the last pose.
    rclcpp::QoS pose_qos(10);
    pose_qos.transient_local();
    for (const auto& robot_id : robot_ids_) {
      const std::string topic = "/hospital/" + robot_id + "/pose";
      auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic, pose_qos,
          [this,
           robot_id](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
            onPose(robot_id, msg);
          });
      pose_subs_.push_back(sub);
      tracks_[robot_id];  // default-construct the per-robot track.
      RCLCPP_INFO(this->get_logger(), "Tracking robot '%s' on %s.",
                  robot_id.c_str(), topic.c_str());
    }

    task_event_sub_ =
        this->create_subscription<hospital_interfaces::msg::TaskEvent>(
            task_event_topic_, 10,
            std::bind(&ActivityLoggerNode::onTaskEvent, this,
                      std::placeholders::_1));

    set_recording_srv_ = this->create_service<std_srvs::srv::SetBool>(
        set_recording_service_,
        std::bind(&ActivityLoggerNode::onSetRecording, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Activity logger ready (%zu robot(s), %zu room region(s)). "
                "Toggle recording via %s; logs -> %s.",
                robot_ids_.size(), regions_.size(),
                set_recording_service_.c_str(), log_directory_.c_str());
  }

  // Shutting down while recording must still close the files and write
  // run.json, otherwise the summary of the run is lost.
  ~ActivityLoggerNode() override {
    std::scoped_lock lock(mutex_);
    if (recording_) {
      stopRecording();
    }
  }

 private:
  // Per-robot room-occupancy tracking with hysteresis.
  struct Track {
    // The room the robot is currently considered to be in ("" = not yet known).
    std::string committed_room;
    rclcpp::Time committed_enter;
    // Candidate room seen but not yet committed, and when it first appeared.
    std::string candidate_room;
    rclcpp::Time candidate_since;
    bool initialised{false};
  };

  // One completed room visit. Each visit is recorded on its own; re-entering a
  // room later produces a SEPARATE visit and its dwell is never summed with the
  // earlier one.
  struct RoomVisit {
    std::string room;
    rclcpp::Time enter;
    rclcpp::Time exit;
    double dwell_s{0.0};
  };

  // One completed ward task. duration_s is the authoritative value forwarded by
  // the RViz panel (matches its Mission dispatch table).
  struct TaskRecord {
    std::string task_label;
    std::string task_id;
    rclcpp::Time start;
    rclcpp::Time end;
    double duration_s{0.0};
    bool success{false};
  };

  // ---- Parameters -----------------------------------------------------------

  void declareParameters() {
    robot_ids_ = this->declare_parameter<std::vector<std::string>>(
        "robot_ids", std::vector<std::string>{"robot_1"});
    std::erase_if(robot_ids_, [this](const std::string& robot_id) {
      if (isSafeRobotId(robot_id)) {
        return false;
      }
      RCLCPP_ERROR(this->get_logger(),
                   "Ignoring invalid robot id '%s' (allowed: 1-64 of "
                   "[A-Za-z0-9_-]).",
                   robot_id.c_str());
      return true;
    });

    // Axis-aligned rooms as parallel arrays: room_names[i] is bounded by
    // [room_min_x[i], room_max_x[i]] x [room_min_y[i], room_max_y[i]].
    rect_names_ = this->declare_parameter<std::vector<std::string>>(
        "room_names", std::vector<std::string>{});
    rect_min_x_ = this->declare_parameter<std::vector<double>>(
        "room_min_x", std::vector<double>{});
    rect_min_y_ = this->declare_parameter<std::vector<double>>(
        "room_min_y", std::vector<double>{});
    rect_max_x_ = this->declare_parameter<std::vector<double>>(
        "room_max_x", std::vector<double>{});
    rect_max_y_ = this->declare_parameter<std::vector<double>>(
        "room_max_y", std::vector<double>{});

    // One optional non-rectangular room (the L-shaped clinical corridor) as a
    // flat [x0, y0, x1, y1, ...] polygon.
    polygon_room_name_ =
        this->declare_parameter<std::string>("polygon_room_name", "");
    polygon_points_ = this->declare_parameter<std::vector<double>>(
        "polygon_points", std::vector<double>{});

    task_event_topic_ = this->declare_parameter<std::string>(
        "task_event_topic", "/hospital/task_events");
    set_recording_service_ = this->declare_parameter<std::string>(
        "set_recording_service", "/hospital/logging/set_recording");
    in_transit_label_ =
        this->declare_parameter<std::string>("in_transit_label", "in_transit");
    debounce_seconds_ =
        this->declare_parameter<double>("debounce_seconds", 0.5);

    log_directory_ = this->declare_parameter<std::string>("log_directory", "");
    if (log_directory_.empty()) {
      const char* home = std::getenv("HOME");
      log_directory_ =
          (home != nullptr ? std::string(home) : std::string(".")) +
          "/hospital_logs";
    }
  }

  void loadRegions() {
    const std::size_t count =
        std::min({rect_names_.size(), rect_min_x_.size(), rect_min_y_.size(),
                  rect_max_x_.size(), rect_max_y_.size()});
    if (rect_names_.size() != count) {
      RCLCPP_WARN(this->get_logger(),
                  "Room rectangle arrays have mismatched lengths; using the "
                  "first %zu.",
                  count);
    }
    for (std::size_t i = 0; i < count; ++i) {
      RoomRegion r;
      r.name = rect_names_[i];
      r.min_x = std::min(rect_min_x_[i], rect_max_x_[i]);
      r.max_x = std::max(rect_min_x_[i], rect_max_x_[i]);
      r.min_y = std::min(rect_min_y_[i], rect_max_y_[i]);
      r.max_y = std::max(rect_min_y_[i], rect_max_y_[i]);
      regions_.push_back(std::move(r));
    }

    if (!polygon_room_name_.empty() && polygon_points_.size() >= 6 &&
        polygon_points_.size() % 2 == 0) {
      RoomRegion r;
      r.name = polygon_room_name_;
      r.is_polygon = true;
      for (std::size_t i = 0; i + 1 < polygon_points_.size(); i += 2) {
        r.polygon.emplace_back(polygon_points_[i], polygon_points_[i + 1]);
      }
      regions_.push_back(std::move(r));
    } else if (!polygon_room_name_.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "polygon_room_name '%s' set but polygon_points is invalid "
                  "(need an even count >= 6); ignoring.",
                  polygon_room_name_.c_str());
    }
  }

  // Resolve the room a point lies in, or the in-transit label if none.
  std::string roomAt(double x, double y) const {
    for (const auto& region : regions_) {
      if (region.contains(x, y)) {
        return region.name;
      }
    }
    return in_transit_label_;
  }

  // ---- Pose handling / room detection --------------------------------------

  void onPose(const std::string& robot_id,
              geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
    // A robot that never sets a stamp would report time 0; fall back to now.
    if (stamp.nanoseconds() == 0) {
      stamp = this->now();
    }
    const std::string room = roomAt(msg->pose.position.x, msg->pose.position.y);

    std::scoped_lock lock(mutex_);
    Track& track = tracks_[robot_id];

    if (!track.initialised) {
      track.committed_room = room;
      track.committed_enter = stamp;
      track.candidate_room = room;
      track.candidate_since = stamp;
      track.initialised = true;
      return;
    }

    if (room == track.committed_room) {
      // Still in the committed room; drop any pending candidate.
      track.candidate_room = room;
      track.candidate_since = stamp;
      return;
    }

    // A different room than the committed one.
    if (room != track.candidate_room) {
      track.candidate_room = room;
      track.candidate_since = stamp;
      return;
    }

    // Same candidate as last time: commit once it has been stable long enough.
    if ((stamp - track.candidate_since).seconds() >= debounce_seconds_) {
      // The visit boundary is when the new room first appeared, so the dwell in
      // the old room excludes the debounce window.
      const rclcpp::Time boundary = track.candidate_since;
      if (recording_ && !track.committed_room.empty()) {
        recordRoomVisit(robot_id, track.committed_room, track.committed_enter,
                        boundary);
      }
      track.committed_room = track.candidate_room;
      track.committed_enter = boundary;
    }
  }

  // ---- Task events ----------------------------------------------------------

  void onTaskEvent(hospital_interfaces::msg::TaskEvent::ConstSharedPtr msg) {
    // robot_id arrives on an unauthenticated topic and becomes a directory
    // name below the run folder, so reject anything that could escape it.
    if (!isSafeRobotId(msg->robot_id)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Dropping task event with invalid robot id '%s'.",
                           msg->robot_id.c_str());
      return;
    }
    std::scoped_lock lock(mutex_);
    if (!recording_) {
      return;
    }
    rclcpp::Time stamp(msg->stamp, RCL_ROS_TIME);
    if (stamp.nanoseconds() == 0) {
      stamp = this->now();
    }
    if (msg->kind == hospital_interfaces::msg::TaskEvent::START) {
      // Remember when the task began so tasks.csv can carry a real start time.
      rememberTaskStart(msg->robot_id, msg->task_id, stamp);
    } else if (msg->kind == hospital_interfaces::msg::TaskEvent::END) {
      recordTask(msg->robot_id, msg->task_label, msg->task_id, msg->success,
                 msg->elapsed_seconds, stamp);
    }
  }

  // ---- Recording control ----------------------------------------------------

  // Store a task START stamp until its END arrives. STARTs whose END never
  // comes (lost message, crashed robot) would otherwise accumulate for the
  // whole run, so the total is capped and the oldest START is evicted when the
  // cap is reached (its task then falls back to the END's duration).
  void rememberTaskStart(const std::string& robot_id,
                         const std::string& task_id,
                         const rclcpp::Time& stamp) {
    auto& robot_starts = pending_task_starts_[robot_id];
    if (robot_starts.find(task_id) == robot_starts.end()) {
      std::size_t pending = 0;
      for (const auto& [id, starts] : pending_task_starts_) {
        pending += starts.size();
      }
      if (pending >= kMaxPendingTaskStarts) {
        evictOldestTaskStart();
      }
    }
    // Re-look up: the eviction may have erased this robot's (empty) map.
    pending_task_starts_[robot_id][task_id] = stamp;
  }

  void evictOldestTaskStart() {
    auto oldest_robot = pending_task_starts_.end();
    std::unordered_map<std::string, rclcpp::Time>::iterator oldest_task;
    for (auto robot_it = pending_task_starts_.begin();
         robot_it != pending_task_starts_.end(); ++robot_it) {
      for (auto task_it = robot_it->second.begin();
           task_it != robot_it->second.end(); ++task_it) {
        if (oldest_robot == pending_task_starts_.end() ||
            task_it->second.nanoseconds() < oldest_task->second.nanoseconds()) {
          oldest_robot = robot_it;
          oldest_task = task_it;
        }
      }
    }
    if (oldest_robot == pending_task_starts_.end()) {
      return;
    }
    RCLCPP_WARN(this->get_logger(),
                "Too many tasks without an END; forgetting the start of task "
                "'%s' (robot '%s').",
                oldest_task->first.c_str(), oldest_robot->first.c_str());
    oldest_robot->second.erase(oldest_task);
    if (oldest_robot->second.empty()) {
      pending_task_starts_.erase(oldest_robot);
    }
  }

  void onSetRecording(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    std::scoped_lock lock(mutex_);
    if (request->data) {
      if (recording_) {
        response->success = true;
        response->message = "Already recording to " + current_run_dir_;
        return;
      }
      if (!startRecording()) {
        response->success = false;
        response->message =
            "Failed to open a log directory in " + log_directory_;
        return;
      }
      response->success = true;
      response->message = "Recording to " + current_run_dir_;
    } else {
      if (!recording_) {
        response->success = true;
        response->message = "Not recording";
        return;
      }
      const std::string path = current_run_dir_;
      stopRecording();
      response->success = true;
      response->message = "Stopped; wrote " + path;
    }
  }

  bool startRecording() {
    std::error_code ec;
    std::filesystem::create_directories(log_directory_, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Cannot create log directory '%s': %s",
                   log_directory_.c_str(), ec.message().c_str());
      return false;
    }

    // One folder per recording session, named by wall-clock start time.
    current_run_dir_ =
        log_directory_ + "/run_" + wallTimestamp("%Y%m%d_%H%M%S");
    std::filesystem::create_directories(current_run_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Cannot create run directory '%s': %s",
                   current_run_dir_.c_str(), ec.message().c_str());
      return false;
    }

    // Fresh output streams + in-memory records for this run.
    rooms_files_.clear();
    tasks_files_.clear();
    visits_.clear();
    task_records_.clear();
    pending_task_starts_.clear();
    // Pre-create a folder + empty CSVs for every rostered robot so a run always
    // has a complete, predictable layout even for robots that never move.
    for (const auto& robot_id : robot_ids_) {
      ensureRobotFiles(robot_id);
    }

    // Start every known room dwell from now so a robot already sitting in a
    // room is timed from the moment recording began.
    const rclcpp::Time now = this->now();
    run_start_ = now;
    for (auto& [robot_id, track] : tracks_) {
      (void)robot_id;
      if (track.initialised) {
        track.committed_enter = now;
        // A room change pending since before the start must not produce a
        // visit boundary earlier than committed_enter (negative dwell).
        track.candidate_since = now;
      }
    }
    recording_ = true;
    RCLCPP_INFO(this->get_logger(), "Started recording to %s.",
                current_run_dir_.c_str());
    return true;
  }

  void stopRecording() {
    // Close out any in-progress room visits as a final (partial) visit.
    const rclcpp::Time now = this->now();
    for (auto& [robot_id, track] : tracks_) {
      if (track.initialised && !track.committed_room.empty()) {
        const double dwell = (now - track.committed_enter).seconds();
        if (dwell > 0.0) {
          recordRoomVisit(robot_id, track.committed_room, track.committed_enter,
                          now);
        }
        track.committed_enter = now;
      }
    }
    writeRunJson(now);
    for (auto& [robot_id, f] : rooms_files_) {
      (void)robot_id;
      if (f.is_open()) {
        f.close();
      }
    }
    for (auto& [robot_id, f] : tasks_files_) {
      (void)robot_id;
      if (f.is_open()) {
        f.close();
      }
    }
    recording_ = false;
    RCLCPP_INFO(this->get_logger(), "Stopped recording (%s).",
                current_run_dir_.c_str());
  }

  // ---- CSV / JSON writing (all callers hold mutex_) -------------------------

  // Open a robot's folder and CSV files (with headers) once per run. Called for
  // every rostered robot at start, and lazily for any unexpected robot id.
  void ensureRobotFiles(const std::string& robot_id) {
    if (rooms_files_.find(robot_id) != rooms_files_.end()) {
      return;
    }
    const std::string dir = current_run_dir_ + "/" + robot_id;
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Cannot create '%s': %s", dir.c_str(),
                   ec.message().c_str());
    }
    auto& rooms = rooms_files_[robot_id];
    rooms.open(dir + "/rooms.csv", std::ios::out | std::ios::trunc);
    if (rooms.is_open()) {
      rooms << "enter_time,exit_time,room,dwell_s\n";
      rooms.flush();
    }
    auto& tasks = tasks_files_[robot_id];
    tasks.open(dir + "/tasks.csv", std::ios::out | std::ios::trunc);
    if (tasks.is_open()) {
      tasks << "start_time,end_time,task_label,task_id,duration_s,success\n";
      tasks.flush();
    }
  }

  void recordRoomVisit(const std::string& robot_id, const std::string& room,
                       const rclcpp::Time& enter, const rclcpp::Time& exit) {
    ensureRobotFiles(robot_id);
    const double dwell_s = (exit - enter).seconds();
    auto& rooms = rooms_files_[robot_id];
    if (rooms.is_open()) {
      rooms << isoTime(enter) << ',' << isoTime(exit) << ',' << csvField(room)
            << ',' << oneDecimal(dwell_s) << '\n';
      rooms.flush();
    }
    // Each visit is kept separately (never summed with earlier visits to the
    // same room) for the run.json summary.
    visits_[robot_id].push_back(RoomVisit{room, enter, exit, dwell_s});
  }

  void recordTask(const std::string& robot_id, const std::string& task_label,
                  const std::string& task_id, bool success, double duration_s,
                  const rclcpp::Time& end) {
    ensureRobotFiles(robot_id);
    // Pair the END with its earlier START if we saw one; otherwise derive the
    // start from the panel's authoritative duration (logger began mid-task).
    rclcpp::Time start = end - rclcpp::Duration::from_seconds(duration_s);
    auto robot_it = pending_task_starts_.find(robot_id);
    if (robot_it != pending_task_starts_.end()) {
      auto id_it = robot_it->second.find(task_id);
      if (id_it != robot_it->second.end()) {
        start = id_it->second;
        robot_it->second.erase(id_it);
      }
      if (robot_it->second.empty()) {
        pending_task_starts_.erase(robot_it);
      }
    }
    auto& tasks = tasks_files_[robot_id];
    if (tasks.is_open()) {
      tasks << isoTime(start) << ',' << isoTime(end) << ','
            << csvField(task_label) << ',' << csvField(task_id) << ','
            << oneDecimal(duration_s) << ',' << (success ? "true" : "false")
            << '\n';
      tasks.flush();
    }
    task_records_[robot_id].push_back(
        TaskRecord{task_label, task_id, start, end, duration_s, success});
  }

  // Structured per-run summary. Rooms are listed per visit (not summed), so a
  // robot that returns to a room shows each stay individually.
  void writeRunJson(const rclcpp::Time& stopped) {
    const std::string path = current_run_dir_ + "/run.json";
    std::ofstream json(path, std::ios::out | std::ios::trunc);
    if (!json.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot write '%s'.", path.c_str());
      return;
    }

    // Robots to report: the roster plus any that produced records.
    std::vector<std::string> robots = robot_ids_;
    for (const auto& [robot_id, _] : visits_) {
      if (std::find(robots.begin(), robots.end(), robot_id) == robots.end()) {
        robots.push_back(robot_id);
      }
    }
    for (const auto& [robot_id, _] : task_records_) {
      if (std::find(robots.begin(), robots.end(), robot_id) == robots.end()) {
        robots.push_back(robot_id);
      }
    }

    json << "{\n";
    json << "  \"run\": {\n";
    json << "    \"started\": \"" << isoTime(run_start_) << "\",\n";
    json << "    \"stopped\": \"" << isoTime(stopped) << "\",\n";
    json << "    \"robots\": [";
    for (std::size_t i = 0; i < robots.size(); ++i) {
      json << (i != 0 ? ", " : "") << '"' << jsonEscape(robots[i]) << '"';
    }
    json << "]\n";
    json << "  },\n";
    json << "  \"robots\": {\n";
    for (std::size_t r = 0; r < robots.size(); ++r) {
      const std::string& robot_id = robots[r];
      json << "    \"" << jsonEscape(robot_id) << "\": {\n";

      json << "      \"rooms\": [";
      const auto vit = visits_.find(robot_id);
      if (vit != visits_.end() && !vit->second.empty()) {
        const auto& vv = vit->second;
        json << "\n";
        for (std::size_t i = 0; i < vv.size(); ++i) {
          json << "        {\"room\": \"" << jsonEscape(vv[i].room)
               << "\", \"enter\": \"" << isoTime(vv[i].enter)
               << "\", \"exit\": \"" << isoTime(vv[i].exit)
               << "\", \"dwell_s\": " << oneDecimal(vv[i].dwell_s) << "}"
               << (i + 1 < vv.size() ? "," : "") << "\n";
        }
        json << "      ],\n";
      } else {
        json << "],\n";
      }

      json << "      \"tasks\": [";
      const auto tit = task_records_.find(robot_id);
      if (tit != task_records_.end() && !tit->second.empty()) {
        const auto& tt = tit->second;
        json << "\n";
        for (std::size_t i = 0; i < tt.size(); ++i) {
          json << "        {\"task_label\": \"" << jsonEscape(tt[i].task_label)
               << "\", \"task_id\": \"" << jsonEscape(tt[i].task_id)
               << "\", \"start\": \"" << isoTime(tt[i].start)
               << "\", \"end\": \"" << isoTime(tt[i].end)
               << "\", \"duration_s\": " << oneDecimal(tt[i].duration_s)
               << ", \"success\": " << (tt[i].success ? "true" : "false") << "}"
               << (i + 1 < tt.size() ? "," : "") << "\n";
        }
        json << "      ]\n";
      } else {
        json << "]\n";
      }

      json << "    }" << (r + 1 < robots.size() ? "," : "") << "\n";
    }
    json << "  }\n";
    json << "}\n";
    json.flush();
  }

  // ---- Small helpers --------------------------------------------------------

  // A robot id is used as a path component and in topic names, so only allow
  // a conservative character set (this also excludes "." and "..").
  static bool isSafeRobotId(const std::string& robot_id) {
    constexpr std::size_t kMaxLength = 64;
    if (robot_id.empty() || robot_id.size() > kMaxLength) {
      return false;
    }
    return std::all_of(robot_id.begin(), robot_id.end(), [](char c) {
      return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
             (c >= '0' && c <= '9') || c == '_' || c == '-';
    });
  }

  // One-decimal seconds so the logged value reads identically to the panel's
  // Mission dispatch table (which formats elapsed as "X.X s").
  static std::string oneDecimal(double seconds) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", seconds);
    return buf;
  }

  // Quote a free-text CSV field if it contains a delimiter, quote, or line
  // break. Values starting with a spreadsheet formula trigger (= + - @) are
  // prefixed with a single quote so they are not evaluated when opened in a
  // spreadsheet application.
  static std::string csvField(const std::string& value) {
    std::string safe = value;
    if (!safe.empty() && (safe.front() == '=' || safe.front() == '+' ||
                          safe.front() == '-' || safe.front() == '@')) {
      safe.insert(safe.begin(), '\'');
    }
    if (safe.find_first_of(",\"\r\n") == std::string::npos) {
      return safe;
    }
    std::string escaped = "\"";
    for (const char c : safe) {
      if (c == '"') {
        escaped += '"';
      }
      escaped += c;
    }
    escaped += '"';
    return escaped;
  }

  static std::string wallTimestamp(const char* format) {
    const std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);
    char buf[64];
    std::strftime(buf, sizeof(buf), format, &tm_buf);
    return buf;
  }

  // Format a ROS time as local wall-clock ISO 8601 with milliseconds. ROS time
  // maps to the system clock here (no sim time), so this is a real timestamp.
  static std::string isoTime(const rclcpp::Time& t) {
    const double secs = t.seconds();
    const std::time_t whole = static_cast<std::time_t>(secs);
    std::tm tm_buf{};
    localtime_r(&whole, &tm_buf);
    char date[32];
    std::strftime(date, sizeof(date), "%Y-%m-%dT%H:%M:%S", &tm_buf);
    int ms = static_cast<int>((secs - static_cast<double>(whole)) * 1000.0);
    if (ms < 0) {
      ms = 0;
    }
    char out[48];
    std::snprintf(out, sizeof(out), "%s.%03d", date, ms);
    return out;
  }

  // Escape a string for embedding in a JSON string literal.
  static std::string jsonEscape(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (const char c : value) {
      switch (c) {
        case '"':
          out += "\\\"";
          break;
        case '\\':
          out += "\\\\";
          break;
        case '\n':
          out += "\\n";
          break;
        case '\r':
          out += "\\r";
          break;
        case '\t':
          out += "\\t";
          break;
        default:
          if (static_cast<unsigned char>(c) < 0x20) {
            char buf[8];
            std::snprintf(
                buf, sizeof(buf), "\\u%04x",
                static_cast<unsigned int>(static_cast<unsigned char>(c)));
            out += buf;
          } else {
            out += c;
          }
      }
    }
    return out;
  }

  // ---- Members --------------------------------------------------------------

  std::vector<std::string> robot_ids_;
  std::vector<std::string> rect_names_;
  std::vector<double> rect_min_x_;
  std::vector<double> rect_min_y_;
  std::vector<double> rect_max_x_;
  std::vector<double> rect_max_y_;
  std::string polygon_room_name_;
  std::vector<double> polygon_points_;
  std::vector<RoomRegion> regions_;

  std::string task_event_topic_;
  std::string set_recording_service_;
  std::string in_transit_label_;
  double debounce_seconds_{0.5};
  std::string log_directory_;

  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      pose_subs_;
  rclcpp::Subscription<hospital_interfaces::msg::TaskEvent>::SharedPtr
      task_event_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_recording_srv_;

  // All state below is guarded by mutex_ (callbacks may share a multi-threaded
  // executor).
  std::mutex mutex_;
  std::unordered_map<std::string, Track> tracks_;
  bool recording_{false};
  std::string current_run_dir_;
  rclcpp::Time run_start_;
  // Per-robot output streams (rooms.csv / tasks.csv) and the in-memory records
  // dumped to run.json when recording stops.
  std::unordered_map<std::string, std::ofstream> rooms_files_;
  std::unordered_map<std::string, std::ofstream> tasks_files_;
  std::unordered_map<std::string, std::vector<RoomVisit>> visits_;
  std::unordered_map<std::string, std::vector<TaskRecord>> task_records_;
  // Task START stamps awaiting their END, keyed [robot_id][task_id]; at most
  // kMaxPendingTaskStarts in total.
  static constexpr std::size_t kMaxPendingTaskStarts = 1024;
  std::unordered_map<std::string, std::unordered_map<std::string, rclcpp::Time>>
      pending_task_starts_;
};

}  // namespace hospital_robot

RCLCPP_COMPONENTS_REGISTER_NODE(hospital_robot::ActivityLoggerNode)

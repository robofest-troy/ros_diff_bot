// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS_DIFF_BOT__DIFFBOT_SYSTEM_HPP_
#define ROS_DIFF_BOT__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstdint>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// CAN communication headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

namespace ros_diff_bot
{

// Structure to hold CAN wheel configuration
struct CANWheelConfig {
  uint32_t can_id;
  std::string joint_name;
  double velocity_scale;
  double direction_multiplier;
};

// Structure to hold CAN interface configuration
struct CANInterfaceConfig {
  std::string device_name;
  uint32_t bitrate;
  uint32_t timeout_ms;
  uint32_t broadcast_interval_ms;
  uint32_t max_retries;
  std::unordered_map<std::string, CANWheelConfig> wheels;
  
  // Message format configuration
  uint8_t data_length;
  uint8_t velocity_byte_start;
  uint8_t velocity_byte_length;
  uint8_t status_byte;
  uint8_t checksum_byte;
  
  // Safety configuration
  double max_velocity_command;
  uint32_t emergency_stop_can_id;
  bool enable_watchdog;
  uint32_t watchdog_timeout_ms;
  
  // Monitoring configuration
  bool enable_can_monitoring;
  bool enable_heartbeat;
  uint32_t heartbeat_interval_ms;
  uint32_t heartbeat_can_id;
};

class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  
  // Logger
  // rclcpp::Logger has a private default constructor, so initialize here
  rclcpp::Logger logger_{rclcpp::get_logger("DiffBotSystemHardware")};
  rclcpp::Clock::SharedPtr clock_;
  
  // CAN communication members
  CANInterfaceConfig can_config_;
  int can_socket_{-1};
  std::atomic<bool> can_active_;
  std::thread can_broadcast_thread_;
  std::chrono::steady_clock::time_point last_command_time_;
  
  // Wheel velocity commands (thread-safe access)
  std::unordered_map<std::string, std::atomic<double>> wheel_velocity_commands_;
  
  // CAN communication methods
  bool initializeCANInterface();
  void closeCANInterface();
  bool sendCANMessage(uint32_t can_id, const uint8_t* data, uint8_t length);
  void canBroadcastLoop();
  bool loadCANConfiguration();
  void createVelocityCANMessage(const CANWheelConfig& wheel_config, double velocity, can_frame& frame);
  void sendHeartbeatMessage();
  bool isCommandTimeout();
  uint8_t calculateChecksum(const uint8_t* data, uint8_t length);
};

}  // namespace ros_diff_bot

#endif  // ROS_DIFF_BOT__DIFFBOT_SYSTEM_HPP_

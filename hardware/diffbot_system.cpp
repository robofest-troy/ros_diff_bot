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

#include "ros_diff_bot/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <cstring>
#include <cerrno>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros_diff_bot
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Clock initialization (logger_ initialized in header)
  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // Load CAN configuration from parameters
  if (!loadCANConfiguration())
  {
    RCLCPP_ERROR(logger_, "Failed to load CAN configuration");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize velocity command storage for each wheel
  for (const auto& [wheel_name, wheel_config] : can_config_.wheels)
  {
    wheel_velocity_commands_[wheel_name] = 0.0;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(logger_, "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(logger_, "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // Initialize CAN interface
  if (!initializeCANInterface())
  {
    RCLCPP_ERROR(logger_, "Failed to initialize CAN interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  
  // Reset wheel velocity commands
  for (auto& [wheel_name, velocity] : wheel_velocity_commands_)
  {
    velocity = 0.0;
  }
  
  RCLCPP_INFO(logger_, "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(logger_, "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(logger_, "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  // Start CAN broadcast thread
  can_active_ = true;
  can_broadcast_thread_ = std::thread(&DiffBotSystemHardware::canBroadcastLoop, this);
  last_command_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(logger_, "Successfully activated! CAN broadcast started.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(logger_, "Deactivating ...please wait...");

  // Stop CAN broadcast thread
  can_active_ = false;
  if (can_broadcast_thread_.joinable())
  {
    can_broadcast_thread_.join();
  }

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(logger_, "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // Close CAN interface
  closeCANInterface();

  RCLCPP_INFO(logger_, "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";
  ss << std::fixed << std::setprecision(2);
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      // Simulate DiffBot wheels's movement as a first-order system
      // Update the joint status: this is a revolute joint without any limit.
      // Simply integrates
      auto velo = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_VELOCITY);
      set_state(name, get_state(name) + period.seconds() * velo);

      ss << std::endl
         << "\t position " << get_state(name) << " and velocity " << velo << " for '" << name
         << "'!";
    }
  }
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    // Simulate sending commands to the hardware
    set_state(name, get_command(name));
    
    // Update wheel velocity commands for CAN broadcasting
    std::string joint_name = descr.get_prefix_name();
    for (const auto& [wheel_name, wheel_config] : can_config_.wheels)
    {
      if (wheel_config.joint_name == joint_name)
      {
        double velocity = get_command(name);
        
        // Apply safety limits
        if (std::abs(velocity) > can_config_.max_velocity_command)
        {
          velocity = (velocity > 0) ? can_config_.max_velocity_command : -can_config_.max_velocity_command;
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, 
                               "Velocity command for %s exceeded limit, clamped to %.2f", 
                               wheel_name.c_str(), velocity);
        }
        
        wheel_velocity_commands_[wheel_name] = velocity * wheel_config.direction_multiplier;
        break;
      }
    }

    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << "command " << get_command(name) << " for '" << name << "'!";
  }
  
  // Update last command time for watchdog
  last_command_time_ = std::chrono::steady_clock::now();
  
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros_diff_bot

// CAN Communication Methods Implementation
namespace ros_diff_bot
{

bool DiffBotSystemHardware::loadCANConfiguration()
{
  // Load parameters from hardware_interface::HardwareInfo
  try {
    // CAN Interface settings
    can_config_.device_name = info_.hardware_parameters.at("can_device");
    can_config_.bitrate = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_bitrate")));
    can_config_.timeout_ms = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.count("can_timeout_ms") ? 
                                       info_.hardware_parameters.at("can_timeout_ms") : "100"));
    can_config_.broadcast_interval_ms = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("broadcast_interval_ms")));
    can_config_.max_retries = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.count("max_retries") ? 
                                        info_.hardware_parameters.at("max_retries") : "3"));
    
    // Wheel configurations
    CANWheelConfig left_wheel;
    left_wheel.can_id = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("left_wheel_can_id"), nullptr, 16));
    left_wheel.joint_name = "left_wheel_joint";
    left_wheel.velocity_scale = std::stod(info_.hardware_parameters.count("velocity_scale") ? 
                                         info_.hardware_parameters.at("velocity_scale") : "1000.0");
    left_wheel.direction_multiplier = std::stod(info_.hardware_parameters.count("left_wheel_direction") ? 
                                               info_.hardware_parameters.at("left_wheel_direction") : "1.0");
    can_config_.wheels["left_wheel"] = left_wheel;
    
    CANWheelConfig right_wheel;
    right_wheel.can_id = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("right_wheel_can_id"), nullptr, 16));
    right_wheel.joint_name = "right_wheel_joint";
    right_wheel.velocity_scale = std::stod(info_.hardware_parameters.count("velocity_scale") ? 
                                          info_.hardware_parameters.at("velocity_scale") : "1000.0");
    right_wheel.direction_multiplier = std::stod(info_.hardware_parameters.count("right_wheel_direction") ? 
                                                info_.hardware_parameters.at("right_wheel_direction") : "1.0");
    can_config_.wheels["right_wheel"] = right_wheel;
    
    // Message format
    can_config_.data_length = static_cast<uint8_t>(std::stoul(info_.hardware_parameters.count("data_length") ? 
                                        info_.hardware_parameters.at("data_length") : "8"));
    can_config_.velocity_byte_start = static_cast<uint8_t>(std::stoul(info_.hardware_parameters.count("velocity_byte_start") ? 
                                                info_.hardware_parameters.at("velocity_byte_start") : "0"));
    can_config_.velocity_byte_length = static_cast<uint8_t>(std::stoul(info_.hardware_parameters.count("velocity_byte_length") ? 
                                                 info_.hardware_parameters.at("velocity_byte_length") : "4"));
    can_config_.status_byte = static_cast<uint8_t>(std::stoul(info_.hardware_parameters.count("status_byte") ? 
                                        info_.hardware_parameters.at("status_byte") : "4"));
    can_config_.checksum_byte = static_cast<uint8_t>(std::stoul(info_.hardware_parameters.count("checksum_byte") ? 
                                          info_.hardware_parameters.at("checksum_byte") : "7"));
    
    // Safety configuration
    can_config_.max_velocity_command = std::stod(info_.hardware_parameters.count("max_velocity_command") ? 
                                                info_.hardware_parameters.at("max_velocity_command") : "10.0");
    can_config_.emergency_stop_can_id = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.count("emergency_stop_can_id") ? 
                                                  info_.hardware_parameters.at("emergency_stop_can_id") : "0x080", nullptr, 16));
    
    std::string enable_watchdog_str = info_.hardware_parameters.count("enable_watchdog") ? 
                                     info_.hardware_parameters.at("enable_watchdog") : "true";
    can_config_.enable_watchdog = (enable_watchdog_str == "true");
    can_config_.watchdog_timeout_ms = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.count("watchdog_timeout_ms") ? 
                                                 info_.hardware_parameters.at("watchdog_timeout_ms") : "500"));
    
    // Monitoring configuration
    std::string enable_can_monitoring_str = info_.hardware_parameters.count("enable_can_monitoring") ? 
                                           info_.hardware_parameters.at("enable_can_monitoring") : "true";
    can_config_.enable_can_monitoring = (enable_can_monitoring_str == "true");
    
    std::string enable_heartbeat_str = info_.hardware_parameters.count("enable_heartbeat") ? 
                                      info_.hardware_parameters.at("enable_heartbeat") : "true";
    can_config_.enable_heartbeat = (enable_heartbeat_str == "true");
    can_config_.heartbeat_interval_ms = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.count("heartbeat_interval_ms") ? 
                                                  info_.hardware_parameters.at("heartbeat_interval_ms") : "1000"));
    can_config_.heartbeat_can_id = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.count("heartbeat_can_id") ? 
                                             info_.hardware_parameters.at("heartbeat_can_id") : "0x700", nullptr, 16));
    
    RCLCPP_INFO(logger_, "CAN Configuration loaded successfully:");
    RCLCPP_INFO(logger_, "  Device: %s", can_config_.device_name.c_str());
    RCLCPP_INFO(logger_, "  Bitrate: %u bps", can_config_.bitrate);
    RCLCPP_INFO(logger_, "  Broadcast Interval: %u ms", can_config_.broadcast_interval_ms);
    RCLCPP_INFO(logger_, "  Left Wheel CAN ID: 0x%03X", can_config_.wheels["left_wheel"].can_id);
    RCLCPP_INFO(logger_, "  Right Wheel CAN ID: 0x%03X", can_config_.wheels["right_wheel"].can_id);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to load CAN configuration: %s", e.what());
    
    // Set default values as fallback
    can_config_.device_name = "can0";
    can_config_.bitrate = 500000;
    can_config_.timeout_ms = 100;
    can_config_.broadcast_interval_ms = 10;
    can_config_.max_retries = 3;
    
    // Default wheel configurations
    CANWheelConfig left_wheel;
    left_wheel.can_id = 0x101;
    left_wheel.joint_name = "left_wheel_joint";
    left_wheel.velocity_scale = 1000.0;
    left_wheel.direction_multiplier = 1.0;
    can_config_.wheels["left_wheel"] = left_wheel;
    
    CANWheelConfig right_wheel;
    right_wheel.can_id = 0x102;
    right_wheel.joint_name = "right_wheel_joint";
    right_wheel.velocity_scale = 1000.0;
    right_wheel.direction_multiplier = 1.0;
    can_config_.wheels["right_wheel"] = right_wheel;
    
    // Default message format
    can_config_.data_length = 8;
    can_config_.velocity_byte_start = 0;
    can_config_.velocity_byte_length = 4;
    can_config_.status_byte = 4;
    can_config_.checksum_byte = 7;
    
    // Default safety configuration
    can_config_.max_velocity_command = 10.0;
    can_config_.emergency_stop_can_id = 0x080;
    can_config_.enable_watchdog = true;
    can_config_.watchdog_timeout_ms = 500;
    
    // Default monitoring configuration
    can_config_.enable_can_monitoring = true;
    can_config_.enable_heartbeat = true;
    can_config_.heartbeat_interval_ms = 1000;
    can_config_.heartbeat_can_id = 0x700;
    
    RCLCPP_WARN(logger_, "Using default CAN configuration due to parameter loading failure");
  }
  
  return true;
}

bool DiffBotSystemHardware::initializeCANInterface()
{
  // Create CAN socket
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0)
  {
    RCLCPP_ERROR(logger_, "Failed to create CAN socket: %s", strerror(errno));
    return false;
  }
  
  // Get interface index
  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, can_config_.device_name.c_str());
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_ERROR(logger_, "Failed to get CAN interface index for %s: %s", 
                 can_config_.device_name.c_str(), strerror(errno));
    close(can_socket_);
    return false;
  }
  
  // Bind socket to CAN interface
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  
  if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(logger_, "Failed to bind CAN socket to %s: %s", 
                 can_config_.device_name.c_str(), strerror(errno));
    close(can_socket_);
    return false;
  }
  
  // Set socket timeout
  struct timeval timeout;
  timeout.tv_sec = can_config_.timeout_ms / 1000;
  timeout.tv_usec = (can_config_.timeout_ms % 1000) * 1000;
  
  if (setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
  {
    RCLCPP_WARN(logger_, "Failed to set CAN socket timeout: %s", strerror(errno));
  }
  
  RCLCPP_INFO(logger_, "CAN interface %s initialized successfully (socket fd: %d)", 
              can_config_.device_name.c_str(), can_socket_);
  return true;
}

void DiffBotSystemHardware::closeCANInterface()
{
  if (can_socket_ >= 0)
  {
    close(can_socket_);
    can_socket_ = -1;
    RCLCPP_INFO(logger_, "CAN interface closed");
  }
}

bool DiffBotSystemHardware::sendCANMessage(uint32_t can_id, const uint8_t* data, uint8_t length)
{
  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = length;
  std::memcpy(frame.data, data, length);
  
  // Use global POSIX ::write, not the class's write() method
  ssize_t bytes_sent = ::write(can_socket_, &frame, sizeof(frame));
  if (bytes_sent != sizeof(frame))
  {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, 
                          "Failed to send CAN message ID: 0x%03X", can_id);
    return false;
  }
  
  // Log successful CAN message transmission (throttled to avoid spam)
  RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000,
                         "CAN TX: ID=0x%03X, DLC=%u, Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
                         can_id, length,
                         data[0], data[1], data[2], data[3],
                         data[4], data[5], data[6], data[7]);
  
  return true;
}

void DiffBotSystemHardware::createVelocityCANMessage(const CANWheelConfig& wheel_config, 
                                                     double velocity, can_frame& frame)
{
  frame.can_id = wheel_config.can_id;
  frame.can_dlc = can_config_.data_length;
  
  // Clear frame data
  std::memset(frame.data, 0, sizeof(frame.data));
  
  // Convert velocity to scaled integer
  int32_t scaled_velocity = static_cast<int32_t>(velocity * wheel_config.velocity_scale);
  
  // Pack velocity data (little-endian)
  frame.data[can_config_.velocity_byte_start] = static_cast<uint8_t>(scaled_velocity & 0xFF);
  frame.data[can_config_.velocity_byte_start + 1] = static_cast<uint8_t>((scaled_velocity >> 8) & 0xFF);
  frame.data[can_config_.velocity_byte_start + 2] = static_cast<uint8_t>((scaled_velocity >> 16) & 0xFF);
  frame.data[can_config_.velocity_byte_start + 3] = static_cast<uint8_t>((scaled_velocity >> 24) & 0xFF);
  
  // Set status byte (0x01 = normal operation)
  frame.data[can_config_.status_byte] = 0x01;
  
  // Calculate and set checksum
  frame.data[can_config_.checksum_byte] = calculateChecksum(frame.data, can_config_.checksum_byte);
}

uint8_t DiffBotSystemHardware::calculateChecksum(const uint8_t* data, uint8_t length)
{
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length; i++)
  {
    checksum ^= data[i];
  }
  return checksum;
}

void DiffBotSystemHardware::canBroadcastLoop()
{
  auto last_heartbeat = std::chrono::steady_clock::now();
  uint64_t loop_count = 0;
  
  RCLCPP_INFO(logger_, "CAN broadcast loop started - sending messages every %u ms", 
              can_config_.broadcast_interval_ms);
  
  while (can_active_)
  {
    auto loop_start = std::chrono::steady_clock::now();
    loop_count++;
    
    // Check for command timeout (watchdog)
    if (can_config_.enable_watchdog && isCommandTimeout())
    {
      // Stop all wheels on timeout
      for (auto& [wheel_name, velocity] : wheel_velocity_commands_)
      {
        velocity = 0.0;
      }
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "Command timeout - stopping all wheels");
    }
    
    // Send velocity commands for each wheel
    for (const auto& [wheel_name, wheel_config] : can_config_.wheels)
    {
      double velocity = wheel_velocity_commands_[wheel_name].load();
      
      can_frame frame;
      createVelocityCANMessage(wheel_config, velocity, frame);
      
      if (!sendCANMessage(frame.can_id, frame.data, frame.can_dlc))
      {
        RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, 
                              "Failed to send velocity command for %s", wheel_name.c_str());
      }
      else
      {
        // Log periodic status of CAN transmission
        if (loop_count % 1000 == 0) // Every 10 seconds at 10ms intervals
        {
          RCLCPP_INFO(logger_, "CAN Status: %s velocity=%.3f m/s, CAN_ID=0x%03X, scaled_vel=%d", 
                     wheel_name.c_str(), velocity, wheel_config.can_id, 
                     static_cast<int32_t>(velocity * wheel_config.velocity_scale));
        }
      }
    }
    
    // Send heartbeat message if enabled
    if (can_config_.enable_heartbeat)
    {
      auto now = std::chrono::steady_clock::now();
      auto heartbeat_interval = std::chrono::milliseconds(can_config_.heartbeat_interval_ms);
      
      if (now - last_heartbeat >= heartbeat_interval)
      {
        sendHeartbeatMessage();
        last_heartbeat = now;
        RCLCPP_DEBUG(logger_, "Heartbeat message sent on CAN ID 0x%03X", can_config_.heartbeat_can_id);
      }
    }
    
    // Sleep for the remaining time to maintain broadcast interval
    auto loop_end = std::chrono::steady_clock::now();
    auto loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
    auto sleep_duration = std::chrono::milliseconds(can_config_.broadcast_interval_ms) - loop_duration;
    
    if (sleep_duration > std::chrono::milliseconds(0))
    {
      std::this_thread::sleep_for(sleep_duration);
    }
    else if (loop_count % 100 == 0) // Log timing issues every second
    {
      RCLCPP_WARN(logger_, "CAN broadcast loop running behind schedule: took %ld ms", 
                 loop_duration.count());
    }
  }
  
  RCLCPP_INFO(logger_, "CAN broadcast loop stopped after %lu iterations", loop_count);
}

void DiffBotSystemHardware::sendHeartbeatMessage()
{
  uint8_t heartbeat_data[8] = {0};
  auto now = std::chrono::steady_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  
  // Pack timestamp in heartbeat message
  heartbeat_data[0] = static_cast<uint8_t>(timestamp & 0xFF);
  heartbeat_data[1] = static_cast<uint8_t>((timestamp >> 8) & 0xFF);
  heartbeat_data[2] = static_cast<uint8_t>((timestamp >> 16) & 0xFF);
  heartbeat_data[3] = static_cast<uint8_t>((timestamp >> 24) & 0xFF);
  heartbeat_data[4] = 0xAA; // Heartbeat signature
  heartbeat_data[5] = 0x55; // Heartbeat signature
  
  if (!sendCANMessage(can_config_.heartbeat_can_id, heartbeat_data, 8))
  {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 5000, "Failed to send heartbeat message");
  }
}

bool DiffBotSystemHardware::isCommandTimeout()
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time_).count();
  return elapsed > can_config_.watchdog_timeout_ms;
}

}  // namespace ros_diff_bot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros_diff_bot::DiffBotSystemHardware, hardware_interface::SystemInterface)

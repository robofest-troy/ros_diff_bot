# ROS DiffBot - Standalone Differential Drive Robot Package with CAN Communication

This package is a standalone implementation of a differential drive robot (DiffBot) with enhanced CAN communication capabilities. It provides a complete ROS 2 package that broadcasts wheel velocity commands via CAN bus at precise intervals.

## Overview

The ROS DiffBot package demonstrates:
- Hardware interface implementation for a differential drive robot with CAN communication
- Real-time CAN message broadcasting (10ms intervals)
- ros2_control integration with configurable CAN parameters
- Controller configuration (diff_drive_controller, joint_state_broadcaster)  
- Simulation capabilities with mock hardware
- URDF robot description
- Launch files for robot visualization and control
- CAN message monitoring and debugging tools

## Features

- **CAN Communication**: Real-time wheel velocity commands via CAN bus
- **Hardware Interface**: Custom DiffBotSystemHardware class with CAN integration
- **Controllers**: Differential drive controller for mobile base control
- **Configuration**: YAML-based CAN parameter configuration
- **Visualization**: RViz configuration for robot visualization
- **Mock Hardware**: Option to run with simulated hardware for testing
- **Complete URDF**: Robot description with differential drive kinematics
- **Monitoring Tools**: CAN message logging and debugging utilities

## Prerequisites

- ROS 2 (Humble, Iron, Jazz, Rolling)
- ros2_control packages
- diff_drive_controller package
- joint_state_broadcaster package
- can-utils package (`sudo apt install can-utils`)
- Linux SocketCAN support

## Quick Start with CAN Logging

### 1. Setup Virtual CAN (for testing)
```bash
cd ~/ros2_ws/src/ros_diff_bot
./setup_can.sh setup-vcan vcan0
```

### 2. Launch with CAN Message Logging
```bash
# Terminal 1: Launch robot with debug logging
ros2 launch ros_diff_bot diffbot_can.launch.py \
  can_device:=vcan0 \
  use_mock_hardware:=true \
  log_level:=debug

# Terminal 2: Monitor CAN messages
./setup_can.sh monitor vcan0

# Terminal 3: Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.5}}' --rate 1
```

### 3. View CAN Messages in Logs

The enhanced logging shows detailed CAN communication:

```bash
# View real-time logs with CAN messages
ros2 node info /controller_manager

# Check log files for CAN communication details
./test_can_messages.sh
# Choose option 6 to show logs with CAN messages
```

## Installation

1. Clone this repository to your ROS 2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository_url> ros_diff_bot
```

2. Install CAN utilities:
```bash
sudo apt update
sudo apt install can-utils
```

3. Build the package:
```bash
cd ~/ros2_ws/src/ros_diff_bot
colcon build --packages-select ros_diff_bot
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Viewing CAN Messages in Logs

The enhanced hardware interface provides detailed logging of CAN communication. Here's how to see CAN messages in the logs:

### 1. Enable Debug Logging

Launch with debug logging to see detailed CAN message information:

```bash
ros2 launch ros_diff_bot diffbot_can.launch.py \
  can_device:=vcan0 \
  use_mock_hardware:=true \
  log_level:=debug
```

### 2. Monitor Console Output

You'll see log messages like:

```
[INFO] [diffbot_system_hardware]: CAN Configuration loaded successfully:
[INFO] [diffbot_system_hardware]:   Device: vcan0
[INFO] [diffbot_system_hardware]:   Bitrate: 500000 bps
[INFO] [diffbot_system_hardware]:   Broadcast Interval: 10 ms
[INFO] [diffbot_system_hardware]:   Left Wheel CAN ID: 0x101
[INFO] [diffbot_system_hardware]:   Right Wheel CAN ID: 0x102
[INFO] [diffbot_system_hardware]: CAN broadcast loop started - sending messages every 10 ms
[INFO] [diffbot_system_hardware]: CAN Status: left_wheel velocity=1.500 m/s, CAN_ID=0x101, scaled_vel=1500
[INFO] [diffbot_system_hardware]: CAN Status: right_wheel velocity=0.500 m/s, CAN_ID=0x102, scaled_vel=500
[DEBUG] [diffbot_system_hardware]: CAN TX: ID=0x101, DLC=8, Data=[DC 05 00 00 01 00 00 5D]
[DEBUG] [diffbot_system_hardware]: CAN TX: ID=0x102, DLC=8, Data=[F4 01 00 00 01 00 00 F5]
[DEBUG] [diffbot_system_hardware]: Heartbeat message sent on CAN ID 0x700
```

### 3. Monitor CAN Bus Directly

In a separate terminal, monitor the actual CAN messages:

```bash
# Setup virtual CAN if not already done
./setup_can.sh setup-vcan vcan0

# Monitor CAN messages
candump vcan0

# Output will show:
#  vcan0  101   [8]  DC 05 00 00 01 00 00 5D
#  vcan0  102   [8]  F4 01 00 00 01 00 00 F5
#  vcan0  700   [8]  00 00 00 00 00 AA 55 00
```

### 4. Understanding CAN Message Format

Each CAN message contains:
- **Bytes 0-3**: Velocity (int32, little-endian, scaled by 1000)
- **Byte 4**: Status (0x01 = normal operation)
- **Bytes 5-6**: Reserved (0x00)
- **Byte 7**: Checksum (XOR of bytes 0-6)

Example: `DC 05 00 00 01 00 00 5D`
- Velocity: 0x000005DC = 1500 (1.5 m/s * 1000 scale)
- Status: 0x01 (normal)
- Checksum: 0x5D

### 5. Log File Analysis

Check ROS2 log files for detailed CAN communication history:

```bash
# Use the test script to find and analyze logs
./test_can_messages.sh
# Choose option 6: "Show logs with CAN messages"

# Or manually check logs
find ~/.ros/log -name "*.log" -mmin -10 -exec grep -l "CAN\|diffbot" {} \;
```

### 6. Performance Monitoring

The logs also show timing information:

```
[WARN] [diffbot_system_hardware]: CAN broadcast loop running behind schedule: took 15 ms
[INFO] [diffbot_system_hardware]: CAN broadcast loop stopped after 25000 iterations
```

This helps monitor if the 10ms broadcast interval is being maintained.

## Usage

### 4. View Robot Description

To visualize the robot in RViz:
```bash
ros2 launch ros_diff_bot diffbot.launch.py
```

5. Control the Robot

Once the robot is running, you can control it using:
```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "
  header: auto
  twist:
    linear:
      x: 0.7
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 1.0"
```

## Package Structure

```
ros_diff_bot/
├── CMakeLists.txt
├── package.xml
├── ros_diff_bot.xml                    # Plugin description
├── README.md
├── bringup/
│   ├── config/
│   │   └── diffbot_controllers.yaml    # Controller configuration
│   └── launch/
│       └── diffbot.launch.py           # Main launch file
├── description/
│   ├── launch/
│   │   └── view_robot.launch.py        # Visualization launch file
│   ├── ros2_control/
│   │   └── diffbot.ros2_control.xacro  # ros2_control description
│   ├── rviz/
│   │   ├── diffbot.rviz                # RViz configuration
│   │   └── diffbot_view.rviz           # RViz view configuration
│   └── urdf/
│       ├── diffbot.urdf.xacro          # Main robot URDF
│       ├── diffbot_description.urdf.xacro  # Robot geometry
│       └── diffbot.materials.xacro     # Material definitions
├── hardware/
│   ├── diffbot_system.cpp              # Hardware interface implementation
│   └── include/
│       └── ros_diff_bot/
│           └── diffbot_system.hpp      # Hardware interface header
└── test/                               # Test files
    ├── test_diffbot_launch.py
    ├── test_urdf_xacro.py
    └── test_view_robot_launch.py
```

## Hardware Interface

The package includes a custom hardware interface (`DiffBotSystemHardware`) that:
- Implements the ros2_control SystemInterface
- Provides velocity command interfaces for left and right wheels
- Provides position and velocity state interfaces
- Simulates simple differential drive dynamics

## Controllers

- **joint_state_broadcaster**: Publishes joint states
- **diffbot_base_controller**: Differential drive controller that accepts Twist commands and controls wheel velocities

## Robot Description

The robot is described as:
- Base link with box geometry
- Two wheel joints (left_wheel_joint, right_wheel_joint) with continuous rotation
- Two caster wheels for stability
- Proper inertial properties and collision geometry

## Testing

Run tests using:
```bash
colcon test --packages-select ros_diff_bot
colcon test-result --verbose
```

## Configuration

### Controller Parameters

The controller parameters can be modified in `bringup/config/diffbot_controllers.yaml`:
- Update rate: 10 Hz
- Wheel separation: 0.10 m
- Wheel radius: 0.015 m
- Velocity and acceleration limits

### Hardware Parameters

Hardware parameters can be modified in the ros2_control URDF description:
- Start/stop duration parameters
- Interface configurations

## Troubleshooting

1. **Build errors**: Ensure all dependencies are installed with `rosdep install`
2. **Controller not starting**: Check controller configuration in YAML file
3. **Robot not moving**: Verify command topic is `/cmd_vel` and hardware is active
4. **RViz not showing robot**: Check if robot_state_publisher is running

## Contributing

Feel free to contribute improvements, bug fixes, or additional features through pull requests.

## License

This package is licensed under the Apache 2.0 License.

## Original Source

This package is derived from [ros2_control_demos example_2](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2) and has been modified to be a standalone package.

## References

- [ros2_control Documentation](https://control.ros.org/)
- [DiffBot Tutorial](https://control.ros.org/rolling/doc/ros2_control_demos/example_2/doc/userdoc.html)
- [Differential Drive Controller](https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)

# ROS DiffBot - Standalone Differential Drive Robot Package

This package is a standalone implementation of a differential drive robot (DiffBot) extracted from the ros2_control_demos example_2. It provides a complete ROS 2 package that can be built and run independently.

## Overview

The ROS DiffBot package demonstrates:
- Hardware interface implementation for a differential drive robot
- ros2_control integration
- Controller configuration (diff_drive_controller, joint_state_broadcaster)
- Simulation capabilities with mock hardware
- URDF robot description
- Launch files for robot visualization and control

## Features

- **Hardware Interface**: Custom DiffBotSystemHardware class implementing SystemInterface
- **Controllers**: Differential drive controller for mobile base control
- **Visualization**: RViz configuration for robot visualization
- **Mock Hardware**: Option to run with simulated hardware
- **Complete URDF**: Robot description with differential drive kinematics

## Prerequisites

- ROS 2 (Humble, Iron, Jazz, Rolling)
- ros2_control packages
- diff_drive_controller package
- joint_state_broadcaster package

## Installation

1. Clone this repository to your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url> ros_diff_bot
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select ros_diff_bot
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### 1. View Robot Description

To visualize the robot in RViz:
```bash
ros2 launch ros_diff_bot view_robot.launch.py
```

### 2. Run with Mock Hardware

To run the complete system with mock hardware:
```bash
ros2 launch ros_diff_bot diffbot.launch.py use_mock_hardware:=true
```

### 3. Run with Custom Hardware

To run with the custom hardware interface:
```bash
ros2 launch ros_diff_bot diffbot.launch.py use_mock_hardware:=false
```

### 4. Control the Robot

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

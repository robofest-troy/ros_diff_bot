#!/bin/bash

# ROS DiffBot Quick Start Script
# This script demonstrates different ways to run the ros_diff_bot package

set -e

show_help() {
    echo "🤖 ROS DiffBot Quick Start"
    echo ""
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  view          View robot in RViz (robot description only)"
    echo "  mock          Run with mock hardware"
    echo "  hardware      Run with custom hardware interface"
    echo "  control       Show example control commands"
    echo "  help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 view       # Just visualize the robot"
    echo "  $0 mock       # Run complete system with simulation"
    echo "  $0 hardware   # Run with real hardware interface"
}

check_sourced() {
    if [ -z "$ROS_DISTRO" ]; then
        echo "❌ Error: ROS not sourced. Please run:"
        echo "source /opt/ros/<distro>/setup.bash"
        echo "source install/setup.bash  # if in workspace"
        exit 1
    fi
}

if [ $# -eq 0 ]; then
    show_help
    exit 1
fi

case "$1" in
    "view")
        echo "🎨 Starting robot visualization..."
        check_sourced
        ros2 launch ros_diff_bot view_robot.launch.py
        ;;
    "mock")
        echo "🔧 Starting DiffBot with mock hardware..."
        check_sourced
        ros2 launch ros_diff_bot diffbot.launch.py use_mock_hardware:=true
        ;;
    "hardware")
        echo "🤖 Starting DiffBot with custom hardware interface..."
        check_sourced
        ros2 launch ros_diff_bot diffbot.launch.py use_mock_hardware:=false
        ;;
    "control")
        echo "🎮 Example control commands:"
        echo ""
        echo "# Move forward:"
        echo 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "header: auto; twist: {linear: {x: 0.5}}"'
        echo ""
        echo "# Turn in place:"
        echo 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "header: auto; twist: {angular: {z: 1.0}}"'
        echo ""
        echo "# Move in circle:"
        echo 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "header: auto; twist: {linear: {x: 0.5}, angular: {z: 1.0}}"'
        echo ""
        echo "# Stop:"
        echo 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "header: auto; twist: {}"'
        echo ""
        echo "# Check topics:"
        echo "ros2 topic list"
        echo "ros2 topic echo /joint_states"
        echo "ros2 topic echo /odom"
        ;;
    "help"|"-h"|"--help")
        show_help
        ;;
    *)
        echo "❌ Unknown option: $1"
        show_help
        exit 1
        ;;
esac

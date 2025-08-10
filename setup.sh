#!/bin/bash

# ROS DiffBot Setup Script
# This script helps set up the ros_diff_bot package in a new ROS workspace

set -e

echo "🤖 ROS DiffBot Setup Script"
echo "=========================="

# Function to detect ROS distribution
detect_ros() {
    if [ -f "/opt/ros/rolling/setup.bash" ]; then
        echo "rolling"
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        echo "jazzy"
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        echo "iron"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "humble"
    else
        echo "none"
    fi
}

# Check if ROS is installed
ROS_DISTRO_DETECTED=$(detect_ros)
if [ "$ROS_DISTRO_DETECTED" = "none" ]; then
    echo "❌ No ROS installation detected!"
    echo "Please install ROS 2 first: https://docs.ros.org/en/rolling/Installation.html"
    exit 1
fi

echo "📦 Detected ROS $ROS_DISTRO_DETECTED"

# Check if we need to create a workspace
if [ ! -f "package.xml" ]; then
    echo "❌ Not in package directory. This should be run from the ros_diff_bot package root."
    exit 1
fi

CURRENT_DIR=$(pwd)
PACKAGE_DIR=$(basename "$CURRENT_DIR")

if [ "$PACKAGE_DIR" != "ros_diff_bot" ]; then
    echo "⚠️  Warning: Current directory is not named 'ros_diff_bot'"
    echo "Current directory: $PACKAGE_DIR"
fi

# Check if we're in a proper workspace structure
if [ ! -d "../.." ] || [ ! -d "../../.." ]; then
    echo "🏗️  Setting up ROS workspace structure..."
    
    # Suggest workspace setup
    echo "It looks like you might need to set up a ROS workspace."
    echo "Here's how to do it:"
    echo ""
    echo "1. Create a workspace:"
    echo "   mkdir -p ~/ros2_ws/src"
    echo "   cd ~/ros2_ws/src"
    echo ""
    echo "2. Clone or copy this package:"
    echo "   cp -r $CURRENT_DIR ."
    echo "   # OR git clone <repository_url> ros_diff_bot"
    echo ""
    echo "3. Source ROS and build:"
    echo "   cd ~/ros2_ws"
    echo "   source /opt/ros/$ROS_DISTRO_DETECTED/setup.bash"
    echo "   rosdep install --from-paths src --ignore-src -r -y"
    echo "   colcon build --packages-select ros_diff_bot"
    echo "   source install/setup.bash"
    echo ""
    read -p "Do you want to check dependencies now? (y/n): " -n 1 -r
    echo
    if [[ $R =~ ^[Yy]$ ]]; then
        echo "📦 Checking dependencies..."
    else
        echo "⏭️  Skipping dependency check. Run manually with: rosdep install --from-paths src --ignore-src -r -y"
        exit 0
    fi
else
    echo "✅ Appears to be in a ROS workspace structure"
    
    # Go to workspace root
    cd ../..
    WS_ROOT=$(pwd)
    echo "📁 Workspace: $WS_ROOT"
    
    # Source ROS
    echo "🔧 Sourcing ROS $ROS_DISTRO_DETECTED..."
    source /opt/ros/$ROS_DISTRO_DETECTED/setup.bash
    export ROS_DISTRO=$ROS_DISTRO_DETECTED
    
    # Install dependencies
    echo "📦 Installing dependencies..."
    if command -v rosdep &> /dev/null; then
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y
    else
        echo "⚠️  rosdep not found. Install with: sudo apt install python3-rosdep"
        echo "Then run: rosdep init && rosdep update"
    fi
    
    echo ""
    echo "✅ Setup completed!"
    echo ""
    echo "🚀 To build and run:"
    echo "source /opt/ros/$ROS_DISTRO_DETECTED/setup.bash"
    echo "colcon build --packages-select ros_diff_bot"
    echo "source install/setup.bash"
    echo "ros2 launch ros_diff_bot diffbot.launch.py"
    echo ""
    echo "📖 See README.md for detailed usage instructions."
fi

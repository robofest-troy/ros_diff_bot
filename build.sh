#!/bin/bash

# ROS DiffBot Build Script
# This script builds the ros_diff_bot package

set -e

echo "🤖 Building ROS DiffBot Package..."

# Check if we're in a ROS workspace
if [ ! -f "package.xml" ]; then
    echo "❌ Error: package.xml not found. Please run this script from the package root directory."
    exit 1
fi

# Navigate to workspace root (assuming we're in src/ros_diff_bot)
if [ -d "../../install" ] || [ -d "../../build" ]; then
    WS_ROOT="../../"
    echo "📁 Found workspace at: $(realpath $WS_ROOT)"
else
    echo "❌ Error: Not in a ROS workspace. Please ensure this package is in src/ directory of a ROS workspace."
    exit 1
fi

cd $WS_ROOT

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ Error: ROS not sourced. Please source ROS setup.bash first:"
    echo "source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "🔧 ROS Distro: $ROS_DISTRO"

# Install dependencies
echo "📦 Installing dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build the package
echo "🔨 Building package..."
colcon build --packages-select ros_diff_bot --cmake-args -DCMAKE_BUILD_TYPE=Release

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo ""
    echo "🚀 To run the package:"
    echo "source install/setup.bash"
    echo "ros2 launch ros_diff_bot diffbot.launch.py"
    echo ""
    echo "📖 For more information, see the README.md file."
else
    echo "❌ Build failed!"
    exit 1
fi

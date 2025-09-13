#!/bin/bash

# ROS2 DiffBot CAN Setup Script
# This script helps set up CAN interfaces for testing and production use

set -e

echo "=== ROS2 DiffBot CAN Setup ==="

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTION] [INTERFACE_NAME]"
    echo ""
    echo "Options:"
    echo "  setup-vcan [name]    Set up virtual CAN interface (default: vcan0)"
    echo "  setup-can [name]     Set up physical CAN interface (default: can0)"
    echo "  status [name]        Show CAN interface status"
    echo "  monitor [name]       Monitor CAN messages"
    echo "  test [name]          Send test CAN messages"
    echo "  cleanup              Remove virtual CAN interfaces"
    echo "  help                 Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 setup-vcan vcan0              # Set up virtual CAN for testing"
    echo "  $0 setup-can can0                # Set up physical CAN interface"
    echo "  $0 monitor vcan0                 # Monitor CAN messages"
    echo "  $0 test vcan0                    # Send test messages"
}

# Function to set up virtual CAN interface
setup_vcan() {
    local interface=${1:-vcan0}
    
    echo "Setting up virtual CAN interface: $interface"
    
    # Load vcan module
    sudo modprobe vcan
    
    # Remove existing interface if it exists
    sudo ip link delete $interface 2>/dev/null || true
    
    # Create and bring up virtual CAN interface
    sudo ip link add dev $interface type vcan
    sudo ip link set up $interface
    
    echo "Virtual CAN interface $interface is ready!"
    echo "You can now use: can_device:=$interface in your launch files"
}

# Function to set up physical CAN interface
setup_can() {
    local interface=${1:-can0}
    local bitrate=${2:-500000}
    
    echo "Setting up physical CAN interface: $interface"
    echo "Bitrate: $bitrate bps"
    
    # Bring down interface first
    sudo ip link set down $interface 2>/dev/null || true
    
    # Configure CAN interface
    sudo ip link set $interface type can bitrate $bitrate
    
    # Optional: Set restart automatically on bus-off
    echo 100 | sudo tee /sys/class/net/$interface/can_restart_ms > /dev/null
    
    # Bring up interface
    sudo ip link set up $interface
    
    echo "Physical CAN interface $interface is ready!"
    echo "Bitrate: $(cat /sys/class/net/$interface/can_bittiming 2>/dev/null || echo 'Unknown')"
}

# Function to show CAN interface status
show_status() {
    local interface=${1:-can0}
    
    echo "=== CAN Interface Status: $interface ==="
    
    if ip link show $interface &>/dev/null; then
        echo "Interface exists: YES"
        echo "Status: $(ip link show $interface | grep -o 'state [A-Z]*' | cut -d' ' -f2)"
        
        if [ -f "/sys/class/net/$interface/can_bittiming" ]; then
            echo "Bitrate: $(cat /sys/class/net/$interface/can_bittiming)"
        fi
        
        if [ -f "/sys/class/net/$interface/statistics/rx_packets" ]; then
            echo "RX Packets: $(cat /sys/class/net/$interface/statistics/rx_packets)"
            echo "TX Packets: $(cat /sys/class/net/$interface/statistics/tx_packets)"
        fi
    else
        echo "Interface exists: NO"
    fi
    
    echo ""
    echo "All CAN interfaces:"
    ip link show type can 2>/dev/null || echo "No CAN interfaces found"
}

# Function to monitor CAN messages
monitor_can() {
    local interface=${1:-vcan0}
    
    if ! command -v candump &> /dev/null; then
        echo "Error: candump not found. Install with: sudo apt install can-utils"
        exit 1
    fi
    
    if ! ip link show $interface &>/dev/null; then
        echo "Error: CAN interface $interface not found"
        echo "Available interfaces:"
        ip link show type can 2>/dev/null || echo "No CAN interfaces found"
        exit 1
    fi
    
    echo "Monitoring CAN messages on $interface (Press Ctrl+C to stop)..."
    echo "Expected message IDs for DiffBot:"
    echo "  0x101 - Left wheel velocity"
    echo "  0x102 - Right wheel velocity"
    echo "  0x700 - Heartbeat (if enabled)"
    echo ""
    
    candump $interface
}

# Function to send test CAN messages
test_can() {
    local interface=${1:-vcan0}
    
    if ! command -v cansend &> /dev/null; then
        echo "Error: cansend not found. Install with: sudo apt install can-utils"
        exit 1
    fi
    
    if ! ip link show $interface &>/dev/null; then
        echo "Error: CAN interface $interface not found"
        exit 1
    fi
    
    echo "Sending test CAN messages to $interface..."
    
    # Send test messages for both wheels
    echo "Sending left wheel velocity command (ID: 0x101)..."
    cansend $interface 101#E803000001000000  # Velocity: 1000 (0x03E8), Status: 0x01
    
    echo "Sending right wheel velocity command (ID: 0x102)..."
    cansend $interface 102#E803000001000000  # Velocity: 1000 (0x03E8), Status: 0x01
    
    echo "Sending heartbeat message (ID: 0x700)..."
    cansend $interface 700#0000000000AA5500  # Heartbeat signature
    
    echo "Test messages sent! Monitor with: $0 monitor $interface"
}

# Function to cleanup virtual CAN interfaces
cleanup() {
    echo "Cleaning up virtual CAN interfaces..."
    
    for vcan in vcan0 vcan1 vcan2; do
        if ip link show $vcan &>/dev/null; then
            echo "Removing $vcan..."
            sudo ip link delete $vcan
        fi
    done
    
    echo "Cleanup complete!"
}

# Function to check dependencies
check_dependencies() {
    local missing=0
    
    if ! command -v ip &> /dev/null; then
        echo "Error: 'ip' command not found"
        missing=1
    fi
    
    if ! command -v candump &> /dev/null; then
        echo "Warning: 'candump' not found. Install with: sudo apt install can-utils"
    fi
    
    if ! command -v cansend &> /dev/null; then
        echo "Warning: 'cansend' not found. Install with: sudo apt install can-utils"
    fi
    
    if [ $missing -eq 1 ]; then
        exit 1
    fi
}

# Main script logic
main() {
    check_dependencies
    
    case "${1:-help}" in
        "setup-vcan")
            setup_vcan "$2"
            ;;
        "setup-can")
            setup_can "$2" "$3"
            ;;
        "status")
            show_status "$2"
            ;;
        "monitor")
            monitor_can "$2"
            ;;
        "test")
            test_can "$2"
            ;;
        "cleanup")
            cleanup
            ;;
        "help"|"-h"|"--help")
            show_usage
            ;;
        *)
            echo "Error: Unknown option '$1'"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

main "$@"

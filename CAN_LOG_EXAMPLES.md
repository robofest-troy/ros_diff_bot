# CAN Message Logging Examples

This file shows examples of what you should see in the logs when running the DiffBot with CAN communication.

## Startup Sequence

When launching the system, you should see these log messages:

```
[INFO] [1694123456.789] [controller_manager]: Loading controller_manager
[INFO] [1694123456.812] [diffbot_system_hardware]: CAN Configuration loaded successfully:
[INFO] [1694123456.812] [diffbot_system_hardware]:   Device: vcan0
[INFO] [1694123456.812] [diffbot_system_hardware]:   Bitrate: 500000 bps
[INFO] [1694123456.812] [diffbot_system_hardware]:   Broadcast Interval: 10 ms
[INFO] [1694123456.812] [diffbot_system_hardware]:   Left Wheel CAN ID: 0x101
[INFO] [1694123456.812] [diffbot_system_hardware]:   Right Wheel CAN ID: 0x102
[INFO] [1694123456.813] [diffbot_system_hardware]: Configuring ...please wait...
[INFO] [1694123456.813] [diffbot_system_hardware]: CAN interface vcan0 initialized successfully (socket fd: 7)
[INFO] [1694123456.813] [diffbot_system_hardware]: Successfully configured!
[INFO] [1694123456.814] [diffbot_system_hardware]: Activating ...please wait...
[INFO] [1694123456.814] [diffbot_system_hardware]: CAN broadcast loop started - sending messages every 10 ms
[INFO] [1694123456.814] [diffbot_system_hardware]: Successfully activated! CAN broadcast started.
```

## Normal Operation (with velocity commands)

When receiving velocity commands, you'll see:

```
[INFO] [1694123465.123] [diffbot_system_hardware]: Writing commands:
[INFO] [1694123465.123] [diffbot_system_hardware]:     command 1.50 for 'left_wheel_joint/velocity'!
[INFO] [1694123465.123] [diffbot_system_hardware]:     command 0.50 for 'right_wheel_joint/velocity'!

# Periodic status updates (every 10 seconds)
[INFO] [1694123475.234] [diffbot_system_hardware]: CAN Status: left_wheel velocity=1.500 m/s, CAN_ID=0x101, scaled_vel=1500
[INFO] [1694123475.234] [diffbot_system_hardware]: CAN Status: right_wheel velocity=0.500 m/s, CAN_ID=0x102, scaled_vel=500
```

## Debug-Level CAN Message Details

With debug logging enabled (`--ros-args --log-level debug`):

```
[DEBUG] [1694123465.124] [diffbot_system_hardware]: CAN TX: ID=0x101, DLC=8, Data=[DC 05 00 00 01 00 00 5D]
[DEBUG] [1694123465.124] [diffbot_system_hardware]: CAN TX: ID=0x102, DLC=8, Data=[F4 01 00 00 01 00 00 F5]
[DEBUG] [1694123466.124] [diffbot_system_hardware]: CAN TX: ID=0x101, DLC=8, Data=[DC 05 00 00 01 00 00 5D]
[DEBUG] [1694123466.124] [diffbot_system_hardware]: CAN TX: ID=0x102, DLC=8, Data=[F4 01 00 00 01 00 00 F5]
[DEBUG] [1694123475.234] [diffbot_system_hardware]: Heartbeat message sent on CAN ID 0x700
```

## CAN Monitor Output (candump)

Running `candump vcan0` in parallel will show:

```
  vcan0  101   [8]  DC 05 00 00 01 00 00 5D    # Left wheel: 1.5 m/s
  vcan0  102   [8]  F4 01 00 00 01 00 00 F5    # Right wheel: 0.5 m/s
  vcan0  101   [8]  DC 05 00 00 01 00 00 5D
  vcan0  102   [8]  F4 01 00 00 01 00 00 F5
  vcan0  700   [8]  00 00 00 00 00 AA 55 00    # Heartbeat
  vcan0  101   [8]  00 00 00 00 01 00 00 01    # Stopped: 0.0 m/s
  vcan0  102   [8]  00 00 00 00 01 00 00 01    # Stopped: 0.0 m/s
```

## Message Decoding Examples

### Velocity Command: `DC 05 00 00 01 00 00 5D`
- **Bytes 0-3**: `DC 05 00 00` = 0x000005DC = 1500 decimal
- **Velocity**: 1500 / 1000 scale = 1.5 m/s
- **Byte 4**: `01` = Normal operation status
- **Bytes 5-6**: `00 00` = Reserved
- **Byte 7**: `5D` = Checksum (0xDC ⊕ 0x05 ⊕ 0x00 ⊕ 0x00 ⊕ 0x01 ⊕ 0x00 ⊕ 0x00 = 0x5D)

### Zero Velocity: `00 00 00 00 01 00 00 01`
- **Bytes 0-3**: `00 00 00 00` = 0 decimal
- **Velocity**: 0 / 1000 scale = 0.0 m/s
- **Byte 4**: `01` = Normal operation status
- **Byte 7**: `01` = Checksum

### Heartbeat: `00 00 00 00 00 AA 55 00`
- **Bytes 0-3**: Timestamp (varies)
- **Bytes 4-5**: `AA 55` = Heartbeat signature
- **Bytes 6-7**: Additional heartbeat data

## Warning Messages

### Command Timeout
When no velocity commands are received:
```
[WARN] [1694123480.567] [diffbot_system_hardware]: Command timeout - stopping all wheels
```

### Timing Issues
If the system can't maintain 10ms intervals:
```
[WARN] [1694123465.140] [diffbot_system_hardware]: CAN broadcast loop running behind schedule: took 15 ms
```

### CAN Interface Problems
If CAN interface is not available:
```
[ERROR] [1694123456.800] [diffbot_system_hardware]: Failed to create CAN socket: Network is down
[ERROR] [1694123456.800] [diffbot_system_hardware]: Failed to initialize CAN interface
```

## Shutdown Sequence

When stopping the system:
```
[INFO] [1694123500.123] [diffbot_system_hardware]: Deactivating ...please wait...
[INFO] [1694123503.123] [diffbot_system_hardware]: CAN broadcast loop stopped after 37234 iterations
[INFO] [1694123503.123] [diffbot_system_hardware]: CAN interface closed
[INFO] [1694123503.123] [diffbot_system_hardware]: Successfully deactivated!
```

## Testing Commands

To generate the above logs, use these commands:

```bash
# 1. Setup and launch
./setup_can.sh setup-vcan vcan0
ros2 launch ros_diff_bot diffbot_can.launch.py can_device:=vcan0 --ros-args --log-level debug

# 2. Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.5}, angular: {z: 0.0}}'

# 3. Monitor CAN messages
candump vcan0

# 4. Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

The logs will show the exact CAN message transmission, timing, and any issues that occur during operation.

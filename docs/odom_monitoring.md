# Odometry Monitoring and Diagnostic System

This system provides comprehensive monitoring for odometry jumps in your antrobot, specifically designed to monitor both wheel odometry (`odom_wheel`) and kinematic ICP odometry (`odom`) along with lidar health.

## Features

- **Real-time jump detection** for position, velocity, and angular changes
- **Lidar health monitoring** to ensure ICP has good input data
- **Cross-odometry consistency checking** between wheel and ICP sources
- **TF tree state capture** when jumps occur
- **Comprehensive diagnostics** via ROS 2 diagnostic system
- **Detailed logging** with pose differences, timing analysis, and sensor states

## Components

### 1. `odom_jump_monitor.py` - Main Monitoring Node

Continuously monitors odometry topics and detects jumps based on configurable thresholds.

**Monitored metrics:**
- Position jumps (sudden large changes in x,y position)
- Angular jumps (sudden orientation changes)
- Velocity jumps (impossible acceleration)
- Lidar point count and scan rate
- Cross-odometry divergence

**When a jump is detected, it captures:**
- Exact pose before and after the jump
- Time differences (ROS time vs wall time)
- TF tree state at the moment of jump
- Lidar scan statistics
- Velocity and acceleration analysis

### 2. `quick_odom_monitor.py` - Command Line Tool

Simple real-time monitoring tool for immediate feedback during testing.

### 3. `tf_timing_capture.py` - TF Timing Issue Capture

Specialized tool that monitors for TF timing errors (like extrapolation errors and message filter drops) and captures the exact state of both odometry sources when these issues occur.

### 4. `tf_timing_diagnostic.py` - Advanced TF Diagnostics

More comprehensive TF timing diagnostic tool with detailed analysis and logging capabilities.

### 5. Diagnostic Integration

Publishes to `/diagnostics` topic for integration with ROS 2 diagnostic tools like `rqt_robot_monitor`.

## Usage

### Quick Start - Command Line Monitor

For immediate monitoring during testing:

```bash
# Monitor with default topics
ros2 run antrobot_ros quick_odom_monitor

# Monitor with custom topics
ros2 run antrobot_ros quick_odom_monitor --wheel-topic odom_wheel --icp-topic odom --lidar-topic scan
```

### Capture TF Timing Issues

For debugging the specific timing issues you're experiencing:

```bash
# Monitor and capture TF timing errors
ros2 run antrobot_ros tf_timing_capture

# Advanced TF diagnostics with detailed analysis
ros2 run antrobot_ros tf_timing_diagnostic
```

### Full Monitoring System

Launch the comprehensive monitoring system:

```bash
# Start with default parameters
ros2 launch antrobot_ros odom_monitor.launch.py

# Start with custom thresholds
ros2 launch antrobot_ros odom_monitor.launch.py max_linear_jump:=0.3 max_angular_jump:=0.3

# Start with parameter file
ros2 run antrobot_ros odom_monitor --ros-args --params-file config/odom_monitor_params.yaml
```

### Integration with Your Robot Launch

Add to your main robot launch file:

```python
# In your launch file
odom_monitor_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(FindPackageShare('antrobot_ros').find('antrobot_ros'), 
                    'launch', 'odom_monitor.launch.py')
    ),
    launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
)
```

## Configuration

Edit `config/odom_monitor_params.yaml` to adjust thresholds:

```yaml
odom_monitor:
  ros__parameters:
    # Adjust these based on your robot's characteristics
    max_linear_jump: 0.5        # 50cm position jump threshold
    max_angular_jump: 0.785     # 45 degree angular jump threshold  
    max_velocity_jump: 2.0      # 2 m/s velocity change threshold
    max_cross_odom_diff: 1.0    # 1m difference between odometry sources
    min_lidar_points: 50        # Minimum points for healthy lidar scan
    timestamp_sync_threshold: 0.5  # Only warn for timestamp differences > 500ms
```

## Understanding the Output

### Jump Detection Messages

When a jump is detected, you'll see detailed logs like:

```
[ERROR] ================================================================================
[ERROR] ODOMETRY JUMP DETECTED - ICP
[ERROR] ================================================================================
[ERROR] Jump in ICP odometry:
[ERROR]   Position jump: 0.847000m
[ERROR]   Angular jump: 0.123000rad (7.05°)
[ERROR]   Velocity change: 12.450000m/s
[ERROR]   Time difference (ROS): 0.050000s
[ERROR]   Time difference (wall): 0.051234s
[ERROR] 
[ERROR] ICP ODOMETRY POSES:
[ERROR]   Previous: x=1.234567, y=2.567890, z=0.000000
[ERROR]             yaw=45.200°
[ERROR]             quat=[0.000000, 0.000000, 0.382683, 0.923880]
[ERROR]             vel=[0.500000, 0.000000, 0.000000]
[ERROR]             ang_vel=[0.000000, 0.000000, 0.200000]
[ERROR]             timestamp: 1752064559.103000000
[ERROR]   Current:  x=2.081567, y=2.890123, z=0.000000
[ERROR]             yaw=52.300°
[ERROR]             quat=[0.000000, 0.000000, 0.423500, 0.905800]
[ERROR]             vel=[0.520000, 0.000000, 0.000000]
[ERROR]             ang_vel=[0.000000, 0.000000, 0.180000]
[ERROR]             timestamp: 1752064559.153000000
[ERROR] 
[ERROR] WHEEL ODOMETRY AT SAME TIME:
[ERROR]   Topic: odom_wheel
[ERROR]   Pose:     x=1.245000, y=2.570000, z=0.000000
[ERROR]             yaw=45.500°
[ERROR]             quat=[0.000000, 0.000000, 0.385000, 0.922900]
[ERROR]             vel=[0.495000, 0.000000, 0.000000]
[ERROR]             ang_vel=[0.000000, 0.000000, 0.195000]
[ERROR]             timestamp: 1752064559.148000000
[ERROR] 
[ERROR] CROSS-ODOMETRY COMPARISON:
[ERROR]   Position difference: 0.836000m
[ERROR]   Yaw difference: 6.800°
[ERROR]   Timestamp difference: 0.005000s
[ERROR]   ⚠️  LARGE CROSS-ODOM DIVERGENCE: 0.836m!
[ERROR] 
[ERROR] ⚠️  TF timing error occurred 1.234s before this jump!
[ERROR] 
[ERROR] Total TF extrapolation errors: 15
[ERROR] 
[ERROR] MESSAGE TIMING ANALYSIS:
[ERROR]   WHEEL_ODOM: avg=0.050s, min=0.045s, max=0.065s
[ERROR]   ICP_ODOM: avg=0.048s, min=0.040s, max=0.080s
[ERROR]   LIDAR: avg=0.100s, min=0.095s, max=0.120s
[ERROR] 
[ERROR] LIDAR STATUS AT JUMP:
[ERROR]   Point count: 89
[ERROR]   Scan rate: 10.1Hz
[ERROR]   Range stats: min=0.21m, max=3.45m, mean=1.23m, std=0.45m
[ERROR] ================================================================================
```

### TF Timing Capture Output

When TF timing issues are detected, you'll see:

```
================================================================================
TF TIMING ISSUE DETECTED - 2025-07-09 15:28:21.403664
================================================================================
Error: Message Filter dropping message: frame 'rplidar_link' at time 1752064558.957 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
Node: global_costmap.global_costmap
Level: INFO
Error timestamp: 1752064558.957000
Current time: 1752064559.403664
Time difference: 0.446664s

WHEEL ODOMETRY:
  Topic timestamp: 1752064559.148000000
  Frame: odom_wheel -> base_link
  Position: x=1.245000, y=2.570000, z=0.000000
  Orientation: yaw=45.500°
  Quaternion: [0.000000, 0.000000, 0.385000, 0.922900]
  Linear vel: [0.495000, 0.000000, 0.000000]
  Angular vel: [0.000000, 0.000000, 0.195000]

ICP ODOMETRY:
  Topic timestamp: 1752064559.103000000
  Frame: odom -> base_link
  Position: x=1.234567, y=2.567890, z=0.000000
  Orientation: yaw=45.200°
  Quaternion: [0.000000, 0.000000, 0.382683, 0.923880]
  Linear vel: [0.500000, 0.000000, 0.000000]
  Angular vel: [0.000000, 0.000000, 0.200000]

LIDAR:
  Timestamp: 1752064558.957000000
  Frame: rplidar_link
  Valid points: 89/360
  Range stats: min=0.21m, max=3.45m, avg=1.23m

TF TREE STATE:
  map -> base_link:
    Translation: [1.234567, 2.567890, 0.000000]
    Rotation: [0.000000, 0.000000, 0.382683, 0.923880]
    Age: 0.446664s
    ⚠️  STALE TRANSFORM (>0.447s old)
  odom -> base_link: EXTRAPOLATION ERROR - Lookup would require extrapolation into the past
  odom_wheel -> base_link:
    Translation: [1.245000, 2.570000, 0.000000]
    Rotation: [0.000000, 0.000000, 0.385000, 0.922900]
    Age: 0.255664s
================================================================================
```

### Quick Monitor Output

The command-line tool shows immediate alerts:

```
🚨 ICP POSITION JUMP: 0.847m
   Previous: (1.234, 2.567)
   Current:  (2.081, 2.890)
   Time diff: 0.050s

⚠️  LIDAR: Low point count 45 (rate: 9.8Hz)
```

### Diagnostics

View in `rqt_robot_monitor` or check programmatically:

```bash
ros2 topic echo /diagnostics
```

## Troubleshooting Common Issues

### High False Positive Rate

If you're getting too many jump alerts:

1. **Increase thresholds** - Your robot might naturally have larger motions
2. **Check timing** - Irregular message timing can cause false positives
3. **Verify topics** - Ensure you're monitoring the correct odometry topics

### Missing Jump Detection

If jumps are not being detected:

1. **Decrease thresholds** - Your jumps might be smaller than expected
2. **Check topic names** - Verify the monitor is subscribed to correct topics
3. **Check message rates** - Very slow odometry updates can mask jumps

### Timestamp Synchronization Warnings

The monitor checks for large differences between wheel and ICP odometry timestamps. Normal behavior:

- **Expected delay**: ICP processing typically has 50-200ms delay after wheel odometry
- **Warning threshold**: Only warns if difference exceeds `timestamp_sync_threshold` (default 500ms)
- **Normal operation**: Small delays are expected and logged at DEBUG level only

If you see frequent timestamp sync warnings:
1. **Increase threshold** in config file if delays are normal for your setup
2. **Check processing load** - high CPU usage can cause larger delays
3. **Check message rates** - verify both odometry sources are publishing regularly

### Lidar Health Issues

Common lidar-related problems:

- **Low point count**: Check lidar range settings, obstacles, or hardware issues
- **Low scan rate**: Check lidar driver configuration or USB/serial connection
- **Data timeout**: Verify lidar driver is running and publishing

## Advanced Features

### Custom Jump Detection Logic

You can extend the monitoring by modifying the `detect_jump()` method to include:

- Acceleration-based detection
- Covariance analysis
- Multi-frame temporal filtering
- Machine learning-based anomaly detection

### Integration with Robot State

The monitor can be extended to integrate with:
- Robot state machine (pause motion on jumps)
- Recovery behaviors (reset odometry, re-initialize localization)
- Data logging for offline analysis

## Performance Notes

- **CPU usage**: Minimal - only processes odometry and lidar messages
- **Memory usage**: Small - keeps limited history (last 100 messages)
- **Network impact**: Low - only publishes diagnostics at 1Hz

## Related Tools

This system works well with:
- `odom_eval_node` - For trajectory recording and analysis
- `evo` toolkit - For detailed odometry evaluation
- `rqt_robot_monitor` - For viewing diagnostic status
- `rqt_plot` - For plotting odometry data in real-time

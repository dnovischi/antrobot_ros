# Odometry Evaluation System

This system provides tools for recording and evaluating multiple odometry sources using the evo toolkit.

## Components

### 1. `odom_eval_node`
Records multiple odometry topics to TUM format files for analysis with evo.

### 2. `odom_eval_params.yaml`
Configuration file specifying:
- Topics to record
- Output directory
- Reference frame

### 3. `odom_eval_analysis.sh`
Script for comparing two trajectory files using evo tools.

## Usage

### 1. Configure topics
Edit `config/odom_eval_params.yaml` to specify which topics to record:

```yaml
odom_eval_node:
  ros__parameters:
    output_dir: "/tmp/odom_eval"
    reference_frame: "odom"
    topics:
      - "/wheel_odom"    # Your wheel odometry topic
      - "/odom"          # Your KISS-ICP odometry topic
```

### 2. Start recording
```bash
# Using launch file (recommended)
ros2 launch antrobot_ros odom_eval.launch.py

# Or run node directly
ros2 run antrobot_ros odom_eval_node --ros-args --params-file config/odom_eval_params.yaml
```

### 3. Record trajectories
Move your robot around while the node is running. Press **Ctrl+C** to stop recording and save files.

### 4. Analyze trajectories
```bash
# Navigate to the analysis script
cd scripts/evaluation/odom/

# Compare two trajectory files
./odom_eval_analysis.sh /tmp/odom_eval/wheel_odom_20250628_143000.tum /tmp/odom_eval/odom_20250628_143000.tum

# With custom options
./odom_eval_analysis.sh traj1.tum traj2.tum --output-dir ~/results --plot-mode xyz --no-align
```

## Output Files

### Recording
- `{topic_name}_{timestamp}.tum` - Trajectory files in TUM format

### Analysis
- `trajectory_comparison.png` - Trajectory overlay plot
- `ape_comparison.png` - Absolute Pose Error plots
- `rpe_comparison.png` - Relative Pose Error plots
- `*.zip` - Detailed results that can be viewed with `evo_res`

## Dependencies

The system requires:
- `evo` toolkit (install with `pip install evo`)
- Standard ROS2 packages (tf2, geometry_msgs, nav_msgs)

## Example Workflow

1. Configure your topics in `odom_eval_params.yaml`
2. Start your robot and odometry sources
3. Launch recording: `ros2 launch antrobot_ros odom_eval.launch.py`
4. Drive your robot around for data collection
5. Stop with Ctrl+C when done
6. Analyze: `./odom_eval_analysis.sh wheel_odom_*.tum odom_*.tum`
7. View results in the generated analysis directory

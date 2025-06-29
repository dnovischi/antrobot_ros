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

# Compare two trajectory files (standard alignment)
./odom_eval_analysis.sh /tmp/odom_eval/wheel_odom_20250628_143000.tum /tmp/odom_eval/odom_20250628_143000.tum

# With custom options and specific alignment
./odom_eval_analysis.sh traj1.tum traj2.tum --output-dir ~/results --plot-mode xy --align-type se3

# For 2D robot navigation (recommended for differential drive)
./odom_eval_analysis.sh wheel_odom.tum kiss_icp.tum --align-type pos_yaw

# No alignment (absolute positioning accuracy)
./odom_eval_analysis.sh wheel_odom.tum kiss_icp.tum --align-type none
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

## Alignment Types

When comparing trajectories, evo can align them in different ways to account for coordinate frame differences and initial pose uncertainties. The alignment type affects how the reference and estimated trajectories are matched before computing errors.

### Available Alignment Types

#### 1. `none` - No Alignment
- **What it does**: Compares trajectories as-is, without any transformation
- **Use when**: 
  - Both trajectories are already in the same coordinate frame
  - You want to measure absolute positioning accuracy
  - Initial poses are exactly aligned
- **Pros**: Shows true absolute positioning performance
- **Cons**: Sensitive to coordinate frame differences and initialization errors
- **Example**: Comparing two odometry sources that start from the exact same pose

#### 2. `se3` - Full SE(3) Alignment (Default)
- **What it does**: Aligns translation (x,y,z) and rotation (roll,pitch,yaw) using Horn's method
- **Use when**: 
  - Standard trajectory comparison (most common)
  - Initial poses may differ slightly
  - You want to focus on trajectory shape rather than absolute positioning
- **Pros**: Robust to initialization differences, shows relative accuracy
- **Cons**: May hide systematic translation/rotation errors
- **Example**: Comparing wheel odometry vs LIDAR odometry with different starting orientations

#### 3. `sim3` - Similarity Transform (SE(3) + Scale)
- **What it does**: Aligns translation, rotation, AND scale
- **Use when**: 
  - Comparing trajectories that may have different scales
  - One system has systematic scaling errors
  - Visual odometry vs metric odometry
- **Pros**: Accounts for scale differences
- **Cons**: May hide important scale-related errors
- **Example**: Comparing visual odometry (unknown scale) with wheel odometry (metric scale)

#### 4. `pos` - Position-Only Alignment
- **What it does**: Aligns only translation (x,y,z), ignores rotation
- **Use when**: 
  - Initial positions differ but orientations are correct
  - You want to preserve orientation errors in the analysis
  - Focusing on translational accuracy
- **Pros**: Preserves rotation errors for analysis
- **Cons**: May not work well if trajectories have different orientations
- **Example**: Comparing odometry sources with same orientation but different starting positions

#### 5. `pos_yaw` - Position + Yaw Alignment
- **What it does**: Aligns translation and yaw (z-axis rotation), preserves roll/pitch
- **Use when**: 
  - 2D robot navigation (most differential drive robots)
  - Initial heading may differ but robot moves in a plane
  - You want to focus on 2D path accuracy
- **Pros**: Ideal for ground robots, preserves 2D shape analysis
- **Cons**: Limited to planar motion analysis
- **Example**: Comparing wheel odometry vs GPS for a differential drive robot

### Choosing the Right Alignment

#### For Your Differential Drive Robot:

**Recommended: `se3` or `pos_yaw`**

- **Use `se3`** when:
  - Comparing different odometry algorithms
  - Initial poses are uncertain
  - You want standard trajectory comparison

- **Use `pos_yaw`** when:
  - Robot operates in 2D (ground robots)
  - You specifically want to analyze 2D navigation performance
  - Comparing wheel odometry vs LIDAR/GPS

- **Use `none`** when:
  - Both odometry sources start from exactly the same pose
  - You want to measure absolute positioning accuracy
  - Testing repeatability

- **Use `sim3`** when:
  - One odometry source might have scaling issues
  - Wheel calibration problems suspected

### Visual Impact

Different alignments will produce different error metrics:

```bash
# No alignment - shows absolute errors
./odom_eval_analysis.sh wheel_odom.tum kiss_icp.tum --align-type none

# Standard alignment - focuses on trajectory shape
./odom_eval_analysis.sh wheel_odom.tum kiss_icp.tum --align-type se3

# 2D robot alignment - ideal for ground robots
./odom_eval_analysis.sh wheel_odom.tum kiss_icp.tum --align-type pos_yaw
```

**Key Point**: Choose alignment based on what you want to evaluate. For most robotics applications with differential drive robots, `se3` or `pos_yaw` provides the most meaningful comparison.

## Evaluation Metrics Explained

The evo toolkit provides several types of trajectory evaluation metrics. Understanding what each measures helps interpret your odometry performance correctly.

### APE (Absolute Pose Error)

APE measures the **global consistency** of your trajectory by comparing each pose to the ground truth after optimal alignment.

**Formula**: `APE = ||T_ref^{-1} * T_est||` at each timestamp

#### APE Results Breakdown:

**Statistical Measures:**
- **mean**: Average error across all poses - overall drift magnitude
- **median**: Middle value when sorted - less sensitive to outliers than mean
- **std**: Standard deviation - consistency of errors (low = consistent, high = erratic)
- **min**: Best performance achieved during trajectory
- **max**: Worst error encountered - indicates maximum drift
- **rmse**: Root Mean Square Error - heavily penalizes large errors

**Error Components:**
- **Translation APE (meters)**: How far your estimated position drifts from true position
- **Rotation APE (degrees)**: How much your estimated orientation differs from true heading

#### APE Interpretation:
```
Translation APE Guidelines:
< 0.1m    : Excellent (precision robotics)
0.1-0.5m  : Good (indoor navigation)
0.5-2.0m  : Acceptable (outdoor, large spaces)
> 2.0m    : Poor (needs improvement)

Rotation APE Guidelines:
< 1°      : Excellent orientation tracking
1-5°      : Good for navigation
5-15°     : Acceptable but may affect planning
> 15°     : Poor orientation accuracy
```

**What APE tells you:**
- **Low APE**: Your odometry stays globally consistent
- **High APE**: Significant drift accumulation over time
- **Increasing APE over time**: Systematic drift (wheel slip, calibration errors)

### RPE (Relative Pose Error)

RPE measures **local accuracy** by comparing the relative motion between consecutive poses over different time intervals.

**Formula**: `RPE = ||(T_ref,i^{-1} * T_ref,j)^{-1} * (T_est,i^{-1} * T_est,j)||`

#### RPE Results Breakdown:

**Statistical Measures** (same as APE):
- **mean/median/std/min/max/rmse**: Same interpretation as APE but for relative motions

**Error Components:**
- **Translation RPE (m/m or m/s)**: Error per unit distance or time traveled
- **Rotation RPE (deg/m or deg/s)**: Angular error per unit distance or time traveled

#### RPE Interpretation:
```
Translation RPE Guidelines:
< 1% of distance    : Excellent (< 0.01 m/m)
1-5% of distance    : Good (0.01-0.05 m/m)
5-10% of distance   : Acceptable (0.05-0.10 m/m)
> 10% of distance   : Poor (> 0.10 m/m)

Rotation RPE Guidelines:
< 1°/m     : Excellent angular accuracy
1-3°/m     : Good for differential drive
3-10°/m    : Acceptable
> 10°/m    : Poor (check wheel calibration)
```

**What RPE tells you:**
- **Low RPE**: Good short-term accuracy, consistent motion estimation
- **High RPE**: Poor local tracking, noisy measurements
- **Constant RPE**: Systematic error (miscalibrated wheels, encoder issues)

### Trajectory Statistics

**Additional metrics provided:**

- **Path Length**: Total distance traveled by each trajectory
- **Duration**: Total time of trajectory recording
- **Average Speed**: Mean velocity during trajectory
- **Bounding Box**: Spatial extent of trajectory (min/max x,y,z)

### Practical Example Interpretation

```bash
# Example results for a differential drive robot:

APE Translation (m):
mean: 0.15, median: 0.12, std: 0.08, rmse: 0.17, max: 0.45

APE Rotation (deg):
mean: 2.3, median: 1.8, std: 1.5, rmse: 2.7, max: 8.1

RPE Translation (m/m):
mean: 0.03, median: 0.025, std: 0.02, rmse: 0.036, max: 0.12

RPE Rotation (deg/m):
mean: 1.2, median: 0.9, std: 0.8, rmse: 1.4, max: 4.5
```

**Interpretation:**
- **Good overall performance**: APE < 0.5m, RPE < 5%
- **Slight systematic drift**: Mean APE 15cm suggests gradual accumulation
- **Consistent local tracking**: Low RPE std indicates reliable short-term motion
- **Acceptable orientation**: 2.3° average heading error is reasonable for navigation

### Choosing Analysis Focus

**Use APE when:**
- Evaluating long-term drift and global accuracy
- Comparing different odometry algorithms
- Assessing suitability for long-duration missions

**Use RPE when:**
- Evaluating short-term accuracy and sensor noise
- Diagnosing systematic calibration errors
- Comparing motion estimation consistency

**Use both when:**
- Complete odometry system evaluation
- Understanding both local and global performance characteristics

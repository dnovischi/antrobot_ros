#!/bin/bash

# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

# Odometry Analysis Script using evo
# This script compares two trajectory files using the evo toolkit

set -e

# Default parameters
TRAJECTORY1=""
TRAJECTORY2=""
OUTPUT_DIR=""
PLOT_MODE="xy"
ALIGN_TYPE="se3"
TIME_TOLERANCE="0.01"

# Set environment to avoid matplotlib 3D issues
export MPLBACKEND="Agg"
export PYTHONPATH="/home/dan/.local/lib/python3.10/site-packages:$PYTHONPATH"

# Function to display help
show_help() {
    echo "Usage: $0 TRAJECTORY1 TRAJECTORY2 [OPTIONS]"
    echo ""
    echo "Arguments:"
    echo "  TRAJECTORY1       First trajectory file (TUM format)"
    echo "  TRAJECTORY2       Second trajectory file (TUM format)"
    echo ""
    echo "Options:"
    echo "  --output-dir DIR  Output directory for results (default: same as trajectory files)"
    echo "  --plot-mode MODE  Plot mode: xy, xz, yz (default: xy, 3D modes disabled)"
    echo "  --time-tol SEC    Time tolerance for timestamp matching (default: 0.01)"
    echo "  --align-type TYPE Alignment type: none, se3, sim3, pos, pos_yaw (default: se3)"
    echo "                    - none: no alignment"
    echo "                    - se3: full SE(3) alignment (translation + rotation)"
    echo "                    - sim3: Sim(3) alignment (translation + rotation + scale)"
    echo "                    - pos: position-only alignment"
    echo "                    - pos_yaw: position + yaw alignment"
    echo "  --help, -h        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 wheel_odom_20250628_143000.tum odom_20250628_143000.tum"
    echo "  $0 traj1.tum traj2.tum --output-dir ~/results --plot-mode xz"
    echo "  $0 traj1.tum traj2.tum --align-type pos_yaw --time-tol 0.5"
    echo "  $0 traj1.tum traj2.tum --align-type none  # no alignment"
}

# Check if evo is installed
check_evo() {
    if ! command -v evo_traj &> /dev/null; then
        echo "Error: evo is not installed"
        echo "Install it with: pip install evo"
        exit 1
    fi
}

# Function to get alignment arguments for evo commands
get_alignment_args() {
    local cmd_type="$1"  # "traj", "ape", or "rpe"
    
    case "$ALIGN_TYPE" in
        none)
            echo ""
            ;;
        se3)
            echo "--align"
            ;;
        sim3)
            echo "--align --correct_scale"
            ;;
        pos|pos_yaw)
            echo "--align_origin"
            ;;
    esac
}

# Parse arguments
if [[ $# -lt 1 ]]; then
    echo "Error: At least one argument is required"
    show_help
    exit 1
fi

# Check for help first
for arg in "$@"; do
    if [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
        show_help
        exit 0
    fi
done

if [[ $# -lt 2 ]]; then
    echo "Error: Two trajectory files are required"
    show_help
    exit 1
fi

TRAJECTORY1="$1"
TRAJECTORY2="$2"
shift 2

# Parse options
while [[ $# -gt 0 ]]; do
    case $1 in
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --plot-mode)
            PLOT_MODE="$2"
            # Avoid 3D plot modes due to matplotlib issues
            if [[ "$PLOT_MODE" == "xyz" ]]; then
                echo "Warning: 3D plotting disabled due to matplotlib conflicts, using xy mode"
                PLOT_MODE="xy"
            fi
            shift 2
            ;;
        --time-tol)
            TIME_TOLERANCE="$2"
            shift 2
            ;;
        --align-type)
            ALIGN_TYPE="$2"
            case "$ALIGN_TYPE" in
                none|se3|sim3|pos|pos_yaw)
                    # Valid alignment types
                    if [[ "$ALIGN_TYPE" == "pos_yaw" ]]; then
                        echo "Note: pos_yaw uses origin alignment (best approximation available in evo)"
                    fi
                    ;;
                *)
                    echo "Error: Invalid alignment type '$ALIGN_TYPE'"
                    echo "Valid options: none, se3, sim3, pos, pos_yaw"
                    exit 1
                    ;;
            esac
            shift 2
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Validate input files
if [[ ! -f "$TRAJECTORY1" ]]; then
    echo "Error: Trajectory file '$TRAJECTORY1' not found"
    exit 1
fi

if [[ ! -f "$TRAJECTORY2" ]]; then
    echo "Error: Trajectory file '$TRAJECTORY2' not found"
    exit 1
fi

# Set output directory
if [[ -z "$OUTPUT_DIR" ]]; then
    OUTPUT_DIR="$(dirname "$TRAJECTORY1")/analysis_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUTPUT_DIR"

# Check evo installation
check_evo

echo "Odometry Analysis using evo"
echo "=========================="
echo "Trajectory 1: $TRAJECTORY1"
echo "Trajectory 2: $TRAJECTORY2"
echo "Output dir:   $OUTPUT_DIR"
echo "Plot mode:    $PLOT_MODE"
echo "Time tol:     $TIME_TOLERANCE s"
echo "Align type:   $ALIGN_TYPE"
echo ""

# Get base names for labels
TRAJ1_NAME=$(basename "$TRAJECTORY1" .tum)
TRAJ2_NAME=$(basename "$TRAJECTORY2" .tum)

# 1. Trajectory comparison and visualization
echo "1. Comparing trajectories..."
ALIGN_ARGS=$(get_alignment_args "traj")
evo_traj tum "$TRAJECTORY1" "$TRAJECTORY2" \
    --ref "$TRAJECTORY1" \
    $ALIGN_ARGS \
    --sync \
    --correct_scale \
    --plot_mode "$PLOT_MODE" \
    --t_max_diff "$TIME_TOLERANCE" \
    --save_plot "$OUTPUT_DIR/trajectory_comparison.png" \
    --save_table "$OUTPUT_DIR/trajectory_comparison_stats.csv" \
    2>/dev/null || {
        echo "Warning: Plot generation failed, trying without plots..."
        evo_traj tum "$TRAJECTORY1" "$TRAJECTORY2" \
            --ref "$TRAJECTORY1" \
            $ALIGN_ARGS \
            --sync \
            --correct_scale \
            --plot_mode "$PLOT_MODE" \
            --t_max_diff "$TIME_TOLERANCE" \
            --save_table "$OUTPUT_DIR/trajectory_comparison_stats.csv" \
            || echo "Warning: Trajectory comparison failed"
    }

# 2. Absolute Pose Error (APE)
echo ""
echo "2. Computing Absolute Pose Error (APE)..."
ALIGN_ARGS=$(get_alignment_args "ape")
evo_ape tum "$TRAJECTORY1" "$TRAJECTORY2" \
    $ALIGN_ARGS \
    --plot_mode "$PLOT_MODE" \
    --correct_scale \
    --t_max_diff "$TIME_TOLERANCE" \
    --save_plot "$OUTPUT_DIR/ape_comparison.png" \
    --save_results "$OUTPUT_DIR/ape_results.zip" \
    2>/dev/null || {
        echo "Warning: APE plot generation failed, trying without plots..."
        evo_ape tum "$TRAJECTORY1" "$TRAJECTORY2" \
            $ALIGN_ARGS \
            --plot_mode "$PLOT_MODE" \
            --correct_scale \
            --t_max_diff "$TIME_TOLERANCE" \
            --save_results "$OUTPUT_DIR/ape_results.zip" \
            || echo "Warning: APE analysis failed"
    }

# 3. Relative Pose Error (RPE)
echo ""
echo "3. Computing Relative Pose Error (RPE)..."
ALIGN_ARGS=$(get_alignment_args "rpe")
evo_rpe tum "$TRAJECTORY1" "$TRAJECTORY2" \
    $ALIGN_ARGS \
    --plot_mode "$PLOT_MODE" \
    --correct_scale \
    --t_max_diff "$TIME_TOLERANCE" \
    --save_plot "$OUTPUT_DIR/rpe_comparison.png" \
    --save_results "$OUTPUT_DIR/rpe_results.zip" \
    2>/dev/null || {
        echo "Warning: RPE plot generation failed, trying without plots..."
        evo_rpe tum "$TRAJECTORY1" "$TRAJECTORY2" \
            $ALIGN_ARGS \
            --plot_mode "$PLOT_MODE" \
            --correct_scale \
            --t_max_diff "$TIME_TOLERANCE" \
            --save_results "$OUTPUT_DIR/rpe_results.zip" \
            || echo "Warning: RPE analysis failed"
    }

# 4. Generate summary
echo ""
echo "4. Generating analysis summary..."
{
    echo "Odometry Analysis Summary"
    echo "========================"
    echo "Generated on: $(date)"
    echo ""
    echo "Input Files:"
    echo "- Reference: $TRAJECTORY1"
    echo "- Estimate:  $TRAJECTORY2"
    echo ""
    echo "Configuration:"
    echo "- Plot mode: $PLOT_MODE"
    echo "- Alignment: $ALIGN_TYPE"
    echo ""
    echo "Output Files:"
    echo "- trajectory_comparison.png         - Trajectory overlay plot"
    echo "- trajectory_comparison_stats.csv   - Trajectory statistics"
    echo "- ape_comparison.png                - Absolute Pose Error plot"
    echo "- ape_results.zip                   - Absolute Pose Error analysis"
    echo "- rpe_comparison.png                - Relative Pose Error plot"
    echo "- rpe_results.zip                   - Relative Pose Error analysis"
    echo ""
    echo "To view detailed results:"
    echo "  evo_res $OUTPUT_DIR/*.zip --use_filenames"
} > "$OUTPUT_DIR/analysis_summary.txt"

echo ""
echo "Analysis complete!"
echo "Results saved to: $OUTPUT_DIR"
echo ""
echo "Generated files:"
ls -la "$OUTPUT_DIR"
echo ""
echo "To view detailed results run:"
echo "  evo_res $OUTPUT_DIR/*.zip --use_filenames"

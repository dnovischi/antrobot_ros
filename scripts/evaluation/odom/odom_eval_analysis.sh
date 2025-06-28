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
ALIGN="--align"
SHOW_PLOT="--plot"

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
    echo "  --plot-mode MODE  Plot mode: xy, xyz, xz, yz (default: xy)"
    echo "  --no-align        Don't align trajectories"
    echo "  --no-plot         Don't show interactive plots"
    echo "  --help, -h        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 wheel_odom_20250628_143000.tum odom_20250628_143000.tum"
    echo "  $0 traj1.tum traj2.tum --output-dir ~/results --plot-mode xyz"
    echo "  $0 traj1.tum traj2.tum --no-plot --no-align"
}

# Check if evo is installed
check_evo() {
    if ! command -v evo_traj &> /dev/null; then
        echo "Error: evo is not installed"
        echo "Install it with: pip install evo"
        exit 1
    fi
}

# Parse arguments
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
            shift 2
            ;;
        --no-align)
            ALIGN=""
            shift
            ;;
        --no-plot)
            SHOW_PLOT=""
            shift
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
echo "Align:        $([ -n "$ALIGN" ] && echo "yes" || echo "no")"
echo "Show plots:   $([ -n "$SHOW_PLOT" ] && echo "yes" || echo "no")"
echo ""

# Get base names for labels
TRAJ1_NAME=$(basename "$TRAJECTORY1" .tum)
TRAJ2_NAME=$(basename "$TRAJECTORY2" .tum)

# 1. Trajectory comparison and visualization
echo "1. Comparing trajectories..."
evo_traj tum "$TRAJECTORY1" "$TRAJECTORY2" \
    --ref "$TRAJECTORY1" \
    $ALIGN \
    $SHOW_PLOT \
    --plot_mode "$PLOT_MODE" \
    --save_plot "$OUTPUT_DIR/trajectory_comparison.png" \
    --save_results "$OUTPUT_DIR/trajectory_comparison.zip" \
    || echo "Warning: Trajectory comparison failed"

# 2. Absolute Pose Error (APE)
echo ""
echo "2. Computing Absolute Pose Error (APE)..."
evo_ape tum "$TRAJECTORY1" "$TRAJECTORY2" \
    $ALIGN \
    $SHOW_PLOT \
    --plot_mode "$PLOT_MODE" \
    --save_plot "$OUTPUT_DIR/ape_comparison.png" \
    --save_results "$OUTPUT_DIR/ape_results.zip" \
    || echo "Warning: APE analysis failed"

# 3. Relative Pose Error (RPE)
echo ""
echo "3. Computing Relative Pose Error (RPE)..."
evo_rpe tum "$TRAJECTORY1" "$TRAJECTORY2" \
    $ALIGN \
    $SHOW_PLOT \
    --plot_mode "$PLOT_MODE" \
    --save_plot "$OUTPUT_DIR/rpe_comparison.png" \
    --save_results "$OUTPUT_DIR/rpe_results.zip" \
    || echo "Warning: RPE analysis failed"

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
    echo "- Alignment: $([ -n "$ALIGN" ] && echo "enabled" || echo "disabled")"
    echo ""
    echo "Output Files:"
    echo "- trajectory_comparison.png/zip - Trajectory overlay and statistics"
    echo "- ape_comparison.png/zip        - Absolute Pose Error analysis"
    echo "- rpe_comparison.png/zip        - Relative Pose Error analysis"
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

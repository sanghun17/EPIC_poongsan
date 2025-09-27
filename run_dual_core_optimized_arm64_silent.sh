#!/bin/bash

# EPIC ARM64 Dual-Core Optimized Runner (Silent Mode)
# Optimized for ARM64 architecture with minimal output for maximum performance

echo "🚀 Starting EPIC with ARM64 Dual-Core Optimizations (Silent Mode)..."

# Check if we're on ARM64
if [ "$(uname -m)" != "aarch64" ]; then
    echo "⚠️  Warning: This script is optimized for ARM64 (aarch64), detected: $(uname -m)"
fi

# Fixed IPs
TARGET_BOARD_IP="192.168.10.2"
HOST_PC_IP="192.168.10.3"

# Get parameters
MAP_NAME=${1:-garage}
RUN_SAMPLER=${2:-true}

# Use ARM64 dual-core optimized config
TARGET_WS="/home/root/catkin_ws"
EPIC_DIR="$TARGET_WS/src/EPIC/src/global_planner"
CONFIG_FILE="$EPIC_DIR/exploration_manager/config/target_board_garage.yaml"
TSP_DIR="$EPIC_DIR/utils/lkh_tsp_solver/resource"

echo "🗺️  Map: $MAP_NAME | Config: dual_core | Sampler: $RUN_SAMPLER"

# ARM64 system optimizations (silent)
echo "⚙️  Applying ARM64 optimizations..."

# 1. ARM64 CPU governor optimization
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    if [ -w "$cpu" ]; then
        echo performance > "$cpu" 2>/dev/null
    fi
done

# 2. ARM64 CPU frequency optimization
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_min_freq; do
    if [ -w "$cpu" ]; then
        cat "${cpu%/*}/cpuinfo_max_freq" > "$cpu" 2>/dev/null
    fi
done

# 3. ARM64 cache optimization
echo 1 > /proc/sys/vm/zone_reclaim_mode 2>/dev/null
echo 15 > /proc/sys/vm/dirty_background_ratio 2>/dev/null
echo 20 > /proc/sys/vm/dirty_ratio 2>/dev/null

# 4. ARM64 memory optimization
echo 1 > /proc/sys/vm/overcommit_memory 2>/dev/null
echo 10 > /proc/sys/vm/swappiness 2>/dev/null
echo 60 > /proc/sys/vm/vfs_cache_pressure 2>/dev/null

# 5. ARM64 power optimization
for cpu in /sys/devices/system/cpu/cpu*/power/energy_perf_bias; do
    if [ -w "$cpu" ]; then
        echo 0 > "$cpu" 2>/dev/null
    fi
done

# 6. ARM64 interrupt optimization
for irq in /proc/irq/*/smp_affinity; do
    if [ -w "$irq" ]; then
        echo 3 > "$irq" 2>/dev/null
    fi
done

# 7. Disable unnecessary services
systemctl stop bluetooth avahi-daemon cups ModemManager 2>/dev/null

# Set ROS environment
export ROS_MASTER_URI=http://$TARGET_BOARD_IP:11311
export ROS_IP=$TARGET_BOARD_IP
export ROS_HOSTNAME=$TARGET_BOARD_IP

# ARM64 ROS optimizations
export ROS_PYTHON_LOG_CONFIG_FILE=""
export ROSCONSOLE_CONFIG_FILE=""
export ROS_LOG_LEVEL=ERROR  # Minimal logging for performance

# ARM64 memory optimization
export MALLOC_MMAP_THRESHOLD_=65536
export MALLOC_TRIM_THRESHOLD_=65536
export MALLOC_TOP_PAD_=65536
export MALLOC_ARENA_MAX=2

# ARM64 threading optimization
export OMP_NUM_THREADS=2
export MKL_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=2

# Source ROS
source /opt/ros/noetic/setup.bash

# Set up ARM64 environment
export LD_LIBRARY_PATH=$TARGET_WS/devel/lib:$LD_LIBRARY_PATH
export PATH=$TARGET_WS/devel/lib/epic_planner:$TARGET_WS/devel/lib/plan_manage:$TARGET_WS/devel/lib/traj_utils:$PATH
export ROS_PACKAGE_PATH=$TARGET_WS/src:$ROS_PACKAGE_PATH

# Topic configuration
ODOM_TOPIC="/quad_0/lidar_slam/odom"
CLOUD_TOPIC="/quad0_pcl_render_node/cloud"

# Load ROS parameters
rosparam load $CONFIG_FILE /exploration_node
rosparam set /exploration_node/odometry_topic "$ODOM_TOPIC"
rosparam set /exploration_node/cloud_topic "$CLOUD_TOPIC"

# LiDAR parameter sync
RENDER_NODE="/quad0_pcl_render_node"
REQUIRED_PARAMS=("is_360lidar" "vertical_fov" "sensing_horizon" "lidar_pitch" "yaw_fov")

MISSING_PARAMS=()
for param in "${REQUIRED_PARAMS[@]}"; do
    if ! rosparam get $RENDER_NODE/$param > /dev/null 2>&1; then
        MISSING_PARAMS+=("$RENDER_NODE/$param")
    fi
done

if [ ${#MISSING_PARAMS[@]} -ne 0 ]; then
    echo "❌ Error: Required LiDAR parameters not found"
    exit 1
fi

# Read and set LiDAR parameters
IS_360_LIDAR=$(rosparam get $RENDER_NODE/is_360lidar)
VERTICAL_FOV=$(rosparam get $RENDER_NODE/vertical_fov)
SENSING_HORIZON=$(rosparam get $RENDER_NODE/sensing_horizon)
LIDAR_PITCH=$(rosparam get $RENDER_NODE/lidar_pitch)
YAW_FOV=$(rosparam get $RENDER_NODE/yaw_fov)

# Calculate FOV values
if [ "$IS_360_LIDAR" = "1" ]; then
    FOV_UP="90.0"
    FOV_DOWN="-90.0"
    FOV_VP_UP="90.0"
    FOV_VP_DOWN="-90.0"
else
    FOV_UP=$(echo "$VERTICAL_FOV / 2" | bc -l)
    FOV_DOWN=$(echo "-1 * $VERTICAL_FOV / 2" | bc -l)
    VP_MARGIN="2.0"
    FOV_VP_UP=$(echo "$FOV_UP - $VP_MARGIN" | bc -l)
    FOV_VP_DOWN=$(echo "$FOV_DOWN + $VP_MARGIN" | bc -l)
fi

# Set exploration parameters
rosparam set /exploration_node/lidar_perception/max_ray_length "$SENSING_HORIZON"
rosparam set /exploration_node/lidar_perception/fov_up "$FOV_UP"
rosparam set /exploration_node/lidar_perception/fov_down "$FOV_DOWN"
rosparam set /exploration_node/lidar_perception/fov_viewpoint_up "$FOV_VP_UP"
rosparam set /exploration_node/lidar_perception/fov_viewpoint_down "$FOV_VP_DOWN"
rosparam set /exploration_node/lidar_perception/lidar_pitch "$LIDAR_PITCH"

# ARM64 CPU affinity function (silent)
set_arm64_cpu_affinity_and_rt() {
    local pid=$1
    local cpu_core=$2
    local priority=$3
    local name=$4
    
    # Set CPU affinity, real-time scheduling, and process priority silently
    taskset -cp $cpu_core $pid &>/dev/null
    chrt -p -f $priority $pid &>/dev/null
    renice -n -10 $pid &>/dev/null
    echo -500 > /proc/$pid/oom_score_adj 2>/dev/null
}

echo "🚀 Starting ARM64 optimized EPIC nodes..."

# 1. Start exploration_node on ARM64 core 0
cd $TSP_DIR && \
nice -n -15 $TARGET_WS/devel/lib/epic_planner/exploration_node \
    __name:=exploration_node &

EXPLORATION_PID=$!
sleep 1
set_arm64_cpu_affinity_and_rt $EXPLORATION_PID 0 85 "exploration_node"

# 2. Start traj_server on ARM64 core 1
nice -n -15 $TARGET_WS/devel/lib/plan_manage/traj_server \
    __name:=traj_server \
    _config_file:=$CONFIG_FILE \
    /position_cmd:=/planning/pos_cmd &

TRAJ_SERVER_PID=$!
sleep 1
set_arm64_cpu_affinity_and_rt $TRAJ_SERVER_PID 1 75 "traj_server"

# 3. Start fast_planner_node on ARM64 core 1
nice -n -15 $TARGET_WS/devel/lib/plan_manage/fast_planner_node \
    __name:=fast_planner_node \
    _config_file:=$CONFIG_FILE &

FAST_PLANNER_PID=$!
sleep 1
set_arm64_cpu_affinity_and_rt $FAST_PLANNER_PID 1 65 "fast_planner_node"

# 4. Start trajectory sampler if requested
if [ "$RUN_SAMPLER" = "true" ]; then
    nice -n -10 $TARGET_WS/devel/lib/epic_planner/custom_trajectory_sampler \
        __name:=trajectory_sampler \
        _sample_interval:=0.1 \
        _total_duration:=10.0 \
        _auto_duration:=true &
    
    SAMPLER_PID=$!
    sleep 1
    set_arm64_cpu_affinity_and_rt $SAMPLER_PID 0 55 "trajectory_sampler"
fi

# Display minimal process information
echo "✅ ARM64 nodes started:"
echo "   exploration_node (PID: $EXPLORATION_PID) → Core 0"
echo "   traj_server (PID: $TRAJ_SERVER_PID) → Core 1"
echo "   fast_planner_node (PID: $FAST_PLANNER_PID) → Core 1"
if [ "$RUN_SAMPLER" = "true" ]; then
    echo "   trajectory_sampler (PID: $SAMPLER_PID) → Core 0"
fi

echo ""
echo "🎯 ARM64 optimizations applied - running in silent mode for maximum performance"
echo "🛑 Press Ctrl+C to stop"

# Wait for interrupt with clean shutdown
trap "echo '🛑 Stopping nodes...'; kill $EXPLORATION_PID $TRAJ_SERVER_PID $FAST_PLANNER_PID $SAMPLER_PID 2>/dev/null; echo '🏁 Stopped'; exit 0" INT

# Run silently without performance monitoring
wait 
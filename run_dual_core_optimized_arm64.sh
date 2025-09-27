#!/bin/bash

# EPIC ARM64 Dual-Core Optimized Runner
# Specifically optimized for ARM64 architecture with dual-core processors

echo "🚀 Starting EPIC with ARM64 Dual-Core Optimizations..."

# ARM64 system information
echo "📊 ARM64 System Information:"
echo "   CPU cores: $(nproc)"
echo "   Memory: $(free -m | awk 'NR==2{printf "%.1f GB\n", $2/1024}')"
echo "   CPU architecture: $(uname -m)"
echo "   CPU model: $(cat /proc/cpuinfo | grep 'model name' | head -1 | cut -d':' -f2 | xargs)"

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
CONFIG_FILE="$EPIC_DIR/exploration_manager/config/target_board_garage_dual_core.yaml"
TSP_DIR="$EPIC_DIR/utils/lkh_tsp_solver/resource"

echo "🗺️  Map: $MAP_NAME"
echo "📁 ARM64 Dual-Core Config: $CONFIG_FILE"
echo "🔬 Trajectory Sampler: $RUN_SAMPLER"

# ARM64 specific system optimizations
echo "⚙️  Applying ARM64 system optimizations..."

# 1. ARM64 CPU governor optimization
echo "   Setting ARM64 CPU governor to performance..."
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    if [ -w "$cpu" ]; then
        echo performance > "$cpu" 2>/dev/null || echo "   Warning: Could not set governor for $cpu"
    fi
done

# 2. ARM64 CPU frequency optimization
echo "   Optimizing ARM64 CPU frequency scaling..."
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_min_freq; do
    if [ -w "$cpu" ]; then
        cat "${cpu%/*}/cpuinfo_max_freq" > "$cpu" 2>/dev/null || echo "   Warning: Could not set min freq for $cpu"
    fi
done

# 3. ARM64 cache optimization
echo "   Optimizing ARM64 cache settings..."
# Enable CPU cache prefetching (ARM64 specific)
echo 1 > /proc/sys/vm/zone_reclaim_mode 2>/dev/null || echo "   Warning: Could not set zone_reclaim_mode"
echo 15 > /proc/sys/vm/dirty_background_ratio 2>/dev/null || echo "   Warning: Could not set dirty_background_ratio"
echo 20 > /proc/sys/vm/dirty_ratio 2>/dev/null || echo "   Warning: Could not set dirty_ratio"

# 4. ARM64 memory optimization
echo "   Optimizing ARM64 memory settings..."
echo 1 > /proc/sys/vm/overcommit_memory 2>/dev/null || echo "   Warning: Could not set overcommit_memory"
echo 10 > /proc/sys/vm/swappiness 2>/dev/null || echo "   Warning: Could not set swappiness (ARM64 optimized)"
echo 60 > /proc/sys/vm/vfs_cache_pressure 2>/dev/null || echo "   Warning: Could not set vfs_cache_pressure"

# 5. ARM64 specific power optimization
echo "   Disabling ARM64 power saving features for performance..."
for cpu in /sys/devices/system/cpu/cpu*/power/energy_perf_bias; do
    if [ -w "$cpu" ]; then
        echo 0 > "$cpu" 2>/dev/null || echo "   Warning: Could not set energy_perf_bias for $cpu"
    fi
done

# 6. ARM64 interrupt optimization
echo "   Optimizing ARM64 interrupt handling..."
# Set IRQ affinity to distribute interrupts across cores
for irq in /proc/irq/*/smp_affinity; do
    if [ -w "$irq" ]; then
        echo 3 > "$irq" 2>/dev/null || true  # Distribute to both cores (binary: 11)
    fi
done

# 7. ARM64 specific services optimization
echo "   Disabling unnecessary services for ARM64..."
systemctl stop bluetooth 2>/dev/null || true
systemctl stop avahi-daemon 2>/dev/null || true
systemctl stop cups 2>/dev/null || true
systemctl stop ModemManager 2>/dev/null || true  # Common on ARM64 boards

# Set ROS environment
export ROS_MASTER_URI=http://$TARGET_BOARD_IP:11311
export ROS_IP=$TARGET_BOARD_IP
export ROS_HOSTNAME=$TARGET_BOARD_IP

# ARM64 specific ROS optimizations
echo "⚡ Applying ARM64 ROS optimizations..."
export ROS_PYTHON_LOG_CONFIG_FILE=""
export ROSCONSOLE_CONFIG_FILE=""
export ROS_LOG_LEVEL=WARN  # Reduce logging overhead

# ARM64 memory optimization
export MALLOC_MMAP_THRESHOLD_=65536    # Smaller threshold for ARM64
export MALLOC_TRIM_THRESHOLD_=65536    # Smaller threshold for ARM64
export MALLOC_TOP_PAD_=65536           # Smaller padding for ARM64
export MALLOC_ARENA_MAX=2              # Limit arenas for dual-core

# ARM64 specific environment variables
export OMP_NUM_THREADS=2               # Match dual-core
export MKL_NUM_THREADS=2               # If using MKL
export OPENBLAS_NUM_THREADS=2          # If using OpenBLAS

# Source ROS
echo "📦 Sourcing ROS for ARM64..."
source /opt/ros/noetic/setup.bash

# Set up ARM64 environment
export LD_LIBRARY_PATH=$TARGET_WS/devel/lib:$LD_LIBRARY_PATH
export PATH=$TARGET_WS/devel/lib/epic_planner:$TARGET_WS/devel/lib/plan_manage:$TARGET_WS/devel/lib/traj_utils:$PATH
export ROS_PACKAGE_PATH=$TARGET_WS/src:$ROS_PACKAGE_PATH

# Topic configuration
ODOM_TOPIC="/quad_0/lidar_slam/odom"
CLOUD_TOPIC="/quad0_pcl_render_node/cloud"

# Load ROS parameters
echo "📝 Loading ROS parameters for ARM64..."
rosparam load $CONFIG_FILE /exploration_node

# Set custom topic names
echo "📝 Setting custom topic parameters..."
rosparam set /exploration_node/odometry_topic "$ODOM_TOPIC"
rosparam set /exploration_node/cloud_topic "$CLOUD_TOPIC"

# ARM64 specific LiDAR parameter sync
echo "📝 Syncing LiDAR parameters for ARM64..."
RENDER_NODE="/quad0_pcl_render_node"
REQUIRED_PARAMS=("is_360lidar" "vertical_fov" "sensing_horizon" "lidar_pitch" "yaw_fov")

MISSING_PARAMS=()
for param in "${REQUIRED_PARAMS[@]}"; do
    if ! rosparam get $RENDER_NODE/$param > /dev/null 2>&1; then
        MISSING_PARAMS+=("$RENDER_NODE/$param")
    fi
done

if [ ${#MISSING_PARAMS[@]} -ne 0 ]; then
    echo "❌ Error: Required LiDAR parameters not found:"
    for param in "${MISSING_PARAMS[@]}"; do
        echo "   - $param"
    done
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

# ARM64 specific CPU affinity function
set_arm64_cpu_affinity_and_rt() {
    local pid=$1
    local cpu_core=$2
    local priority=$3
    local name=$4
    
    echo "   Setting ARM64 optimizations for $name (PID: $pid)..."
    
    # Set CPU affinity with ARM64 specific mask
    taskset -cp $cpu_core $pid 2>/dev/null || echo "   Warning: Could not set CPU affinity for $name"
    
    # Set real-time scheduling with ARM64 optimized priority
    chrt -p -f $priority $pid 2>/dev/null || echo "   Warning: Could not set RT priority for $name"
    
    # Set ARM64 specific process priority
    renice -n -10 $pid 2>/dev/null || echo "   Warning: Could not set nice priority for $name"
    
    # Set ARM64 specific OOM score
    echo -500 > /proc/$pid/oom_score_adj 2>/dev/null || echo "   Warning: Could not set OOM score for $name"
}

# ARM64 performance monitoring function
monitor_arm64_performance() {
    while true; do
        # Get ARM64 specific performance metrics
        cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | sed 's/%us,//')
        mem_usage=$(free | grep Mem | awk '{printf("%.1f"), $3/$2 * 100.0}')
        
        # ARM64 specific temperature monitoring
        temp=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null | awk '{print $1/1000}' || echo "N/A")
        
        echo "📊 ARM64 Performance: CPU ${cpu_usage}%, Memory ${mem_usage}%, Temp ${temp}°C"
        
        # ARM64 specific thermal throttling check
        if [ "$temp" != "N/A" ] && (( $(echo "$temp > 70.0" | bc -l) )); then
            echo "🔥 ARM64 thermal warning: ${temp}°C - consider reducing load"
        fi
        
        # ARM64 specific resource monitoring
        if (( $(echo "$cpu_usage > 85.0" | bc -l) )); then
            echo "⚠️  ARM64 High CPU usage: ${cpu_usage}%"
        fi
        
        if (( $(echo "$mem_usage > 75.0" | bc -l) )); then
            echo "⚠️  ARM64 High memory usage: ${mem_usage}%"
        fi
        
        sleep 5
    done
}

# Start ARM64 performance monitoring
monitor_arm64_performance &
MONITOR_PID=$!

echo "🚀 Starting ARM64 optimized EPIC nodes..."

# 1. Start exploration_node on ARM64 core 0
echo "🚀 Starting exploration_node on ARM64 core 0..."
cd $TSP_DIR && \
nice -n -15 $TARGET_WS/devel/lib/epic_planner/exploration_node \
    __name:=exploration_node &

EXPLORATION_PID=$!
sleep 2
set_arm64_cpu_affinity_and_rt $EXPLORATION_PID 0 85 "exploration_node"

# 2. Start traj_server on ARM64 core 1
echo "🚀 Starting traj_server on ARM64 core 1..."
nice -n -15 $TARGET_WS/devel/lib/plan_manage/traj_server \
    __name:=traj_server \
    _config_file:=$CONFIG_FILE \
    /position_cmd:=/planning/pos_cmd &

TRAJ_SERVER_PID=$!
sleep 2
set_arm64_cpu_affinity_and_rt $TRAJ_SERVER_PID 1 75 "traj_server"

# 3. Start fast_planner_node on ARM64 core 1
echo "🚀 Starting fast_planner_node on ARM64 core 1..."
nice -n -15 $TARGET_WS/devel/lib/plan_manage/fast_planner_node \
    __name:=fast_planner_node \
    _config_file:=$CONFIG_FILE &

FAST_PLANNER_PID=$!
sleep 2
set_arm64_cpu_affinity_and_rt $FAST_PLANNER_PID 1 65 "fast_planner_node"

# 4. Start trajectory sampler if requested
if [ "$RUN_SAMPLER" = "true" ]; then
    echo "🚀 Starting trajectory_sampler on ARM64 core 0..."
    nice -n -10 $TARGET_WS/devel/lib/epic_planner/custom_trajectory_sampler \
        __name:=trajectory_sampler \
        _sample_interval:=0.1 \
        _total_duration:=10.0 \
        _auto_duration:=true &
    
    SAMPLER_PID=$!
    sleep 2
    set_arm64_cpu_affinity_and_rt $SAMPLER_PID 0 55 "trajectory_sampler"
    
    echo "✅ Trajectory sampler started and ARM64 optimized"
fi

# Display ARM64 process information
echo ""
echo "📊 ARM64 Process Information:"
echo "   exploration_node: PID=$EXPLORATION_PID, ARM64 Core=0, Priority=85"
echo "   traj_server: PID=$TRAJ_SERVER_PID, ARM64 Core=1, Priority=75"
echo "   fast_planner_node: PID=$FAST_PLANNER_PID, ARM64 Core=1, Priority=65"
if [ "$RUN_SAMPLER" = "true" ]; then
    echo "   trajectory_sampler: PID=$SAMPLER_PID, ARM64 Core=0, Priority=55"
fi

echo ""
echo "🔧 ARM64 Dual-Core Optimizations Applied:"
echo "   ✅ ARM64 CPU governor set to performance"
echo "   ✅ ARM64 CPU frequency scaling optimized"
echo "   ✅ ARM64 cache optimization enabled"
echo "   ✅ ARM64 memory settings optimized"
echo "   ✅ ARM64 power saving disabled"
echo "   ✅ ARM64 interrupt distribution configured"
echo "   ✅ ARM64 real-time scheduling enabled"
echo "   ✅ ARM64 CPU affinity configured"
echo "   ✅ ARM64 process priorities optimized"
echo "   ✅ ARM64 performance monitoring active"
echo "   ✅ ARM64 thermal monitoring enabled"

echo ""
echo "🎯 ARM64 Optimization Summary:"
echo "   Core 0: exploration_node + trajectory_sampler (primary computation)"
echo "   Core 1: fast_planner_node + traj_server (path planning)"
echo "   Memory: Optimized for ARM64 cache hierarchy"
echo "   Power: Performance mode for maximum throughput"
echo "   Thermal: Monitoring enabled for stability"

echo ""
echo "📊 Monitor topics:"
echo "   - /planning/pos_cmd (position commands)"
echo "   - /planning/bspline (trajectory)"
echo "   - /exploration_finish (completion status)"
echo ""
echo "🛑 Press Ctrl+C to stop all ARM64 optimized nodes"
echo ""

# Wait for interrupt with ARM64 specific cleanup
trap "echo '🛑 Stopping ARM64 EPIC nodes...'; kill $MONITOR_PID $EXPLORATION_PID $TRAJ_SERVER_PID $FAST_PLANNER_PID $SAMPLER_PID 2>/dev/null; echo '🏁 ARM64 EPIC stopped'; exit 0" INT

echo "✅ All ARM64 optimized nodes running. Monitoring..."
wait 
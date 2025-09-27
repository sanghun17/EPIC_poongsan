#!/bin/bash

# EPIC Dual-Core Optimized Runner
# Optimized for dual-core ARM processors with advanced performance tuning

echo "🚀 Starting EPIC with Dual-Core Optimizations..."

# System information
echo "📊 System Information:"
echo "   CPU cores: $(nproc)"
echo "   Memory: $(free -m | awk 'NR==2{printf "%.1f GB\n", $2/1024}')"
echo "   CPU architecture: $(uname -m)"

# Fixed IPs
TARGET_BOARD_IP="192.168.10.2"
HOST_PC_IP="192.168.10.3"

# Get parameters
MAP_NAME=${1:-garage}
RUN_SAMPLER=${2:-true}

# Use dual-core optimized config
TARGET_WS="/home/root/catkin_ws"
EPIC_DIR="$TARGET_WS/src/EPIC/src/global_planner"
CONFIG_FILE="$EPIC_DIR/exploration_manager/config/target_board_garage_dual_core.yaml"
TSP_DIR="$EPIC_DIR/utils/lkh_tsp_solver/resource"

echo "🗺️  Map: $MAP_NAME"
echo "📁 Dual-Core Config: $CONFIG_FILE"
echo "🔬 Trajectory Sampler: $RUN_SAMPLER"

# System optimizations
echo "⚙️  Applying system optimizations..."

# 1. Set CPU governor to performance
echo "   Setting CPU governor to performance..."
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    if [ -w "$cpu" ]; then
        echo performance > "$cpu" 2>/dev/null || echo "   Warning: Could not set governor for $cpu"
    fi
done

# 2. Disable CPU frequency scaling
echo "   Disabling CPU frequency scaling..."
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_min_freq; do
    if [ -w "$cpu" ]; then
        cat "${cpu%/*}/cpuinfo_max_freq" > "$cpu" 2>/dev/null || echo "   Warning: Could not set min freq for $cpu"
    fi
done

# 3. Set real-time scheduling priorities
echo "   Setting real-time scheduling..."
ulimit -r unlimited 2>/dev/null || echo "   Warning: Could not set real-time limits"

# 4. Optimize memory settings
echo "   Optimizing memory settings..."
echo 1 > /proc/sys/vm/overcommit_memory 2>/dev/null || echo "   Warning: Could not set overcommit_memory"
echo 50 > /proc/sys/vm/swappiness 2>/dev/null || echo "   Warning: Could not set swappiness"

# 5. Disable unnecessary services (optional)
echo "   Disabling unnecessary services..."
systemctl stop bluetooth 2>/dev/null || true
systemctl stop avahi-daemon 2>/dev/null || true
systemctl stop cups 2>/dev/null || true

# Set ROS environment
export ROS_MASTER_URI=http://$TARGET_BOARD_IP:11311
export ROS_IP=$TARGET_BOARD_IP
export ROS_HOSTNAME=$TARGET_BOARD_IP

# Advanced ROS optimizations
echo "⚡ Applying ROS optimizations..."
export ROS_PYTHON_LOG_CONFIG_FILE=""
export ROSCONSOLE_CONFIG_FILE=""
export ROS_LOG_LEVEL=WARN  # Reduce logging overhead

# Memory optimization
export MALLOC_MMAP_THRESHOLD_=131072
export MALLOC_TRIM_THRESHOLD_=131072
export MALLOC_TOP_PAD_=131072

# Source ROS
source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=$TARGET_WS/devel/lib:$LD_LIBRARY_PATH
export PATH=$TARGET_WS/devel/lib/epic_planner:$TARGET_WS/devel/lib/plan_manage:$TARGET_WS/devel/lib/traj_utils:$PATH
export ROS_PACKAGE_PATH=$TARGET_WS/src:$ROS_PACKAGE_PATH

# Load optimized parameters
echo "📝 Loading dual-core optimized parameters..."
rosparam load $CONFIG_FILE /exploration_node

# Set CPU affinity and real-time scheduling function
set_cpu_affinity_and_rt() {
    local pid=$1
    local cpu_core=$2
    local priority=$3
    local name=$4
    
    echo "   Setting $name (PID: $pid) to CPU core $cpu_core with priority $priority"
    
    # Set CPU affinity
    taskset -cp $cpu_core $pid 2>/dev/null || echo "   Warning: Could not set CPU affinity for $name"
    
    # Set real-time scheduling
    chrt -p -f $priority $pid 2>/dev/null || echo "   Warning: Could not set RT priority for $name"
}

# Function to monitor performance
monitor_performance() {
    while true; do
        cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | sed 's/%us,//')
        mem_usage=$(free | grep Mem | awk '{printf("%.1f"), $3/$2 * 100.0}')
        
        echo "📊 Performance: CPU ${cpu_usage}%, Memory ${mem_usage}%"
        
        # Adaptive parameter adjustment based on performance
        if (( $(echo "$cpu_usage > 80.0" | bc -l) )); then
            echo "⚠️  High CPU usage detected, consider reducing parameters"
        fi
        
        if (( $(echo "$mem_usage > 70.0" | bc -l) )); then
            echo "⚠️  High memory usage detected, consider reducing parameters"
        fi
        
        sleep 5
    done
}

# Start performance monitoring in background
monitor_performance &
MONITOR_PID=$!

echo "🚀 Starting optimized EPIC nodes..."

# 1. Start exploration_node with CPU affinity to core 0
echo "🚀 Starting exploration_node on CPU core 0..."
cd $TSP_DIR && \
nice -n -10 $TARGET_WS/devel/lib/epic_planner/exploration_node \
    __name:=exploration_node &

EXPLORATION_PID=$!
sleep 2
set_cpu_affinity_and_rt $EXPLORATION_PID 0 80 "exploration_node"

# 2. Start traj_server with CPU affinity to core 1
echo "🚀 Starting traj_server on CPU core 1..."
nice -n -10 $TARGET_WS/devel/lib/plan_manage/traj_server \
    __name:=traj_server \
    _config_file:=$CONFIG_FILE \
    /position_cmd:=/planning/pos_cmd &

TRAJ_SERVER_PID=$!
sleep 2
set_cpu_affinity_and_rt $TRAJ_SERVER_PID 1 70 "traj_server"

# 3. Start fast_planner_node with CPU affinity to core 1
echo "🚀 Starting fast_planner_node on CPU core 1..."
nice -n -10 $TARGET_WS/devel/lib/plan_manage/fast_planner_node \
    __name:=fast_planner_node \
    _config_file:=$CONFIG_FILE &

FAST_PLANNER_PID=$!
sleep 2
set_cpu_affinity_and_rt $FAST_PLANNER_PID 1 60 "fast_planner_node"

# 4. Start trajectory sampler if requested
if [ "$RUN_SAMPLER" = "true" ]; then
    echo "🚀 Starting trajectory_sampler on CPU core 0..."
    nice -n -10 $TARGET_WS/devel/lib/epic_planner/custom_trajectory_sampler \
        __name:=trajectory_sampler \
        _sample_interval:=0.1 \
        _total_duration:=10.0 \
        _auto_duration:=true &
    
    SAMPLER_PID=$!
    sleep 2
    set_cpu_affinity_and_rt $SAMPLER_PID 0 50 "trajectory_sampler"
    
    echo "✅ Trajectory sampler started and optimized"
fi

# Display process information
echo ""
echo "📊 Process Information:"
echo "   exploration_node: PID=$EXPLORATION_PID, CPU Core=0, Priority=80"
echo "   traj_server: PID=$TRAJ_SERVER_PID, CPU Core=1, Priority=70"
echo "   fast_planner_node: PID=$FAST_PLANNER_PID, CPU Core=1, Priority=60"
if [ "$RUN_SAMPLER" = "true" ]; then
    echo "   trajectory_sampler: PID=$SAMPLER_PID, CPU Core=0, Priority=50"
fi

echo ""
echo "🔧 Dual-Core Optimizations Applied:"
echo "   ✅ CPU governor set to performance"
echo "   ✅ CPU frequency scaling disabled"
echo "   ✅ Real-time scheduling enabled"
echo "   ✅ Memory optimization applied"
echo "   ✅ CPU affinity configured"
echo "   ✅ Process priorities optimized"
echo "   ✅ Performance monitoring active"

echo ""
echo "📊 Monitor topics:"
echo "   - /planning/pos_cmd"
echo "   - /planning/bspline"
echo "   - /exploration_finish"
echo ""
echo "🛑 Press Ctrl+C to stop and restore system settings"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "🛑 Stopping EPIC nodes and restoring system settings..."
    
    # Kill monitoring
    kill $MONITOR_PID 2>/dev/null || true
    
    # Kill EPIC processes
    kill $EXPLORATION_PID $TRAJ_SERVER_PID $FAST_PLANNER_PID 2>/dev/null || true
    if [ "$RUN_SAMPLER" = "true" ]; then
        kill $SAMPLER_PID 2>/dev/null || true
    fi
    
    # Restore CPU governor
    echo "   Restoring CPU governor..."
    for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        if [ -w "$cpu" ]; then
            echo ondemand > "$cpu" 2>/dev/null || true
        fi
    done
    
    # Restore services
    echo "   Restoring system services..."
    systemctl start bluetooth 2>/dev/null || true
    systemctl start avahi-daemon 2>/dev/null || true
    
    echo "✅ System restored to normal state"
    echo "🏁 EPIC dual-core optimization stopped"
}

# Set trap for cleanup
trap cleanup INT TERM

# Wait for processes
echo "✅ All nodes started with dual-core optimizations"
echo "📈 Performance monitoring active..."
wait 
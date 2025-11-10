#!/bin/bash

# EPIC ARM Target Board Deployment Script
# This script copies all necessary ARM binaries and configurations to the target board

echo "🚀 EPIC ARM Target Board Deployment Script"

# Check if target board IP is provided
if [ -z "$1" ]; then
    echo "❌ Error: Please provide the target board IP address"
    echo "Usage: $0 <TARGET_BOARD_IP> [username]"
    echo "Example: $0 192.168.1.200 ubuntu"
    exit 1
fi

TARGET_IP=$1
USERNAME=${2:-root}
TARGET_PATH="/home/root/catkin_ws"

echo "🎯 Target Board: $USERNAME@$TARGET_IP"
echo "📁 Target Path: $TARGET_PATH"

# Test SSH connection
echo "🔍 Testing SSH connection..."
if ! ssh $USERNAME@$TARGET_IP "echo SSH connection successful"; then
    echo "❌ Error: Could not connect to target board"
    exit 1
fi
echo "✅ SSH connection successful"

# Create necessary directories on target board
echo "📁 Creating directory structure on target board..."
ssh $USERNAME@$TARGET_IP "mkdir -p $TARGET_PATH/devel/lib/{epic_planner,plan_manage,traj_utils,lkh_tsp_solver}"
ssh $USERNAME@$TARGET_IP "mkdir -p $TARGET_PATH/src/EPIC/src/global_planner/exploration_manager/{config,launch}"
ssh $USERNAME@$TARGET_IP "mkdir -p $TARGET_PATH/src/EPIC/src/global_planner/utils/lkh_tsp_solver"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"

echo "📂 Script location: $SCRIPT_DIR"
echo "📂 Workspace location: $WORKSPACE_DIR"

# Copy ARM executable files
echo "📦 Copying ARM executable files..."

# Main executables
EXECUTABLES=(
    "epic_planner/exploration_node"
    "epic_planner/custom_trajectory_sampler"
    "plan_manage/fast_planner_node" 
    "plan_manage/traj_server"
    "traj_utils/process_msg"
    "lkh_tsp_solver/lkh_tsp"
)

for exe in "${EXECUTABLES[@]}"; do
    if [ -f "$WORKSPACE_DIR/devel/lib/$exe" ]; then
        echo "  📄 Copying $exe..."
        scp "$WORKSPACE_DIR/devel/lib/$exe" $USERNAME@$TARGET_IP:$TARGET_PATH/devel/lib/$exe
    else
        echo "  ⚠️  Warning: $exe not found"
    fi
done

# Copy shared libraries
echo "📚 Copying ARM shared libraries..."
LIBRARIES=(
    "libepic_planner.so"
    "libplan_manage.so"
    "libtraj_utils.so"
    "libfrontier_manager.so"
    "libpath_searching.so"
    "libpointcloud_topo.so"
    "libpoly_traj.so"
    "liblkh_tsp_solver.so"
    "libikd_tree.so"
    "libunit_shpere.so"
    "libdecode_msgs.so"
    "libencode_msgs.so"
    "libparallel_bubble_astar.so"
)

for lib in "${LIBRARIES[@]}"; do
    if [ -f "$WORKSPACE_DIR/devel/lib/$lib" ]; then
        echo "  📄 Copying $lib..."
        scp "$WORKSPACE_DIR/devel/lib/$lib" $USERNAME@$TARGET_IP:$TARGET_PATH/devel/lib/$lib
    else
        echo "  ⚠️  Warning: $lib not found"
    fi
done

# Copy configuration files
echo "⚙️  Copying configuration files..."
CONFIG_FILES=(
    "target_board_garage_v2.yaml"
    "target_board_garage.yaml"
    "garage.yaml"
    "cave.yaml"
    "factory.yaml"
)

for config in "${CONFIG_FILES[@]}"; do
    if [ -f "$SCRIPT_DIR/src/EPIC/src/global_planner/exploration_manager/config/$config" ]; then
        echo "  📄 Copying $config..."
        scp "$SCRIPT_DIR/src/EPIC/src/global_planner/exploration_manager/config/$config" \
            $USERNAME@$TARGET_IP:$TARGET_PATH/src/EPIC/src/global_planner/exploration_manager/config/$config
    else
        echo "  ⚠️  Warning: $config not found"
    fi
done

# Copy launch files
echo "🚀 Copying launch files..."
LAUNCH_FILES=(
    "launch_trajectory_sampler.launch"
)

for launch in "${LAUNCH_FILES[@]}"; do
    if [ -f "$SCRIPT_DIR/src/EPIC/src/global_planner/exploration_manager/launch/$launch" ]; then
        echo "  📄 Copying $launch..."
        scp "$SCRIPT_DIR/src/EPIC/src/global_planner/exploration_manager/launch/$launch" \
            $USERNAME@$TARGET_IP:$TARGET_PATH/src/EPIC/src/global_planner/exploration_manager/launch/$launch
    else
        echo "  ⚠️  Warning: $launch not found"
    fi
done

# Copy LKH TSP solver resources
echo "🧩 Copying LKH TSP solver resources..."
if [ -d "$SCRIPT_DIR/src/EPIC/src/global_planner/utils/lkh_tsp_solver/resource" ]; then
    scp -r "$SCRIPT_DIR/src/EPIC/src/global_planner/utils/lkh_tsp_solver/resource" \
        $USERNAME@$TARGET_IP:$TARGET_PATH/src/EPIC/src/global_planner/utils/lkh_tsp_solver/
    echo "  ✅ LKH resources copied"
else
    echo "  ⚠️  Warning: LKH resources not found"
fi

# Copy run script and setup script
echo "📜 Copying scripts..."

# Copy run_target_board.sh
if [ -f "$SCRIPT_DIR/run_target_board.sh" ]; then
    scp "$SCRIPT_DIR/run_target_board.sh" $USERNAME@$TARGET_IP:$TARGET_PATH/
    ssh $USERNAME@$TARGET_IP "chmod +x $TARGET_PATH/run_target_board.sh"
    echo "  ✅ run_target_board.sh copied and made executable"
else
    echo "  ⚠️  Warning: run_target_board.sh not found in $SCRIPT_DIR"
fi


# Copy setup_epic_arm.sh
if [ -f "$SCRIPT_DIR/setup_epic_arm.sh" ]; then
    scp "$SCRIPT_DIR/setup_epic_arm.sh" $USERNAME@$TARGET_IP:$TARGET_PATH/
    ssh $USERNAME@$TARGET_IP "chmod +x $TARGET_PATH/setup_epic_arm.sh"
    echo "  ✅ setup_epic_arm.sh copied and made executable"
else
    echo "  ⚠️  Warning: setup_epic_arm.sh not found in $SCRIPT_DIR"
fi

# Verify the files exist on target
echo "🔍 Verifying scripts on target..."
ssh $USERNAME@$TARGET_IP "ls -la $TARGET_PATH/run_target_board.sh $TARGET_PATH/setup_epic_arm.sh" || echo "  ⚠️  Warning: Scripts not found on target"

# Copy package.xml files for ROS package discovery
echo "📋 Copying ROS package metadata..."
PACKAGE_DIRS=(
    "epic_planner"
    "plan_manage" 
    "traj_utils"
    "frontier_manager"
    "path_searching"
    "pointcloud_topo"
    "poly_traj"
    "lkh_tsp_solver"
    "lidar_map"
)

for pkg in "${PACKAGE_DIRS[@]}"; do
    # Find the package directory
    PKG_PATH=$(find "$SCRIPT_DIR/src" -name "$pkg" -type d | head -1)
    if [ -n "$PKG_PATH" ]; then
        if [ -f "$PKG_PATH/package.xml" ]; then
            # Remove SCRIPT_DIR from the path for target
            REL_PATH=${PKG_PATH#$SCRIPT_DIR/}
            TARGET_PKG_PATH="$TARGET_PATH/$REL_PATH"
            ssh $USERNAME@$TARGET_IP "mkdir -p $TARGET_PKG_PATH"
            scp "$PKG_PATH/package.xml" $USERNAME@$TARGET_IP:$TARGET_PKG_PATH/
            echo "  📄 Copied $pkg/package.xml"
        fi
    fi
done

# Verify deployment
echo "🔍 Verifying deployment..."
ssh $USERNAME@$TARGET_IP "ls -la $TARGET_PATH/devel/lib/epic_planner/exploration_node" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Main executable verified on target board"
    
    # Check if it's ARM binary
    ARCH_CHECK=$(ssh $USERNAME@$TARGET_IP "file $TARGET_PATH/devel/lib/epic_planner/exploration_node | grep -o 'ARM aarch64'")
    if [ -n "$ARCH_CHECK" ]; then
        echo "✅ Confirmed ARM aarch64 binary"
    else
        echo "⚠️  Warning: Binary architecture could not be verified"
    fi
else
    echo "❌ Error: Main executable not found on target board"
    exit 1
fi

echo ""
echo "🎉 Deployment completed successfully!"
echo ""
echo "📋 Next steps on target board:"
echo "   1. cd $TARGET_PATH"
echo "   2. bash setup_epic_arm.sh"
echo "   3. ./run_target_board.sh <HOST_PC_IP> garage"
echo ""
echo "🆕 NEW: Trajectory Sampler Usage:"
echo "   4. ./run_target_board.sh garage true"
echo ""
echo "📊 Deployed files:"
echo "   - $(echo "${EXECUTABLES[@]}" | wc -w) executable files"
echo "   - $(echo "${LIBRARIES[@]}" | wc -w) shared libraries"
echo "   - Launch and configuration files"
echo "   - LKH TSP solver resources"
echo "   - Setup and run scripts" 

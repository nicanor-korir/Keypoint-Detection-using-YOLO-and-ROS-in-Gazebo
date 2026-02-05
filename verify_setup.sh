#!/bin/bash
# Verification Script for DeepMind Bot Setup

echo "=================================================="
echo "   DeepMind Bot - Setup Verification"
echo "=================================================="

# Initialize Conda
eval "$(conda shell.bash hook)"

# Check if environment exists
echo "[1/6] Checking conda environment..."
if conda env list | grep -q "ros2_humble_env"; then
    echo "  ✓ ros2_humble_env exists"
    conda activate ros2_humble_env
else
    echo "  ✗ ros2_humble_env not found"
    echo "  → Run: ./setup_and_launch.sh"
    exit 1
fi

# Check workspace build
echo "[2/6] Checking workspace build..."
if [ -d "ros2_ws/install" ]; then
    echo "  ✓ Workspace is built"
else
    echo "  ✗ Workspace not built"
    echo "  → Run: ./setup_and_launch.sh"
    exit 1
fi

# Source workspace
unset PYTHONPATH
if [ -f "ros2_ws/install/setup.bash" ]; then
    source ros2_ws/install/setup.bash
else
    echo "  ✗ Cannot source workspace"
    exit 1
fi

# Check if packages are available
echo "[3/6] Checking ROS 2 packages..."
PACKAGES=("deepmind_bot_gazebo" "deepmind_bot_description" "advanced_perception" "custom_msgs")
for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg prefix $pkg > /dev/null 2>&1; then
        echo "  ✓ $pkg"
    else
        echo "  ✗ $pkg not found"
        echo "  → Rebuild workspace: cd ros2_ws && colcon build"
        exit 1
    fi
done

# Check Gazebo installation
echo "[4/6] Checking Gazebo installation..."
if command -v gazebo > /dev/null 2>&1; then
    echo "  ✓ Gazebo found: $(which gazebo)"
else
    echo "  ✗ Gazebo not found"
    exit 1
fi

# Check critical Gazebo plugins
echo "[5/6] Checking Gazebo ROS plugins..."
PLUGINS=("libgazebo_ros_state.so" "libgazebo_ros_camera.so" "libgazebo_ros_diff_drive.so")
for plugin in "${PLUGINS[@]}"; do
    if [ -f "$CONDA_PREFIX/lib/$plugin" ]; then
        echo "  ✓ $plugin"
    else
        echo "  ✗ $plugin not found"
        exit 1
    fi
done

# Check world and model files
echo "[6/6] Checking world and model files..."
WORLD_FILE="$(ros2 pkg prefix deepmind_bot_gazebo)/share/deepmind_bot_gazebo/worlds/perception1.world"
if [ -f "$WORLD_FILE" ]; then
    echo "  ✓ perception1.world found"
else
    echo "  ✗ perception1.world not found"
    exit 1
fi

MODELS_DIR="$(ros2 pkg prefix deepmind_bot_gazebo)/share/deepmind_bot_gazebo/models"
if [ -d "$MODELS_DIR" ]; then
    MODEL_COUNT=$(ls -1 "$MODELS_DIR" | wc -l)
    echo "  ✓ Models directory found ($MODEL_COUNT models)"
else
    echo "  ✗ Models directory not found"
    exit 1
fi

echo "=================================================="
echo "  ✓ All checks passed!"
echo "=================================================="
echo "You can now run: ./launch_simulation.sh"
echo "=================================================="

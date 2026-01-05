#!/bin/bash

# SLAM Bench Orchestrator - Automated Installer
# Supported OS: Ubuntu 22.04 LTS

set -e

# --- Colors ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}==============================================${NC}"
echo -e "${GREEN}     SLAM Bench Orchestrator Installer        ${NC}"
echo -e "${GREEN}==============================================${NC}"

# 1. Check OS
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "22.04" ]; then
        echo -e "${YELLOW}[WARNING] This script is optimized for Ubuntu 22.04. You are on $VERSION_ID.${NC}"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then exit 1; fi
    fi
else
    echo -e "${RED}[ERROR] Could not detect OS version.${NC}"
    exit 1
fi

# 2. System Updates
echo -e "${YELLOW}[1/5] Updating system packages...${NC}"
sudo apt update && sudo apt upgrade -y

# 3. ROS 2 Dependencies
echo -e "${YELLOW}[2/5] Installing ROS 2 Humble dependencies...${NC}"
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-simulations \
    ros-humble-tf-transformations \
    python3-colcon-common-extensions \
    python3-rosdep

# 4. Python Dependencies
echo -e "${YELLOW}[3/5] Installing Python libraries...${NC}"
pip3 install --user \
    PyQt5 \
    pyyaml \
    "numpy<2" \
    matplotlib \
    reportlab \
    psutil \
    transforms3d \
    pyqtgraph \
    PyOpenGL \
    scikit-image

# 5. Build Local Slams (GMapping)
echo -e "${YELLOW}[4/5] Building local workspaces (deps/)...${NC}"
if [ -d "deps/gmapping_ws" ]; then
    cd deps/gmapping_ws
    # Check if we need to source ROS first
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    colcon build --symlink-install
    cd ../..
    echo -e "${GREEN}[SUCCESS] GMapping build complete.${NC}"
else
    echo -e "${RED}[ERROR] deps/gmapping_ws not found!${NC}"
fi

# 6. Finalizing
echo -e "${YELLOW}[5/5] Finalizing setup...${NC}"
# Add execution permission to runners
chmod +x runner/run_matrix.py
chmod +x runner/run_one.py

echo -e "${GREEN}==============================================${NC}"
echo -e "${GREEN}        INSTALLATION COMPLETE!                ${NC}"
echo -e "${GREEN}==============================================${NC}"
echo -e "To start the GUI, run:"
echo -e "  ${YELLOW}python3 gui/main.py${NC}"
echo -e ""
echo -e "To run a matrix in headless mode:"
echo -e "  ${YELLOW}python3 runner/run_matrix.py configs/matrices/test_headless_ci.yaml${NC}"
echo -e "=============================================="

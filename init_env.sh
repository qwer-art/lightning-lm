#!/bin/bash
# 编译并初始化环境
# 用法: source init_env.sh

# 退出 conda 环境，避免 Python 路径冲突
if [[ -n "$CONDA_PREFIX" ]] && type conda &>/dev/null; then
    conda deactivate
fi

# source ROS2 环境
source /opt/ros/humble/setup.bash

# 切到项目根目录（source 模式下用 BASH_SOURCE）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MAKEFLAGS=-j4 colcon build --packages-select lightning
source install/setup.bash

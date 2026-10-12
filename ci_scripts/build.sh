#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../") # project: lejulab_platform

source /opt/ros/noetic/setup.bash

LOCAL_DEVELOPMENT=0  # 1: local development, 0: CI
if [ "$LOCAL_DEVELOPMENT" -eq 0 ]; then
    export CC="ccache gcc"
    export CXX="ccache g++"
fi


cd $PROJECT_DIR

# Clean previous builds to ensure fresh compilation
if [ -d "$PROJECT_DIR/build" ]; then
        echo "Previous catkin build detected, running catkin clean..."
        cd "$PROJECT_DIR"
        catkin clean -y
        if [ $? -ne 0 ]; then
            echo "Error: Initial catkin clean failed"
            exit 1
        fi
fi

# 正常编译所有包防止依赖丢失
catkin config --no-install -DCMAKE_BUILD_TYPE=Release -DCMAKE_ASM_COMPILER=/usr/bin/as
if [ $? -ne 0 ]; then
    echo "Error: catkin config failed"
    exit 1
fi

catkin build  -j$(nproc)
if [ $? -ne 0 ]; then
    echo "Error: catkin build failed"
    exit 1
fi

# source devel/setup.bash

# 安装模式编译需要安装的包
catkin clean -y
if [ $? -ne 0 ]; then
    echo "Error: catkin clean failed"
    exit 1
fi

catkin config --install --install-space installed
if [ $? -ne 0 ]; then
    echo "Error: catkin config --install failed"
    exit 1
fi

catkin build
if [ $? -ne 0 ]; then
    echo "Error: catkin build hardware_node and related packages failed"
    exit 1
fi

# 恢复安装模式
catkin config --no-install
if [ $? -ne 0 ]; then
    echo "Error: catkin config --no-install failed"
    exit 1
fi

# 清理build、devel，保留install
catkin clean -y
EXIT_CODE=$?
if [ $EXIT_CODE -ne 0 ]; then
    echo "Error: catkin clean -b -d -y failed"
    exit $EXIT_CODE
fi

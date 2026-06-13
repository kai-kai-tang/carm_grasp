#!/bin/bash


source /opt/ros/foxy/setup.bash     # Ubuntu 20.04 使用 foxy
source /opt/ros/humble/setup.bash   # Ubuntu 22.04 使用 humble
export ROS_DOMAIN_ID=1              # 设置 ROS_DOMAIN_ID，确保与其他设备不冲突

script_dir=$(dirname "$(realpath "$0")")
echo "当前脚本所在的目录: $script_dir"
echo

############################################## 参数配置 ##############################################

frame_id="base_link"
# frame_id="odom_frame"

# 相机点云所在的坐标系名称
pc_frame_id="d405_depth_optical_frame"

############################################## 可执行程序 ##############################################

python3 ${script_dir}/../src/arm_node.py \
    --frame_id "${frame_id}" \
    --pc_frame_id "${pc_frame_id}"

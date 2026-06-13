#!/bin/bash

source /opt/ros/foxy/setup.bash     # Ubuntu 20.04 使用 foxy
source /opt/ros/humble/setup.bash   # Ubuntu 22.04 使用 humble
export ROS_DOMAIN_ID=1

script_dir=$(dirname "$(realpath "$0")")
echo "当前脚本所在的目录: $script_dir"
echo

############################################## 参数配置 ##############################################


############################################## 可执行程序 ##############################################

rviz2 -d ${script_dir}/../rviz/check.rviz
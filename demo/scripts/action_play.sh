#!/bin/bash


source /opt/ros/foxy/setup.bash     # Ubuntu 20.04 使用 foxy
source /opt/ros/humble/setup.bash   # Ubuntu 22.04 使用 humble
export ROS_DOMAIN_ID=1              # 设置 ROS_DOMAIN_ID,确保与其他设备不冲突

script_dir=$(dirname "$(realpath "$0")")
echo "当前脚本所在的目录: $script_dir"

root_dir="$(realpath "${script_dir}/../../")"
echo "项目根目录: $root_dir"
echo


############################################## 参数配置 ##############################################

# 机械臂动作模板文件夹路径( 根据需要修改为 calib_handeye 或 calib_camera 或其他 )
tmpl_dir="${root_dir}/demo/data/action/calib_handeye"


############################################## 可执行程序 ##############################################

python3 ${root_dir}/examples/common/src/action_play.py \
    --tmpl_dir "${tmpl_dir}" 

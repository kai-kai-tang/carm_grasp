#!/bin/bash


source /opt/ros/foxy/setup.bash     # Ubuntu 20.04 使用 foxy
source /opt/ros/humble/setup.bash   # Ubuntu 22.04 使用 humble
export ROS_DOMAIN_ID=1              # 设置 ROS_DOMAIN_ID，确保与其他设备不冲突

script_dir=$(dirname "$(realpath "$0")")
echo "当前脚本所在的目录: $script_dir"
echo

############################################## 参数配置 ##############################################

# 彩色图像话题名称
color_img_topic="/realsense/d405/color/image_rect_raw"

# 深度图像话题名称
depth_img_topic="/realsense/d405/aligned_depth_to_color/image_raw"

# 相机点云所在的坐标系名称
pc_frame_id="d405_depth_optical_frame"

# 夹爪的宽度和厚度(单位: m), 格式: [width,thickness]
gripper_size='[0.015,0.005]'  


############################################## 可执行程序 ##############################################

python3 ${script_dir}/../src/calib_gripper.py \
    --color_img_topic ${color_img_topic} \
    --depth_img_topic ${depth_img_topic} \
    --pc_frame_id ${pc_frame_id} \
    --gripper_size ${gripper_size}

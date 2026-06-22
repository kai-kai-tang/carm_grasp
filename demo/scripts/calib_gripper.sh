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

# RGB-D 相机内参文件路径
cam_params_path="${root_dir}/demo/data/calib/cam_params.json"

# 机械臂手眼标定结果文件路径
handeye_calib_path="${root_dir}/demo/data/calib/calib_handeye.json"

# 夹爪参数文件路径( 输入输出 )
gripper_path="${root_dir}/demo/data/calib/gripper_body.json"

# 彩色图像话题名称
color_img_topic="/realsense/d405/color/image_rect_raw"

# 深度图像话题名称
depth_img_topic="/realsense/d405/aligned_depth_to_color/image_raw"

# 相机点云所在的坐标系名称
pc_frame_id="d405_depth_optical_frame"

# 夹爪的宽度和厚度(单位: m), 格式: [width,thickness]
gripper_size='[0.015,0.005]'  


############################################## 可执行程序 ##############################################

python3 ${root_dir}/examples/common/src/calib_gripper.py \
    --cam_params_path "${cam_params_path}" \
    --handeye_calib_path "${handeye_calib_path}" \
    --gripper_path "${gripper_path}" \
    --color_img_topic "${color_img_topic}" \
    --depth_img_topic "${depth_img_topic}" \
    --pc_frame_id "${pc_frame_id}" \
    --gripper_size "${gripper_size}"

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

# 彩色图像话题名称
color_img_topic="/realsense/d405/color/image_rect_raw"

# RGB-D 相机参数文件路径
cam_params_path="${root_dir}/demo/data/calib/cam_params.json" 

# 手眼标定文件路径
handeye_calib_path="${root_dir}/demo/data/calib/calib_handeye.json"

# 模板文件的目录
tmpl_dir="${root_dir}/demo/data/benchmark/grasp_2d"  

# 检测状态下的位姿 [tx, ty, tz, qx, qy, qz, qw]
detect_pose="[0.27591300, -0.00070758, 0.40509600, 0.99748184, 0.07091509, -0.00101194, 0.00001724]"  

# 放置状态下的位姿 [tx, ty, tz, qx, qy, qz, qw]
place_pose="[0.06029420, 0.28073100, 0.25423200, 0.72462909, 0.68913908, -0.00001507, 0.00008348]"   


############################################## 可执行程序 ##############################################

python3 ${root_dir}/examples/benchmark/src/test_tmpl_grasp_2d.py \
    --cam_params_path ${cam_params_path} \
    --handeye_calib_path ${handeye_calib_path} \
    --color_img_topic ${color_img_topic} \
    --tmpl_dir ${tmpl_dir} \
    --detect_pose "${detect_pose}" \
    --place_pose "${place_pose}" \
    # --debug

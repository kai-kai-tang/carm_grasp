#!/bin/bash


source /opt/ros/foxy/setup.bash     # Ubuntu 20.04 使用 foxy
source /opt/ros/humble/setup.bash   # Ubuntu 22.04 使用 humble
export ROS_DOMAIN_ID=1              # 设置 ROS_DOMAIN_ID，确保与其他设备不冲突

script_dir=$(dirname "$(realpath "$0")")
echo "当前脚本所在的目录: $script_dir"
echo

root_dir="$(realpath "${script_dir}/../../../")"
echo "项目根目录: $root_dir"
echo

############################################## 参数配置 ##############################################

# 彩色图像话题名称
color_img_topic="/realsense/d405/color/image_rect_raw"

# 深度图像话题名称
depth_img_topic="/realsense/d405/aligned_depth_to_color/image_raw"

# 模板文件的目录
tmpl_dir="${root_dir}/data/benchmark/tmpl/grasp_3d"  


############################################## 可执行程序 ##############################################

python3 ${script_dir}/../src/test_tmpl_grasp_3d.py \
    --color_img_topic ${color_img_topic} \
    --depth_img_topic ${depth_img_topic} \
    --tmpl_dir ${tmpl_dir} \
    --debug

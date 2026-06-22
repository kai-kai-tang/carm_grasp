#!/bin/bash

script_dir=$(dirname "$(realpath "$0")")
echo "当前脚本所在的目录: $script_dir"

root_dir="$(realpath "${script_dir}/../../")"
echo "项目根目录: $root_dir"
echo


############################################## 参数配置 ##############################################

# 标定板信息: [tag_size( m ), space_size( m ), tag_rows, tag_cols]
calib_board_info='[0.0352, 0.01056, 6, 6]'  

# 图像目录
img_dir="${root_dir}/demo/data/collect/calib_camera/cam0"  


############################################## 可执行程序 ##############################################

python3 ${root_dir}/examples/common/src/calib_camera.py \
    --calib_board_info "${calib_board_info}" \
    --img_dir "${img_dir}"

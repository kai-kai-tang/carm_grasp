"""
功能: 创建基于 AprilTag2 的 2D 抓取模板数据, 包含物体在图像上的位置和朝向信息, 以及机械臂末端位姿和夹爪距离等状态信息
"""

import argparse
import os
import sys
import logging
import time
import json

from typing_extensions import List, Tuple, Dict

import cv2
import numpy as np
import transforms3d

import rclpy

# 导入第三方模块
import apriltag2

# 导入本工程的模块
code_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.normpath(f'{code_dir}/../../../')
sys.path.append(root_dir)

from core.utils import (
    GREEN, YELLOW, BLUE, RED, RESET,
    KeyboardReader, read_rgbd_params
)
from core.arm_wrapper import ArmWrapper
from core.arm_utils import compute_axis_aligned_pose
from core.cam_ros_utils import CamNode
from core.vision_utils import Matcher2D


######################################################### 全局变量 #########################################################


######################################################### 函数定义 #########################################################


def save_state(cam_node: CamNode,
               arm: ArmWrapper,
               save_dir: str,
               matcher: Matcher2D = None):
    """
    保存当前状态为模板文件
    Args:
        cam_node (CamNode): 相机 ROS2 节点
        arm (ArmWrapper): 机械臂对象
        save_dir (str): 保存路径
        matcher (Matcher2D): 可选的 2D 匹配器对象, 若提供则会检测物体位置并保存到模板中
    """

    # 获取图像
    imgs = cam_node.get_frames(do_spin_once=True)
    if imgs is None:
        logging.error(f'{RED}failed to get images, skip saving tmpl{RESET}')
        return
    # end if
    rgb_img = imgs[0][0]  # 取第一帧第一摄像头图像

    # 获取机械臂末端位姿
    T_base_end = arm.get_pose()

    # 获取夹爪距离
    gripper_dist = arm.get_gripper_dist()

    data_dict = {
        'T_base_end': T_base_end.tolist(),
        'gripper_dist': gripper_dist
    }

    if matcher is not None:  # 获取物体位置

        result_list, msg = matcher.match(rgb_img, top_k=1)
        if len(result_list) == 0:
            logging.warning(f'{YELLOW}no match found, skip saving tmpl, {msg}{RESET}')
            return
        # end if

        pose_2d = result_list[0].pose_2d
        data_dict['obj_pose_2d'] = [float(x) for x in pose_2d]

        bgr_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
        matcher.draw(bgr_img, result_list)  # 绘制匹配结果
        img_path = os.path.join(save_dir, 'tag.png')
        cv2.imwrite(img_path, bgr_img)
    # end if

    # 保存模板数据
    img_path = os.path.join(save_dir, f'color.png')
    cv2.imwrite(img_path, cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR))
    logging.info(f'saved image to: {GREEN}{img_path}{RESET}')

    file_path = os.path.join(save_dir, f'state.json')
    with open(file_path, 'w') as f:
        json.dump(data_dict, f, indent=4)
    logging.info(f'saved state to: {GREEN}{file_path}{RESET}')
# end def save_state


######################################################### 主函数 #########################################################

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--color_img_topic", type=str, required=True,
                        help="RGB 图像的 ROS2 话题名称")

    parser.add_argument("--tmpl_dir", type=str,
                        help="模板文件的目录")

    args = parser.parse_args()

    color_img_topic = args.color_img_topic
    if color_img_topic is None:
        logging.error("Error: color_img_topic is not provided.")
        exit(0)
    # end if

    tmpl_dir = args.tmpl_dir
    if tmpl_dir is None:
        logging.error('no tmpl_dir specified, exiting')
        exit(1)
    # end if

    print()
    print(f"color image topic: {BLUE}{color_img_topic}{RESET}")
    print(f'grasp_2d template will be saved to: {GREEN}{tmpl_dir}{RESET}')
    print()

    # 读取相机参数
    rgbd_params_path = os.path.join(root_dir, 'data/calib/cam_params.json')
    intrinsic, distortion, _ = read_rgbd_params(rgbd_params_path)

    config = Matcher2D.Config(
        intrinsic=intrinsic,
        distortion=distortion,
    )
    matcher = Matcher2D(config)

    tmpl_dir = os.path.normpath(tmpl_dir)  # 规范化路径
    os.makedirs(tmpl_dir, exist_ok=True)

    # 初始化 AprilTag2 检测器
    detector = apriltag2.Detector()
    logging.info(f'initialized apriltag2 detector')

    # 创建机械臂对象
    arm = ArmWrapper()
    if not arm.is_connected():
        logging.error(f'{RED}failed to connect to arm, exiting {RESET}')
        exit(1)
    # end if

    # 初始化 ROS2 节点
    rclpy.init(args=None)
    cam_node = CamNode([color_img_topic])

    # 创建键盘读取对象
    keyboard_reader = KeyboardReader()

    print()
    logging.info(f'use keyboard to control: \n{BLUE}'
                 f'  a: adjust arm z axis\n'
                 f'  g: save grasp data\n'
                 f'  n: save near( with object ) data\n'
                 f'  b: save next near( with object ) data\n'
                 f'  f: save far( with object ) data\n'
                 f'  d: save next far( with object ) data\n{RESET}')

    while rclpy.ok():

        key = keyboard_reader.read_key()
        if key is None:
            time.sleep(0.03)
            continue
        # end if
        # logging.info(f'pressed: {key}')
        print()

        if key == 'a':   # 调整末端的 z 轴方向, 使它与基座的 z 轴平行
            logging.info(f'{BLUE}start to adjust arm z axis ...{RESET}')

            T_base_end = arm.get_pose()
            target_T_base_end = compute_axis_aligned_pose(T_base_end, base_axis_idx=-3, obj_axis_idx=3)
            if target_T_base_end is None:
                continue
            # end if

            logging.info(f'try move to new T_base_end:\n{target_T_base_end}')
            is_ok = arm.set_pose(target_T_base_end)
            if not is_ok:
                logging.warning('failed to move to new T_base_end')
            # end if
            print()
        # end if

        if key == 'g':  # 保存抓取时的状态
            logging.info(f'{BLUE}start to save grasp data ...{RESET}')

            save_dir = os.path.join(tmpl_dir, 'grasp')
            os.makedirs(save_dir, exist_ok=True)

            save_state(cam_node,
                       arm,
                       save_dir)
        # end if

        if key == 'n':  # 保存相机距离物体较近时的状态
            logging.info(f'{BLUE}start to save near( with object ) data ...{RESET}')

            save_dir = os.path.join(tmpl_dir, 'near')
            os.makedirs(save_dir, exist_ok=True)

            save_state(cam_node,
                       arm,
                       save_dir,
                       matcher=matcher)
        # end if

        if key == 'b':  # 保存相机距离物体较近时的状态, 即在当前位置的基础上在XY平面上移动一小段距离
            logging.info(f'{BLUE}start to save next near( with object ) data ...{RESET}')

            save_dir = os.path.join(tmpl_dir, 'next_near')
            os.makedirs(save_dir, exist_ok=True)

            save_state(cam_node,
                       arm,
                       save_dir,
                       matcher=matcher)
        # end if

        if key == 'f':  # 保存相机距离物体较远时的状态
            logging.info(f'{BLUE}start to save far( with object ) data ...{RESET}')

            save_dir = os.path.join(tmpl_dir, 'far')
            os.makedirs(save_dir, exist_ok=True)

            save_state(cam_node,
                       arm,
                       save_dir,
                       matcher=matcher)
        # end if

        if key == 'd':  # 保存相机距离物体较远时的状态, 即在当前位置的基础上在XY平面上移动一小段距离
            logging.info(f'{BLUE}start to save next far( with object ) data ...{RESET}')

            save_dir = os.path.join(tmpl_dir, 'next_far')
            os.makedirs(save_dir, exist_ok=True)

            save_state(cam_node,
                       arm,
                       save_dir,
                       matcher=matcher)
        # end if
    # end while
# end if __name__ == '__main__'

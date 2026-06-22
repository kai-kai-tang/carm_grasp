"""
文件说明: 
    自动收集数据的脚本, 读取给定的机械臂动作模板,依次执行每个模板,并在每个模板的不同阶段保存相应的数据, 包括机械臂状态、相机数据等. 
    该脚本可以帮助用户快速收集大量的机械臂操作数据, 用于后续的分析和训练.
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

# 导入本工程的模块

code_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.normpath(f'{code_dir}/../../../')
sys.path.append(root_dir)

from core.utils import (
    GREEN, YELLOW, BLUE, RED, RESET,
    wait_key
)
from core.arm_wrapper import ArmWrapper
from core.cam_ros_utils import CamNode
from examples.common.src.action_play import read_action_list  # 同目录下的模块


######################################################### 函数定义 #########################################################


######################################################### 主函数 #########################################################

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description="自动数据采集")

    parser.add_argument(
        "--tmpl_dir",
        type=str,
        help="保存模板文件的目录")

    parser.add_argument(
        "--img_topic_list",
        nargs='+',
        help="要保存的图像话题名称列表")

    parser.add_argument(
        "--data_dir",
        type=str,
        required=True,
        help="结果保存目录, 会在该目录下创建 img 子目录保存图像")

    parser.add_argument(
        "--debug",
        action='store_true',
        help="是否开启调试模式")

    args = parser.parse_args()
    tmpl_dir = args.tmpl_dir
    img_topic_list = args.img_topic_list
    data_dir = args.data_dir
    debug = args.debug

    print()
    print(f'tmpl_dir: {BLUE}{tmpl_dir}{RESET}')
    print(f"img_topic_list: {BLUE}{img_topic_list}{RESET}")
    print(f"data_dir: {BLUE}{data_dir}{RESET}")
    print(f"debug: {BLUE}{debug}{RESET}")
    print()

    # 读取模板文件
    action_tmpl_list = read_action_list(tmpl_dir)
    if len(action_tmpl_list) == 0:
        logging.warning(f'{YELLOW}no valid tmpl found in tmpl_dir: {tmpl_dir}{RESET}')
        exit(1)
    # end if

    # 创建机械臂对象
    arm = ArmWrapper()
    if not arm.is_connected():
        logging.error(f'{RED}failed to connect to arm, exiting {RESET}')
        exit(1)
    # end if
    # arm.set_speed_level(10)
    arm.set_gripper_dist(0.08)  # 先打开夹爪, 避免碰撞
    time.sleep(1)  # 等待机械臂动作完成

    # 切换到位置力控制模式, 以便在执行模板时能够适当顺应环境, 避免碰撞
    is_ok = arm.set_control_mode(ArmWrapper.ControlMode.PF)
    if not is_ok:
        logging.error(f'{RED}failed to switch to [position force] mode{RESET}')
        exit(1)
    # end if

    # 初始化 ROS2 节点
    rclpy.init(args=None)
    cam_node = CamNode(img_topic_list=img_topic_list)

    # 准备保存结果的目录
    cam_num = len(img_topic_list)
    for i in range(cam_num):
        img_dir = os.path.join(data_dir, f'cam{i}')
        os.makedirs(img_dir, exist_ok=True)
    # end for

    # 执行每个模板, 并在每个模板的不同阶段保存相应的数据
    is_all_ok = True
    pose_dict = {}
    for idx, action_tmpl in enumerate(action_tmpl_list):
        if not wait_key(debug):
            exit(0)
        # end if

        print()
        logging.info(f'Executing template [{idx}] ...')

        # 执行模板的动作, 包括机械臂末端位姿, 关节角
        T_base_end = action_tmpl.get('T_base_end', None)
        joints = action_tmpl.get('joints', None)

        if joints is not None:
            is_ok = arm.set_joints(joints)
            if not is_ok:
                logging.error(f'{RED}failed to move joints for template {idx}{RESET}')
                is_all_ok = False
                break
            # end if
        elif T_base_end is not None:
            is_ok = arm.set_pose(T_base_end)
            if not is_ok:
                logging.error(f'{RED}failed to move to pose for template {idx}{RESET}')
                is_all_ok = False
                break
            # end if
        else:
            logging.error(f'{RED}no valid action found in template {idx}, quit{RESET}')
            is_all_ok = False
            break
        # end if

        time.sleep(1)  # 等待机械臂动作完成

        # 获取机械臂末端位姿
        T_base_end = arm.get_pose()
        pose = ArmWrapper.matrix_to_array(T_base_end)

        pose_dict[f'{idx}'] = pose

        # 获取图像
        frames = cam_node.get_frames(do_spin_once=True)
        if frames is None:
            logging.error(f'{RED}failed to get frames{RESET}')
            is_all_ok = False
            break
        # end if

        for cam_idx in range(cam_num):
            raw_img = frames[0][cam_idx]  # 取第一帧的图像
            if len(raw_img.shape) == 3:  # 如果是彩色图像, 则 RGB --> BGR
                img = cv2.cvtColor(raw_img, cv2.COLOR_RGB2BGR)
            else:
                img = raw_img
            # end if
            img_save_path = os.path.join(data_dir, f'cam{cam_idx}', f'{idx}.png')
            cv2.imwrite(img_save_path, img)
        # end for
    # end for

    if not is_all_ok:
        logging.error(f'{RED}data collection failed, exiting...{RESET}')
        exit(1)
    # end if

    # 保存机械臂末端位姿数据
    pose_dict["PoseNote"] = "Meaning: transformation from the end of the arm to the base of the arm; Format: tx,ty,tz,qx,qy,qz,qw"
    pose_dict["eye_in_hand"] = True  # 说明位姿数据是眼在手的
    pose_save_path = os.path.join(data_dir, 'arm_pose.json')
    with open(pose_save_path, 'w') as f:
        json.dump(pose_dict, f, indent=4)
    # end if

    logging.info(f'{GREEN}data collection completed successfully!{RESET}')

    # 回到初始位置
    arm.set_joints(arm.init_joints)

# end if __name__ == '__main__':

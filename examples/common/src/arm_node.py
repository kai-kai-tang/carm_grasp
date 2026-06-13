# -*- coding: utf-8 -*-
"""
功能说明: 启动机械臂 ROS2 节点, 发布机械臂状态信息, 并且可以通过键盘调整机械臂姿态
"""

import logging
import argparse
import os
import sys
import mmengine
import time

import numpy as np

import rclpy
from tf2_ros import TransformBroadcaster


# 导入本工程的模块
code_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.normpath(f'{code_dir}/../../../')
sys.path.append(root_dir)

from core.utils import (
    GREEN, YELLOW, BLUE, RED, RESET,
    KeyboardReader, read_handeye_calib,
    reset_empty_str
)
from core.arm_wrapper import ArmWrapper
from core.arm_utils import (
    GripperBody,
    compute_axis_aligned_pose
)
from core.arm_ros_utils import ArmNode, pose_to_transform_stamped


######################################################### 主函数 #########################################################

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--frame_id", type=str, required=True,
                        default="base_link",
                        help="机械臂发布位姿的坐标系的名称")

    parser.add_argument("--pc_frame_id", type=str, required=True,
                        help="点云所在的坐标系名称")

    args = parser.parse_args()

    frame_id = args.frame_id
    pc_frame_id = args.pc_frame_id

    print(f'frame_id: {GREEN}{frame_id}{RESET}')
    print(f'pc_frame_id: {GREEN}{pc_frame_id}{RESET}')
    print()

    calib_dir = os.path.normpath(os.path.join(root_dir, 'data/calib'))  # 标定文件夹路径

    # 读取手眼标定矩阵
    handeye_calib_path = os.path.join(calib_dir, 'calib_handeye.json')
    T_end_cam, _ = read_handeye_calib(handeye_calib_path)
    print()

    # 读取夹爪模型
    gripper_path = os.path.join(root_dir, 'data/calib/gripper_body.json')
    if not os.path.exists(gripper_path):
        logging.error(f'no gripper body file found at: {gripper_path}, exiting')
        exit(1)
    # end if
    gripper_data_dict = mmengine.load(gripper_path)
    gripper_width = gripper_data_dict['width']
    gripper_thickness = gripper_data_dict['thickness']
    T_cam_gripper = np.array(gripper_data_dict['T_cam_gripper'], dtype=np.float32)
    gripper_body = GripperBody(width=gripper_width,
                               thickness=gripper_thickness,
                               T_cam_gripper=T_cam_gripper)
    logging.info(f"gripper width: {GREEN}{gripper_body.width}{RESET}, thickness: {GREEN}{gripper_body.thickness}{RESET}")
    logging.info(f"T_cam_gripper: \n{GREEN}{gripper_body.T_cam_gripper}{RESET}")
    print()

    # 创建机械臂对象
    arm = ArmWrapper()
    if not arm.is_connected():
        logging.error('\033[91mfailed to connect to arm, exiting \033[0m')   # 红色打印
        exit(1)
    # end if

    # 初始化 ROS2 节点
    rclpy.init(args=None)
    arm_node = ArmNode(pub_arm_joints=True, pub_gripper_msg=True, frame_id=frame_id)
    tf_broadcaster = TransformBroadcaster(arm_node)

    # 创建键盘读取对象
    keyboard_reader = KeyboardReader()

    print()

    while rclpy.ok():

        time.sleep(0.02)

        T_base_end = arm.get_pose()
        gripper_dist = arm.get_gripper_dist()
        joints = arm.get_joints()

        arm_node.publish_pose(T_base_end)  # 发布机械臂末端位姿
        arm_node.publish_joints(joints)  # 发布机械臂关节角度
        arm_node.publish_grippers(gripper_body,
                                  gripper_dist,
                                  T_base_end,
                                  T_end_cam)  # 发布机械爪

        T_base_cam = T_base_end @ T_end_cam
        ts = pose_to_transform_stamped(arm_node.frame_id, pc_frame_id, T_base_cam)
        ts.header.stamp = arm_node.get_clock().now().to_msg()
        tf_broadcaster.sendTransform(ts)

        rclpy.spin_once(arm_node, timeout_sec=0.05)

        key = keyboard_reader.read_key()
        if key is None:
            continue
        # end if
        # logging.info(f'pressed: {key}')

        if key == 'q':  # 退出
            logging.info('exiting...')
            break
        # end if

        # 打印当前机械臂状态
        if key == 'v':
            joins = arm.get_joints()
            T_base_end = arm.get_pose()
            gripper_dist = arm.get_gripper_dist()
            pose = arm._matrix_to_array(T_base_end)
            logging.info(f'current joints: {GREEN}{joins}{RESET}')
            logging.info(f'current pose: {GREEN}{pose}{RESET}')
            logging.info(f'current gripper dist: {GREEN}{gripper_dist:.4f}{RESET}')
        # end if

        # 调整末端,使末端坐标系的 Z 轴指向基座坐标系的 -Z 轴
        if key == 'a':
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

        # 调整末端,使末端坐标系的 X 轴指向基座坐标系的 Z 轴
        if key == 'b':
            target_T_base_end = compute_axis_aligned_pose(T_base_end, base_axis_idx=3, obj_axis_idx=1)
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

        # 调整末端,使相机坐标系的 Z 轴指向下方
        if key == 'c':
            target_T_base_end = compute_axis_aligned_pose(T_base_end, base_axis_idx=-3, obj_axis_idx=3, T_end_obj=T_end_cam)
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

        # 夹爪控制
        gripper_step = 0.001  # 夹爪每次移动的步长
        if key == ',':  # 缩小夹爪
            set_dist = gripper_dist - gripper_step
            arm.set_gripper_dist(set_dist)
            logging.info(f'set gripper {gripper_dist:.3f} -->> {set_dist:.3f}, actual: {arm.get_gripper_dist():.3f}')
        elif key == '.':  # 放大夹爪
            set_dist = gripper_dist + gripper_step
            arm.set_gripper_dist(set_dist)
            logging.info(f'set gripper {gripper_dist:.3f} -->> {set_dist:.3f}, actual: {arm.get_gripper_dist():.3f}')
        # end if

    # end while

    arm_node.destroy_node()
    rclpy.shutdown()
    logging.info('shutdown')

# end if __name__ == '__main__'

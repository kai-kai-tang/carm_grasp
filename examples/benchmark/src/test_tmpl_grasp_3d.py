# -*- coding: utf-8 -*-
"""
功能说明: 基于自研机械臂 CARM 的 3D 抓取( 6 个自由度 )示例 ROS2 节点    
"""

import rclpy

import logging
import argparse
import os
import sys
import time
import json
import mmengine
from typing_extensions import List, Tuple, Dict

import numpy as np
import transforms3d


# 导入本工程的模块

code_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.normpath(f'{code_dir}/../../../')
sys.path.append(root_dir)

from core.utils import (
    GREEN, YELLOW, BLUE, RED, RESET,
    wait_key, reset_empty_str,
    read_rgbd_params, read_handeye_calib, inv_tf
)

from core.arm_wrapper import ArmWrapper

from core.arm_utils import (
    TH_ANGLE_Z,
    TH_GRIPPER_HEIGHT,
    GripperBody,
    CollisionDetector,
    check_arm_pose,
)

from core.arm_ros_utils import TargetArmNode

from core.cam_ros_utils import (
    CamNode,
)

from core.vision_utils import (
    compute_locate_error
)

from core.vision_utils import Matcher3D, depth_mean_filter


######################################################### 全局常量( 仅本文件使用 ) #########################################################

CHECK_HEIGHT = 0.18
"""检查位姿时物体到相机的垂直距离, 单位: 米"""


######################################################### 函数定义 #########################################################


def read_tmpl_grasp(tmpl_dir: str) -> Dict:
    """
    读取抓取模板数据
    Args:
        tmpl_dir (str): 抓取模板文件夹路径
    Returns:
        (Dict): 抓取模板数据字典
    """

    # 1. 读取相机处于检测位置时的机械臂状态
    detect_path = os.path.join(tmpl_dir, 'detect.json')
    if not os.path.exists(detect_path):
        logging.error(f'file not found: {detect_path}')
        return None
    # end if

    with open(detect_path, 'r') as f:
        detect_dict = json.load(f)
    # end with
    detect_T_base_end = np.array(detect_dict['T_base_end'], dtype=np.float32)
    detect_joints = detect_dict["joints"]
    detect_gripper_dist = detect_dict["gripper_dist"]
    logging.info(f'arm detect_joints: {detect_joints}')
    logging.info(f'arm detect_gripper_dist: {detect_gripper_dist}')

    # 2. 读取相机处于放置位置时的机械臂状态
    place_path = os.path.join(tmpl_dir, 'place.json')
    if not os.path.exists(place_path):
        logging.error(f'file not found: {place_path}')
        return None
    # end if

    with open(place_path, 'r') as f:
        place_dict = json.load(f)
    # end with
    place_T_base_end = np.array(place_dict['T_base_end'], dtype=np.float32)
    place_joints = place_dict["joints"]
    place_gripper_dist = place_dict["gripper_dist"]
    logging.info(f'arm place_joints: {place_joints}')
    logging.info(f'arm place_gripper_dist: {place_gripper_dist}')

    # 3. 读取抓取模板位姿
    grasp_path = os.path.join(tmpl_dir, 'grasp.json')
    if not os.path.exists(grasp_path):
        logging.warning(f'file not found: {grasp_path}')
        return None
    # end if

    with open(grasp_path, 'r') as f:
        grasp_data = json.load(f)
    # end with

    grasp_T_base_end = np.array(grasp_data['T_base_end'], dtype=np.float32)
    grasp_gripper_dist = grasp_data['gripper_dist']

    # 4. 读取预备位姿数据,以及根据预备位姿数据构建抓取模板列表
    ready_path = os.path.join(tmpl_dir, 'ready.json')
    if not os.path.exists(ready_path):
        logging.warning(f'file not found: {ready_path}')
        return None
    # end if

    with open(ready_path, 'r') as f:
        ready_data = json.load(f)
    # end with

    ready_T_base_end = np.array(ready_data['T_base_end'], dtype=np.float32)
    ready_T_cam_model = np.array(ready_data['T_cam_model'], dtype=np.float32)

    tmpl_dict = {
        'detect_T_base_end': detect_T_base_end,
        'detect_joints': detect_joints,
        'detect_gripper_dist': detect_gripper_dist,

        'place_T_base_end': place_T_base_end,
        'place_joints': place_joints,
        'place_gripper_dist': place_gripper_dist,

        'grasp_T_base_end': grasp_T_base_end,
        'grasp_gripper_dist': grasp_gripper_dist,

        'ready_T_base_end': ready_T_base_end,
        'ready_T_cam_model': ready_T_cam_model,
    }

    logging.info(f'read grasp tmpl successfully!')
    print()

    return tmpl_dict
# end def read_tmpl_grasp


def compute_ready_pose(T_end_cam: np.ndarray,
                       ready_T_cam_model: np.ndarray,
                       cur_T_base_end: np.ndarray,
                       cur_T_cam_model: np.ndarray) -> np.ndarray:
    """
    计算机械臂位姿,使得机械臂从当前位姿移动到设定位姿时, cur_T_cam_model = ready_T_cam_model    
    Args:
        T_end_cam (np.ndarray): 从相机到机械臂末端的变换矩阵, 4*4
        ready_T_cam_model (np.ndarray): 准备阶段从物体到相机的变换矩阵, 4*4
        cur_T_base_end (np.ndarray): 当前从机械臂末端到基座的变换矩阵, 4*4
        cur_T_cam_model (np.ndarray): 当前从物体到相机的变换矩阵, 4*4
    Returns:
        (np.ndarray): 计算得到的机械臂位姿 T_base_end
    """

    ready_T_model_cam = inv_tf(ready_T_cam_model)
    T_cam_end = inv_tf(T_end_cam)

    target_T_base_end = cur_T_base_end @ T_end_cam @ cur_T_cam_model @ ready_T_model_cam @ T_cam_end

    return target_T_base_end
# end def compute_ready_pose


def match(cam_node: CamNode,
          matcher: Matcher3D,
          debug_level: int) -> np.ndarray:
    """
    匹配物体并返回位姿
    Args:
        cam_node (CamNode): 相机节点
        matcher (Matcher3D): 3D 匹配器
        debug_level (int): 调试级别
    Returns:
        (np.ndarray): 匹配得到的从物体到相机的变换矩阵
    """

    frames = cam_node.get_frames(do_spin_once=True)
    if frames is None:
        logging.error(f"{RED}get frames failed.{RESET}")
        return None
    # end if

    color_img, depth_img = frames[0][0], frames[0][1]

    result_list, msg = matcher.match(bgr_img=color_img,
                                     depth_img=depth_img,
                                     top_k=1,
                                     debug_level=debug_level)

    if len(result_list) == 0:
        logging.error(f'{RED}match failed, msg: {msg}{RESET}')
        return None
    # end if

    T_cam_model = result_list[0].T_cam_tag

    return T_cam_model
# end def match


def track(cam_node: CamNode,
          matcher: Matcher3D,
          init_T_cam_model: np.ndarray,
          debug_level: int) -> np.ndarray:
    """
    跟踪物体并返回位姿
    Args:
        cam_node (CamNode): 相机节点
        matcher (Matcher3D): 3D 匹配器
        init_T_cam_model (np.ndarray): 初始的从物体到相机的变换矩阵, 4*4
        debug_level (int): 调试级别
    Returns:
        (np.ndarray): 跟踪得到的从物体到相机的变换矩阵
    """

    frames = cam_node.get_frames(do_spin_once=True)
    if frames is None:
        logging.error(f"{RED}get frames failed.{RESET}")
        return None
    # end if

    color_img, depth_img = frames[0][0], frames[0][1]

    T_cam_model, msg = matcher.track(bgr_img=color_img,
                                     depth_img=depth_img,
                                     init_T_cam_tag=init_T_cam_model,
                                     debug_level=debug_level)

    if T_cam_model is None:
        logging.error(f'{RED}track failed, msg: {msg}{RESET}')
        return None
    # end if

    return T_cam_model
# end def track


def do_grasp(T_end_cam: np.ndarray,
             gripper_body: GripperBody,
             tmpl_dict: Dict,
             arm: ArmWrapper,
             cam_node: CamNode,
             arm_node: TargetArmNode,
             matcher: Matcher3D,
             debug_level: int,
             debug: bool = False) -> bool:
    """
    执行一次 3D 抓取任务
    Args:
        T_end_cam (np.ndarray): 从相机到机械臂末端的变换矩阵, 4*4
        tmpl_dict (Dict): 抓取模板数据字典, 包含检测位姿、预备位姿和抓取位姿等信息
        arm (ArmWrapper): 机械臂
        cam_node (CamNode): 相机节点
        arm_node (TargetArmNode): 机械臂目标节点
        matcher (Matcher3D): 3D 匹配器
        debug_level (int): 调试级别
        debug (bool): 是否启用调试模式, 启用后会在每个步骤等待用户按键确认, 并显示更多日志信息
    Returns:
        (bool): 执行结果, False: 任务失败; True: 任务成功
    """

    T_cam_end = inv_tf(T_end_cam)

    grasp_T_base_end = tmpl_dict['grasp_T_base_end']
    grasp_gripper_dist = tmpl_dict['grasp_gripper_dist']

    ready_T_cam_model = tmpl_dict['ready_T_cam_model']
    ready_T_base_end = tmpl_dict['ready_T_base_end']

    # 计算从 ready 位姿到 grasp 位姿的增量
    delta_T_end = inv_tf(ready_T_base_end) @ grasp_T_base_end  # 末端坐标系下的位姿增量
    logging.info(f'delta_T_end: \n{GREEN}{delta_T_end}{RESET}')

    max_refine_cnt = 2  # 最大细化次数

    show_locate_err = False  # 是否显示定位误差

    # 临时变量
    prev_T_cam_model = None
    prev_T_base_end = None
    target_T_base_end = None

    ######## 1. 定位物体( 通过匹配 ) ########
    print()
    logging.info(f'grasp-step [1], {BLUE}locate model by matching{RESET}')
    if not wait_key(debug):
        return False
    # end if

    # 匹配物体
    cur_T_cam_model = match(cam_node=cam_node,
                            matcher=matcher,
                            debug_level=debug_level)
    if cur_T_cam_model is None:
        return False
    # end if

    logging.info(f'match T_cam_model: \n{GREEN}{cur_T_cam_model}{RESET}')

    ######## 2. 计算预备位姿 ########
    print()
    logging.info(f'grasp-step [2], {BLUE}compute ready pose{RESET}')

    cur_T_base_end = arm.get_pose()
    target_T_base_end = compute_ready_pose(T_end_cam=T_end_cam,
                                           ready_T_cam_model=ready_T_cam_model,
                                           cur_T_base_end=cur_T_base_end,
                                           cur_T_cam_model=cur_T_cam_model)
    logging.info(f'ready T_base_end: \n{GREEN}{target_T_base_end}{RESET}')

    target_gripper_dist = grasp_gripper_dist + 0.03  # 预备位姿时先稍微放开一点夹爪,方便检查和移动

    # 发布将要到达的位姿( 用于可视化 )
    arm_node.publish_pose(target_T_base_end)

    # 检查机械臂位姿合理性
    if not check_arm_pose(T_base_end=target_T_base_end,
                          T_end_cam=T_end_cam,
                          gripper_body=gripper_body,
                          gripper_dist=target_gripper_dist,
                          th_angle_z=TH_ANGLE_Z,
                          th_gripper_height=TH_GRIPPER_HEIGHT):
        logging.warning(f"{RED}arm pose check failed at ready pose.{RESET} try next label.")
        return False
    # end if

    ######## 3. 移动到预备位置 ########
    print()
    logging.info(f'grasp-step [3], {BLUE}move to ready pose{RESET}')
    if not wait_key(debug):
        return False
    # end if

    # 移动之前更新临时变量
    prev_T_cam_model = cur_T_cam_model
    prev_T_base_end = cur_T_base_end

    is_ok = arm.set_pose(target_T_base_end)
    if not is_ok:
        logging.error(f"{RED}move arm to ready pose failed. try next label.{RESET}")
        return False
    # end if

    # 计算定位偏差
    if show_locate_err:
        cur_T_base_end = arm.get_pose()
        cur_T_end_base = inv_tf(cur_T_base_end)
        init_T_cam_model = T_cam_end @ cur_T_end_base @ prev_T_base_end @ T_end_cam @ prev_T_cam_model

        cur_T_cam_model = track(cam_node=cam_node,
                                matcher=matcher,
                                init_T_cam_model=init_T_cam_model,
                                debug_level=debug_level)
        if cur_T_cam_model is None:
            logging.error(f"{RED}locate model failed at ready pose.{RESET}")
            return False
        # end if

        pos_err, rot_err = compute_locate_error(ready_T_cam_model, cur_T_cam_model)
        logging.info(f'locate error at ready pose, pos_err(mm): {pos_err:.2f}, rot_err(deg): {rot_err:.2f}')
    # end if

    ######## 迭代细化预备位姿 ########
    refine_cnt = 0
    while refine_cnt < max_refine_cnt:
        refine_cnt += 1

        ######## 4. 跟踪物体并计算预备位姿 ########
        print()
        logging.info(f'grasp-step [4-{refine_cnt}] , {BLUE}track model{RESET}')
        if not wait_key(debug):
            return False
        # end if

        # 跟踪物体
        cur_T_base_end = arm.get_pose()
        cur_T_end_base = inv_tf(cur_T_base_end)
        init_T_cam_model = T_cam_end @ cur_T_end_base @ prev_T_base_end @ T_end_cam @ prev_T_cam_model

        cur_T_cam_model = track(cam_node=cam_node,
                                matcher=matcher,
                                init_T_cam_model=init_T_cam_model,
                                debug_level=debug_level)
        if cur_T_cam_model is None:
            logging.error(f"{RED}locate model failed at ready pose.{RESET}")
            return False
        # end if
        logging.info(f'track T_cam_model: \n{GREEN}{cur_T_cam_model}{RESET}')

        # 计算定位偏差
        if show_locate_err:
            pos_err, rot_err = compute_locate_error(ready_T_cam_model, cur_T_cam_model)
            logging.info(f'locate error at ready pose, pos_err(mm): {pos_err:.2f}, rot_err(deg): {rot_err:.2f}')
        # end if

        # 计算预备位姿
        target_T_base_end = compute_ready_pose(T_end_cam=T_end_cam,
                                               ready_T_cam_model=ready_T_cam_model,
                                               cur_T_base_end=cur_T_base_end,
                                               cur_T_cam_model=cur_T_cam_model)
        logging.info(f'ready T_base_end: \n{GREEN}{target_T_base_end}{RESET}')

        # 发布将要到达的位姿( 用于可视化 )
        arm_node.publish_pose(target_T_base_end)

        # 检查机械臂位姿合理性
        if not check_arm_pose(T_base_end=target_T_base_end,
                              T_end_cam=T_end_cam,
                              gripper_body=gripper_body,
                              gripper_dist=arm.get_gripper_dist(),
                              th_angle_z=TH_ANGLE_Z,
                              th_gripper_height=TH_GRIPPER_HEIGHT):
            logging.error(f"{RED}arm pose check failed at ready pose. try next label{RESET}")
            return False
        # end if

        ######## 5. 再次移动到预备位置并计算抓取位姿 ########
        print()
        logging.info(f'grasp-step [5-{refine_cnt}] , {BLUE}move to ready pose again{RESET}')
        if not wait_key(debug):
            return False
        # end if

        # 移动之前更新临时变量
        prev_T_cam_model = cur_T_cam_model
        prev_T_base_end = cur_T_base_end

        # 移动到预备位置
        is_ok = arm.set_pose(target_T_base_end)
        if not is_ok:
            logging.error(f"{RED}move arm to ready pose failed. try next label.{RESET}")
            return False
        # end if
    # end while

    # 达到最大细化次数时,计算抓取位姿
    logging.info('reached max refine count.')

    cur_T_base_end = arm.get_pose()

    # 计算定位偏差
    if show_locate_err and debug:
        # 跟踪物体
        cur_T_end_base = inv_tf(cur_T_base_end)
        init_T_cam_model = T_cam_end @ cur_T_end_base @ prev_T_base_end @ T_end_cam @ prev_T_cam_model
        cur_T_cam_model = track(cam_node=cam_node,
                                matcher=matcher,
                                init_T_cam_model=init_T_cam_model,
                                debug_level=debug_level)
        if cur_T_cam_model is None:
            logging.error('track model failed.')
            return False
        # end if
        logging.info(f'track T_cam_model: \n{GREEN}{cur_T_cam_model}{RESET}')

        # 计算定位偏差
        pos_err, rot_err = compute_locate_error(ready_T_cam_model, cur_T_cam_model)
        logging.info(f'locate error at ready pose, pos_err(mm): {pos_err:.2f}, rot_err(deg): {rot_err:.2f}')
    # end if

    # 计算抓取位姿
    target_T_base_end = cur_T_base_end @ delta_T_end
    logging.info(f'grasp T_base_end: \n{GREEN}{target_T_base_end}{RESET}')

    target_gripper_dist = grasp_gripper_dist + 0.01  # 刚好比物体宽一点,不会碰到其他物体

    # 设置夹爪位置
    is_ok = arm.set_gripper_dist(target_gripper_dist)
    if not is_ok:
        logging.error(f"{RED}set gripper to grasp position failed.{RESET}")
        return False
    # end if
    time.sleep(0.3)  # 等待夹爪动作完成

    # 发布将要到达的位姿( 用于可视化 )
    arm_node.publish_pose(target_T_base_end)

    # 检查机械臂位姿合理性
    if not check_arm_pose(T_base_end=target_T_base_end,
                          T_end_cam=T_end_cam,
                          gripper_body=gripper_body,
                          gripper_dist=target_gripper_dist,
                          th_angle_z=TH_ANGLE_Z,
                          th_gripper_height=TH_GRIPPER_HEIGHT):
        logging.warning(f"{RED}arm pose check failed at ready pose.{RESET} try next label.")
        return False
    # end if

    ######## 11. 抓取 ########
    print()
    logging.info(f'grasp-step [11] , {BLUE}move to grasp pose{RESET}')
    if not wait_key(debug):
        return False
    # end if

    # 移动到抓取位姿
    is_ok = arm.set_pose(target_T_base_end, move_line=True)
    if not is_ok:
        logging.error(f"{RED}move arm to grasp pose failed. try next label.{RESET}")
        return False
    # end if

    # 闭合夹爪
    is_ok = arm.set_gripper_dist(grasp_gripper_dist - 0.005)
    if not is_ok:
        logging.error(f"{RED}close gripper failed.{RESET}")
        return False
    # end if

    time.sleep(0.3)  # 等待夹爪闭合完成

    # 提高 Z 轴高度, 避免碰撞
    target_T_base_end[2, 3] += 0.1
    logging.info(f"{GREEN} try move arm to higher pose...{RESET}")
    is_ok = arm.set_pose(target_T_base_end, move_line=True)
    if not is_ok:
        logging.warning(f"{YELLOW}move arm to higher pose failed. release gripper and try next label.{RESET}")
        return False
    # end if

    return True
# end def do_grasp


def run(T_end_cam: np.ndarray,
        gripper_body: GripperBody,
        tmpl_dict: Dict,
        arm: ArmWrapper,
        cam_node: CamNode,
        arm_node: TargetArmNode,
        matcher: Matcher3D,
        debug_level: int,
        debug: bool = False):
    """
    循环执行 3D 抓取任务    
    """

    detect_joints = tmpl_dict['detect_joints']               # 机械臂处于检测状态时的位置关节角度
    detect_gripper_dist = tmpl_dict['detect_gripper_dist']   # 机械臂处于检测状态时的夹爪位置
    place_T_base_end = tmpl_dict['place_T_base_end']         # 机械臂处于放置状态时的末端位姿
    place_joints = tmpl_dict['place_joints']                 # 机械臂处于放置状态时的位置关节角度
    place_gripper_dist = tmpl_dict['place_gripper_dist']     # 机械臂处于放置状态时的夹爪位置

    while rclpy.ok():

        print(f"\n{GREEN}start loop {RESET}")

        ######## 0. 移动到检测位置 ########
        logging.info(f'step [0] , {BLUE}move to detect pose{RESET}')
        if not wait_key(debug):
            return -1
        # end if

        logging.info(f"{GREEN}try move arm to detect pose...{RESET}")
        is_ok = arm.set_gripper_dist(detect_gripper_dist)
        if not is_ok:
            logging.error(f"{RED}set gripper to detect pose failed.{RESET}")
            break
        # end if

        detect_T_base_end = tmpl_dict['detect_T_base_end']
        is_ok = arm.set_pose(detect_T_base_end)
        if not is_ok:
            logging.error(f"{RED}move arm to detect pose failed, try again.{RESET}")
            break
        # end if

        # is_ok = arm.set_joints(detect_joints, th_angle_err=0.01)
        # if not is_ok:
        #     logging.error(f"{RED}move arm to detect pose failed, try again.{RESET}")
        #     break
        # # end if

        is_ok = do_grasp(T_end_cam=T_end_cam,
                         gripper_body=gripper_body,
                         tmpl_dict=tmpl_dict,
                         arm=arm,
                         cam_node=cam_node,
                         arm_node=arm_node,
                         matcher=matcher,
                         debug_level=debug_level,
                         debug=debug)

        if not is_ok:
            logging.error(f"{RED}grasp failed at current loop.{RESET}")
            return -1
        # end if

        ######## -1. 放置 ########
        print()
        logging.info(f'step [-1] , {BLUE}move to place pose{RESET}')
        if not wait_key(debug):
            return -1
        # end if

        # 移动到放置位置
        logging.info(f"{GREEN} try move arm to place pose...{RESET}")

        target_T_base_end = place_T_base_end.copy()
        target_T_base_end[2, 3] = arm.get_pose()[2, 3]  # 保持当前高度, 只移动 XY 平面位置和姿态
        is_ok = arm.set_pose(target_T_base_end, move_line=True)
        if not is_ok:
            logging.error(f"{RED}move arm to place pose failed.{RESET}")
            break
        # end if

        is_ok = arm.set_joints(place_joints)
        if not is_ok:
            logging.error(f"{RED}move arm to place pose failed.{RESET}")
            break
        # end if

        # 打开夹爪
        is_ok = arm.set_gripper_dist(place_gripper_dist)
        if not is_ok:
            logging.error(f"{RED}open gripper to place pose failed.{RESET}")
            break
        # end if

    # end while

    # 机械臂回到零点
    arm.set_joints(arm.init_joints)

    logging.info('run finished.')

# end def run


######################################################### 主函数 #########################################################


if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument("--color_img_topic", type=str, required=True,
                        help="彩色图像的 ROS2 话题名称")

    parser.add_argument("--depth_img_topic", type=str, required=True,
                        help="深度图像的 ROS2 话题名称")

    parser.add_argument("--tmpl_dir", type=str, required=True,
                        help="模板文件的目录")

    parser.add_argument("--debug", action='store_true',
                        help="是否开启调试模式")
    args = parser.parse_args()

    color_img_topic = args.color_img_topic
    if color_img_topic is None:
        logging.error("color_img_topic is not provided.")
        exit(0)
    # end if

    depth_img_topic = args.depth_img_topic
    if depth_img_topic is None:
        logging.error("depth_img_topic is not provided.")
        exit(0)
    # end if

    tmpl_dir = args.tmpl_dir
    if tmpl_dir is None:
        logging.error('no tmpl_dir specified, exiting')
        sys.exit(-1)
    # end if

    debug = args.debug
    debug_level = 0
    if debug is True:
        debug_level = 3
    # end if

    print()
    print(f"color image topic: {BLUE}{color_img_topic}{RESET}")
    print(f"depth image topic: {BLUE}{depth_img_topic}{RESET}")
    print(f'load grasp template from: {BLUE}{tmpl_dir}{RESET}')
    print(f'enable debug mode: {BLUE}{debug}{RESET}')
    print(f'debug level: {BLUE}{debug_level}{RESET}')
    print()

    # 读取相机参数
    camera_param_path = os.path.join(root_dir, 'data/calib/cam_params.json')
    camera_param_path = os.path.normpath(camera_param_path)
    intrinsic, distortion, depth_scale = read_rgbd_params(camera_param_path)
    if intrinsic is None or depth_scale is None:
        exit(1)
    # end if

    # 读取手眼标定矩阵
    print()
    handeye_calib_path = os.path.join(root_dir, 'data/calib/calib_handeye.json')
    T_end_cam, _ = read_handeye_calib(handeye_calib_path)

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

    # 读取抓取模板
    print()
    tmpl_dir = os.path.normpath(tmpl_dir)  # 规范化路径
    tmpl_dict = read_tmpl_grasp(tmpl_dir)
    if tmpl_dict is None:
        logging.error('read grasp tmpl failed.')
        sys.exit(-1)
    # end if

    # 创建机械臂对象
    arm = ArmWrapper()
    if not arm.is_connected():
        logging.error(f'{RED}failed to connect to arm, exiting{RESET}')   # 红色打印
        exit(1)
    # end if

    # 设置夹爪先闭合再打开,表明程序已经启动
    is_ok = arm.set_gripper_dist(0.02)
    if not is_ok:
        logging.error('set gripper initial position failed, exiting')
        exit(1)
    # end if
    time.sleep(0.5)
    is_ok = arm.set_gripper_dist(0.07)
    if not is_ok:
        logging.error('set gripper initial position failed, exiting')
        exit(1)
    # end if

    # 初始化匹配器
    config = Matcher3D.Config(
        intrinsic=intrinsic,
        depth_scale=depth_scale,
        distortion=distortion,
    )
    matcher = Matcher3D(config)

    # 初始化 ROS2 节点
    rclpy.init(args=None)

    cam_node = CamNode(img_topic_list=[color_img_topic, depth_img_topic])
    arm_node = TargetArmNode()

    # 运行
    run(T_end_cam=T_end_cam,
        gripper_body=gripper_body,
        tmpl_dict=tmpl_dict,
        arm=arm,
        cam_node=cam_node,
        arm_node=arm_node,
        matcher=matcher,
        debug_level=debug_level,
        debug=debug)

    cam_node.destroy_node()
    arm_node.destroy_node()
    rclpy.shutdown()
    logging.info('shutdown')

# end if __name__ == '__main__'

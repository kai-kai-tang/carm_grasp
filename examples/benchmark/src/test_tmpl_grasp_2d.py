"""
功能: 测试基于 AprilTag2 的 2D 抓取模板数据的使用
"""

import argparse
import os
import sys
import logging
import time
import json

from typing_extensions import List, Tuple, Dict

import numpy as np
import transforms3d

import rclpy


# 导入本工程的模块
code_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.normpath(f'{code_dir}/../../../')
sys.path.append(root_dir)

from core.utils import (
    GREEN, YELLOW, BLUE, RED, RESET,
    wait_key, inv_tf, read_cam_params, read_handeye_calib
)
from core.arm_wrapper import ArmWrapper
from core.arm_utils import compute_axis_aligned_pose
from core.arm_ros_utils import TargetArmNode
from core.cam_ros_utils import CamNode
from core.vision_utils import TagMatcher2D


######################################################### 全局变量 #########################################################


######################################################### 函数定义 #########################################################


def read_tmpl_grasp_2d(tmpl_dir: str) -> Dict:
    """
    读取抓取模板数据
    """

    # 读取处于夹取状态的模板数据
    file_path = f'{tmpl_dir}/grasp/state.json'
    with open(file_path, 'r') as f:
        grasp_state = json.load(f)
    # end with

    grasp_gripper_dist = grasp_state['gripper_dist']
    grasp_T_base_end = np.array(grasp_state['T_base_end'])

    # 读取处于近距离状态的模板数据
    file_path = f'{tmpl_dir}/near/state.json'
    with open(file_path, 'r') as f:
        near_state = json.load(f)
    # end with

    near_gripper_dist = near_state['gripper_dist']
    near_T_base_end = np.array(near_state['T_base_end'])
    near_obj_pose_2d = np.array(near_state['obj_pose_2d'])

    file_path = f'{tmpl_dir}/next_near/state.json'
    with open(file_path, 'r') as f:
        near_state = json.load(f)
    # end with

    next_near_T_base_end = np.array(near_state['T_base_end'])
    next_near_obj_pose_2d = np.array(near_state['obj_pose_2d'])

    # 读取处于远距离状态的模板数据
    file_path = f'{tmpl_dir}/far/state.json'
    with open(file_path, 'r') as f:
        far_state = json.load(f)
    # end with

    far_gripper_dist = far_state['gripper_dist']
    far_T_base_end = np.array(far_state['T_base_end'])
    far_obj_pose_2d = np.array(far_state['obj_pose_2d'])

    file_path = f'{tmpl_dir}/next_far/state.json'
    with open(file_path, 'r') as f:
        far_state = json.load(f)
    # end with

    next_far_T_base_end = np.array(far_state['T_base_end'])
    next_far_obj_pose_2d = np.array(far_state['obj_pose_2d'])

    tmpl_dict = {
        'grasp_gripper_dist': grasp_gripper_dist,
        'grasp_T_base_end': grasp_T_base_end,

        'near_gripper_dist': near_gripper_dist,
        'near_T_base_end': near_T_base_end,
        'near_obj_pose_2d': near_obj_pose_2d,
        'next_near_T_base_end': next_near_T_base_end,
        'next_near_obj_pose_2d': next_near_obj_pose_2d,

        'far_gripper_dist': far_gripper_dist,
        'far_T_base_end': far_T_base_end,
        'far_obj_pose_2d': far_obj_pose_2d,
        'next_far_T_base_end': next_far_T_base_end,
        'next_far_obj_pose_2d': next_far_obj_pose_2d,
    }

    return tmpl_dict
# end def read_tmpl_grasp_2d


def transform_pose_2d(R_dst_src: np.ndarray,
                      src_pose_2d: List[float]) -> np.ndarray:
    """将物体在 src 像素坐标系下的位姿转换到 dst 归一化坐标系下的位姿
    Args:
        R_dst_src (np.ndarray): 从 src 坐标系到 dst 坐标系的旋转矩阵 (3,3)
        src_pose_2d (List[float]): 物体在 src 坐标系下的位姿 [nx, ny, theta], 其中 (nx, ny) 是归一化坐标, theta 是物体的朝向角, 单位为弧度
    Returns:
        (np.ndarray): 物体在 dst 坐标系下的位姿 [nx, ny, theta], 其中 (nx, ny) 是归一化坐标, theta 是物体的朝向角, 单位为弧度
    """

    # 为便于区分, 这里将三维空间中的位移用 xyz 表示, 将归一化坐标中的位移用 nx,ny 表示, 图像/归一化坐标中的旋转用 theta 表示

    src_nx, src_ny, src_theta = src_pose_2d

    step = 0.01  # 用于计算物体朝向的辅助点与物体中心的距离, 单位为归一化坐标系下的距离, 这个值必须足够小, 以保证物体朝向的计算足够精确, 但又不能太小, 以避免数值误差的影响
    delta_src_nx = src_nx + step * np.cos(src_theta)
    delta_src_ny = src_ny + step * np.sin(src_theta)

    src_pts = np.array([
        [src_nx, src_ny, 1.0],
        [delta_src_nx, delta_src_ny, 1.0]
    ], dtype=np.float64).T  # (3,2)

    dst_rays = R_dst_src @ src_pts  # (3,2)
    dst_rays = dst_rays / (dst_rays[2:3, :] + 1e-6)  # 归一化 (3,2)

    dst_nx, dst_ny = dst_rays[0, 0], dst_rays[1, 0]
    delta_dst_nx, delta_dst_ny = dst_rays[0, 1], dst_rays[1, 1]
    dst_theta = np.arctan2(delta_dst_ny - dst_ny, delta_dst_nx - dst_nx)

    return np.array([dst_nx, dst_ny, dst_theta], dtype=np.float64)
# end def transform_pose_2d


def compute_delta_end_pose(T_end_cam: np.ndarray,
                           tmpl_dict: Dict,
                           cur_T_base_end: np.ndarray,
                           cur_obj_pose_2d: List[float],
                           z_step: float = 0.05,
                           ) -> np.ndarray:
    """
    计算机械臂末端的位姿增量

    Args:
        T_end_cam (np.ndarray): 从相机到机械臂末端的射影变换矩阵
        tmpl_dict (Dict): 模板数据字典
        cur_T_base_end (np.ndarray): 当前机械臂末端位姿
        cur_obj_pose_2d (List[float]): 当前物体在图像上的位姿 [nx, ny, theta], 其中 (nx, ny) 是归一化坐标, theta 是物体相对于相机的朝向角, 单位为弧度
        z_step (float): 每次控制机械臂在 z 轴上的移动步长, 单位为米

    Returns:
        (np.ndarray): 机械臂末端的位姿增量
    """

    near_T_base_end = tmpl_dict['near_T_base_end']
    near_obj_pose_2d = tmpl_dict['near_obj_pose_2d']
    next_near_T_base_end = tmpl_dict['next_near_T_base_end']
    next_near_obj_pose_2d = tmpl_dict['next_near_obj_pose_2d']

    far_T_base_end = tmpl_dict['far_T_base_end']
    far_obj_pose_2d = tmpl_dict['far_obj_pose_2d']
    next_far_T_base_end = tmpl_dict['next_far_T_base_end']
    next_far_obj_pose_2d = tmpl_dict['next_far_obj_pose_2d']

    # 设置一个虚拟相机, 满足:
    # 1) 虚拟相机的内参 K 为 3*3 的单位矩阵, 即虚拟相机坐标系下的二维坐标为归一化坐标
    # 2) 虚拟相机与末端的坐标轴同向 T_end_virtual[0:3, 3] = T_end_cam[0:3, 3]
    # 3) 虚拟相机与原始相机的原点重合  R_virtual_cam = T_end_cam[:3, :3]

    T_end_virtual = np.eye(4)
    T_end_virtual[0:3, 3] = T_end_cam[0:3, 3]
    R_virtual_cam = T_end_cam[:3, :3]

    # 将位姿转换到虚拟相机坐标系下
    near_T_base_virtual = near_T_base_end @ T_end_virtual
    next_near_T_base_virtual = next_near_T_base_end @ T_end_virtual
    far_T_base_virtual = far_T_base_end @ T_end_virtual
    next_far_T_base_virtual = next_far_T_base_end @ T_end_virtual
    cur_T_base_virtual = cur_T_base_end @ T_end_virtual

    # 将物体位姿转换到虚拟相机的归一化坐标系下
    near_virtual_pose_2d = transform_pose_2d(R_virtual_cam, near_obj_pose_2d)
    next_near_virtual_pose_2d = transform_pose_2d(R_virtual_cam, next_near_obj_pose_2d)
    far_virtual_pose_2d = transform_pose_2d(R_virtual_cam, far_obj_pose_2d)
    next_far_virtual_pose_2d = transform_pose_2d(R_virtual_cam, next_far_obj_pose_2d)
    cur_virtual_pose_2d = transform_pose_2d(R_virtual_cam, cur_obj_pose_2d)

    # 计算 ready 和 detect 两个时刻的平移与归一化坐标增量, 以及它们的比值
    near_z = near_T_base_virtual[2, 3]
    near_delta_xy = (inv_tf(near_T_base_virtual) @ next_near_T_base_virtual)[0:2, 3]      # 1 时刻末端在 0 时刻坐标系下的平移
    near_delta_uv = next_near_virtual_pose_2d[:2] - near_virtual_pose_2d[:2]              # 0-->>1 物体在虚拟相机上的归一化坐标增量
    near_ratio = np.linalg.norm(near_delta_xy) / (np.linalg.norm(near_delta_uv) + 1e-6)   # 平移与归一化坐标增量的比值

    far_z = far_T_base_virtual[2, 3]
    far_delta_xy = (inv_tf(far_T_base_virtual) @ next_far_T_base_virtual)[0:2, 3]       # 1 时刻末端在 0 时刻坐标系下的平移
    far_delta_uv = next_far_virtual_pose_2d[:2] - far_virtual_pose_2d[:2]               # 0-->>1 物体在虚拟相机上的归一化坐标增量
    far_ratio = np.linalg.norm(far_delta_xy) / (np.linalg.norm(far_delta_uv) + 1e-6)    # 平移与归一化坐标增量的比值

    # 计算虚拟相机的旋转增量
    delta_theta = cur_virtual_pose_2d[2] - near_virtual_pose_2d[2]
    while delta_theta > np.pi:
        delta_theta -= 2 * np.pi
    while delta_theta < -np.pi:
        delta_theta += 2 * np.pi
    # end while
    delta_R = transforms3d.axangles.axangle2mat([0, 0, 1], delta_theta)

    # 计算仅旋转后的 cur_virtual_pose_2d[:2]
    R_2d = delta_R.T[:2, :2]
    cur_virtual_pose_2d[:2] = R_2d @ cur_virtual_pose_2d[:2]

    # 计算虚拟相机的平移增量
    cur_z = cur_T_base_virtual[2, 3]
    alpha = (cur_z - near_z) / (far_z - near_z)
    target_ratio = near_ratio + alpha * (far_ratio - near_ratio)
    target_uv = near_virtual_pose_2d[:2] + alpha * (far_virtual_pose_2d[:2] - near_virtual_pose_2d[:2])
    delta_uv = cur_virtual_pose_2d[:2] - target_uv
    delta_xy = delta_uv * target_ratio

    diff_z = cur_z - near_z
    if abs(diff_z) > z_step:
        delta_z = z_step * np.sign(diff_z)
    else:
        delta_z = diff_z
    # end if

    # delta_z = 0   # 调试: 先不控制 z 轴移动, 只控制 x,y 平移和旋转
    delta_xyz = np.array([delta_xy[0], delta_xy[1], delta_z])

    # 计算最终的虚拟相机的位姿增量
    delta_T0 = np.eye(4)
    delta_T0[0:3, 0:3] = delta_R

    delta_T1 = np.eye(4)
    delta_T1[0:3, 3] = delta_xyz

    delta_T_virtual = delta_T0 @ delta_T1

    # 将虚拟相机的位姿增量转换到末端坐标系下
    delta_T_end = T_end_virtual @ delta_T_virtual @ inv_tf(T_end_virtual)

    return delta_T_end
# end def compute_delta_end_pose


def do_grasp(T_end_cam: np.ndarray,
             tmpl_dict: Dict,
             matcher: TagMatcher2D,
             arm: ArmWrapper,
             cam_node: CamNode,
             arm_node: TargetArmNode,
             debug: bool = False
             ) -> bool:
    """
    执行一次 2D 抓取动作
    Returns:
        bool: 是否抓取成功
    """

    grasp_gripper_dist = tmpl_dict['grasp_gripper_dist']
    grasp_T_base_end = tmpl_dict['grasp_T_base_end']
    near_T_base_end = tmpl_dict['near_T_base_end']
    final_T_end = inv_tf(near_T_base_end) @ grasp_T_base_end

    max_try_cnt = 20
    try_cnt = 0
    while rclpy.ok():
        ######## 1. 检测物体 ########
        print()
        logging.info(f'grasp-step [1-{try_cnt}] , {BLUE}detect pose_2d{RESET}')
        if not wait_key(debug):
            return False
        # end if

        # 获取图像
        frame_list = cam_node.get_frames(do_spin_once=True)
        if frame_list is None:
            logging.error(f'{RED}failed to get images, skip this grasp try{RESET}')
            return False
        # end if

        rgb_img = frame_list[0][0]  # 取第一帧第一摄像头图像

        # 检测物体位置
        result_list, msg = matcher.match(rgb_img, top_k=1)
        if len(result_list) == 0:
            logging.warning(f'{YELLOW}no match found, skip this grasp try, {msg}{RESET}')
            return False
        # end if

        cur_obj_pose_2d = result_list[0].pose_2d
        logging.info(f'detected obj_pose_2d: {cur_obj_pose_2d}')

        # 计算目标末端位姿增量
        cur_T_base_end = arm.get_pose()
        delta_T_end = compute_delta_end_pose(T_end_cam,
                                             tmpl_dict,
                                             cur_T_base_end,
                                             cur_obj_pose_2d)
        logging.info(f'computed delta_T_end: \n{delta_T_end}')

        delta_dist = np.linalg.norm(delta_T_end[0:3, 3])
        delta_angle = np.arccos((np.trace(delta_T_end[0:3, 0:3]) - 1) / 2)
        logging.info(f'delta_dist(m): {delta_dist:.4f}, delta_angle(deg): {delta_angle * 180.0 / np.pi:.4f}')
        if delta_dist < 0.0005 and delta_angle < 1.0 / 180.0 * np.pi:
            logging.info(f'delta_T_end is small enough, no need to move, break detecting loop')
            break
        # end if

        if delta_dist < 0.01:
            delta_T_end[0:3, 3] = delta_T_end[0:3, 3] * 0.5  # 距离目标很近时,需要缩小增量, 避免过冲
            time.sleep(0.2)  # 如果增量较小, 则先等待一段时间再执行下一个循环, 避免频繁地发送小增量的控制命令
        # end if

        target_T_base_end = cur_T_base_end @ delta_T_end

        # 发布目标位姿, 供 rviz 显示
        arm_node.publish_pose(target_T_base_end)

        ######## 2. 控制机械臂运动到目标位姿 ########
        print()
        logging.info(f'grasp-step [2-{try_cnt}] , {BLUE}move to target pose{RESET}')
        if not wait_key(debug):
            return False
        # end if

        is_ok = arm.set_pose(target_T_base_end)
        if not is_ok:
            logging.error(f"{RED}move arm to target pose failed, try again.{RESET}")
            return False
        # end if

        try_cnt += 1
        if try_cnt >= max_try_cnt:
            logging.error(f'{RED}reach max try cnt {max_try_cnt}{RESET}')
            return False
        # end if
    # end while

    cur_T_base_end = arm.get_pose()
    target_T_base_end = cur_T_base_end @ final_T_end

    # 发布目标位姿, 供 rviz 显示
    arm_node.publish_pose(target_T_base_end)

    ######## 3. 抓取物体 ########
    print()
    logging.info(f'grasp-step [3] , {BLUE}grasp object{RESET}')
    if not wait_key(debug):
        return False
    # end if

    # 运动到抓取位姿
    is_ok = arm.set_pose(target_T_base_end)
    if not is_ok:
        logging.error(f"{RED}move arm to target pose failed{RESET}")
        return False
    # end if

    # 合拢夹爪
    is_ok = arm.set_gripper_dist(grasp_gripper_dist - 0.01)
    if not is_ok:
        logging.error(f"{RED}grasp object failed.{RESET}")
        return False
    # end if

    ######## 4. 升高 ########
    print()
    logging.info(f'grasp-step [4] , {BLUE}move up{RESET}')
    if not wait_key(debug):
        return False
    # end if

    # 原地提高高度
    target_T_base_end = arm.get_pose()
    target_T_base_end[2, 3] += 0.1
    is_ok = arm.set_pose(target_T_base_end)
    if not is_ok:
        logging.error(f"{RED}move arm to higher pose failed.{RESET}")
        return False
    # end if

    return True
# end def do_grasp


def run(T_end_cam: np.ndarray,
        tmpl_dict: Dict,
        detect_T_base_end: np.ndarray,
        place_T_base_end: np.ndarray,
        cam_node: CamNode,
        arm_node: TargetArmNode,
        matcher: TagMatcher2D,
        arm: ArmWrapper,
        debug: bool = False):
    """
    执行平面上物体抓取任务
    """

    max_gripper_dist = 0.08
    initial_T_base_end = compute_axis_aligned_pose(detect_T_base_end, base_axis_idx=-3, obj_axis_idx=3)  # 机械臂末端坐标系的 z 轴与基座的 -z 轴平行的位姿
    if initial_T_base_end is None:
        logging.error(f'{RED}failed to compute initial_T_base_end, exiting...{RESET}')
        return
    # end if

    while rclpy.ok():
        print(f"\n{GREEN}start loop {RESET}")

        ######## 0. 移动到检测位置 ########
        print()
        logging.info(f'step [0] , {BLUE}move to detect pose{RESET}')
        if not wait_key(debug):
            break
        # end if

        logging.info(f"{GREEN}try move arm to detect pose...{RESET}")
        is_ok = arm.set_gripper_dist(max_gripper_dist)
        if not is_ok:
            logging.error(f"{RED}set gripper to ready dist failed.{RESET}")
            break
        # end if

        is_ok = arm.set_pose(initial_T_base_end)
        if not is_ok:
            logging.error(f"{RED}move arm to detect pose failed{RESET}")
            break
        # end if

        ######## 执行一次抓取 ########
        is_ok = do_grasp(T_end_cam,
                         tmpl_dict,
                         matcher,
                         arm,
                         cam_node,
                         arm_node,
                         debug)
        if not is_ok:
            break
        # end if

        ######## -1. 丢下物体 ########
        print()
        logging.info(f'step [-1] , {BLUE}release object{RESET}')
        if not wait_key(debug):
            break
        # end if

        # 移动到放置位置
        is_ok = arm.set_pose(place_T_base_end)
        if not is_ok:
            logging.error(f"{RED}move arm to place pose failed{RESET}")
            break
        # end if

        # 打开夹爪
        is_ok = arm.set_gripper_dist(max_gripper_dist)
        if not is_ok:
            logging.error(f"{RED}open gripper failed.{RESET}")
            break
        # end if

        time.sleep(0.5)
    # end while
# end def run


######################################################### 主函数 #########################################################

if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument("--cam_params_path", type=str, required=True,
                        help="相机参数文件的路径, 包含内参和畸变参数")

    parser.add_argument("--handeye_calib_path", type=str, required=True,
                        help="手眼标定文件的路径, 包含相机与机械臂的位姿关系")

    parser.add_argument("--color_img_topic", type=str, required=True,
                        help="RGB 图像的 ROS2 话题名称")

    parser.add_argument("--tmpl_dir", type=str, required=True,
                        help="模板文件的目录")

    parser.add_argument("--detect_pose", type=str, required=True,
                        help="检测状态下的位姿, 格式[tx,ty,tz,qx,qy,qz,qw], 其中 t 是位移, q 是旋转四元数")

    parser.add_argument("--place_pose", type=str, required=True,
                        help="放置状态下的位姿, 格式[tx,ty,tz,qx,qy,qz,qw], 其中 t 是位移, q 是旋转四元数")

    parser.add_argument("--debug", action='store_true',
                        help="是否开启调试模式")

    args = parser.parse_args()

    cam_params_path = args.cam_params_path
    handeye_calib_path = args.handeye_calib_path

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

    detect_pose = json.loads(args.detect_pose)
    detect_T_base_end = ArmWrapper.array_to_matrix(detect_pose)

    place_pose = json.loads(args.place_pose)
    place_T_base_end = ArmWrapper.array_to_matrix(place_pose)

    debug = args.debug

    print()
    print(f"camera parameters file: {BLUE}{cam_params_path}{RESET}")
    print(f"handeye calib file: {BLUE}{handeye_calib_path}{RESET}")
    print(f"color image topic: {BLUE}{color_img_topic}{RESET}")
    print(f'grasp_2d template dir: {BLUE}{tmpl_dir}{RESET}')
    print(f'detect pose: {BLUE}{detect_pose}{RESET}')
    print(f'place pose: {BLUE}{place_pose}{RESET}')
    print(f"debug: {BLUE}{debug}{RESET}")
    print()

    # 读取相机参数
    intrinsic, distortion = read_cam_params(cam_params_path)

    # 初始化匹配器
    config = TagMatcher2D.Config(
        intrinsic=intrinsic,
        distortion=distortion,
    )
    matcher = TagMatcher2D(config)

    # 读取手眼标定结果
    T_end_cam, _ = read_handeye_calib(handeye_calib_path)
    if T_end_cam is None:
        exit(1)
    # end if

    # 读取抓取模板数据
    tmpl_dir = os.path.normpath(tmpl_dir)  # 规范化路径
    tmpl_dict = read_tmpl_grasp_2d(tmpl_dir)
    if tmpl_dict is None:
        logging.error(f'{RED}failed to read grasp tmpl from {tmpl_dir}, exiting...{RESET}')
        exit(1)
    # end if

    # 创建机械臂对象
    arm = ArmWrapper()
    if not arm.is_connected():
        logging.error(f'{RED}failed to connect to arm, exiting {RESET}')
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

    # 初始化 ROS2 节点
    rclpy.init(args=None)
    cam_node = CamNode([color_img_topic])
    arm_node = TargetArmNode()

    # 运行
    run(T_end_cam,
        tmpl_dict,
        detect_T_base_end,
        place_T_base_end,
        cam_node,
        arm_node,
        matcher,
        arm,
        debug=debug)

# end if __name__ == '__main__':

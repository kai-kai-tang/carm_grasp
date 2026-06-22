# -*- coding: utf-8 -*-
"""
功能说明: 录制机械臂的行动模板文件, 包含机械臂末端位姿, 关节角, 夹爪距离
"""

import argparse
import os
import sys
import logging
import time
import json

from typing_extensions import List, Tuple

import numpy as np


# 导入本工程的模块
code_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.normpath(f'{code_dir}/../../../')
sys.path.append(root_dir)

from core.utils import (
    GREEN, YELLOW, BLUE, RED, RESET,
)
from core.arm_wrapper import ArmWrapper
from core.arm_utils import compute_axis_aligned_pose


######################################################### 全局变量 #########################################################


######################################################### 函数定义 #########################################################

def save_action(arm: ArmWrapper,
                file_path: str):
    """
    保存当前机械臂状态为一个行动模板, 包含机械臂末端位姿, 关节角, 夹爪距离
    Args:
        arm (ArmWrapper): 机械臂对象
        save_dir (str): 保存路径
    """

    # 获取机械臂末端位姿
    T_base_end = arm.get_pose()

    # 获取机械臂关节角
    joints = arm.get_joints()

    # 获取夹爪距离
    gripper_dist = arm.get_gripper_dist()

    data_dict = {
        'T_base_end': T_base_end.tolist(),
        'joints': joints,
        'gripper_dist': gripper_dist
    }

    logging.info(f'joints: {GREEN}{joints}{RESET} ...')

    # 保存模板数据
    with open(file_path, 'w') as f:
        json.dump(data_dict, f, indent=4)
    logging.info(f'saved action to: {GREEN}{file_path}{RESET}')
# end def save_action


def load_action(file_path: str) -> Tuple[np.ndarray, List[float], float]:
    """
    从模板文件中加载一个行动模板, 包含机械臂末端位姿, 关节角, 夹爪距离
    Args:
        file_path (str): 模板文件路径

    Returns:
        (Tuple[np.ndarray, List[float], float]): 机械臂末端位姿, 关节角, 夹爪距离
    """

    if not os.path.exists(file_path):
        logging.warning(f'{YELLOW}action tmpl file not found: {file_path}{RESET}')
        return None, None, None
    # end if

    try:
        with open(file_path, 'r') as f:
            data_dict = json.load(f)
    except Exception as e:
        logging.warning(f'{YELLOW}failed to load action from file: {file_path}, error: {e}{RESET}')
        return None, None, None
    # end try

    if 'T_base_end' not in data_dict or 'joints' not in data_dict or 'gripper_dist' not in data_dict:
        logging.error(f"{RED}invalid action tmpl file: {file_path}, missing required fields 'T_base_end'/'joints'/'gripper_dist'{RESET}")
        return None, None, None
    # end if

    T_base_end = np.array(data_dict['T_base_end'])
    joints = data_dict['joints']
    gripper_dist = data_dict['gripper_dist']

    return T_base_end, joints, gripper_dist
# end def load_action


######################################################### 主函数 #########################################################

if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument("--tmpl_dir", type=str,
                        help="保存模板文件的目录")

    args = parser.parse_args()

    # 解析参数
    tmpl_dir = args.tmpl_dir
    if tmpl_dir is None:
        logging.error('no tmpl_dir specified, exiting')
        exit(1)
    # end if

    print()
    print(f'action template will be saved to: {GREEN}{tmpl_dir}{RESET}')
    print()

    # 创建模板文件夹
    tmpl_dir = os.path.normpath(tmpl_dir)
    os.makedirs(tmpl_dir, exist_ok=True)

    # 创建机械臂对象
    arm = ArmWrapper()
    if not arm.is_connected():
        logging.error(f'{RED}failed to connect to arm, exiting {RESET}')
        exit(1)
    # end if
    # arm.set_speed_level(10)

    # 主循环
    print()
    logging.info(f'use keyboard to control: \n{BLUE}'
                 f'  q: 退出 \n'
                 f'  z: 位置控制模式 \n'
                 f'  x: 拖动模式 \n'
                 f'  a: 对齐 arm_end z 轴到 arm_base -z 轴 \n'
                 f'  >: 转到下一个模板 \n'
                 f'  <: 转到上一个模板 \n'
                 f'  e: 执行当前模板 \n'
                 f'  s: 保存当前模板 \n'
                 f'  d: 删除当前模板 \n{RESET}')

    current_tmpl_idx = 0
    while True:

        print()
        key = input(f'press: ')
        if key is None:
            time.sleep(0.03)
            continue
        # end if
        logging.info(f'you pressed: [{key}]')

        if key == 'q':
            logging.info(f'{BLUE}exit program ...{RESET}')
            break
        # end if

        if key == 'z':
            logging.info(f'{BLUE}切换到位置控制模式 ...{RESET}')
            is_ok = arm.set_control_mode(ArmWrapper.ControlMode.POSITION)
            if not is_ok:
                logging.warning(f'{YELLOW}切换到位置控制模式失败{RESET}')
            # end if
            print()
        # end if

        if key == 'x':
            logging.info(f'{BLUE}切换到拖动模式 ...{RESET}')
            is_ok = arm.set_control_mode(ArmWrapper.ControlMode.TEACH)
            if not is_ok:
                logging.warning(f'{YELLOW}切换到拖动模式失败{RESET}')
            # end if
            print()
        # end if

        if key == 'a':
            logging.info(f'{BLUE}开始对齐 arm_end z 轴到 arm_base -z 轴 ...{RESET}')

            T_base_end = arm.get_pose()
            target_T_base_end = compute_axis_aligned_pose(T_base_end, base_axis_idx=-3, obj_axis_idx=3)
            if target_T_base_end is None:
                continue
            # end if

            logging.info(f'尝试移动到 T_base_end:\n{target_T_base_end}')
            is_ok = arm.set_pose(target_T_base_end)
            if not is_ok:
                logging.warning('移动到 T_base_end 失败')
            # end if
            print()
        # end if

        file_path = os.path.join(tmpl_dir, f'{current_tmpl_idx}.json')

        if key == '.':   # 注意: 键盘上是 > 键, 但是输入时是 . 键
            if not os.path.exists(file_path):
                logging.warning(f'{YELLOW}当前模板未找到, idx: [{current_tmpl_idx}], 无法转到下一个模板{RESET}')
                continue
            # end if

            current_tmpl_idx += 1
            logging.info(f'{BLUE}转到下一个模板, idx: [{current_tmpl_idx}]{RESET}')
        # end if

        if key == ',':   # 注意: 键盘上是 < 键, 但是输入时是 , 键
            if current_tmpl_idx == 0:
                logging.warning(f'{YELLOW}已经是第一个模板, 无法返回{RESET}')
                continue
            # end if

            current_tmpl_idx -= 1
            logging.info(f'{BLUE}转到上一个模板, idx: [{current_tmpl_idx}]{RESET}')
        # end if

        if key == 'e':
            logging.info(f'{BLUE}执行当前模板, idx: [{current_tmpl_idx}]{RESET}')

            if not os.path.exists(file_path):
                logging.warning(f'{YELLOW}当前模板未找到, idx: [{current_tmpl_idx}]{RESET}')
                continue
            # end if

            T_base_end, joints, gripper_dist = load_action(file_path)

            logging.info(f'尝试执行当前模板, idx: [{current_tmpl_idx}] ...{RESET}')
            is_ok = arm.set_joints(joints)
            if not is_ok:
                logging.warning(f'{YELLOW}移动到模板关节位置失败{RESET}')
                continue
            # end if

            # arm.set_speed_level(10)  # 设置较慢的速度, 以免执行模板时机械臂运动过快

            is_ok = arm.set_control_mode(ArmWrapper.ControlMode.POSITION)
            if not is_ok:
                logging.warning(f'{YELLOW}切换到位置控制模式失败{RESET}')
                continue
            # end if

            is_ok = arm.set_gripper_dist(gripper_dist)
            if not is_ok:
                logging.warning(f'{YELLOW}移动到模板夹爪距离失败{RESET}')
                continue
            # end if
        # end if

        if key == 's':
            logging.info(f'{BLUE}保存模板, idx: [{current_tmpl_idx}]{RESET}')

            file_path = os.path.join(tmpl_dir, f'{current_tmpl_idx}.json')
            save_action(arm, file_path)
        # end if

        if key == 'd':
            logging.info(f'{BLUE}删除模板, idx: [{current_tmpl_idx}]{RESET}')

            file_path = os.path.join(tmpl_dir, f'{current_tmpl_idx}.json')
            if os.path.exists(file_path):
                os.remove(file_path)
                logging.info(f'{BLUE}已删除模板, idx: [{current_tmpl_idx}]{RESET}')
            else:
                logging.warning(f'{RED}模板未找到, idx: [{current_tmpl_idx}]{RESET}')
            # end if
        # end if
    # end while

# end if __name__ == '__main__'

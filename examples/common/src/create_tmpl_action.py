# -*- coding: utf-8 -*-
"""
功能说明: 创建机械臂的行动模板文件, 包含机械臂末端位姿, 关节角, 夹爪距离
"""

import argparse
import os
import sys
import logging
import time
import json

from typing_extensions import List, Tuple, Dict

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

    # 主循环
    print()
    logging.info(f'use keyboard to control: \n{BLUE}'
                 f'  q: quit \n'
                 f'  a: align arm_end z axis to arm_base -z axis \n'
                 f'  b: align arm_end x axis to arm_base z axis \n'
                 f'  >: go to next tmpl \n'
                 f'  <: go to previous tmpl \n'
                 f'  e: execute current tmpl \n'
                 f'  s: save current tmpl \n'
                 f'  d: delete current tmpl \n{RESET}')

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

        if key == 'a':
            logging.info(f'{BLUE}start to align arm_end z axis to arm_base -z axis ...{RESET}')

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

        if key == 'b':
            logging.info(f'{BLUE}start to align arm_end x axis to arm_base z axis ...{RESET}')

            T_base_end = arm.get_pose()
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

        file_path = os.path.join(tmpl_dir, f'{current_tmpl_idx}.json')

        if key == '.':   # 注意: 键盘上是 > 键, 但是输入时是 . 键
            if not os.path.exists(file_path):
                logging.warning(f'{YELLOW}current tmpl not found, idx: [{current_tmpl_idx}], cannot go to next tmpl{RESET}')
                continue
            # end if

            current_tmpl_idx += 1
            logging.info(f'{BLUE}go to next tmpl, idx: [{current_tmpl_idx}]{RESET}')
        # end if

        if key == ',':   # 注意: 键盘上是 < 键, 但是输入时是 , 键
            if current_tmpl_idx == 0:
                logging.warning(f'{YELLOW}already at first tmpl, cannot go back{RESET}')
                continue
            # end if

            current_tmpl_idx -= 1
            logging.info(f'{BLUE}go to previous tmpl, idx: [{current_tmpl_idx}]{RESET}')
        # end if

        if key == 'e':
            logging.info(f'{BLUE}execute tmpl, idx: [{current_tmpl_idx}]{RESET}')

            if not os.path.exists(file_path):
                logging.warning(f'{YELLOW}tmpl not found, idx: [{current_tmpl_idx}]{RESET}')
                continue
            # end if

            T_base_end, joints, gripper_dist = load_action(file_path)

            logging.info(f'try execute tmpl, idx: [{current_tmpl_idx}] ...{RESET}')
            is_ok = arm.set_joints(joints)
            if not is_ok:
                logging.warning(f'{YELLOW}failed to move to tmpl joints{RESET}')
                continue
            # end if

            # arm.set_speed_level(10)  # 设置较慢的速度, 以免执行模板时机械臂运动过快

            is_ok = arm.set_control_mode(ArmWrapper.ControlMode.POSITION)
            if not is_ok:
                logging.warning(f'{YELLOW}failed to switch to position control mode{RESET}')
                continue
            # end if

            is_ok = arm.set_gripper_dist(gripper_dist)
            if not is_ok:
                logging.warning(f'{YELLOW}failed to move to tmpl gripper dist{RESET}')
                continue
            # end if
        # end if

        if key == 's':
            logging.info(f'{BLUE}save tmpl, idx: [{current_tmpl_idx}]{RESET}')

            file_path = os.path.join(tmpl_dir, f'{current_tmpl_idx}.json')
            save_action(arm, file_path)
        # end if

        if key == 'd':
            logging.info(f'{BLUE}delete tmpl, idx: [{current_tmpl_idx}]{RESET}')

            file_path = os.path.join(tmpl_dir, f'{current_tmpl_idx}.json')
            if os.path.exists(file_path):
                os.remove(file_path)
                logging.info(f'{BLUE}deleted tmpl, idx: [{current_tmpl_idx}]{RESET}')
            else:
                logging.warning(f'{RED}tmpl not found, idx: [{current_tmpl_idx}]{RESET}')
            # end if
        # end if
    # end while
# end if __name__ == '__main__'

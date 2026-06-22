# -*- coding: utf-8 -*-
"""
功能说明: 执行行动模板文件, 包含机械臂末端位姿, 关节角, 夹爪距离
"""

import argparse
import os
import sys
import logging
import time
import json

from typing_extensions import List, Tuple, Dict


# 导入本工程的模块

code_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.normpath(f'{code_dir}/../../../')
sys.path.append(root_dir)

from core.utils import (
    GREEN, YELLOW, BLUE, RED, RESET,
    wait_key
)
from core.arm_wrapper import ArmWrapper

from examples.common.src.action_record import load_action  # 同目录下的模块


######################################################### 全局变量 #########################################################


######################################################### 函数定义 #########################################################

def read_action_list(tmpl_dir: str) -> List[Dict]:
    """
    读取文件夹内的所有行动模板文件, 包含机械臂末端位姿, 关节角, 夹爪距离
    Args:
        tmpl_dir (str): 模板文件夹路径

    Returns:
        List[Dict]: 包含所有行动模板的列表,每个模板为一个字典,包含机械臂末端位姿、关节角和夹爪距离
    """
    action_tmpl_list = []
    tmpl_idx = 0
    while True:
        file_path = os.path.join(tmpl_dir, f'{tmpl_idx}.json')
        T_base_end, joints, gripper_dist = load_action(file_path)
        if T_base_end is None or joints is None or gripper_dist is None:
            break
        # end if

        action_tmpl_list.append({
            'T_base_end': T_base_end,
            'joints': joints,
            'gripper_dist': gripper_dist
        })

        tmpl_idx += 1
    # end while

    return action_tmpl_list
# end def read_action_list


def do_action(action_tmpl_list: List[Dict],
              arm: ArmWrapper,
              debug: bool = False) -> bool:
    """
    执行行动模板列表中的每一个模板, 包含机械臂末端位姿, 关节角, 夹爪距离
    Args:
        action_tmpl_list (List[Dict]): 包含所有行动模板的列表,每个模板为一个字典,包含机械臂末端位姿、关节角和夹爪距离
        arm (ArmWrapper): 机械臂对象
        debug (bool): 是否开启调试模式, 如果开启, 则在执行每一个模板前会等待用户输入
    """
    is_ok = arm.set_control_mode(ArmWrapper.ControlMode.PF)
    if not is_ok:
        logging.error(f'{RED}failed to switch to [position force] mode{RESET}')
        return False
    # end if

    for idx, action_tmpl in enumerate(action_tmpl_list):

        print()
        logging.info(f'action-idx {BLUE}[{idx}]{RESET}')
        if not wait_key(debug):
            return False
        # end if

        joints = action_tmpl['joints']
        gripper_dist = action_tmpl['gripper_dist']

        is_ok = arm.set_joints(joints, move_line=False)
        if not is_ok:
            logging.warning(f'{YELLOW}failed to move to tmpl joints{RESET}')
            return False
        # end if

        is_ok = arm.set_gripper_dist(gripper_dist)
        if not is_ok:
            logging.warning(f'{YELLOW}failed to move to tmpl gripper dist{RESET}')
            return False
        # end if

        # time.sleep(0.5)  # 等待机械臂运动完成
    # end for

    return True
# end def do_action


def run(action_tmpl_list: List[Dict],
        arm: ArmWrapper,
        debug: bool = False) -> bool:
    """
    执行行动模板列表中的每一个模板, 包含机械臂末端位姿, 关节角, 夹爪距离
    Args:
        action_tmpl_list (List[Dict]): 包含所有行动模板的列表,每个模板为一个字典,包含机械臂末端位姿、关节角和夹爪距离
        arm (ArmWrapper): 机械臂对象
        debug (bool): 是否开启调试模式, 如果开启, 则在执行每一个模板前会等待用户输入
    """
    # if debug:
    #     arm.set_speed_level(10)  # 设置较慢的速度, 以免执行模板时机械臂运动过快
    # # end if

    while True:
        print(f"\n{GREEN}start loop {RESET}")
        if not wait_key(True):
            break
        # end if

        is_ok = do_action(action_tmpl_list, arm, debug)
        if not is_ok:
            break
        # end if
    # end while

    logging.info(f'{GREEN}action loop finished{RESET}, move to the first action template')
    action_tmpl = action_tmpl_list[0]
    joints = action_tmpl['joints']
    gripper_dist = action_tmpl['gripper_dist']

    is_ok = arm.set_joints(joints, move_line=False)
    if not is_ok:
        logging.warning(f'{YELLOW}failed to move to tmpl joints{RESET}')
        return False
    # end if

    is_ok = arm.set_gripper_dist(gripper_dist)
    if not is_ok:
        logging.warning(f'{YELLOW}failed to move to tmpl gripper dist{RESET}')
        return False
    # end if

# end def run

######################################################### 主函数 #########################################################


if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument("--tmpl_dir", type=str,
                        help="保存模板文件的目录")

    parser.add_argument("--debug", action='store_true',
                        help="是否开启调试模式")

    args = parser.parse_args()

    # 解析参数
    tmpl_dir = args.tmpl_dir
    if tmpl_dir is None:
        logging.error('no tmpl_dir specified, exiting')
        exit(1)
    # end if

    debug = args.debug

    print()
    print(f'action template will be read from: {BLUE}{tmpl_dir}{RESET}')
    print(f'enable debug mode: {BLUE}{debug}{RESET}')
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

    # 执行任务
    run(action_tmpl_list, arm, debug=debug)
# end if __name__ == '__main__'

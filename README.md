# carm_grasp

基于 CARM 机械臂、ROS2 和 RGB-D 相机的抓取示例工程，包含基础机械臂操作、夹爪标定、动作模板录制，以及 2D / 3D 抓取模板的创建与测试。    
注意: **当前抓取示例仅仅适用于表明贴有 apriltag 的物体**

## TODO
添加视频说明

## 依赖

- [apriltag2](https://github.com/kai-kai-tang/apriltag2)
- [carm](https://pypi.org/project/carm/0.1.20260512/)
- ROS2（脚本中兼容 Foxy / Humble）
- Python 依赖：numpy、opencv-python、open3d、mmengine、transforms3d、rclpy

## 目录说明

- `core/`：机械臂、相机、视觉匹配、ROS2 通信等基础封装。
- `examples/common/src/`：基础示例，包括机械臂状态发布、夹爪标定、动作模板录制与回放。
- `examples/benchmark/src/`：抓取基准示例，包括 2D 抓取模板和 3D 抓取模板的创建与测试。
- `examples/*/scripts/`：对应 Python 示例的启动脚本，已写好 ROS 环境和默认参数。
- `data/calib/`：相机参数、手眼标定结果、夹爪模型等标定文件。
- `data/benchmark/tmpl/`：仓库内置的 2D / 3D 抓取模板样例。

## 运行前准备

1. 确保机械臂可以通过 `carm` 正常连接。
2. 确保 ROS2 环境可用，并且相机话题已经发布。
3. 根据实际设备修改 `examples/common/scripts/*.sh` 和 `examples/benchmark/scripts/*.sh` 中的参数，重点检查：
	- `ROS_DOMAIN_ID`
	- 彩色 / 深度图像话题名
	- `pc_frame_id`
	- 模板保存目录
4. 确保以下标定文件已经就绪：
	- `data/calib/cam_params.json`
	- `data/calib/calib_handeye.json`
	- `data/calib/gripper_body.json`（3D 抓取测试需要；可通过夹爪标定脚本生成）

注意：`examples/common/scripts/create_tmpl_action.sh` 与 `examples/common/scripts/test_tmpl_action.sh` 默认将动作模板保存到仓库外路径，第一次使用前建议先改成你自己的目录。

## examples/common/src 说明

### arm_node.py

用途：启动机械臂 ROS2 节点，持续发布末端位姿、关节角、夹爪 Marker 和相机相关 TF，方便在 RViz 中观察状态。

主要参数：

- `--frame_id`：机械臂基座坐标系名称。
- `--pc_frame_id`：相机点云坐标系名称。

交互按键：

- `q`：退出。
- `v`：打印当前关节角、末端位姿和夹爪距离。
- `a`：对齐末端 Z 轴到基座 -Z 方向。
- `b`：对齐末端 X 轴到基座 Z 方向。
- `c`：对齐相机 Z 轴到下方。
- `,` / `.`：缩小 / 放大夹爪开口。

运行方式：

```bash
bash examples/common/scripts/arm_node.sh
```

### calib_gripper.py

用途：根据 AprilTag 平面和深度图，估计夹爪在相机坐标系下的位姿，并生成 `data/calib/gripper_body.json`。

主要参数：

- `--color_img_topic`：彩色图像话题。
- `--depth_img_topic`：深度图像话题。
- `--pc_frame_id`：点云坐标系名称。
- `--gripper_size`：夹爪宽度和厚度，格式为 `[width,thickness]`，单位为米。

交互按键：

- `q`：退出。
- `a`：调整末端姿态，使末端朝下。
- `,` / `.`：缩小 / 放大夹爪开口。
- `t`：采集当前 RGB-D 数据并估计夹爪位姿。
- `s`：保存标定结果到 `data/calib/gripper_body.json`。

运行方式：

```bash
bash examples/common/scripts/calib_gripper.sh
```

### create_tmpl_action.py

用途：录制一组通用动作模板。每个模板保存为一个 JSON 文件，包含末端位姿、关节角和夹爪开口，可用于非视觉动作回放。

主要参数：

- `--tmpl_dir`：模板保存目录。

交互按键：

- `q`：退出。
- `a`：对齐末端 Z 轴到基座 -Z。
- `b`：对齐末端 X 轴到基座 Z。
- `.`：切换到下一个模板编号。
- `,`：切换到上一个模板编号。
- `e`：执行当前编号对应的模板。
- `s`：保存当前机械臂状态为模板。
- `d`：删除当前模板。

运行方式：

```bash
bash examples/common/scripts/create_tmpl_action.sh
```

### test_tmpl_action.py

用途：顺序读取动作模板目录中的 `0.json`、`1.json`、`2.json`...，并循环回放。

主要参数：

- `--tmpl_dir`：模板目录。

运行方式：

```bash
bash examples/common/scripts/test_tmpl_action.sh
```

使用方式：

- 启动后会先读取模板目录。
- 每轮执行前会等待一次确认。
- 然后按模板编号顺序依次运动关节并设置夹爪开口。

## examples/benchmark/src 说明

### create_tmpl_grasp_2d.py

用途：录制 2D 抓取模板。脚本会保存抓取位姿，以及 near / next_near / far / next_far 等带目标观测的状态，用于后续根据图像中目标的 2D 位姿修正机械臂运动。

主要参数：

- `--color_img_topic`：彩色图像话题。
- `--tmpl_dir`：模板目录。

输出内容：

- `grasp/state.json`
- `near/state.json`
- `next_near/state.json`
- `far/state.json`
- `next_far/state.json`
- 采样时保存的 `color.png` 和目标检测示意图 `tag.png`

交互按键：

- `a`：调整末端朝向。
- `g`：保存抓取位姿模板。
- `n`：保存 near 模板。
- `b`：保存 next_near 模板。
- `f`：保存 far 模板。
- `d`：保存 next_far 模板。

运行方式：

```bash
bash examples/benchmark/scripts/create_tmpl_grasp_2d.sh
```

推荐录制顺序：

1. 将机械臂移动到最终抓取位姿，按 `g` 保存 `grasp`。
2. 抬高到较近观察位姿，且画面中能看到目标，按 `n` 保存 `near`。
3. 在 near 位姿基础上做一小段平面内移动，按 `b` 保存 `next_near`。
4. 移动到更远的观察位姿，按 `f` 保存 `far`。
5. 在 far 位姿基础上再做一小段平面内移动，按 `d` 保存 `next_far`。

### test_tmpl_grasp_2d.py

用途：读取 2D 模板，检测图像中目标的 2D 位姿，计算末端修正量，逐步逼近目标并完成抓取。

主要参数：

- `--color_img_topic`：彩色图像话题。
- `--tmpl_dir`：模板目录。
- - `--debug`：开启调试模式后，会在每一步等待确认，并提高匹配调试输出等级。

特点：

- 启动后会读取 `grasp / near / next_near / far / next_far` 模板。
- 每轮流程包括：移动到检测位姿、反复检测并修正、执行抓取、抬升物体。

运行方式：

```bash
bash examples/benchmark/scripts/test_tmpl_grasp_2d.sh
```

### create_tmpl_grasp_3d.py

用途：录制 3D 抓取模板，包括检测位姿、放置位姿、抓取位姿和预备位姿，并在预备位姿下保存目标的 3D 匹配结果。

主要参数：

- `--color_img_topic`：彩色图像话题。
- `--depth_img_topic`：深度图像话题。
- `--tmpl_dir`：模板目录。

输出内容：

- `detect.json`：检测位姿、关节角、夹爪距离。
- `place.json`：放置位姿、关节角、夹爪距离。
- `grasp.json`：抓取位姿和夹爪距离。
- `ready.json`：预备位姿、夹爪距离和 `T_cam_model`。
- 配套保存的 RGB / 深度图。

交互按键：

- `q`：退出。
- `,` / `.`：缩小 / 放大夹爪开口。
- `a`：对齐末端 Z 轴到下方。
- `c`：对齐相机 Z 轴到下方。
- `d`：保存 detect 数据。
- `p`：保存 place 数据。
- `g`：保存 grasp 数据。
- `r`：保存 ready 数据，并执行一次 3D 匹配得到 `T_cam_model`。
- `n`：切换到下一个模板编号计数。

运行方式：

```bash
bash examples/benchmark/scripts/create_tmpl_grasp_3d.sh
```

推荐录制顺序：

1. 将机械臂移动到观察位姿，按 `d` 保存 `detect.json`。
2. 将机械臂移动到放置位姿，按 `p` 保存 `place.json`。
3. 将机械臂移动到实际抓取位姿，按 `g` 保存 `grasp.json`。
4. 将机械臂移动到抓取前的预备位姿，确保画面和深度稳定，按 `r` 保存 `ready.json`。

### test_tmpl_grasp_3d.py

用途：读取 3D 抓取模板和夹爪模型，通过 3D 匹配与跟踪计算预备位姿和抓取位姿，完成完整的 6D 抓取流程。

主要参数：

- `--color_img_topic`：彩色图像话题。
- `--depth_img_topic`：深度图像话题。
- `--tmpl_dir`：模板目录。
- `--debug`：开启调试模式后，会在每一步等待确认，并提高匹配调试输出等级。

依赖文件：

- `data/calib/cam_params.json`
- `data/calib/calib_handeye.json`
- `data/calib/gripper_body.json`
- `tmpl_dir` 下的 `detect.json`、`place.json`、`grasp.json`、`ready.json`

运行方式：

```bash
bash examples/benchmark/scripts/test_tmpl_grasp_3d.sh
```

默认流程：

1. 移动到检测位姿。
2. 通过 3D 匹配定位物体。
3. 计算并移动到预备位姿。
4. 使用 3D 跟踪迭代细化预备位姿。
5. 计算抓取位姿并执行直线抓取。
6. 合拢夹爪、抬升物体、移动到放置位姿并释放。

## 推荐使用步骤

### 基础调试流程

1. 启动机械臂状态发布：

```bash
bash examples/common/scripts/arm_node.sh
```

2. 如需 3D 抓取，先做夹爪标定：

```bash
bash examples/common/scripts/calib_gripper.sh
```

3. 如需录制通用动作模板：

```bash
bash examples/common/scripts/create_tmpl_action.sh
```

4. 验证动作模板回放：

```bash
bash examples/common/scripts/test_tmpl_action.sh
```

### 2D 抓取流程

1. 录制 2D 抓取模板：

```bash
bash examples/benchmark/scripts/create_tmpl_grasp_2d.sh
```

2. 运行 2D 抓取测试：

```bash
bash examples/benchmark/scripts/test_tmpl_grasp_2d.sh
```

### 3D 抓取流程

1. 确保已完成夹爪标定并生成 `data/calib/gripper_body.json`。
2. 录制 3D 抓取模板：

```bash
bash examples/benchmark/scripts/create_tmpl_grasp_3d.sh
```

3. 运行 3D 抓取测试：

```bash
bash examples/benchmark/scripts/test_tmpl_grasp_3d.sh
```

## 补充说明

- 示例脚本里的相机话题默认使用 RealSense D405，请按你的设备修改。
- `data/benchmark/tmpl/` 中已经提供了一套模板结构，可作为录制时的参考。
- 2D / 3D 抓取脚本都依赖手眼标定结果，若抓取位姿明显不对，优先检查 `calib_handeye.json` 和相机话题是否正确。


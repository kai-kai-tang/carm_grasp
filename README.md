# carm_grasp

基于 CARM 机械臂、ROS2 和 RGB-D 相机的抓取示例工程，覆盖机械臂控制、相机与手眼标定、夹爪标定、动作模板录制回放，以及基于 AprilTag 的 2D / 3D 抓取示例。

注意：

- 当前 2D / 3D 抓取示例都假设目标表面可稳定检测到 AprilTag。
- 3D 抓取不仅依赖 RGB-D 图像，还依赖手眼标定和夹爪几何模型。

## TODO

添加视频说明

## 依赖

- [apriltag2](https://github.com/kai-kai-tang/apriltag2)
- [carm](https://pypi.org/project/carm/0.1.20260615/)
- ROS2（代码里使用到 `rclpy`、`cv_bridge`、`message_filters`、`tf2_ros` 等 ROS2 Python 组件）
- Python 依赖：`numpy`、`opencv-python`、`open3d`、`mmengine`、`transforms3d`

示例安装：

```bash
pip install numpy==1.26.4 opencv-python==4.7.0.72 open3d==0.19.0 mmengine transforms3d
pip install carm==0.1.20260615
```

说明：

- `apriltag2` 的安装方式取决于你的环境，请按各自项目说明配置。
- `examples/*/scripts/*.sh` 默认同时写了 Foxy / Humble 的 `source` 语句，实际使用时建议只保留你本机存在的 ROS2 发行版。

## 目录说明

- `core/`：机械臂控制、相机 ROS2 订阅、视觉匹配、几何变换、RViz 可视化等基础封装。
- `examples/common/src/`：基础能力示例，包括状态发布、动作模板录制（`action_record.py`）与回放（`action_play.py`）、自动采集（`auto_collect.py`）、相机标定、手眼标定、夹爪标定。
- `examples/benchmark/src/`：抓取基准示例，包括 2D 抓取模板录制/测试和 3D 抓取模板录制/测试。
- `examples/*/scripts/`：对应 Python 示例的 shell 启动脚本，已预填 ROS 环境和默认参数。
- `demo/`：开箱即用的演示目录，包含预录制的动作模板（`data/action/`）、采集数据样例（`data/collect/`）、标定结果（`data/calib/`）和可直接运行的启动脚本（`scripts/`）。
- `data/calib/`：相机参数、手眼标定结果、夹爪模型等标定文件。
- `data/benchmark/tmpl/`：2D / 3D 抓取模板目录与仓库内置样例。
- `rviz/`：RViz 配置。

## core 模块说明

### arm_wrapper.py

机械臂 SDK 的统一封装，当前对接 `carm`。主要提供：

- 控制模式切换：`POSITION`、`TEACH`、`PF` 等。
- 状态读取：关节角、末端位姿、夹爪开口、外力。
- 运动控制：`set_joints`、`set_pose`、`track_pose`、`set_gripper_dist`。
- 工具函数：位姿矩阵与 `[tx, ty, tz, qx, qy, qz, qw]` 之间的转换，批量逆解。

### arm_utils.py

机械臂与夹爪相关的几何工具：

- `GripperBody`：夹爪几何模型，保存 `T_cam_gripper` 并生成夹爪矩形顶点。
- `CollisionDetector`：基于深度图和夹爪投影的碰撞检测工具。
- `check_arm_pose`：检查末端朝向和夹爪高度是否合理。
- `compute_axis_aligned_pose`：将末端或相机某个轴对齐到基座目标方向。

### arm_ros_utils.py

机械臂状态和目标状态的 ROS2 可视化工具：

- 位姿 / TF / Marker 转 ROS 消息。
- `ArmNode`：发布实际机械臂位姿、关节角、夹爪 Marker。
- `TargetArmNode`：发布目标位姿与目标夹爪 Marker，方便在 RViz 中验证规划结果。

### cam_ros_utils.py

相机 ROS2 工具：

- `CamNode` 可同步订阅多路图像话题。
- 支持按需拉取同步帧 `get_frames()`。
- 支持等待相机内参消息 `get_cam_infos()`。

### vision_utils.py

视觉与定位相关工具：

- RGB-D / 深度图转点云。
- 深度均值滤波 `depth_mean_filter()`。
- AprilTag 2D / 3D 位姿计算。
- `ImageUndistorter`：图像和点去畸变。
- `TagMatcher2D`：识别 AprilTag 并估计平面位姿 `[tx, ty, theta]`。
- `TagMatcher3D`：识别 / 跟踪 AprilTag，并输出 `T_cam_tag`。
- `compute_locate_error()`：评估 6D 定位误差。

### utils.py

通用工具：

- 读取 `cam_params.json`、`calib_handeye.json` 等配置文件。
- 常用坐标变换：`inv_tf()`、`transform_delta_pose()`、`compute_aligned_pose()`。
- 键盘与调试辅助：`wait_key()`、`get_key()`、`KeyboardReader`。

## 运行前准备

1. 确保机械臂可以通过 `carm` 正常连接。
2. 确保 ROS2 环境可用，并且相机相关话题已经发布。
3. 根据实际设备修改 `examples/common/scripts/*.sh` 和 `examples/benchmark/scripts/*.sh` 中的参数，重点检查：
	- `ROS_DOMAIN_ID`
	- 彩色 / 深度图像话题名
	- `pc_frame_id`
	- 模板目录和结果目录
	- `detect_pose` / `place_pose`
4. 根据要运行的脚本准备标定文件：
	- `data/calib/cam_params.json`：2D / 3D 视觉脚本都需要。
	- `data/calib/calib_handeye.json`：`arm_node.py`、2D / 3D 抓取、夹爪标定都需要。
	- `data/calib/gripper_body.json`：`arm_node.py` 发布夹爪 Marker 时需要，3D 抓取测试也需要。
5. 如果你希望快速体验完整流程，可直接使用 `demo/` 目录下的预置数据和脚本：
	- `demo/scripts/` 中提供了所有核心脚本的副本，参数已预配置指向 `demo/data/`。
	- 先运行 `demo/scripts/action_record.sh` 录制采集轨迹，再运行 `demo/scripts/auto_collect.sh` 执行自动采集，最后运行 `calib_handeye.sh` / `calib_camera.sh` 完成标定。

## examples/common/src 说明

### arm_node.py

用途：持续发布机械臂位姿、关节角、夹爪 Marker，以及从 `frame_id` 到 `pc_frame_id` 的 TF，方便在 RViz 中观察状态。

依赖：

- `data/calib/calib_handeye.json`
- `data/calib/gripper_body.json`

主要参数：

- `--frame_id`：机械臂基座坐标系名称。
- `--pc_frame_id`：相机点云坐标系名称。

交互按键：

- `q`：退出。
- `v`：打印当前关节角、末端位姿和夹爪距离。
- `a`：对齐末端 Z 轴到基座 -Z 方向。
- `c`：对齐相机 Z 轴到基座 -Z 方向。
- `,` / `.`：缩小 / 放大夹爪开口。

运行方式：

```bash
bash examples/common/scripts/arm_node.sh
```

### action_record.py

用途：录制非视觉动作模板（又名"动作录制"）。通过拖动或位置控制模式将机械臂移动到目标位姿，按 `s` 保存当前状态为模板。每个模板保存为一个 JSON 文件，包含：

- `T_base_end`
- `joints`
- `gripper_dist`

主要参数：

- `--tmpl_dir`：模板保存目录。

交互按键：

- `q`：退出。
- `z`：切换到位置控制模式。
- `x`：切换到拖动模式。
- `a`：对齐末端 Z 轴到基座 -Z。
- `.`：切换到下一个模板编号。
- `,`：切换到上一个模板编号。
- `e`：执行当前编号对应的模板。
- `s`：保存当前机械臂状态为模板。
- `d`：删除当前模板。

说明：

- 录制的模板可以单独用于动作回放（搭配 `action_play.py`），也可以作为 `auto_collect.py` 的采集轨迹输入。
- `demo/scripts/action_record.sh` 默认将 `tmpl_dir` 指向 `demo/data/action/calib_handeye/`，演示了为手眼标定准备采集轨迹的用法。你也可以将其指向其他目录（如 `demo/data/action/calib_camera/`），用于相机标定或其他采集任务。
- **与 `auto_collect.py` 搭配使用**：先用 `action_record.py` 录制一组覆盖标定板不同视角的机械臂位姿模板，再用 `auto_collect.py` 自动执行这些模板并在每个位姿处采集图像和机械臂数据。这是手眼标定与相机标定数据采集的推荐工作流。

运行方式：

```bash
bash examples/common/scripts/action_record.sh
```

### action_play.py

用途：顺序读取模板目录中的 `0.json`、`1.json`、`2.json`...，循环回放模板里的关节角和夹爪开口（又名"动作回放"）。可单独使用，也可用于验证 `action_record.py` 录制的模板是否正确。

主要参数：

- `--tmpl_dir`：模板目录。
- `--debug`：开启后，每个模板执行前都会等待确认。

行为说明：

- 启动后会先切换到 `PF` 控制模式。
- 每轮回放开始前固定等待一次确认。
- 默认 shell 脚本 `demo/scripts/action_play.sh` 指向 `demo/data/action/calib_handeye/`。

运行方式：

```bash
bash examples/common/scripts/action_play.sh
```

### auto_collect.py

用途：读取 `action_record.py` 录制的一组动作模板，自动依次执行每个模板，并在每个位姿处采集同步图像和机械臂位姿。是手眼标定、相机标定及其他批量数据采集任务的核心自动化工具。

> **典型工作流**：`action_record.py`（录制采集轨迹）→ `auto_collect.py`（自动采集数据）→ `calib_handeye.py` / `calib_camera.py`（执行标定）

主要参数：

- `--tmpl_dir`：动作模板目录（通常由 `action_record.py` 录制生成）。
- `--img_topic_list`：需要采集的图像话题列表，可传入多路图像（如彩色+深度）。
- `--data_dir`：结果保存目录。
- `--debug`：开启后，每个模板执行前等待确认。

输出内容：

- `data_dir/cam0/<idx>.png`、`data_dir/cam1/<idx>.png` ...
- `data_dir/arm_pose.json`

说明：

- 机械臂会先打开夹爪，再切换到 `PF` 控制模式执行模板。
- `arm_pose.json` 中会记录 `eye_in_hand=true`，后续 `calib_handeye.py` 会读取这个信息。
- `demo/scripts/auto_collect.sh` 内置了两套配置（注释切换）：
	- **手眼标定采集**：模板目录指向 `demo/data/action/calib_handeye/`，采集彩色+深度图，结果写入 `demo/data/collect/calib_handeye/`。
	- **相机标定采集**：模板目录指向 `demo/data/action/calib_camera/`，仅采集彩色图，结果写入 `demo/data/collect/calib_camera/`。
- 你也可以用 `action_record.py` 为任意采集任务录制自定义模板，然后交给 `auto_collect.py` 批量执行。

运行方式：

```bash
bash examples/common/scripts/auto_collect.sh
```

### calib_camera.py

用途：读取图像目录中的标定板图片，执行针孔相机标定。

主要参数：

- `--calib_board_info`：标定板信息，格式为 `[tag_size, space_size, tag_rows, tag_cols]`。
- `--img_dir`：图像目录。

输入要求：

- 图像目录中读取 `*.png`。
- 至少需要 10 张有效图像。
- 每张有效图像里至少要能得到足够的 AprilTag 角点用于标定。

输出内容：

- 在 `img_dir` 的父目录下生成 `cam_params.json`。

运行方式：

```bash
bash examples/common/scripts/calib_camera.sh
```

### calib_handeye.py

用途：读取机械臂位姿和对应图像，定位标定板后执行手眼标定。

主要参数：

- `--cam_param_path`：相机内参文件路径。
- `--calib_board_info`：标定板信息，格式为 `[tag_size, space_size, tag_rows, tag_cols]`。
- `--img_dir`：图像目录。
- `--arm_pose_path`：机械臂末端位姿文件路径。

输入要求：

- `arm_pose.json` 与图像文件名编号要对应。
- `arm_pose.json` 里的 `eye_in_hand` 会决定输出 `T_armend_cam` 还是 `T_armbase_cam`。

输出内容：

- 在 `arm_pose.json` 同目录下生成 `calib_handeye.json`。

运行方式：

```bash
bash examples/common/scripts/calib_handeye.sh
```

### calib_gripper.py

用途：根据 AprilTag 平面和深度图估计夹爪在相机坐标系下的位姿，并生成 `data/calib/gripper_body.json`。

依赖：

- `data/calib/cam_params.json`
- `data/calib/calib_handeye.json`

主要参数：

- `--color_img_topic`：彩色图像话题。
- `--depth_img_topic`：深度图像话题。
- `--pc_frame_id`：点云坐标系名称。
- `--gripper_size`：夹爪宽度和厚度，格式为 `[width, thickness]`，单位为米。

交互按键：

- `q`：退出。
- `a`：调整末端姿态，使末端朝下。
- `,` / `.`：缩小 / 放大夹爪开口。
- `t`：采集当前 RGB-D 数据并估计夹爪位姿。
- `s`：保存标定结果到 `data/calib/gripper_body.json`。

说明：

- 当前实现会检测 AprilTag `id=0` 所在平面来估计夹爪坐标系。
- 如果磁盘上已有 `gripper_body.json`，脚本会先加载，再允许覆盖保存。

运行方式：

```bash
bash examples/common/scripts/calib_gripper.sh
```

## examples/benchmark/src 说明

### create_tmpl_grasp_2d.py

用途：录制 2D 抓取模板。脚本保存抓取状态，以及 near / next_near / far / next_far 四组带目标观测的状态，用于后续根据图像中的目标 2D 位姿修正机械臂运动。

依赖：

- `data/calib/cam_params.json`

主要参数：

- `--color_img_topic`：彩色图像话题。
- `--tmpl_dir`：模板目录。

输出内容：

- `grasp/state.json`
- `near/state.json`
- `next_near/state.json`
- `far/state.json`
- `next_far/state.json`
- 每个状态目录下保存 `color.png`
- 对带视觉观测的状态额外保存 `tag.png`

其中：

- `grasp/state.json` 包含 `T_base_end` 和 `gripper_dist`。
- `near / next_near / far / next_far` 还会额外保存 `obj_pose_2d`。

交互按键：

- `a`：调整末端朝向。
- `g`：保存抓取位姿模板。
- `n`：保存 `near`。
- `b`：保存 `next_near`。
- `f`：保存 `far`。
- `d`：保存 `next_far`。

运行方式：

```bash
bash examples/benchmark/scripts/create_tmpl_grasp_2d.sh
```

推荐录制顺序：

1. 将机械臂移动到最终抓取位姿，按 `g` 保存 `grasp`。
2. 抬高到较近观察位姿，且画面中能看到目标，按 `n` 保存 `near`。
3. 在 `near` 位姿基础上做一小段平面内移动，按 `b` 保存 `next_near`。
4. 移动到更远的观察位姿，按 `f` 保存 `far`。
5. 在 `far` 位姿基础上再做一小段平面内移动，按 `d` 保存 `next_far`。

### test_tmpl_grasp_2d.py

用途：读取 2D 模板，检测图像中的目标 2D 位姿，计算末端修正量，逐步逼近目标并完成抓取与放置。

依赖：

- `data/calib/cam_params.json`
- `data/calib/calib_handeye.json`
- `tmpl_dir/grasp/state.json`
- `tmpl_dir/near/state.json`
- `tmpl_dir/next_near/state.json`
- `tmpl_dir/far/state.json`
- `tmpl_dir/next_far/state.json`

主要参数：

- `--color_img_topic`：彩色图像话题。
- `--tmpl_dir`：模板目录。
- `--detect_pose`：检测位姿，格式为 `[tx, ty, tz, qx, qy, qz, qw]`。
- `--place_pose`：放置位姿，格式为 `[tx, ty, tz, qx, qy, qz, qw]`。
- `--debug`：开启后，每一步都会等待确认。

说明：

- `detect_pose` 和 `place_pose` 不是从模板目录读取，而是通过命令行参数传入。
- 你可以直接修改 `examples/benchmark/scripts/test_tmpl_grasp_2d.sh` 里的默认 JSON 字符串，也可以先用 `arm_node.py` 的 `v` 按键打印当前位姿后再填入。

默认流程：

1. 张开夹爪并移动到检测位姿。
2. 反复检测 AprilTag，迭代修正末端位姿。
3. 进入抓取位姿并合拢夹爪。
4. 抬升物体。
5. 移动到放置位姿并释放。

运行方式：

```bash
bash examples/benchmark/scripts/test_tmpl_grasp_2d.sh
```

### create_tmpl_grasp_3d.py

用途：录制 3D 抓取所需模板。当前实现实际会保存抓取位姿和预备位姿，并在预备位姿下执行一次 3D 匹配，记录 `T_cam_model`。

依赖：

- `data/calib/cam_params.json`
- `data/calib/calib_handeye.json`

主要参数：

- `--color_img_topic`：彩色图像话题。
- `--depth_img_topic`：深度图像话题。
- `--tmpl_dir`：模板目录。

输出内容：

- `grasp.json`
- `ready.json`
- `grasp-color.png`
- `grasp-depth.png`
- `ready-color.png`
- `ready-depth.png`

交互按键：

- `q`：退出。
- `,` / `.`：缩小 / 放大夹爪开口。
- `a`：对齐末端 Z 轴到下方。
- `c`：对齐相机 Z 轴到下方。
- `g`：保存抓取位姿和夹爪距离。
- `r`：保存预备位姿、夹爪距离，并执行一次 3D 匹配得到 `T_cam_model`。

说明：

- 检测位姿与放置位姿仍然需要在测试阶段通过 `--detect_pose` 和 `--place_pose` 单独提供。

运行方式：

```bash
bash examples/benchmark/scripts/create_tmpl_grasp_3d.sh
```

推荐录制顺序：

1. 将机械臂移动到实际抓取位姿，按 `g` 保存 `grasp.json`。
2. 将机械臂移动到抓取前的预备位姿，确保画面与深度稳定，按 `r` 保存 `ready.json`。

### test_tmpl_grasp_3d.py

用途：读取 3D 抓取模板和夹爪模型，通过 3D 匹配与跟踪计算预备位姿和抓取位姿，完成完整的 6D 抓取流程。

依赖：

- `data/calib/cam_params.json`
- `data/calib/calib_handeye.json`
- `data/calib/gripper_body.json`
- `tmpl_dir/grasp.json`
- `tmpl_dir/ready.json`

主要参数：

- `--color_img_topic`：彩色图像话题。
- `--depth_img_topic`：深度图像话题。
- `--tmpl_dir`：模板目录。
- `--detect_pose`：检测位姿，格式为 `[tx, ty, tz, qx, qy, qz, qw]`。
- `--place_pose`：放置位姿，格式为 `[tx, ty, tz, qx, qy, qz, qw]`。
- `--debug`：开启后，每一步都会等待确认，并将内部调试等级提升到 3。

默认流程：

1. 打开夹爪并移动到检测位姿。
2. 通过 3D 匹配定位物体。
3. 计算并移动到预备位姿。
4. 进行最多 2 轮 3D 跟踪细化。
5. 计算抓取位姿并执行直线抓取。
6. 合拢夹爪、抬升物体。
7. 移动到放置位姿并释放。

说明：

- `detect_pose` 和 `place_pose` 同样通过命令行参数提供，不从模板目录读取。
- 默认 shell 脚本 `examples/benchmark/scripts/test_tmpl_grasp_3d.sh` 已给出示例值，请按实际工位修改。

运行方式：

```bash
bash examples/benchmark/scripts/test_tmpl_grasp_3d.sh
```

## 推荐使用流程

### 标定与基础调试流程

1. 录制一组采集动作模板（拖动机械臂到多个不同视角，在每个视角按 `s` 保存）：

	```bash
	bash examples/common/scripts/action_record.sh
	```

2. 自动执行这些模板并采集图像 / 位姿：

	```bash
	bash examples/common/scripts/auto_collect.sh
	```

3. 用采集结果标定相机内参：

	```bash
	bash examples/common/scripts/calib_camera.sh
	```

4. 用采集结果标定手眼：

	```bash
	bash examples/common/scripts/calib_handeye.sh
	```

5. 如需 RViz 夹爪显示或 3D 抓取，再做夹爪标定：

	```bash
	bash examples/common/scripts/calib_gripper.sh
	```

6. 用 RViz 检查机械臂状态和 TF：

	```bash
	bash examples/common/scripts/arm_node.sh
	```

7. 如需录制和验证通用动作模板，请让录制脚本与回放脚本指向同一目录后再运行：

	```bash
	bash examples/common/scripts/action_record.sh
	bash examples/common/scripts/action_play.sh
	```

### 2D 抓取流程

1. 录制 2D 抓取模板：

	```bash
	bash examples/benchmark/scripts/create_tmpl_grasp_2d.sh
	```

2. 修改 `examples/benchmark/scripts/test_tmpl_grasp_2d.sh` 中的 `detect_pose` 和 `place_pose`，或手动传参。

3. 运行 2D 抓取测试：

	```bash
	bash examples/benchmark/scripts/test_tmpl_grasp_2d.sh
	```

### 3D 抓取流程

1. 确保已完成相机标定、手眼标定和夹爪标定。
2. 录制 3D 抓取模板：

	```bash
	bash examples/benchmark/scripts/create_tmpl_grasp_3d.sh
	```

3. 修改 `examples/benchmark/scripts/test_tmpl_grasp_3d.sh` 中的 `detect_pose` 和 `place_pose`，或手动传参。

4. 运行 3D 抓取测试：

	```bash
	bash examples/benchmark/scripts/test_tmpl_grasp_3d.sh
	```

## 补充说明

- 示例脚本里的相机话题默认使用 RealSense D405，请按你的设备修改。
- `data/benchmark/tmpl/` 中已经提供了一套模板结构，可作为录制时的参考。
- 2D / 3D 抓取脚本都依赖手眼标定结果；若抓取位姿明显不对，优先检查 `calib_handeye.json`、`cam_params.json` 和话题配置是否正确。
- `arm_node.py` 与 3D 抓取都使用 `gripper_body.json`；仓库内可以放样例文件，但在真实设备上建议重新标定。


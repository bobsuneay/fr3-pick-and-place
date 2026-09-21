# 双臂与 mimic 夹爪：RViz Plan & Execute 使用说明

适用：Ubuntu 22.04、ROS 2 Humble、法奥控制柜 3.9.7，以及随原工程提供的
`fairino_hardware_v3_9_7` / C++ SDK。两只夹爪各自接入对应控制柜。
本版为 `FR3 SDK gripper integration v2`，启动日志会打印此标识和模型安装路径。

## 1. 更新工程

如果已有目录的 origin 是 `https://github.com/bobsuneay/fr3-pick-and-place.git`：

```bash
cd ~/fr3-pick-and-place   # 改为你的实际工程目录
git status --short
git pull --ff-only origin main
```

如果原工程来自 `fr3-standard`、`fr3-sim` 或已有无法合并的本地修改，使用新目录：

```bash
cd ~
git clone https://github.com/bobsuneay/fr3-pick-and-place.git
cd ~/fr3-pick-and-place
```

保留现有 `~/fr3_dual_arm.arms.yaml`、`~/fr3_dual_arm.scene.yaml` 和
`~/fr3_dual_arm.hardware.yaml`，不要用示例覆盖已经校准的配置。
厂商 SDK 不在此仓库中；将原始提供目录里的
`frcobot_ros2-v3.0.0_robotV3.9.7` 放入当前工程的 `third_party/`。
目录中至少应有 `fairino_msgs/`、`fairino_hardware_v3_9_7/`，并保留 SDK 库文件。

## 2. 应用 SDK 补丁并编译

先关闭原来的 ROS 控制程序。在新的终端执行，避免加载旧工作空间：

```bash
source /opt/ros/humble/setup.bash
cd ~/fr3-pick-and-place
sudo apt install python3-colcon-common-extensions python3-yaml patch \
  ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers

vendor="$PWD/third_party/frcobot_ros2-v3.0.0_robotV3.9.7"
bash scripts/apply_fairino_patches.sh "$vendor"

# 限定厂商版本，避免同时发现和构建多个 fairino_hardware 版本。
colcon build --symlink-install --cmake-clean-cache \
  --base-paths src "$vendor/fairino_msgs" "$vendor/fairino_hardware_v3_9_7" \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

ros2 pkg prefix fr3_dual_arm_bringup
ros2 pkg prefix fr3_dual_arm_description
ros2 pkg prefix fairino_hardware_v3_9_7
```

三个路径应指向此次编译的工作空间。编译失败时先解决报错，不要继续启动旧二进制。
补丁脚本先在临时副本验证全部补丁，再备份并更新所需源文件；再次运行不重复修改。
若报 hunk/baseline mismatch，它不会覆盖原文件：请换回所提供的 3.9.7 原始源码，
或提供差异以进一步适配。手工改过、旧版本残缺的夹爪补丁不能直接当作当前版本。

## 3. 现场配置

左右机器人 IP 与夹爪编号以现场硬件 YAML 为准，公开仓库不记录实际内网地址。
保留现场已经核实的编号；编号属于各自控制柜，并不要求左右不同。
`commissioned: true` 应表示现场已经确认配置。

`gripper.open_pos`、`closed_pos` 为 SDK 百分比；按你现有配置分别为 `0`、`100`。
`arms.yaml` 中的 `open_gap: 0.1` 对应主指关节最大位置 `0.05 m`。
映射为：主指 `q=0.05` → `open_pos`；`q=0` → `closed_pos`，中间线性换算。
必须以实测开合方向和行程为准，模型尺寸不能替代标定。

法奥 SDK 的 `block=1` 才是非阻塞。本版自动将旧 YAML 中的 `block: 0`
转换为运行时的 `1`，并打印提示，以避免夹爪动作阻塞同侧机械臂循环。
速度与力使用 YAML 的 `vel` / `force` 百分比，RViz 的速度缩放不改变 SDK 夹爪速度。
本版不提供抓取力检测或“夹住物体即成功”的判定。

## 4. 先检查反馈

```bash
ros2 launch fr3_dual_arm_bringup real.launch.py \
  hardware:=$HOME/fr3_dual_arm.hardware.yaml \
  arms:=$HOME/fr3_dual_arm.arms.yaml \
  scene:=$HOME/fr3_dual_arm.scene.yaml \
  enable_execution:=false
```

本版先启动两侧控制管理器和控制器，再检查最近一秒内的 14 个有限位置反馈，
通过后才启动 MoveIt / RViz。缺失夹爪时会打印具体关节名并退出，不再让规划界面
继续使用不完整状态。ROS 消息新鲜度检查不能检测 SDK 内部缓存失效。

`enable_execution:=false` 只禁用运动控制器执行：SDK 硬件仍初始化、夹爪会激活，
机械臂驱动可能发送 ServoJ 保持当前位置。它不是只读模式；ROS Stop 不代替急停。

另开终端并 source 本次 `install/setup.bash`：

```bash
ros2 control list_controllers -c /left_controller_manager
ros2 control list_controllers -c /right_controller_manager
ros2 control list_hardware_interfaces -c /left_controller_manager
ros2 control list_hardware_interfaces -c /right_controller_manager
ros2 topic echo /joint_states
```

每侧 broadcaster 为 active，arm/gripper controller 在此模式为 inactive。
两侧 `/joint_states` 消息可以分开发送，但合计必须包含 12 个 `*_j1..6`
和 `left_left_finger_joint`、`right_left_finger_joint` 的有限 position。
从指是 mimic，不需要独立反馈。机械臂未导出的 velocity/effort 为 NaN 不等于 position 无效。

## 5. 在 RViz 操作真机

退出上一启动程序，确认工作区域可执行后重新运行：

```bash
ros2 launch fr3_dual_arm_bringup real.launch.py \
  hardware:=$HOME/fr3_dual_arm.hardware.yaml \
  arms:=$HOME/fr3_dual_arm.arms.yaml \
  scene:=$HOME/fr3_dual_arm.scene.yaml \
  enable_execution:=true
```

此时启动检查还要求四个 action server 均可连接：

| 规划组 | 执行端点 |
| --- | --- |
| `left_arm` | `/left_arm_controller/follow_joint_trajectory` |
| `right_arm` | `/right_arm_controller/follow_joint_trajectory` |
| `both_arms` | 上述两个机械臂 action |
| `left_gripper` | `/left_gripper_controller/gripper_cmd` |
| `right_gripper` | `/right_gripper_controller/gripper_cmd` |

在 MotionPlanning 面板选择组，Start State 选择当前状态。机械臂设置小范围目标，
先 Plan 检查轨迹，再 Execute。夹爪选择 `open` 或 `closed` 后 Plan & Execute。
也可在关节目标中设置主指的中间位置；另一手指自动跟随。
`both_arms` 不包含夹爪：机械臂运动与夹爪开合分别选组执行，不支持四者一次同步抓取。

验收命令：

```bash
ros2 param get /move_group moveit_simple_controller_manager.controller_names
ros2 action list -t
```

MoveIt 应列出 **4 个**控制器，而不是日志中的 2 个；夹爪 action 类型为
`control_msgs/action/GripperCommand`，后缀是 **gripper_cmd**，不是 `command`。

## 6. 错误定位

- 仍看到只有 2 个控制器 / 缺两个主指：核对启动日志的 v2 标识和上面三个包的安装路径；
  退出旧进程，在新终端只 source `/opt/ros/humble` 和本次 workspace。
- 插件检查失败：确认补丁、厂商编译结果，以及只加载一个 `fairino_hardware_v3_9_7`。
- SDK `ActGripper` / `GetGripperCurPosition` 失败：核对控制柜的夹爪配置、编号和故障信息。
  本版会拒绝初始读数失败，不伪造打开位置。
- `Goal constraints are already satisfied`：目标与当前规划状态相同，此次不会下发动作；
  用不同的目标验证，不能把这条消息当作实物运动成功。
- 夹爪未到目标即停滞：action 返回失败；当前默认 `allow_stalling: false`，
  避免堵转被误报为成功。夹住工件的确认需要额外力/状态反馈逻辑。

## 7. 本次验证范围

已在 Windows 离线运行：模型/控制器/反馈检查回归、实际补丁应用与重复安装、
不兼容源码不修改测试、修改后的 C++ 源码编译与假 SDK 故障测试、真实 SDK 头文件签名检查。
未连接机器人，未执行完整 Ubuntu ROS2 构建或真实 RViz / SDK 动作验收。

```bash
python3 -m unittest discover -s tests -p test_real_contract.py -v
python3 tests/shared_rpc/run.py \
  --vendor third_party/frcobot_ros2-v3.0.0_robotV3.9.7/fairino_hardware_v3_9_7 \
  --bash bash --real-sdk-headers
```

接口核对：[ROS2 Humble 夹爪控制器源码](https://github.com/ros-controls/ros2_controllers/blob/humble/gripper_controllers/include/gripper_controllers/gripper_action_controller_impl.hpp)。
SDK 的阻塞标志与参数签名以随工程提供的 `libfairino/include/robot.h` 为准。

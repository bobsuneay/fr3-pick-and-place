# 架构说明

## 仿真与实机差异

| 模块 | 仿真 | 实机 |
| --- | --- | --- |
| 机器人模型 | `fr3_dual_arm_description` | 同一套模型 |
| MoveIt 配置 | `fr3_dual_arm_moveit_config` | 同一套配置 |
| 硬件接口 | `gazebo_ros2_control/GazeboSystem` | `fairino_hardware/FairinoHardwareInterface`（6 轴）+ `fairino_hardware/FairinoGripperHardwareInterface`（夹爪，待实现） |
| 控制器 | Gazebo 单 controller_manager | 左/右两个独立 controller_manager |
| 抓取应用 | `fr3_dual_arm_grasp` | 同一套上层逻辑 |
| 手眼标定 | 不需要 | `fr3_dual_arm_calibration` |

## 关键规划组

- `left_arm`
- `right_arm`
- `both_arms`
- `left_gripper`
- `right_gripper`

## 关键 action / topic

- `/<side>_arm_controller/follow_joint_trajectory`
- `/<side>_gripper_controller/command`
- `/joint_states`
- `/head_camera/points`
- `/remote_cmd_interface`（法奥字符串指令服务，含夹爪 `MoveGripper`）

## 后端切换原则

`fr3_dual_arm_bringup` 只负责选择硬件插件和控制器参数。上层 `fr3_dual_arm_grasp` 不写死 Gazebo 或厂商 SDK。

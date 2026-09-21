# 从 fr3_bolt_inspection_cell 迁移映射

## 参数来源

以下参数直接来自 `fr3-sim5/sim_ws/src/fr3_bolt_inspection_cell`：

| 新项目位置 | 原位置 |
| --- | --- |
| `fr3_dual_arm_description/config/arms.yaml` | `fr3_bolt_inspection_cell/config/arms.yaml` |
| `fr3_dual_arm_description/config/scene.yaml` | `fr3_bolt_inspection_cell/config/scene.yaml` |
| `fr3_dual_arm_grasp/config/grasp.yaml` | `fr3_bolt_inspection_cell/config/inspection.yaml` |

## 代码迁移

| 新包 | 需要从原包迁移/提炼的内容 |
| --- | --- |
| `fr3_dual_arm_description` | FR3/HKV 网格、`fr3_arm.urdf`、双臂装配脚本；移除动态 Python 生成模型、场景、SRDF 的耦合 |
| `fr3_dual_arm_moveit_config` | 由原 `fr3_dual_bolt_cell.model.moveit_config()` 生成的 SRDF、运动学、控制器配置静态化 |
| `fr3_dual_arm_gazebo` | `world.py`、`inspection_world()`、`model.augment()` 中的相机和夹爪稳定逻辑 |
| `fr3_dual_arm_hardware` | 法奥 `fairino_hardware_v3_9_7` 的 6 轴插件复用，以及基于其夹爪 SDK（`ActGripper`/`MoveGripper`）的 `FairinoGripperHardwareInterface` 适配 |
| `fr3_dual_arm_bringup` | `bringup.launch.py` 的启动顺序，但拆成 `sim/mock/real` 三个入口 |
| `fr3_dual_arm_grasp` | `core.py`、`geometry.py`、`cartesian.py`、`handover.py`、`feedback.py`、`ros_io.py`、`task_node.py` 中的业务算法 |
| `fr3_dual_arm_calibration` | 新增，承接实机相机手眼标定 |

## 迁移顺序

1. 先把模型和 MoveIt 配置静态化，让 RViz 能显示双臂模型。
2. 再把 Gazebo world 和 `gazebo_ros2_control` 控制器跑通。
3. 迁移点云识别与 MoveIt 接口，先跑通右手定点抓取。
4. 接入法奥厂商驱动和机械臂挂载夹爪（`gripper_index` + SDK），先反馈后执行。
5. 最后迁移检测视角、双臂交接和 UI。

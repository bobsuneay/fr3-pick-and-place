# fr3_dual_arm_description

存放双臂 FR3 + HKV 夹爪的机械模型、网格和真实场景参数。

`fr3_arm.urdf` 来自法奥 FR3 单臂模型，网格文件已经复制到本包，并把原 `package://fr3_dual_bolt_cell/...` 引用改为本包。

## 后续工作

双臂装配由 `fr3_dual_arm_description/model.py` 完成，负责：

1. 读取 `arms.yaml` 和 `scene.yaml`。
2. 克隆 `fr3_arm.urdf`，给关节和连杆加 `left_` / `right_` 前缀。
3. 添加左右安装板、HKV 夹爪、世界支撑和相机外参。
4. 生成 `gazebo` / `mock` / `real` 三种 ros2_control 接口，以及 SRDF、运动学和 MoveIt 控制器参数。

不要把原 `fr3_dual_bolt_cell` 的动态模型、MoveIt、Gazebo 逻辑继续堆在这里。

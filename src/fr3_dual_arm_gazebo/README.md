# fr3_dual_arm_gazebo

只保留 Gazebo 仿真需要的 world、控制器和启动文件。真实相机驱动、夹爪硬件接口、业务逻辑不放在这里。

后续从 `fr3_bolt_inspection_cell` 迁移：

- `world.py` 中桌面、螺丝、相机传感器的 world 生成
- `model.augment()` 中夹爪接触稳定和腕部相机几何
- `linked_controllers()` 的 Gazebo 控制器参数

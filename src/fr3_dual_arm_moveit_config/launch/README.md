# MoveIt launch 说明

正式 `demo.launch.py` 需要在 `fr3_dual_arm_description` 生成完整双臂 URDF/xacro 后启用。

建议使用 `moveit_configs_utils.MoveItConfigsBuilder`，参数：

```python
MoveItConfigsBuilder(
    "fr3_dual_arm",
    package_name="fr3_dual_arm_moveit_config",
).to_moveit_configs()
```

该构建器期望：

- `config/fr3_dual_arm.srdf`
- `config/fr3_dual_arm.urdf.xacro`
- `config/kinematics.yaml`
- `config/joint_limits.yaml`
- `config/ros2_controllers.yaml`
- `config/moveit_controllers.yaml`

当前目录已放入配置，待补齐描述包的 `fr3_dual_arm.urdf.xacro`。

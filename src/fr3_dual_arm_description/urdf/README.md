# URDF 说明

`fr3_arm.urdf` 是未加前缀的单臂模型。双臂装配时应由 `fr3_dual_arm_description` 自己的 `model.py` 生成，避免直接依赖 `fr3_dual_bolt_cell` 的运行时 Python 代码。

需要生成的核心命名：

```text
left_j1 ... left_j6
right_j1 ... right_j6
left_left_finger_joint
left_right_finger_joint
right_left_finger_joint
right_right_finger_joint
```

网格文件已经位于 `meshes/fairino3_v6` 和 `meshes/hkv_tg9801`。

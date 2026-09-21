# fr3_dual_arm_calibration

仿真中相机外参由模型直接给出，实机必须标定：

- 相机内参
- 相机到机械臂基座的 hand-eye 变换
- 头部 RGB-D 到 `world` 或 `support_link` 的外参
- 两个腕部 D435i 的外参

可使用 `easy_handeye` 或法奥/相机厂商提供的标定流程。

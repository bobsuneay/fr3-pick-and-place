# 法奥 SDK 驱动补丁

使用用户提供的 `frcobot_ros2-v3.0.0_robotV3.9.7/fairino_hardware_v3_9_7` 和 `fairino_msgs`。
厂商 SDK 二进制不随本仓库发布，许可证保持上游声明。

从项目根目录运行：

```bash
bash scripts/apply_fairino_patches.sh third_party/frcobot_ros2-v3.0.0_robotV3.9.7
```

脚本顺序应用双 IP、夹爪接口和共享连接三个补丁，并验证完整结果。
同侧机械臂和夹爪在一个 controller_manager 进程内共享一个 FRRobot；左右两侧使用不同进程。
`ActGripper` 初始化夹爪；`GetGripperCurPosition` 提供实测位置；
`MoveGripper(..., block=1, ...)` 非阻塞下发目标。主指位置单位为米，SDK 位置为百分比。
首次读数失败会拒绝激活；控制循环中读取失败会返回硬件错误。

脚本先验证临时副本，再备份原源文件，重复执行不改变文件；不兼容版本会退出。
不要手工交叉叠加旧补丁。实际编译及使用见 [使用说明](../docs/REAL_MOVEIT_USAGE.md)。

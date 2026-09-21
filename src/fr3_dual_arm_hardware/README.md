# fr3_dual_arm_hardware

此包保留原工程的硬件接口骨架，真实启动不加载这个占位插件。
`fr3_dual_arm_bringup real.launch.py` 加载修补后的厂商 `FairinoHardwareInterface`
和 `FairinoGripperHardwareInterface`：每侧独立进程，臂和夹爪共享同侧 SDK 连接。
每个夹爪只导出一个主指位置命令及位置/速度反馈，另一指由 URDF mimic 跟随。

更新和编译必须同时包含厂商驱动与描述/启动包，详见 [使用说明](../../docs/REAL_MOVEIT_USAGE.md)。

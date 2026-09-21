# fr3_dual_arm_bringup

统一启动入口：

```bash
ros2 launch fr3_dual_arm_bringup sim.launch.py enable_execution:=true
ros2 launch fr3_dual_arm_bringup mock.launch.py enable_execution:=true
ros2 launch fr3_dual_arm_bringup real.launch.py hardware:=$HOME/fr3_dual_arm.hardware.yaml enable_execution:=false
```

三个入口只改变硬件插件、控制器参数和是否启动 Gazebo。

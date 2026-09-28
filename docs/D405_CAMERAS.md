# 双腕 RealSense D405：模型与 ROS 2 接口

默认 `scene.yaml` 已恢复左右腕部相机，以 `fr3-sim/sim_ws/src/fr3_bolt_inspection_cell/config/inspection.yaml` 的夹爪安装外参为起点，将左右相机及支架沿夹爪掌部 +Z 方向下移 40 mm，避免末端关节旋转时靠近后方连杆。相机连接到各自的 `gripper_palm`，随手臂运动。头部相机保留原配置，腰部相机仅在 `scene.full_cameras.yaml` 启用。

外壳采用简化盒体（相机 +X 朝前，XYZ 为 23 × 42 × 42 mm），质量 60 g；Gazebo 使用 1280×720、30 Hz、水平视场角 87°、裁剪距离 0.07–0.50 m。依据 [D405 官方规格](https://www.intel.com/content/www/us/en/products/sku/229218/intel-realsense-depth-camera-d405/specifications.html)。这是理想深度仿真，不模拟双目噪声、遮挡失配或真实镜头畸变。原 D435i 安装姿态作为初始值，部署时在 scene 中填写实际测量/手眼标定外参。

## 统一话题

下面的 `{camera}` 为 `left_d405` 或 `right_d405`。订阅使用 `sensor_data` QoS（Best Effort、Volatile），按消息 `header.stamp` 同步，按 `header.frame_id` 查询 TF。

| 话题 | ROS 2 类型 | 用途 |
| --- | --- | --- |
| `/{camera}/image_raw` | `sensor_msgs/msg/Image` | 彩色图像 |
| `/{camera}/camera_info` | `sensor_msgs/msg/CameraInfo` | 彩色内参与标定 |
| `/{camera}/depth/image_raw` | `sensor_msgs/msg/Image` | 原始深度 |
| `/{camera}/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 深度内参 |
| `/{camera}/points` | `sensor_msgs/msg/PointCloud2` | 深度点云 |

实机还提供 `/{camera}/aligned_depth/image_raw` 和 `/{camera}/aligned_depth/camera_info`，深度对齐到彩色坐标。Gazebo 的彩色和深度共用理想光学模型，可直接使用原始深度，不额外发布 aligned_depth 别名。

Gazebo 深度通常为 `32FC1`（米），RealSense 为 `16UC1`（毫米）；消费者必须按 `encoding` 转换，不可将同名话题理解为相同字节格式。无效像素按对应格式处理（0、NaN、Inf）。实机 `clip_distance=0.5` 限制远端，应用按需要过滤 <0.07 m 的结果；内参与畸变以实机 CameraInfo 为准。

## TF 所有权

`robot_state_publisher` 发布 `gripper_palm → {camera}_bracket → {camera}_link` 及仿真 `{camera}_optical_frame`。实机驱动设置 `camera_name={camera}`、`base_frame_id=link`，从同一个 `{camera}_link` 发布内部彩色/深度帧和工厂标定，不覆盖腕部安装关节。实机图像一般使用 `{camera}_color_optical_frame`，深度/点云使用 `{camera}_depth_optical_frame`；不要将实机消息硬改为仿真光学帧。

## 构建和仿真

```bash
cd /home/suneasy/fr3-pick-and-place
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fr3_dual_arm_description fr3_dual_arm_bringup fr3_dual_arm_calibration fr3_dual_arm_grasp
source install/setup.bash
ros2 launch fr3_dual_arm_bringup pick_place.launch.py mode:=gazebo
```

Gazebo 在加载机器人时创建相机插件，无需启动 USB 驱动。`mode:=mock` 仅提供模型与 TF，不生成相机图像。

## 实机相机

安装与 ROS Humble 和设备固件匹配的 `realsense2_camera` / librealsense（相机包需要支持 D405 的 `depth_module.color_profile`），参考 [官方 ROS 2 驱动](https://github.com/realsenseai/realsense-ros)。使用 `rs-enumerate-devices` 获取序列号，填入 `fr3_dual_arm_bringup/config/cameras.yaml` 或用启动参数覆盖。左右必须使用不同序列号；空序列号会报错，避免自动选择导致左右交换。

在机器人模型已启动的终端环境中，单独启用相机：

```bash
ros2 launch fr3_dual_arm_bringup cameras.launch.py mode:=real \
  left_serial:=左相机序列号 right_serial:=右相机序列号
```

此入口只启动相机，不连接或驱动机械臂。需要世界坐标 TF 时应同时运行现有 mock/real 机器人 bringup。若机器人使用自定义 scene，相机入口也传 `scene:=同一文件路径`。

与实机示教统一启动：

```bash
ros2 launch fr3_dual_arm_bringup pick_place.launch.py mode:=real \
  hardware:=/绝对路径/fr3_dual_arm.hardware.yaml \
  wrist_cameras:=true left_serial:=左相机序列号 right_serial:=右相机序列号
```

`wrist_cameras` 默认 false；相机设备缺失不影响原示教启动方式。单相机部署可在 `camera_config:=/绝对路径/cameras.yaml` 中禁用另一侧。

检查连接与数据：

```bash
ros2 topic hz /left_d405/image_raw
ros2 topic echo /left_d405/camera_info --once --qos-reliability best_effort
ros2 topic hz /right_d405/depth/image_raw
ros2 run tf2_ros tf2_echo world left_d405_depth_optical_frame
```

工程自带的 RViz 配置已经加入 `Left Wrist RGB`、`Right Wrist RGB` 和
`Right Wrist PointCloud`，启动后自动订阅上述统一话题。左右深度图也已预置为
`Left/Right Wrist Depth`，默认关闭以降低渲染负载，需要时在 Displays 中勾选；
Fixed Frame 保持 `world`。当前开发环境未安装 RealSense 驱动，代码与模型通过
离线验证；双 USB 出流、真实内参和实际 TF 外参仍需接设备验收。

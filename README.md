# FR3 双臂示教抓取展示 Demo

基于 `fr3-standard3ok` 的 MoveIt 2 实机工程，复现 `fr3-sim5` 的点云抓取、展示、交接流程，并增加最终放置。抓取目标由右腕 D405 实时识别，展示和交接仍沿用原流程。

**完整安装、示教与实机操作：[PICK_PLACE_DEMO.md](docs/PICK_PLACE_DEMO.md)**

- 原生中文界面：双臂关节角、TCP、夹爪开度；仅采集 `ready`、`left_place` 两个点；抓取位姿由右腕点云相机识别。
- 关节、TCP、MoveL、夹爪运动经 MoveIt 规划；可先预览再执行。
- 右手抓取 → 展示（关节 6 转 -180°、X 轴 ±30°、底部正对相机）→ 左手接取 → 左手同姿态展示 → 放下。
- 包含双臂、夹爪、支架和桌面碰撞检测；抓取和交接使用夹爪实际开度自动确认。
- 抓取分“转竖直 → 直线下降”两步；展示在光轴前 0.30 m 处让关节 6 转 180° 再补 X 倾斜和底部正对视角；IK 无解时可点“跳过本步”继续。
- 保留已校准的双臂/夹爪/support 参数；默认启用头部相机和双腕 RealSense D405；完整配置另含腰部相机。相机接口见 [D405 部署说明](docs/D405_CAMERAS.md)。
- 默认禁止执行。没有预填实机运动点，必须现场示教；零件和桌面参数需核对。

Ubuntu 22.04 / ROS 2 Humble 编译后：

```bash
# mock 界面与规划
ros2 launch fr3_dual_arm_bringup pick_place.launch.py mode:=mock

# mock 执行调试
ros2 launch fr3_dual_arm_bringup pick_place.launch.py mode:=mock enable_execution:=true

# 实机先读取反馈 / 规划
ros2 launch fr3_dual_arm_bringup pick_place.launch.py \
  mode:=real hardware:=$HOME/fr3_dual_arm.hardware.yaml enable_execution:=false
```

完整 demo 前需采集 `ready`、`left_place` 两点，并核对 35×16 mm 圆柱、两侧分格盒、D405 视野及点云 ROI；右盒默认 3×4 格并全部装件。抓取 TCP 与接近路径完全由右腕点云检测右盒圆柱后生成。实机仍使用基础工程的厂商 SDK 与补丁，SDK 不随 Git 仓库上传，需从原工程复制。

本次通过 Python 编译检查和离线测试（含 Tk 界面构造、流程失败/取消、标定与相机回归）。**尚未进行 ROS 2 在线运行及实机验收**，步骤见完整说明。

- [源码来源与保留内容](docs/SOURCE_PROVENANCE.md)
- [底层驱动与夹爪说明](docs/REAL_MOVEIT_USAGE.md)
- [厂商 SDK 补丁](third_party/README.md)

`docs/ARCHITECTURE.md` / `PORTING_MAP.md` 等是基础工程的历史迁移资料；当前 demo 功能和操作以本 README 与 PICK_PLACE_DEMO.md 为准。

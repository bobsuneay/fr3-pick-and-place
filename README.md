# FR3 双臂示教抓取展示 Demo

基于 `fr3-standard3ok` 的 MoveIt 2 实机工程，复现 `fr3-sim5` 的抓取、展示、交接流程，并增加最终放置。使用示教点替代点云抓取位姿，保留后续相机升级接口。

**完整安装、示教与实机操作：[PICK_PLACE_DEMO.md](docs/PICK_PLACE_DEMO.md)**

- 原生中文界面：双臂关节角、TCP、夹爪开度、16 个关键点采集与保存。
- 关节、TCP、MoveL、夹爪运动经 MoveIt 规划；可先预览再执行。
- 右手抓取 → 多角度展示 → 左手接取 → 多角度展示 → 放下。
- 包含双臂、夹爪、支架、桌面和被抓零件的模型碰撞检测；交接前确认左手夹稳。
- 保留已校准的双臂/夹爪/support 参数；默认仅头部相机，完整相机配置有备份。
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

完整 demo 前需完成全部关键点，核对桌面、零件包围盒和 TCP 偏移，并配置 `demo.yaml`。实机仍使用基础工程的厂商 SDK 与补丁，SDK 不随 Git 仓库上传，需从原工程复制。

本次通过 Python 编译检查和离线测试（含 Tk 界面构造、流程失败/取消、标定与相机回归）。**尚未进行 ROS 2 在线运行及实机验收**，步骤见完整说明。

- [源码来源与保留内容](docs/SOURCE_PROVENANCE.md)
- [底层驱动与夹爪说明](docs/REAL_MOVEIT_USAGE.md)
- [厂商 SDK 补丁](third_party/README.md)

`docs/ARCHITECTURE.md` / `PORTING_MAP.md` 等是基础工程的历史迁移资料；当前 demo 功能和操作以本 README 与 PICK_PLACE_DEMO.md 为准。

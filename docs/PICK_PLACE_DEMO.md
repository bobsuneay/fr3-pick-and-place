# 双臂示教抓取展示 Demo

本版以用户提供的 `fr3-standard3ok/fr3-standard` 为基础，保留其双臂、HKV 夹爪、TCP、support 安装参数和实机控制器补丁。`fr3-sim5` 的“右手抓取→右手展示→左手接取→左手展示”改为示教点驱动，并增加左手放置和撤离。原始目录未修改。

## 1. 已实现与验证范围

- Tk 原生界面显示双臂实测关节角、夹爪开口毫米和闭合百分比；读取 TCP、采集/覆盖/打开/保存示教点。
- 单臂关节目标、TCP 位姿目标、MoveL 直线、夹爪目标均使用 MoveIt。预览与执行分开，启动默认禁止执行。
- 16 个关键点同时保存双臂关节、世界坐标 TCP、夹爪目标开度。展示采用初始姿态和两个不同展示角度，可自行示教。
- 执行时逐段规划。双臂交接预备点使用 `both_arms`；其他段使用对应单臂组，另一臂仍参与碰撞检测。接近、展示和撤离默认是关节空间规划，不保证 TCP 走直线；界面的 MoveL 可用于单独调试直线段。
- 将桌面、桌腿加入规划场景。零件用可配置包围盒表示，随抓取附着到右手、交接后转移到左手、放下后恢复为环境障碍物。
- 仅为当前抓取/交接的手指与零件放开必要接触；不关闭双臂、夹爪之间、支架或其他环境的碰撞检测。零件与桌面接触仅在取件前和放置阶段允许。
- 反馈缺失/过期、规划失败、轨迹不完整、执行错误、目标未到位、取消都会中止后续步骤。停止不主动松开夹爪。
- 保留 `perception.py`，新增 `/grasp/perception/grasp_tcp` 位姿候选接口；当前无需相机。

本次在 Windows 进行了 Python 编译检查、离线行为测试、模型回归和 Tk 界面构造测试。**没有在本机执行 ROS 2 编译、MoveIt 在线规划、厂商 SDK 联调或实机运动验收。** mock 模式只模拟反馈与执行，不模拟真实夹持、摩擦或掉件。

## 2. 安装与编译（Ubuntu 22.04 / ROS 2 Humble）

使用新终端，不要 source 旧工程的 install：

```bash
git clone https://github.com/bobsuneay/fr3-pick-and-place.git
cd fr3-pick-and-place
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install python3-tk python3-numpy python3-scipy python3-yaml \
  python3-colcon-common-extensions python3-rosdep python3-pytest patch \
  ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src
source install/setup.bash
```

以上可用于 mock。实机需要把原基础工程的 `third_party/frcobot_ros2-v3.0.0_robotV3.9.7` 复制到新仓库的 `third_party/`，包括 SDK 库。与基础仓库一致，厂商 SDK 不上传；本地交付目录中保留了用户原有副本。

```bash
vendor="$PWD/third_party/frcobot_ros2-v3.0.0_robotV3.9.7"
bash scripts/apply_fairino_patches.sh "$vendor"
colcon build --symlink-install --cmake-clean-cache \
  --base-paths src "$vendor/fairino_msgs" "$vendor/fairino_hardware_v3_9_7" \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

实机沿用两个独立 controller_manager，每侧 arm/gripper 共用 SDK 连接。不要同时运行旧 ROS 工程或 `fr-sdk3` 直接控制同一机器人。

## 3. 首次打开界面

```bash
# mock：可先采集和预览，不执行
ros2 launch fr3_dual_arm_bringup pick_place.launch.py mode:=mock

# mock：验证运动、夹爪和示教界面操作
ros2 launch fr3_dual_arm_bringup pick_place.launch.py \
  mode:=mock enable_execution:=true
```

实机先读取反馈、检查模型和 TCP：

```bash
ros2 launch fr3_dual_arm_bringup pick_place.launch.py \
  mode:=real hardware:=$HOME/fr3_dual_arm.hardware.yaml \
  enable_execution:=false
```

本地交付目录根部保留了用户给定的 `fr3_dual_arm.hardware.yaml`，该文件被 Git 忽略。现场左右 IP 与夹爪编号以该本地文件为准，公开仓库不记录实际内网地址。复制到 Ubuntu 后使用实际路径；不要改回旧说明中的编号。

当现场已核对左右臂方向、反馈、TCP 和环境后，关闭上次 launch，再以 `enable_execution:=true speed:=0.1` 启动。ROS 控制器和 demo 节点共同使用该开关。界面不会修改机器人使能状态或切换拖动示教模式。

如果已有当前版本 MoveIt bringup，可以单独运行：

```bash
ros2 launch fr3_dual_arm_grasp grasp.launch.py enable_execution:=false
```

此时 `arms_file`、`scene_file` 必须与正在运行的 bringup 完全相同。一个机器人工作空间只能运行一个 demo 后端。

## 4. 坐标、单位与夹爪

界面输入：关节为度，位置为毫米，姿态为固定轴 XYZ 的 Rx/Ry/Rz（度）。内部 JSON：关节 rad、位置 m、四元数 `[qx,qy,qz,qw]`，夹爪完整开口 `gap_m`。

每次采集聚合左右侧 `/joint_states`，检查全部 14 个独立关节的每项反馈在 1 秒内更新，并对同一快照调用 MoveIt `/compute_fk` 得到 `world → left/right_gripper_tcp`。采集过程中机器人移动则拒绝保存。mimic 从指由实测主指推导，不独立命令。

**不是直接抄 SDK 的工具号 1 的 TCP 数字。** 模型 TCP 由 `arms.yaml` 的夹爪安装与 TCP 偏移定义；SDK 工具坐标系或机器人基坐标下的数字不能直接粘贴到 world 输入框。优先用机器人示教器定位后点击“采集”，或者用本界面的 MoveIt 运动定位。

当前校准保留 `open_gap=0.1 m`、单指行程 `0.05 m`。闭合百分比 = `100 × (1-gap/open_gap)`：0% 全开，100% 全闭。SDK 配置沿用 `open_pos=0, closed_pos=100`。

实际夹住零件时不要强令完全闭合。为 `right_grasp` / `left_receive` 保存现场测得的夹持开度，必要时用“只保存开度”修正目标。该按钮只改示教文件，不运动。底层夹爪默认不把堵转视为成功；闭合目标无法达到时流程会停下，不能通过取消碰撞检测解决。

## 5. 采集 16 个点

1. 先采集 `ready`，双臂处于可用的空手就绪状态。
2. 右手：`right_pregrasp`（张开并位于零件上方/接近处）、`right_grasp`（抓取姿态和夹持目标开度）、`right_lift`（抬升后）。
3. 右手展示：`right_display` 是初始展示姿态，`right_view_1/2` 为两个角度。运行后会回到初始展示姿态。
4. `handover_ready` 同时记录右手交接姿态和左手尚未接触的接近姿态。
5. 保持右手不动，只定位左手到 `left_receive`，设置左手夹持开度。程序检查这两点中右臂关节差不超过 0.02 rad。
6. `right_retreat` 是交接后右手撤离位置。
7. 左手展示：`left_display`、`left_view_1/2`。
8. 左手放置：`left_preplace`（接近）、`left_place`（零件处于目标支撑处）、`left_retreat`（撤离并张开）。

每个槽都保存双臂信息，但执行时只使用流程规定的运动组。夹爪只在明确的抓取、接取和释放阶段动作，不会在每个展示点自动改变开度。右手释放使用 `right_pregrasp` 的张开值；左手初始张开和最终释放使用 `left_retreat` 的张开值。

默认自动保存到 `~/.ros/fr3_demo/teach_points.json`。文件包含 arms/scene 配置内容哈希，不允许跨不同安装标定或相机场景直接回放。旧 SDK JSON 的 mm/degree 与新 schema 不兼容，需重新采集。载入失败时不会悄悄覆盖旧文件；选择“另存为”以创建新文件。

## 6. 工作台和零件包围盒

`scene.yaml` 保留原基础工程的桌面示例；**双臂和支架已标定不代表桌面和零件也已核对。** 默认桌面中心 `[0.48, 0] m`、尺寸 `0.7 × 0.65 m`、台面高 `0.75 m`。按现场更新副本并通过 `scene:=...` 传入。

复制 `src/fr3_dual_arm_grasp/config/demo.yaml` 到用户配置目录：

```bash
mkdir -p ~/.ros/fr3_demo
cp src/fr3_dual_arm_grasp/config/demo.yaml ~/.ros/fr3_demo/demo.yaml
```

填写实际 `workpiece.dimensions_m` 与抓取时的 `right_tcp_to_object`（TCP 到零件包围盒中心的变换），核对后设置 `scene_and_object_verified: true`。默认值仅为示例，完整 demo 默认拒绝运行。再启动：

```bash
ros2 launch fr3_dual_arm_bringup pick_place.launch.py \
  mode:=real hardware:=$HOME/fr3_dual_arm.hardware.yaml \
  demo_config:=$HOME/.ros/fr3_demo/demo.yaml \
  enable_execution:=true speed:=0.1
```

点击“检查示教点完整性”，逐点规划并检查 RViz。预览只验证当前状态到一个目标的路径，不等于整条流程已验收。完整流程每段都重新规划，并保留当前零件附着关系。

流程在右手抓取、左手接取和最终放置各等待一次人工确认。尤其在左手确认前右手不松开；当前没有视觉/力传感器，不能把到位反馈说成夹持成功。界面“继续”或 `/grasp/continue` 就是操作者确认。

## 7. 停止和恢复

“停止流程 / 取消运动”取消当前 MoveIt goal，后续步骤不再发送，夹爪不自动打开。超时或目标接收/取消状态不确定时会锁住新的运动，需要检查实机后重启 demo 节点。软件取消不等同于控制柜硬件急停。

失败后不能直接从第一个点重跑。先现场处理机器人和零件，必要时用低速单步规划撤离；确认零件已人工处理后使用“人工恢复后清除任务状态”。它只清理本 demo 的零件碰撞体和流程状态，不运动、不打开夹爪。若仍附着零件，重启节点也不会自动丢弃旧附着记录。

一次完成后重新运行前，需把待抓取零件放回示教抓取位置。

## 8. 相机与未来感知

- `config/scene.yaml`：只保留头部相机。
- `config/scene.full_cameras.yaml`：原头部、胸/腰部 `waist_camera` 和双腕 `d435i` 完整备份。
- 恢复时传 `scene:=.../scene.full_cameras.yaml`。相机碰撞体随配置恢复，不改双臂、夹爪或支架。RViz 保留头部图像/点云显示；实机相机驱动不由本 demo 启动，没有相机不会阻止示教流程。
- 候选话题 `/grasp/perception/grasp_tcp` 类型 `geometry_msgs/PoseStamped`，要求 frame=`world`、有效四元数、当前 ROS 时间戳。上游先完成相机外参变换、零件位姿到右手 TCP 的抓取偏移换算。
- 消息只缓存在候选区，不触发运动。界面按钮可把 2 秒内的候选填入右手 TCP 输入框，先规划，执行定位后重新采集抓取/接近等关联点。当前完整 demo 仍只使用示教文件，不自动重算点云抓取轨迹。
- `perception.py` 原有点云估计函数保留，尚未接入完整闭环识别或自动抓取重试。

## 9. 无 GUI 与接口

`pick_place.launch.py gui:=false` 启动同一后端，示教文件通过 `points:=/path/to/teach_points.json` 传入。

```bash
ros2 service call /grasp/start std_srvs/srv/Trigger '{}'
ros2 service call /grasp/status std_srvs/srv/Trigger '{}'
ros2 service call /grasp/continue std_srvs/srv/Trigger '{}'
ros2 service call /grasp/stop std_srvs/srv/Trigger '{}'
```

`/grasp/status_text` 发布 JSON，包括 busy、阶段、附着手臂、待确认提示、恢复要求和运动状态不确定标志。start 返回成功仅表示任务已接受，最终结果查看 status。

## 10. 验证与首次现场验收

```bash
python3 -m pytest tests -q
python3 -m compileall -q src
```

离线测试覆盖反馈聚合与超时、无效数据拒绝、单位映射、示教文件往返、标定不匹配、双臂交接一致性、失败时不松开右手、取消时不继续、相机备份与机械参数不变、Tk 控件和按钮互锁。无显示器的 Linux 可使用 `xvfb-run -a python3 -m pytest tests -q`。

现场还需完成：Humble 编译；mock 启动与目标规划；实机 14 关节反馈；TCP 与示教器对照；空载低速单步；夹爪实测目标可达；桌面/支架/双臂避碰；带零件抓取、交接、释放；运动中停止与通信中断后的实际控制柜行为。MoveIt 只能检查模型中存在的障碍物，不能感知未建模的人和物。

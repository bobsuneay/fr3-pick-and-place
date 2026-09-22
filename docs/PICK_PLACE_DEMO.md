# 双臂示教抓取展示 Demo

本版以用户提供的 `fr3-standard3ok/fr3-standard` 为基础，保留其双臂、HKV 夹爪、TCP、support 安装参数和实机控制器补丁。`fr3-sim5` 的“右手抓取→右手展示→左手接取→左手展示”改为示教点驱动，并增加左手放置和撤离。原始目录未修改。

## 1. 已实现与验证范围

- Tk 界面每 200 ms 自动刷新左右臂关节角、左右夹爪开口毫米/闭合百分比，并异步读取双臂 TCP。目标输入框独立，使用“当前姿态填入目标”复制当前测量。反馈过期显示错误，不继续显示为实时数据。
- 单臂关节目标、TCP 位姿目标、MoveL 直线、夹爪目标均使用 MoveIt。预览与执行分开，启动默认禁止执行。
- 3 个核心点同时保存双臂关节、世界坐标 TCP、夹爪开度：`ready`、`right_pregrasp`、`left_place`。抓取、展示和交接目标由运行时自动生成。速度滑条 1%–30%，改变后下一段规划生效，不改变正在执行的轨迹；SDK 夹爪闭合速度仍由 hardware.yaml 的 gripper.vel 设置。
- 执行时逐段规划。双臂交接预备点使用 `both_arms`；其他段使用对应单臂组，另一臂仍参与碰撞检测。普通关键点使用关节空间规划；展示旋转、交接后短距离撤离、放置下降/抬升使用完整笛卡尔路径碰撞检查。不可达或路径不完整时停止，不退化为绕行展示。
- UI 的“关键点运行”按钮可在“保存的关节角”和“TCP 位姿反解”之间切换。TCP 模式下，普通单臂点以保存的 `world → gripper_tcp` 位姿交给 MoveIt IK/OMPL；`ready` 和 `handover_ready` 将左右 TCP 作为 `both_arms` 的同一个规划目标。切换同时作用于选中点回放和完整 Demo，流程运行期间锁定。展示、撤离和放置的原有 TCP/笛卡尔逻辑不变。
- 将桌面、桌腿和支撑加入规划场景。默认 `show_workpiece_in_rviz: false`，零件只保留尺寸、TCP 偏移和持有者参数，不发布成 RViz/MoveIt 碰撞体；双臂、夹爪、桌面与支架之间的碰撞检测保持开启。
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

当现场已核对左右臂方向、反馈、TCP 和环境后，关闭上次 launch，再以 `enable_execution:=true speed:=0.1` 启动。ROS 控制器和 demo 节点共同使用该开关。界面提供“进入拖动示教 / 恢复运动控制”，不会修改 RobotEnable。必须先应用本版驱动补丁并重编译 fairino_hardware_v3_9_7。

如果已有当前版本 MoveIt bringup，可以单独运行：

```bash
ros2 launch fr3_dual_arm_grasp grasp.launch.py enable_execution:=false
```

实机独立启动还需传入 `mode:=real hardware:=$HOME/fr3_dual_arm.hardware.yaml`。此时 `arms_file`、`scene_file` 必须与正在运行的 bringup 完全相同。一个机器人工作空间只能运行一个 demo 后端。

## 4. 坐标、单位与夹爪

界面输入：关节为度，位置为毫米，姿态为固定轴 XYZ 的 Rx/Ry/Rz（度）。内部 JSON：关节 rad、位置 m、四元数 `[qx,qy,qz,qw]`，夹爪完整开口 `gap_m`。

每次采集聚合左右侧 `/joint_states`，检查全部 14 个独立关节的每项反馈在 1 秒内更新，并对同一快照调用 MoveIt `/compute_fk` 得到 `world → left/right_gripper_tcp`。采集过程中机器人移动则拒绝保存。mimic 从指由实测主指推导，不独立命令。

**不是直接抄 SDK 的工具号 1 的 TCP 数字。** 模型 TCP 由 `arms.yaml` 的夹爪安装与 TCP 偏移定义；SDK 工具坐标系或机器人基坐标下的数字不能直接粘贴到 world 输入框。优先用机器人示教器定位后点击“采集”，或者用本界面的 MoveIt 运动定位。

当前校准保留 `open_gap=0.1 m`、单指行程 `0.05 m`。闭合百分比 = `100 × (1-gap/open_gap)`：0% 全开，100% 全闭。SDK 配置沿用 `open_pos=0, closed_pos=100`。

完整 Demo 抓取时发送完全闭合目标；TG-9801 由 `hardware.yaml` 的 10 N 限力停止，ROS 夹爪控制器允许因零件稳定堵转。程序随后读取实际总开度，只有连续稳定且位于 `demo.yaml` 的允许范围才继续。手动夹爪测试和张开动作仍可使用界面的目标闭合百分比。

## 5. 采集 3 个核心点

1. `ready`：双臂空手就绪，双夹爪张开；也用于流程结束后的回位。
2. `right_pregrasp`：右手位于零件上方的预夹取位姿。程序从这个点复制 x/y，把 z 改为桌面上方 10 mm，并将 TCP 轴调整为竖直向下，自动生成 `right_grasp`。
3. `left_place`：左手放置位姿。程序自动生成放置上方点、下降和放置后的抬升路径。

展示点由头部相机光轴中心和 30 cm 距离自动生成；交接中心、左右夹爪间距和姿态由 `demo.yaml` 的 `handover_center_xyz`、`handover_separation_m` 自动生成；左手接取位姿再根据右手实时 TCP 动态对齐。旧文件中的 `right_grasp`、`right_display`、`handover_ready`、`left_receive` 会被保留读取但不再要求重新采集。

交接后右手沿自身 TCP 的负 Z 方向撤离 `retreat_distance_m`（默认 6 cm），再回到就绪点；放置前的位置由 `left_place` 沿 world +Z 加 `place_clearance_m`（默认 8 cm）生成，随后直线下降，放下后直线抬升。参数在 demo.yaml 中，方向必须符合现场夹持几何；路径不通会停止。夹爪不会跟随展示点开合：右手释放使用 right_pregrasp 的开度，左手张开/释放使用 ready 的开度。

### 展示方式和相机位置

复用 fr3-sim5 的 `centered_views` / `interpolate_object` 方式：绕零件中心做 XYZ 欧拉角相对旋转，SLERP 插值；每个视角都从初始姿态出发并返回，不累积腕部转角。18 个视角与原 inspection.yaml 一致，包括 pitch ±15°/±35°、roll ±15°/±60°/±120°/180° 和 yaw -30°/±60°/±120°/180°，以及初始视角。任一视角规划失败会停止本次 demo。

头部相机默认绕 world Y 轴向下俯视 45°。零件中心位于 `head_camera_optical_frame` 的 `[0, 0, 0.30] m`，即光轴中心前方 30 cm；默认标定对应 world 约 `[0.2721, 0, 1.2279] m`。这不是 TCP 到相机的距离；程序通过 TCP→零件偏移反算双手目标，左手交接后优先使用实际附着变换。示教文件的原始测量不被替换，避免关节角和 TCP 自相矛盾。展示参考点的“规划/执行选中点”也使用相机派生目标，不能选择 both 同时占用展示中心。

### 执行后继续拖动示教

先等待运动结束；若中途停止且仍持有零件，先人工处理并使用“人工恢复后清除任务状态”。点击“进入拖动示教”：停用双臂和夹爪命令控制器，保持状态广播器及硬件读取，再通过 SDK 同款 XML-RPC 接口执行 Mode(1)、DragTeachSwitch(1)。没有另开 SDK 状态连接，也不需要额外 ros2_cmd_server。

拖动定位后直接采集。点击“恢复运动控制”：先退出双臂拖动模式、切回自动模式，再恢复控制器。驱动将命令同步到最新测量，不回跳到拖动前的位置。切换失败会显示 fault 并禁止界面运动；处理实际状态后可再次点击恢复。不会在 demo 结束或 UI 关闭时自动开启拖动。mock 只切换控制器，不连接机器人，也不模拟手动拖动的物理过程。

旧 16 点文件中已取消的点可载入但不再参与流程；首次更新建议另存为新文件保留原记录。

默认自动保存到当前工程源码目录的 `src/fr3_dual_arm_grasp/config/teach_points.json`，不使用 `~/.ros`。默认的 arms、scene 和 demo 配置也优先从本工作空间 `src/` 读取；安装时不带源码的部署才回退到对应包的 `install/.../share/.../config/`。示教文件包含 arms/scene 配置内容哈希，不允许跨不同安装标定或相机场景直接回放。旧 SDK JSON 的 mm/degree 与新 schema 不兼容，需重新采集。载入失败时不会悄悄覆盖旧文件；选择“另存为”以创建新文件。

## 6. 工作台和零件包围盒

`scene.yaml` 保留原基础工程的桌面示例；**双臂和支架已标定不代表桌面和零件也已核对。** 默认桌面中心 `[0.48, 0] m`、尺寸 `0.7 × 2.0 m`、台面高 `0.75 m`。按现场更新副本并通过 `scene:=...` 传入。

现场参数直接维护在项目配置中：

```bash
nano src/fr3_dual_arm_grasp/config/demo.yaml
```

填写实际 `workpiece.dimensions_m` 与抓取时的 `right_tcp_to_object`（TCP 到零件包围盒中心的变换），核对后设置 `scene_and_object_verified: true`。默认值仅为示例，完整 demo 默认拒绝运行。再启动：

```bash
ros2 launch fr3_dual_arm_bringup pick_place.launch.py \
  mode:=real hardware:=$HOME/fr3_dual_arm.hardware.yaml \
  enable_execution:=true speed:=0.1
```

点击“检查示教点完整性”，逐点规划并检查 RViz。预览只验证当前状态到一个目标的路径，不等于整条流程已验收。完整流程每段都重新规划，并保留当前零件附着关系。

抓取时夹爪命令完全闭合，由 `hardware.yaml` 的夹持力限制停止。实际开度连续稳定并落在 `demo.yaml` 的允许范围后，流程自动继续；右手只在左手也通过检查后松开。该判断依赖开度，不等同于触觉或掉落检测。

交接时以右手实时 `gripper_tcp` 的 Z 中心轴为基准，自动把示教的左手接取 TCP 投影到该中心线上，并把左手 Z 轴调整为与右手反向；示教点只提供轴向间距和左手绕中心轴的滚转参考。修正后的目标仍通过 MoveIt 求逆解、OMPL 和碰撞检测。到位后按默认 5 mm、5° 容差复核；复核失败时左夹爪不会闭合，右夹爪也不会松开。

### 6.1 实机夹爪参数

`hardware.yaml` 建议明确写入：

```yaml
gripper:
  vel: 50
  force_n: 10.0
  rated_force_n: 20.0
  force: 50
  maxtime: 30000
  block: 1
  open_pos: 0
  closed_pos: 100
```

`force_n` 是现场使用的目标力，代码按 `force_n / rated_force_n` 换算成法奥 `MoveGripper` 的百分比，因此 10 N 对应 50%。没有新字段的旧文件也默认按 10 N / 20 N 计算。`demo.yaml` 的下列范围必须按零件实测夹持宽度调整：

```yaml
workpiece:
  grasp_gap_min_m: 0.010
  grasp_gap_max_m: 0.026
```

实际开度小于下限通常表示没有夹到零件而接近全闭；大于上限表示夹到错误位置或其他障碍。这个判断没有零件身份识别、触觉确认或掉落检测。

### 6.2 完整 Demo 的 25 步

每个运动步骤开始前都会检查停止请求、反馈新鲜度和当前机器人状态。普通远距离运动使用 MoveIt IK/OMPL；标记为 MoveL 的步骤使用 `/compute_cartesian_path`，要求路径覆盖率 100%，以 2 mm 步长进行碰撞检查，再交给 `/execute_trajectory`。夹爪通过 MoveIt `GripperCommand`、`ros2_control` 和法奥 `MoveGripper` 到达实机。

| 步骤 | 状态文字 | 动作及内部处理 |
|---:|---|---|
| 1 | `move both ready` | 左右臂同时回到 `ready`。关节模式使用保存的双臂关节角；TCP 模式把两个 TCP 作为 `both_arms` 目标。OMPL 检查双臂互撞、桌面和支撑碰撞。 |
| 2 | `grip right right_pregrasp` | 右夹爪张开到 `right_pregrasp` 保存的开度，为抓取留出空间。 |
| 3 | `grip left ready` | 左夹爪张开到 `ready` 保存的开度，避免交接前处于未知状态。 |
| 4 | `move right right_pregrasp` | 右臂用 OMPL 到达抓取接近点，左臂保持不动但仍参与全机器人碰撞检查。 |
| 5 | `approach right right_grasp` | 右手从接近点沿 TCP 直线 MoveL 到抓取点；路径不完整、关节跳变或碰撞时不执行。 |
| 6 | `grasp right right_grasp` | 右夹爪发送完全闭合目标，按 10 N 限力夹紧。允许稳定堵转，然后读取主指反馈换算总开度；连续 5 次变化不超过 0.5 mm 且落在配置范围才认为成功。 |
| 7 | `attach right` | 软件状态把零件持有者记录为右手，并保存 TCP 到零件的相对变换。默认不向 RViz 发布零件碰撞体。 |
| 8 | `lift right right_pregrasp` | 右手带件沿直线 MoveL 返回 `right_pregrasp`，形成可控抬升，不允许 OMPL 绕行。 |
| 9 | `scan right right_display` | 右手先用 OMPL 到头部相机光轴中心前 30 cm 的展示中心，再按 fr3-sim5 的 18 个相对视角绕零件中心插值旋转；每个视角返回中性姿态。 |
| 10 | `move both handover_ready` | 双臂以 `both_arms` 同时规划到交接预备位，避免分别规划造成另一只手成为动态障碍。 |
| 11 | `touch left` | 若启用零件碰撞体，临时允许左右手指与零件接触；默认隐藏零件时仅更新流程接触状态。 |
| 12 | `receive left left_receive` | 读取右手实时 TCP，以其 Z 轴作为中心线；保留示教的轴向间距和左手滚转参考，自动消除左手横向偏差并令两条 Z 轴反向。修正目标通过左臂 IK、OMPL 和碰撞检测后执行，右臂保持不动。 |
| 13 | `grasp left left_receive` | 到位后复核中心线横向误差 ≤5 mm、角度误差 ≤5°；通过后左夹爪完全闭合并按实际开度自动确认。失败时右手继续夹持。 |
| 14 | `transfer left` | 只有左夹爪确认成功后，软件持有者才从右手切换为左手。 |
| 15 | `grip right right_pregrasp` | 右夹爪张开到预抓取点保存的开度，正式释放零件。 |
| 16 | `retreat right` | 右手沿自身 TCP 负 Z 方向直线撤离 `retreat_distance_m`，避免撤离时扫过左夹爪。 |
| 17 | `touch_only left` | 接触状态收紧为只允许左手持有零件。 |
| 18 | `move right ready` | 右臂用 OMPL 回到安全就绪位，左手继续持件。 |
| 19 | `scan left right_display` | 根据左手当前 TCP 到零件的相对变换，计算同一相机展示中心和 18 个零件视角，由左手完成展示。 |
| 20 | `preplace left` | 从 `left_place` 沿 world +Z 增加 `place_clearance_m` 得到放置上方点，使用 OMPL 到达。 |
| 21 | `place left` | 左手从上方点直线 MoveL 下降到示教的放置位。 |
| 22 | `grip left ready` | 左夹爪自动张开到 `ready` 开度，放下零件；此处已按要求取消人工确认。 |
| 23 | `detach left` | 软件清除左手持有者并记录零件最终世界位姿；默认不显示零件碰撞体。 |
| 24 | `preplace left` | 左手从放置点沿 world +Z 直线抬升到安全高度。 |
| 25 | `move left ready` | 左臂用 OMPL 回到 `ready`，清除恢复锁并报告完整流程完成。 |

任一步失败都会停止后续步骤。抓取后发生失败时不会自动张开当前持有零件的夹爪；交接步骤中，左手未通过中心线和开度检查时右手不会松开。

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
ros2 service call /grasp/stop std_srvs/srv/Trigger '{}'
```

`/grasp/status_text` 发布 JSON，包括 busy、阶段、附着手臂、恢复要求和运动状态不确定标志。start 返回成功仅表示任务已接受，最终结果查看 status。

## 10. 验证与首次现场验收

```bash
python3 -m pytest tests -q
python3 -m compileall -q src
```

离线测试覆盖反馈聚合与超时、无效数据拒绝、单位映射、示教文件往返、标定不匹配、双臂交接一致性、失败时不松开右手、取消时不继续、相机备份与机械参数不变、Tk 控件和按钮互锁。无显示器的 Linux 可使用 `xvfb-run -a python3 -m pytest tests -q`。

现场还需完成：Humble 编译；mock 启动与目标规划；实机 14 关节反馈；TCP 与示教器对照；空载低速单步；夹爪实测目标可达；桌面/支架/双臂避碰；带零件抓取、交接、释放；运动中停止与通信中断后的实际控制柜行为。MoveIt 只能检查模型中存在的障碍物，不能感知未建模的人和物。

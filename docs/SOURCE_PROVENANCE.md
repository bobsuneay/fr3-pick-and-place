# 来源与保留内容

- 工程基础：用户指定的本地 `fr3-standard3ok/fr3-standard`，Git HEAD `96fad21f66e342c288d45c19019858b2ceff140a`，对应 `https://github.com/bobsuneay/fr3-standard0`。以本地文件内容为准，包括用户现场修改。
- 流程参考：本地 `fr3-sim5/fr3-sim`，Git HEAD `d11d3bc8f4c036fb36be78e59d28f9b528ea5935`，对应 `https://github.com/bobsuneay/fr3-sim`；参考 `fr3_bolt_inspection_cell/task_node.py` 的抓取、展示、交接顺序。
- 界面参考：本地 `fr-sdk3/fr-sdk3/fr-sdk/robot_ui.py` 的关节/TCP、夹爪百分比和示教采集操作。新界面使用 MoveIt，不复制直接 SDK 运动通道。
- `arms.yaml`、机械 URDF、模型生成器和控制器/SDK 补丁保留基础版本。相机变化通过 scene 配置完成，原配置备份为 `scene.full_cameras.yaml`。
- 原第三方 SDK 目录保留在本地新工程，并与基础仓库一致排除于 Git 上传。厂商许可证与上游声明保持不变。
- 用户提供的实机 hardware YAML 只复制到本地新工程根部，Git 忽略该文件。原始三个目录和外部 hardware YAML 未修改。

新增实现：`teach_model.py`（数据/单位/反馈）、`motion.py`（MoveIt 运动）、`demo_scene.py`（场景/零件）、`workflow.py`（流程）、`app.py`（后端服务）、`teach_ui.py`（界面）、`pick_place.launch.py`（统一启动）、离线测试与使用说明。

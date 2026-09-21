# fr3_dual_arm_grasp

MoveIt 2 示教与抓取展示应用。完整说明见仓库 `docs/PICK_PLACE_DEMO.md`。

- `teach_model.py`：数据格式、单位、反馈聚合、固定流程。
- `motion.py`：MoveGroup / CartesianPath / ExecuteTrajectory、结果与取消检查。
- `demo_scene.py`：桌面、零件碰撞体、附着/交接/放下。
- `workflow.py`：抓取、展示、确认交接、放置状态流程。
- `app.py`：后台串行任务和 ROS 服务。
- `teach_ui.py`：Tk 示教界面。
- `perception.py`：保留的点云算法；目前不自动接入运动。

启动统一入口：`ros2 launch fr3_dual_arm_bringup pick_place.launch.py`。
`grasp_node.py` / `moveit_interface.py` 保留为兼容导入。

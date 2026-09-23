"""Native Tk teaching panel, inspired by fr-sdk3, backed exclusively by MoveIt."""
import math
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import rclpy
from scipy.spatial.transform import Rotation

from .app import DemoApp
from .teach_model import SLOTS, gap_to_percent, percent_to_gap


class TeachUI:
    def __init__(self, root, app):
        self.root, self.app = root, app
        root.title('FR3 双臂示教 · 抓取 / 展示 / 交接 / 放下 · MoveIt 2')
        root.geometry('1360x940')
        self.last_capture = None
        self.controls = []
        self.side = tk.StringVar(value='right')
        self.status = tk.StringVar()
        self.speed = tk.DoubleVar(value=app.motion.speed*100)
        self.speed_label = tk.StringVar()
        self.keypoint_mode_label = tk.StringVar()
        self.feedback = tk.StringVar(value='等待 /joint_states')
        self.path = tk.StringVar(value=app.points_file)
        self.joints = [tk.StringVar(value='0') for _ in range(6)]
        self.tcp = [tk.StringVar(value='0') for _ in range(6)]
        self.closure = tk.StringVar(value='0')
        self.saved_gap = tk.StringVar(value='0')
        self.point_side = tk.StringVar(value='right')
        self.details = tk.StringVar(value='选择已采集点查看双臂 TCP、关节与夹爪信息')
        self.build()
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.refresh()

    def button(self, parent, text, command):
        button = ttk.Button(parent, text=text, command=lambda: self.attempt(command))
        button.pack(side='left', padx=4, pady=4)
        self.controls.append(button)
        return button

    def attempt(self, function):
        try:
            function()
        except Exception as exc:
            messagebox.showerror('操作未完成', str(exc), parent=self.root)

    def submit(self, label, fn):
        self.app.submit(label, fn)

    def build(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill='x')
        mode = '执行已启用' if self.app.motion.enabled else '只规划 / 采集（enable_execution=false）'
        ttk.Label(top, text='FR3 双臂示教 DEMO', font=('', 18, 'bold')).pack(side='left')
        ttk.Label(top, text=f'  {mode}  |  TCP: world → gripper_tcp').pack(side='left', padx=16)
        speed_row = ttk.Frame(self.root, padding=6)
        speed_row.pack(fill='x')
        ttk.Label(speed_row, text='运行速度（1–30%）').pack(side='left')
        ttk.Scale(speed_row, from_=1, to=30, variable=self.speed, command=self.change_speed).pack(side='left', fill='x', expand=True)
        ttk.Label(speed_row, textvariable=self.speed_label, width=48).pack(side='left')
        self.speed_label.set(f'{self.app.motion.speed:.0%} · 下一段规划生效')
        self.point_mode_button = ttk.Button(
            speed_row, textvariable=self.keypoint_mode_label,
            command=lambda: self.attempt(self.toggle_keypoint_mode))
        self.point_mode_button.pack(side='left', padx=8)
        self.controls.append(self.point_mode_button)
        self.update_keypoint_mode_label()
        ttk.Label(self.root, textvariable=self.feedback, padding=10, font=('TkFixedFont', 10)).pack(fill='x')
        status = ttk.LabelFrame(self.root, text='流程状态', padding=8)
        status.pack(fill='x', padx=10)
        ttk.Label(status, textvariable=self.status, wraplength=1250, font=('', 11)).pack(anchor='w')
        body = ttk.Panedwindow(self.root, orient='horizontal')
        body.pack(fill='both', expand=True, padx=10, pady=10)
        manual_container, teach = ttk.Frame(body), ttk.Frame(body, padding=8)
        canvas = tk.Canvas(manual_container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(manual_container, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        manual = ttk.Frame(canvas, padding=8)
        window = canvas.create_window((0, 0), window=manual, anchor='nw')
        manual.bind('<Configure>', lambda event: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda event: canvas.itemconfigure(window, width=event.width))
        body.add(manual_container, weight=1)
        body.add(teach, weight=2)

        selection = ttk.Frame(manual)
        selection.pack(fill='x')
        ttk.Label(selection, text='当前手臂').pack(side='left')
        ttk.Combobox(selection, textvariable=self.side, values=['right', 'left'], state='readonly', width=10).pack(side='left', padx=8)
        self.button(selection, '当前姿态填入目标', lambda: self.submit('读取当前状态', self.app.capture))
        joint_frame = ttk.LabelFrame(manual, text='关节目标（度）· MoveJ / OMPL', padding=8)
        joint_frame.pack(fill='x', pady=8)
        for i, variable in enumerate(self.joints):
            ttk.Label(joint_frame, text=f'J{i+1}').grid(row=i//3*2, column=i%3, sticky='w')
            ttk.Entry(joint_frame, textvariable=variable, width=12).grid(row=i//3*2+1, column=i%3, padx=4, pady=3)
        row = ttk.Frame(manual)
        row.pack(fill='x')
        self.button(row, '规划关节目标', lambda: self.joint_move(False))
        self.button(row, '执行关节目标', lambda: self.joint_move(True))

        tcp_frame = ttk.LabelFrame(manual, text='TCP 目标 · world · mm / 度（固定轴 XYZ RPY）', padding=8)
        tcp_frame.pack(fill='x', pady=8)
        for i, (label, variable) in enumerate(zip(['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'], self.tcp)):
            ttk.Label(tcp_frame, text=label).grid(row=i//3*2, column=i%3, sticky='w')
            ttk.Entry(tcp_frame, textvariable=variable, width=12).grid(row=i//3*2+1, column=i%3, padx=4, pady=3)
        for linear, label in [(False, 'TCP 自由路径'), (True, 'MoveL 直线')]:
            row = ttk.Frame(manual)
            row.pack(fill='x')
            self.button(row, '规划 ' + label, lambda line=linear: self.pose_move(False, line))
            self.button(row, '执行 ' + label, lambda line=linear: self.pose_move(True, line))
        gripper = ttk.LabelFrame(manual, text='夹爪 · 0% 全开，100% 全闭', padding=8)
        gripper.pack(fill='x', pady=8)
        ttk.Label(gripper, text='目标闭合百分比').pack(side='left')
        ttk.Entry(gripper, textvariable=self.closure, width=8).pack(side='left', padx=4)
        row = ttk.Frame(manual)
        row.pack(fill='x')
        self.button(row, '规划夹爪', lambda: self.gripper_move(False))
        self.button(row, '执行夹爪', lambda: self.gripper_move(True))
        ttk.Label(manual, text='所有运动均经过 MoveIt。\n直线路径必须 100% 成功才执行。\n单点回放只移动关节，不自动改变夹爪。\n示教时可用机器人示教器定位，再采集；\n执行后点击“进入拖动示教”释放保持指令。', wraplength=410).pack(anchor='w', pady=12)
        teaching = ttk.Frame(manual)
        teaching.pack(fill='x')
        self.button(teaching, '进入拖动示教', lambda: self.submit('进入拖动示教', lambda: self.app.set_teach_mode(True)))
        self.button(teaching, '恢复运动控制', lambda: self.submit('恢复运动控制', lambda: self.app.set_teach_mode(False)))
        self.button(manual, '加载感知候选到输入框（不运动）', self.use_candidate)

        files = ttk.Frame(teach)
        files.pack(fill='x')
        ttk.Entry(files, textvariable=self.path, state='readonly').pack(side='left', fill='x', expand=True)
        self.button(files, '打开', self.load)
        self.button(files, '另存为', self.save_as)
        tree_frame = ttk.Frame(teach)
        tree_frame.pack(fill='both', expand=True, pady=6)
        self.tree = ttk.Treeview(tree_frame, columns=('name', 'saved'), show='headings', selectmode='browse', height=7)
        self.tree.heading('name', text='3 个核心示教点（其余运行时自动生成）')
        self.tree.heading('saved', text='状态')
        self.tree.column('name', width=410)
        self.tree.column('saved', width=90)
        for key, label in SLOTS.items():
            self.tree.insert('', 'end', iid=key, values=(label, '未采集'))
        self.tree.pack(side='left', fill='both', expand=True)
        scroll = ttk.Scrollbar(tree_frame, command=self.tree.yview)
        scroll.pack(side='right', fill='y')
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.selection_set('ready')
        self.tree.bind('<<TreeviewSelect>>', lambda event: self.show_point())
        row = ttk.Frame(teach)
        row.pack(fill='x')
        self.button(row, '采集 / 覆盖选中点（双臂）', self.capture)
        ttk.Label(row, text='回放运动组').pack(side='left')
        ttk.Combobox(row, textvariable=self.point_side, values=['right', 'left', 'both'], state='readonly', width=8).pack(side='left')
        self.button(row, '规划选中点', lambda: self.replay(False))
        self.button(row, '执行选中点', lambda: self.replay(True))
        ttk.Label(teach, textvariable=self.details, wraplength=760, font=('TkFixedFont', 9)).pack(fill='x', pady=8)
        row = ttk.Frame(teach)
        row.pack(fill='x')
        ttk.Label(row, text='选中点 / 当前手臂的夹持目标：闭合 %').pack(side='left')
        ttk.Entry(row, textvariable=self.saved_gap, width=8).pack(side='left')
        self.button(row, '只保存开度', self.edit_gap)
        ttk.Label(teach, text='仅采集 ready、right_pregrasp、left_place。\n抓取点自动取预夹取点 x/y，桌面上方 10 mm 且 TCP 竖直；展示点由头部相机前 30 cm 自动生成；交接位姿按配置中心和夹爪间距自动生成。\n原始关节与 TCP 均来自同一次双臂反馈快照，保存单位为 rad / m / 四元数。').pack(anchor='w', pady=6)
        actions = ttk.Frame(self.root, padding=10)
        actions.pack(fill='x')
        self.button(actions, '检查示教点完整性', lambda: self.submit('检查示教点', self.app.book.validate_complete))
        self.button(actions, '运行完整 DEMO', lambda: self.submit('启动完整 demo', self.app.start_demo))
        tk.Button(actions, text='停止流程 / 取消运动', bg='#ba2832', fg='white', command=self.app.motion.cancel).pack(side='left', padx=8)
        tk.Button(actions, text='跳过本步（IK 失败时）', bg='#d99a1c', fg='black',
                  command=lambda: self.attempt(self.app.skip_step)).pack(side='left', padx=8)
        self.button(actions, '人工恢复后清除任务状态', self.recover)
        ttk.Label(self.root, text='软件停止不替代硬件急停。停止后保留夹爪；完整流程不会自动重启。预览只检查当前状态到选中目标的一段路径。', padding=6).pack(fill='x')

    def change_speed(self, value):
        percent = round(float(value))
        self.app.set_speed(percent)
        self.speed_label.set(f'{percent}% · 下一段规划生效')

    def update_keypoint_mode_label(self):
        label = ('TCP 位姿反解' if self.app.keypoint_motion_mode == 'tcp' else '保存的关节角')
        self.keypoint_mode_label.set('关键点运行：' + label + '（点击切换）')

    def toggle_keypoint_mode(self):
        mode = 'tcp' if self.app.keypoint_motion_mode == 'joints' else 'joints'
        self.app.set_keypoint_motion_mode(mode)
        self.update_keypoint_mode_label()

    def selected(self):
        selected = self.tree.selection()
        if not selected:
            raise ValueError('先选择一个关键点')
        return selected[0]

    def capture(self):
        name = self.selected()
        self.submit('采集 ' + SLOTS[name], lambda: self.app.capture(name))

    def replay(self, execute):
        name, side = self.selected(), self.point_side.get()
        self.submit('回放 ' + name, lambda: self.app.move_point(name, side, execute))

    def joint_move(self, execute):
        side, joints = self.side.get(), [math.radians(float(v.get())) for v in self.joints]
        self.submit('关节目标', lambda: self.app.manual_joints(side, joints, execute))

    def pose_move(self, execute, linear):
        values = [float(v.get()) for v in self.tcp]
        pose = [v/1000 for v in values[:3]] + Rotation.from_euler('xyz', values[3:], degrees=True).as_quat().tolist()
        side = self.side.get()
        self.submit('TCP 目标', lambda: self.app.manual_pose(side, pose, execute, linear))

    def gripper_move(self, execute):
        side = self.side.get()
        gap = percent_to_gap(float(self.closure.get()), self.app.open_gap)
        self.submit('夹爪目标', lambda: self.app.manual_gripper(side, gap, execute))

    def edit_gap(self):
        name, side = self.selected(), self.side.get()
        gap = percent_to_gap(float(self.saved_gap.get()), self.app.open_gap)
        def edit():
            self.app.book.points[name][side]['gap_m'] = gap
            self.app.book.save(self.app.points_file)
        self.submit('保存夹持目标开度（不运动）', edit)

    def load(self):
        path = filedialog.askopenfilename(filetypes=[('示教文件', '*.json')])
        if path:
            def load_file():
                self.app.book.load(path)
                self.app.points_file, self.app.load_error = path, ''
            self.submit('载入示教文件', load_file)

    def save_as(self):
        path = filedialog.asksaveasfilename(defaultextension='.json', filetypes=[('示教文件', '*.json')])
        if path:
            def save_file():
                self.app.book.save(path)
                self.app.points_file, self.app.load_error = path, ''
            self.submit('保存示教文件', save_file)

    def recover(self):
        if messagebox.askyesno('人工恢复', '已停止机械臂并人工处理零件，确认当前没有需要保持的抓取状态？\n此操作只清除 MoveIt 中的 demo 零件和流程状态，不控制夹爪。'):
            self.submit('清除任务状态', self.app.recover)

    def fill_pose(self, pose):
        xyz = [x*1000 for x in pose[:3]]
        angles = Rotation.from_quat(pose[3:]).as_euler('xyz', degrees=True).tolist()
        for variable, value in zip(self.tcp, xyz + angles):
            variable.set(f'{value:.3f}')

    def use_candidate(self):
        candidate = self.app.perception_candidate
        if candidate is None:
            raise ValueError('尚无 /grasp/perception/grasp_tcp 候选')
        stamp = candidate['stamp_sec'] + candidate['stamp_nanosec']*1e-9
        age = self.app.get_clock().now().nanoseconds*1e-9 - stamp
        if not 0 <= age <= 2:
            raise ValueError('感知候选超过 2 秒或时间戳无效')
        self.side.set('right')
        self.fill_pose(candidate['tcp'])

    def show_point(self):
        name = self.selected()
        point = self.app.book.points.get(name)
        if point is None:
            self.details.set(name + '：未采集')
            return
        lines = [name]
        for side in ('right', 'left'):
            data = point[side]
            xyz = ' '.join(f'{v*1000:.1f}' for v in data['tcp'][:3])
            rpy = ' '.join(f'{v:.1f}' for v in Rotation.from_quat(data['tcp'][3:]).as_euler('xyz', degrees=True))
            q = ' '.join(f'{math.degrees(v):.1f}' for v in data['joints'])
            lines.append(f'{side}: TCP mm [{xyz}]  RPY° [{rpy}]\n  J° [{q}]  开口 {data["gap_m"]*1000:.1f} mm')
        self.details.set('\n'.join(lines))

    def refresh(self):
        self.status.set(self.app.status_text + f'  |  模式: {self.app.teaching.state}  |  零件附着: {self.app.scene.owner or "无"}')
        self.path.set(self.app.points_file)
        try:
            values = self.app.feedback.snapshot()
            lines = []
            for side in ('right', 'left'):
                joints = '  '.join(f'{math.degrees(values[f"{side}_j{i}"]):7.2f}' for i in range(1, 7))
                gap = values[side + '_left_finger_joint']*2
                lines.append(f'{side:5} J° [{joints}]   开口 {gap*1000:.2f} mm  闭合 {gap_to_percent(max(0, min(self.app.open_gap, gap)), self.app.open_gap):.1f}%')
            if self.app.live_pose_error or time.monotonic() - self.app.live_stamp > 1.0:
                lines.append('TCP: ' + (self.app.live_pose_error or '反馈已过期'))
            else:
                for side in ('right', 'left'):
                    pose = self.app.live_poses[side]
                    xyz = ' '.join(f'{v*1000:.2f}' for v in pose[:3])
                    rpy = ' '.join(f'{v:.2f}' for v in Rotation.from_quat(pose[3:]).as_euler('xyz', degrees=True))
                    lines.append(f'{side:5} TCP mm [{xyz}]  RPY° [{rpy}]')
            self.feedback.set('\n'.join(lines))
        except Exception as exc:
            self.feedback.set(str(exc))
        capture = self.app.latest_capture
        if capture is not None and capture is not self.last_capture:
            self.last_capture = capture
            data = capture[self.side.get()]
            for variable, value in zip(self.joints, data['joints']):
                variable.set(f'{math.degrees(value):.3f}')
            self.fill_pose(data['tcp'])
            self.closure.set(f'{gap_to_percent(data["gap_m"], self.app.open_gap):.2f}')
        for key, label in SLOTS.items():
            self.tree.item(key, values=(label, '已采集' if key in self.app.book.points else '未采集'))
        for control in self.controls:
            control.configure(state='disabled' if self.app.busy else 'normal')
        self.show_point()
        self.root.after(200, self.refresh)

    def close(self):
        self.app.motion.cancel()
        # Main shuts down the executor after cancellation gets a bounded chance.
        self.root.destroy()


def main():
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    app = DemoApp()
    def spin():
        try:
            rclpy.spin(app)
        except ExternalShutdownException:
            pass
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        root = tk.Tk()
        TeachUI(root, app)
        root.mainloop()
    finally:
        app.motion.cancel()
        if app.worker:
            app.worker.join(timeout=6)
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2)
        app.destroy_node()

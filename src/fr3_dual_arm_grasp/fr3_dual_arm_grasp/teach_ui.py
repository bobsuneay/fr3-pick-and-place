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
        ttk.Label(top, text=f'  {mode}  |  速度比例 {self.app.motion.speed:.0%}  |  TCP: world → gripper_tcp').pack(side='left', padx=16)
        ttk.Label(self.root, textvariable=self.feedback, padding=10, font=('TkFixedFont', 10)).pack(fill='x')
        status = ttk.LabelFrame(self.root, text='流程状态', padding=8)
        status.pack(fill='x', padx=10)
        ttk.Label(status, textvariable=self.status, wraplength=1250, font=('', 11)).pack(anchor='w')
        body = ttk.Panedwindow(self.root, orient='horizontal')
        body.pack(fill='both', expand=True, padx=10, pady=10)
        manual, teach = ttk.Frame(body, padding=8), ttk.Frame(body, padding=8)
        body.add(manual, weight=1)
        body.add(teach, weight=2)

        selection = ttk.Frame(manual)
        selection.pack(fill='x')
        ttk.Label(selection, text='当前手臂').pack(side='left')
        ttk.Combobox(selection, textvariable=self.side, values=['right', 'left'], state='readonly', width=10).pack(side='left', padx=8)
        self.button(selection, '读取关节 / TCP', lambda: self.submit('读取当前状态', self.app.capture))
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
        ttk.Label(manual, text='所有运动均经过 MoveIt。\n直线路径必须 100% 成功才执行。\n单点回放只移动关节，不自动改变夹爪。\n示教时可用机器人示教器定位，再采集；\n本界面不直接调用 SDK，不切换拖动模式。', wraplength=410).pack(anchor='w', pady=12)
        self.button(manual, '加载感知候选到输入框（不运动）', self.use_candidate)

        files = ttk.Frame(teach)
        files.pack(fill='x')
        ttk.Entry(files, textvariable=self.path, state='readonly').pack(side='left', fill='x', expand=True)
        self.button(files, '打开', self.load)
        self.button(files, '另存为', self.save_as)
        tree_frame = ttk.Frame(teach)
        tree_frame.pack(fill='both', expand=True, pady=6)
        self.tree = ttk.Treeview(tree_frame, columns=('name', 'saved'), show='headings', selectmode='browse', height=16)
        self.tree.heading('name', text='关键点（按流程顺序采集）')
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
        ttk.Label(teach, text='抓取 / 接取点保存夹持目标开度；接近 / 撤离点保存张开开度。\n原始关节与 TCP 均来自同一次双臂反馈快照，保存单位为 rad / m / 四元数。').pack(anchor='w', pady=6)
        actions = ttk.Frame(self.root, padding=10)
        actions.pack(fill='x')
        self.button(actions, '检查示教点完整性', lambda: self.submit('检查示教点', self.app.book.validate_complete))
        self.button(actions, '运行完整 DEMO', lambda: self.submit('启动完整 demo', self.app.start_demo))
        self.continue_button = ttk.Button(actions, text='确认夹稳 / 放置 · 继续', command=lambda: self.attempt(self.app.continue_demo))
        self.continue_button.pack(side='left', padx=8)
        tk.Button(actions, text='停止流程 / 取消运动', bg='#ba2832', fg='white', command=self.app.motion.cancel).pack(side='left', padx=8)
        self.button(actions, '人工恢复后清除任务状态', self.recover)
        ttk.Label(self.root, text='软件停止不替代硬件急停。停止后保留夹爪；完整流程不会自动重启。预览只检查当前状态到选中目标的一段路径。', padding=6).pack(fill='x')

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
        self.status.set(self.app.status_text + f'  |  零件附着: {self.app.scene.owner or "无"}')
        self.path.set(self.app.points_file)
        try:
            values = self.app.feedback.snapshot()
            lines = []
            for side in ('right', 'left'):
                joints = '  '.join(f'{math.degrees(values[f"{side}_j{i}"]):7.2f}' for i in range(1, 7))
                gap = values[side + '_left_finger_joint']*2
                lines.append(f'{side:5} J° [{joints}]   开口 {gap*1000:.2f} mm  闭合 {gap_to_percent(max(0, min(self.app.open_gap, gap)), self.app.open_gap):.1f}%')
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
        self.continue_button.configure(state='normal' if self.app.awaiting_confirmation else 'disabled')
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

"""Construct the real Tk panel with a fake backend, without importing ROS."""
import importlib
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


def test_panel_builds_and_formats_measured_feedback(monkeypatch):
    tk = pytest.importorskip('tkinter')
    pytest.importorskip('scipy')
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/fr3_dual_arm_grasp'))
    from fr3_dual_arm_grasp.teach_model import Feedback, TeachBook, MEASURED, captured_point
    fake_ros, fake_app = ModuleType('rclpy'), ModuleType('fr3_dual_arm_grasp.app')
    fake_app.DemoApp = object
    monkeypatch.setitem(sys.modules, 'rclpy', fake_ros)
    monkeypatch.setitem(sys.modules, 'fr3_dual_arm_grasp.app', fake_app)
    sys.modules.pop('fr3_dual_arm_grasp.teach_ui', None)
    ui = importlib.import_module('fr3_dual_arm_grasp.teach_ui')
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        pytest.skip('Tk display unavailable: ' + str(exc))
    root.withdraw()
    try:
        feedback = Feedback()
        feedback.update(MEASURED, [0.01]*14)
        point = captured_point(feedback.snapshot(), {s: [0.5, 0, 1, 0, 0, 0, 1] for s in ('left', 'right')}, 0.1)
        book = TeachBook('test', 0.1)
        book.record('ready', point)
        app = SimpleNamespace(motion=SimpleNamespace(enabled=False, speed=0.1, cancel=lambda: None),
                              scene=SimpleNamespace(owner=None), points_file='test.json', feedback=feedback,
                              status_text='test', book=book, latest_capture=point, open_gap=0.1,
                              busy=False, teaching=SimpleNamespace(state='motion'),
                              keypoint_motion_mode='joints',
                              live_poses={s: point[s]['tcp'] for s in ('left', 'right')},
                              live_pose_error='', live_stamp=__import__('time').monotonic(),
                              set_speed=lambda percent: None,
                              set_keypoint_motion_mode=lambda mode: setattr(app, 'keypoint_motion_mode', mode))
        panel = ui.TeachUI(root, app)
        root.update_idletasks()
        assert 'TCP mm [500.00' in panel.feedback.get()
        assert '20.00 mm' in panel.feedback.get()
        assert 'RPY' in panel.details.get()
        assert panel.tcp[0].get() == '500.000'
        assert len(panel.tree.get_children()) == 3
        panel.toggle_keypoint_mode()
        assert app.keypoint_motion_mode == 'tcp'
        assert 'TCP' in panel.keypoint_mode_label.get()
        app.busy = True
        panel.refresh()
        assert all(str(button['state']) == 'disabled' for button in panel.controls)
    finally:
        root.destroy()
        sys.modules.pop('fr3_dual_arm_grasp.teach_ui', None)

"""Offline behavior tests: no ROS, SDK or robot connection."""
from copy import deepcopy
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/fr3_dual_arm_grasp'))
sys.path.insert(0, str(ROOT / 'src/fr3_dual_arm_description'))
from fr3_dual_arm_grasp.teach_model import (
    Feedback, TeachBook, MEASURED, SLOTS, recipe, gap_to_percent,
    percent_to_gap, captured_point, model_fingerprint,
    validate_handover_centerline)
from fr3_dual_arm_grasp.workflow import run_workflow
from fr3_dual_arm_description.model import build_model, semantic, read_yaml


def point():
    values = {name: 0.01 for name in MEASURED}
    return captured_point(values, {s: [0.5, 0, 1, 0, 0, 0, 1] for s in ('left', 'right')}, 0.1)


def book():
    result = TeachBook('model-test', 0.1)
    for name in SLOTS:
        result.record(name, point())
    result.points['right_pregrasp']['right']['gap_m'] = 0.1
    result.points['ready']['left']['gap_m'] = 0.1
    return result


def test_partial_broadcasts_require_both_fresh_arms():
    feedback = Feedback(1)
    left = [j for j in MEASURED if j.startswith('left_')]
    right = [j for j in MEASURED if j.startswith('right_')]
    feedback.update(left, [0.01]*7, now=10)
    with pytest.raises(RuntimeError, match='right_j1'):
        feedback.snapshot(now=10.2)
    feedback.update(right, [0.02]*7, now=10.2)
    snapshot = feedback.snapshot(now=10.4)
    assert snapshot['left_right_finger_joint'] == snapshot['left_left_finger_joint']
    feedback.update(right, [0.02]*7, now=12)
    with pytest.raises(RuntimeError, match='left_j1'):
        feedback.snapshot(now=12)


def test_nan_feedback_does_not_refresh_valid_sample():
    feedback = Feedback()
    feedback.update(MEASURED, [0]*14, now=1)
    feedback.update(MEASURED, [float('nan')]*14, now=2)
    with pytest.raises(RuntimeError):
        feedback.snapshot(now=2.1)


@pytest.mark.parametrize('percent,gap', [(0, 0.1), (50, 0.05), (100, 0)])
def test_gripper_units(percent, gap):
    assert percent_to_gap(percent, 0.1) == pytest.approx(gap)
    assert gap_to_percent(gap, 0.1) == pytest.approx(percent)


@pytest.mark.parametrize('bad', [-1, 101, float('nan'), float('inf')])
def test_invalid_closure_rejected(bad):
    with pytest.raises(ValueError):
        percent_to_gap(bad, 0.1)


def test_save_load_and_wrong_model_is_transactional(tmp_path):
    source = book()
    path = tmp_path / 'points.json'
    source.save(path)
    target = TeachBook('model-test', 0.1)
    target.load(path)
    assert target.points == source.points
    other = TeachBook('other', 0.1)
    with pytest.raises(ValueError, match='schema/model'):
        other.load(path)
    assert other.points == {}


@pytest.mark.parametrize('mutate', [
    lambda p: p.update(frame='base'),
    lambda p: p['right'].update(tcp=[0, 0, 0, 0, 0, 0, 0]),
    lambda p: p['left'].update(joints=[float('inf')]*6),
    lambda p: p['left'].update(tcp_link='tool0'),
    lambda p: p['left'].update(gap_m=-0.01),
])
def test_invalid_point_rejected(mutate):
    p = point()
    mutate(p)
    with pytest.raises(ValueError):
        book().record('ready', p)


def test_missing_points_and_inconsistent_handover():
    b = book()
    b.validate_complete()
    del b.points['ready']
    with pytest.raises(ValueError, match='missing'):
        b.validate_complete()


def test_handover_requires_collinear_opposing_gripper_axes():
    right = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    left = [0.0, 0.0, 1.1, 1.0, 0.0, 0.0, 0.0]
    lateral, angle = validate_handover_centerline(left, right)
    assert lateral == pytest.approx(0.0)
    assert angle == pytest.approx(0.0)
    left[0] = 0.01
    with pytest.raises(ValueError, match='centerlines'):
        validate_handover_centerline(left, right)


class FakeApp:
    def __init__(self, fail_at=None):
        self.book = book()
        self.events = []
        self.stop_event = threading.Event()
        self.recovery_required = False
        self.dwell = 0
        self.fail_at = fail_at
        self.motion = SimpleNamespace(guard=self.guard, gripper=self.grip)
        self.scene = SimpleNamespace(owner=None, place_initial=lambda p: self.event('initial'),
                                     attach=self.attach, detach=self.detach,
                                     set_touch=lambda sides: self.event('touch', *sides),
                                     allow=lambda sides, table=False: self.event('allow', str(table)))

    def guard(self, execute=True):
        if self.stop_event.is_set():
            raise RuntimeError('cancelled')

    def event(self, *args):
        self.events.append(args)
        if self.fail_at == args:
            raise RuntimeError('injected failure')

    def grip(self, side, gap, execute=True):
        self.event('grip', side, gap)

    def verify_grasp(self, side):
        self.event('grasp_ok', side)

    def verify_handover_alignment(self):
        self.event('handover_aligned')

    def attach(self, side, transfer=False):
        self.event('transfer' if transfer else 'attach', side)
        self.scene.owner = side

    def detach(self):
        self.event('detach')
        self.scene.owner = None

    def move_point(self, key, side, execute, linear=False):
        self.event('move', side, key)

    def initialize_scene(self):
        pass

    def grasp_target(self):
        return self.book.points['right_pregrasp']['right']['tcp']

    def scan_display(self, side):
        self.event('scan', side)

    def move_handover_receive(self):
        self.event('move', 'left', 'left_receive')

    def retreat_donor(self):
        self.event('retreat', 'right')

    def place_move(self, above):
        self.event('place', above)

    def publish(self, text):
        pass


def test_recipe_releases_donor_only_after_automatic_receiver_grasp_and_transfer():
    app = FakeApp()
    run_workflow(app)
    e = app.events
    close_left = e.index(('grip', 'left', 0.0))
    transfer = e.index(('transfer', 'left'))
    release_right = e.index(('grip', 'right', 0.1), transfer)
    assert close_left < e.index(('grasp_ok', 'left'), close_left) < transfer < release_right
    assert e.index(('grip', 'left', 0.1), transfer) < e.index(('detach',))
    assert app.scene.owner is None and not app.recovery_required


@pytest.mark.parametrize('failure', [
    ('grip', 'left', 0.0), ('grasp_ok', 'left'), ('transfer', 'left'),
    ('move', 'left', 'left_receive'),
])
def test_handover_failure_never_opens_donor(failure):
    app = FakeApp(fail_at=failure)
    with pytest.raises(RuntimeError, match='injected'):
        run_workflow(app)
    assert app.recovery_required
    # Only the pre-pick opening occurred; no release after handover failure.
    assert app.events.count(('grip', 'right', 0.1)) == 1
    assert ('detach',) not in app.events
    with pytest.raises(RuntimeError, match='Recover'):
        run_workflow(app)


def test_head_only_and_full_camera_backup_preserve_mechanical_calibration():
    share = ROOT / 'src/fr3_dual_arm_description'
    arms = read_yaml(share / 'config/arms.yaml')
    current_scene = read_yaml(share / 'config/scene.yaml')
    full_scene = read_yaml(share / 'config/scene.full_cameras.yaml')
    assert current_scene['cameras'] == {}
    for key in current_scene.keys() - {'cameras'}:
        assert current_scene[key] == full_scene[key]
    current = build_model(share, share / 'config/scene.yaml', arms, mode='mock')
    full = build_model(share, share / 'config/scene.full_cameras.yaml', arms, mode='mock')
    assert current.find("link[@name='head_camera_link']") is not None
    assert current.find("link[@name='left_d435i_link']") is None
    assert current.find("link[@name='waist_camera_link']") is None
    import xml.etree.ElementTree as ET
    for joint in current.findall('joint'):
        name = joint.get('name')
        assert ET.tostring(joint) == ET.tostring(full.find(f"joint[@name='{name}']"))
    assert 'left_d435i' not in semantic(current, arms)


def test_fingerprint_tracks_scene_and_arms(tmp_path):
    arms, scene = tmp_path/'arms', tmp_path/'scene'
    arms.write_text('a')
    scene.write_text('s')
    before = model_fingerprint(arms, scene)
    scene.write_text('changed')
    assert before != model_fingerprint(arms, scene)


def test_old_points_load_without_requiring_obsolete_views(tmp_path):
    import json
    source = book()
    path = tmp_path / 'legacy.json'
    source.save(path)
    document = json.loads(path.read_text())
    document['points']['right_view_1'] = point()
    path.write_text(json.dumps(document))
    target = TeachBook('model-test', 0.1)
    target.load(path)
    target.validate_complete()
    assert len(target.points) == 3
    assert 'right_view_1' in json.loads(path.read_text())['points']

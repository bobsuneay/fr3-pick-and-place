"""ROS-independent teaching file, units, state freshness and demo recipe."""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import threading
import time

SIDES = ('left', 'right')
JOINTS = tuple(f'{s}_j{i}' for s in SIDES for i in range(1, 7))
MEASURED = JOINTS + tuple(f'{s}_left_finger_joint' for s in SIDES)
SLOTS = {
    'ready': '双臂就绪（双夹爪张开）',
    'right_pregrasp': '右手抓取接近点（张开，复用于抬升）',
    'right_grasp': '右手抓取点（夹持开度）',
    'right_display': '展示参考（右手姿态；双臂共用，相机前 30 cm）',
    'handover_ready': '双臂交接预备（左手保持接近距离）',
    'left_receive': '左手接取点（夹持开度，右手不动）',
    'left_place': '左手放置点（保持夹持）',
}
LEGACY_SLOTS = {'right_lift', 'right_view_1', 'right_view_2', 'right_retreat',
                'left_display', 'left_view_1', 'left_view_2', 'left_preplace', 'left_retreat'}


def tcp_z_axis(pose):
    x, y, z, w = pose_vector(pose)[3:]
    return [2.0 * (x*z + y*w),
            2.0 * (y*z - x*w),
            1.0 - 2.0 * (x*x + y*y)]


def handover_centerline_error(left_pose, right_pose):
    """Return lateral axis separation [m] and opposing-axis angle [rad]."""
    left_axis, right_axis = tcp_z_axis(left_pose), tcp_z_axis(right_pose)
    opposing = max(-1.0, min(1.0, -sum(a*b for a, b in zip(left_axis, right_axis))))
    angle = math.acos(opposing)
    delta = [float(left_pose[i]) - float(right_pose[i]) for i in range(3)]
    axial = sum(a*b for a, b in zip(delta, right_axis))
    lateral = math.sqrt(sum((d - axial*a) ** 2 for d, a in zip(delta, right_axis)))
    return lateral, angle


def validate_handover_centerline(left_pose, right_pose,
                                 lateral_tolerance=0.005,
                                 angular_tolerance=math.radians(5.0)):
    lateral, angle = handover_centerline_error(left_pose, right_pose)
    if lateral > lateral_tolerance or angle > angular_tolerance:
        raise ValueError(
            'Handover gripper centerlines do not coincide: '
            f'lateral={lateral*1000:.1f} mm (limit {lateral_tolerance*1000:.1f}), '
            f'angle={math.degrees(angle):.1f} deg '
            f'(limit {math.degrees(angular_tolerance):.1f})')
    return lateral, angle



def finite_vector(value, length, label):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f'{label}: expected {length} numbers')
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(f'{label}: non-finite/non-numeric value')
    return [float(x) for x in value]


def pose_vector(value):
    values = finite_vector(value, 7, 'TCP xyz + quaternion xyzw')
    if abs(sum(x*x for x in values[3:]) - 1.0) > 0.001:
        raise ValueError('TCP quaternion must be normalized')
    return values


def gap_to_percent(gap, open_gap):
    if not math.isfinite(gap) or not 0 <= gap <= open_gap:
        raise ValueError('Gripper gap outside calibrated range')
    return 100.0 * (1.0 - gap / open_gap)


def percent_to_gap(percent, open_gap):
    if not math.isfinite(percent) or not 0 <= percent <= 100:
        raise ValueError('Closure must be 0..100 percent')
    return open_gap * (1.0 - percent / 100.0)


def model_fingerprint(arms_file, scene_file):
    return hashlib.sha256(Path(arms_file).read_bytes() + b'\0' + Path(scene_file).read_bytes()).hexdigest()


class Feedback:
    """Merge partial broadcasts, requiring each commanded joint to be fresh."""
    def __init__(self, max_age=1.0):
        self.max_age = max_age
        self.lock = threading.Lock()
        self.values = {}

    def update(self, names, positions, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            for name, value in zip(names, positions):
                if name in MEASURED and math.isfinite(value):
                    self.values[name] = (float(value), now)

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            bad = [j for j in MEASURED if j not in self.values or not 0 <= now-self.values[j][1] <= self.max_age]
            if bad:
                raise RuntimeError('Missing/stale robot feedback: ' + ', '.join(bad))
            result = {j: self.values[j][0] for j in MEASURED}
        # Mimic joints are computed from their measured masters, never commanded.
        for side in SIDES:
            result[f'{side}_right_finger_joint'] = result[f'{side}_left_finger_joint']
        return result


class TeachBook:
    def __init__(self, fingerprint, open_gap):
        self.fingerprint, self.open_gap = fingerprint, open_gap
        self.points = {}

    def validate_point(self, point):
        if point.get('frame') != 'world':
            raise ValueError('Teaching frame must be world')
        for side in SIDES:
            data = point[side]
            finite_vector(data['joints'], 6, side + ' joints/rad')
            pose_vector(data['tcp'])
            if data['tcp_link'] != side + '_gripper_tcp':
                raise ValueError('TCP link does not match robot model')
            gap_to_percent(data['gap_m'], self.open_gap)

    def record(self, name, point):
        if name not in SLOTS:
            raise ValueError('Unknown teaching slot')
        self.validate_point(point)
        self.points[name] = deepcopy(point)

    def validate_complete(self):
        missing = set(SLOTS) - self.points.keys()
        if missing:
            raise ValueError('Teach missing points: ' + ', '.join(sorted(missing)))
        for point in self.points.values():
            self.validate_point(point)
        # The donor must stay fixed while the receiver approaches.
        donor = self.points['handover_ready']['right']['joints']
        receive_donor = self.points['left_receive']['right']['joints']
        if max(abs(a-b) for a, b in zip(donor, receive_donor)) > 0.02:
            raise ValueError('Right arm changed between handover_ready and left_receive; reteach')
        for side, closed, opened in [('right', 'right_grasp', 'right_pregrasp'),
                                     ('left', 'left_receive', 'ready')]:
            if self.points[opened][side]['gap_m'] <= self.points[closed][side]['gap_m'] + 0.002:
                raise ValueError(f'{side}: teach an open gap at least 2 mm wider than the grasp gap')

    def save(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        document = dict(schema_version=1, model_sha256=self.fingerprint,
                        units=dict(joints='rad', position='m', orientation='quaternion_xyzw', gap='m'),
                        points=self.points)
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        temporary.replace(path)

    def load(self, path):
        document = json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
        if document.get('schema_version') != 1 or document.get('model_sha256') != self.fingerprint:
            raise ValueError('Teaching file schema/model differs; use the matching arms and scene or reteach')
        if document.get('units') != dict(joints='rad', position='m', orientation='quaternion_xyzw', gap='m'):
            raise ValueError('Unsupported units; SDK mm/degree files cannot be replayed directly')
        candidate = TeachBook(self.fingerprint, self.open_gap)
        for name, point in document['points'].items():
            if name in LEGACY_SLOTS:
                candidate.validate_point(point)
                continue  # Preserve old file on disk; obsolete points are not required.
            candidate.record(name, point)
        self.points = candidate.points


def captured_point(joints, poses, open_gap):
    point = dict(frame='world', captured_at=datetime.now(timezone.utc).isoformat())
    for side in SIDES:
        gap = 2.0 * joints[side + '_left_finger_joint']
        # Permit only tiny encoder roundoff at a hard endpoint.
        if gap < -0.0002 or gap > open_gap + 0.0002:
            raise ValueError('Measured gripper opening outside calibration')
        point[side] = dict(joints=[joints[f'{side}_j{i}'] for i in range(1, 7)],
                           tcp=poses[side], tcp_link=side + '_gripper_tcp',
                           gap_m=max(0.0, min(open_gap, gap)))
    return point


def recipe():
    """Explicit ownership transitions; motion never silently changes a gripper."""
    return [
        ('move', 'both', 'ready'),
        ('grip', 'right', 'right_pregrasp'), ('grip', 'left', 'ready'),
        ('move', 'right', 'right_pregrasp'), ('approach', 'right', 'right_grasp'),
        ('grasp', 'right', 'right_grasp'),
        ('attach', 'right', ''), ('lift', 'right', 'right_pregrasp'),
        ('scan', 'right', 'right_display'),
        ('move', 'both', 'handover_ready'), ('touch', 'left', ''),
        ('receive', 'left', 'left_receive'), ('grasp', 'left', 'left_receive'),
        ('transfer', 'left', ''), ('grip', 'right', 'right_pregrasp'),
        ('retreat', 'right', ''), ('touch_only', 'left', ''),
        ('move', 'right', 'ready'), ('scan', 'left', 'right_display'),
        ('preplace', 'left', ''), ('place', 'left', ''),
        ('grip', 'left', 'ready'), ('detach', 'left', ''),
        ('preplace', 'left', ''), ('move', 'left', 'ready'),
    ]

"""Camera model and deployment contracts without USB devices or ROS processes."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import pytest

ROOT = Path(__file__).resolve().parents[1]
for package in ('fr3_dual_arm_description', 'fr3_dual_arm_bringup'):
    sys.path.insert(0, str(ROOT / 'src' / package))
from fr3_dual_arm_description.model import build_model, read_yaml, semantic
from fr3_dual_arm_bringup.cameras import selected_cameras, camera_remappings


@pytest.mark.parametrize('scene_name', ['scene.yaml', 'scene.full_cameras.yaml'])
def test_d405_mounts_sensors_and_collision_links(scene_name):
    share = ROOT / 'src/fr3_dual_arm_description'
    arms = read_yaml(share / 'config/arms.yaml')
    root = build_model(share, share / 'config' / scene_name, arms, mode='gazebo')
    links = {link.get('name') for link in root.findall('link')}
    parents = {j.find('child').get('link'): j.find('parent').get('link') for j in root.findall('joint')}
    for side in ('left', 'right'):
        name = side + '_d405'
        assert parents[name + '_link'] == name + '_bracket'
        assert parents[name + '_bracket'] == side + '_gripper_palm'
        link = root.find(f"link[@name='{name}_link']")
        assert link.find('collision/geometry/box').get('size') == '0.023 0.042 0.042'
        assert float(link.find('inertial/mass').get('value')) == 0.06
        sensor = root.find(f"gazebo[@reference='{name}_link']/sensor")
        assert sensor.get('type') == 'depth'
        assert float(sensor.findtext('camera/clip/near')) == .07
        assert float(sensor.findtext('camera/clip/far')) == .5
        plugin = sensor.find('plugin')
        assert plugin.findtext('camera_name') == name
        assert plugin.findtext('frame_name') in links
        assert name + '_optical_frame' in links
    for pair in ET.fromstring(semantic(root, arms)).findall('disable_collisions'):
        assert pair.get('link1') in links and pair.get('link2') in links


def config(left='123', right='456'):
    return {'cameras': {'left_d405': {'serial_no': left}, 'right_d405': {'serial_no': right}}}


def test_distinct_serials_and_public_topics():
    cameras = selected_cameras(config())
    assert [params['serial_no'] for _, params in cameras] == ['_123', '_456']
    for name, params in cameras:
        assert params['camera_name'] == name
        assert params['base_frame_id'] == 'link'
        topics = dict(camera_remappings(name))
        assert topics['~/depth/color/points'] == f'/{name}/points'
        assert topics['~/color/image_raw'] == f'/{name}/image_raw'
        assert topics['~/depth/image_rect_raw'] == f'/{name}/depth/image_raw'
        assert topics['~/color/camera_info'] == f'/{name}/camera_info'


@pytest.mark.parametrize('left,right', [('', ''), ('123', '_123'), ('abc', '456')])
def test_invalid_device_selection_fails_before_launch(left, right):
    with pytest.raises(ValueError):
        selected_cameras(config(left, right))


def test_single_camera_and_serial_overrides():
    cfg = config('', '')
    cfg['cameras']['right_d405']['enabled'] = False
    cameras = selected_cameras(cfg, {'left': '789'})
    assert len(cameras) == 1
    assert cameras[0][1]['serial_no'] == '_789'


def test_rviz_exposes_both_wrist_images_and_perception_cloud():
    rviz = (ROOT / 'src/fr3_dual_arm_description/config/dual_arm.rviz').read_text(
        encoding='utf-8')
    for side in ('left', 'right'):
        assert f'Name: {side.title()} Wrist RGB' in rviz
        assert f'Value: /{side}_d405/image_raw' in rviz
        assert f'Value: /{side}_d405/depth/image_raw' in rviz
    assert 'Name: Right Wrist PointCloud' in rviz
    assert 'Value: /right_d405/points' in rviz

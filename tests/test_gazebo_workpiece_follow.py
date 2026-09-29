"""Static contracts for Gazebo-only physical workpiece following."""
from pathlib import Path
import ast


ROOT = Path(__file__).resolve().parents[1]


def functions(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    return {node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}


def test_gazebo_follow_uses_owner_target_and_nonblocking_service():
    path = ROOT / 'src/fr3_dual_arm_grasp/fr3_dual_arm_grasp/app.py'
    source = path.read_text(encoding='utf-8')
    methods = functions(path)
    assert '_follow_gazebo_workpiece' in methods
    body = ast.get_source_segment(source, methods['_follow_gazebo_workpiece'])
    assert 'self.scene.owner' in body
    assert 'self.scene.target_id' in body
    assert "owner + '_gripper_tcp'" in body
    assert 'self.set_entity.call_async(request)' in body
    assert 'self.motion.service' not in body


def test_detach_releases_gazebo_entity_identity():
    path = ROOT / 'src/fr3_dual_arm_grasp/fr3_dual_arm_grasp/demo_scene.py'
    source = path.read_text(encoding='utf-8')
    detach = ast.get_source_segment(source, functions(path)['detach'])
    assert 'self.target_id = None' in detach

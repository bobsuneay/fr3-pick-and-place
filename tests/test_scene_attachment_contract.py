"""Guard the MoveIt initial-attachment update ordering contract."""
from pathlib import Path
import ast


ROOT = Path(__file__).resolve().parents[1]


def test_initial_attach_does_not_duplicate_world_remove():
    path = ROOT / 'src/fr3_dual_arm_grasp/fr3_dual_arm_grasp/demo_scene.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    attach = next(node for node in ast.walk(tree)
                  if isinstance(node, ast.FunctionDef) and node.name == 'attach')

    world_appends = []
    for node in ast.walk(attach):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        target = ast.unparse(node.func.value)
        if node.func.attr == 'append' and target == 'req.scene.world.collision_objects':
            world_appends.append(node.lineno)

    assert world_appends == []

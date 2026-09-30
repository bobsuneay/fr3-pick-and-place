"""Contracts for contact-gated fixed-joint Gazebo grasp assistance."""
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/fr3_dual_arm_gazebo'))
from fr3_dual_arm_gazebo.world_builder import load_scene, world_xml


class AssistedGraspPluginTests(unittest.TestCase):
    def test_default_world_loads_fixed_joint_plugin_for_pickup_entity(self):
        scene = load_scene(ROOT / 'src/fr3_dual_arm_description/config/scene.yaml')
        xml = world_xml(scene)
        self.assertIn('libfr3_dual_arm_assisted_grasp.so', xml)
        self.assertIn('<robot_model>fr3_dual_arm</robot_model>', xml)
        self.assertIn('<object_model>right_part</object_model>', xml)

    def test_plugin_requires_bilateral_contact_and_creates_fixed_joint(self):
        source = (ROOT / 'src/fr3_dual_arm_grasp_sim/src/assisted_grasp.cpp').read_text(encoding='utf-8')
        self.assertIn('contacts_[i].ready(now)', source)
        self.assertIn('if(now < p->deadline)return;', source)
        self.assertIn('p->deadline=world_->SimTime().Double()+2.0', source)
        self.assertIn('accepted; waiting for Gazebo contact evidence', source)
        self.assertIn('const bool handover = !owner_.empty() && owner_ != p->side;', source)
        self.assertIn('CreateJoint("fixed",robot)', source)
        self.assertIn('next->Attach(palm,body)', source)
        self.assertIn('grasp_->Detach()', source)

    def test_python_uses_plugin_services_not_periodic_entity_teleport(self):
        source = (ROOT / 'src/fr3_dual_arm_grasp/fr3_dual_arm_grasp/app.py').read_text(encoding='utf-8')
        self.assertIn("'/grasp/sim/{side}_grasp'", source)
        self.assertIn("'/grasp/sim/owner'", source)
        self.assertNotIn('_follow_gazebo_workpiece', source)
        self.assertNotIn('gazebo_follow_future', source)


if __name__ == '__main__':
    unittest.main()

"""Standalone pickup object and legacy gridded-scene contracts."""
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/fr3_dual_arm_gazebo'))
from fr3_dual_arm_gazebo.world_builder import (
    bin_boxes, load_scene, part_poses, random_pickup_xy, world_xml)


class FixedRandom:
    def __init__(self, values):
        self.values = iter(values)

    def random(self):
        return next(self.values)


class SinglePartSceneTests(unittest.TestCase):
    def setUp(self):
        self.config = ROOT / 'src/fr3_dual_arm_description/config'

    def test_default_scene_has_one_standalone_right_part_and_no_right_bin(self):
        scene = load_scene(self.config / 'scene.yaml')
        poses = part_poses(scene)
        self.assertEqual([name for name, _ in poses], ['right_part'])
        self.assertEqual(poses[0][1][:2], [0.50, -0.48])
        names = [name for name, _, _ in bin_boxes(scene)]
        self.assertTrue(any(name.startswith('left_bin_') for name in names))
        self.assertFalse(any(name.startswith('right_bin_') for name in names))
        xml = world_xml(scene)
        self.assertIn('<model name="right_part">', xml)
        self.assertNotIn('<model name="right_part_00_00">', xml)

    def test_backup_restores_right_grid_and_twelve_parts(self):
        scene = load_scene(self.config / 'scene.grid_bins.backup.yaml')
        self.assertEqual(len(part_poses(scene)), 12)
        self.assertTrue(any(name.startswith('right_bin_')
                            for name, _, _ in bin_boxes(scene)))

    def test_random_position_is_inside_ten_centimetre_disc(self):
        scene = load_scene(self.config / 'scene.yaml')
        # sqrt(.25) * .10 = .05 m at angle zero.
        xy = random_pickup_xy(scene, FixedRandom([.25, 0.0]))
        self.assertAlmostEqual(xy[0], .55)
        self.assertAlmostEqual(xy[1], -.48)
        cx, cy = scene['pickup']['center_xy']
        self.assertLessEqual((xy[0]-cx)**2 + (xy[1]-cy)**2, .10**2)


if __name__ == '__main__':
    unittest.main()

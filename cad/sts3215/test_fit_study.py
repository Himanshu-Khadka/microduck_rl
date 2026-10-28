"""CAD-only regressions; no simulator/training environment is required."""

import unittest

import numpy as np
import trimesh
from build_fit_study import (
    assembly_records,
    make_candidate,
    original_upper,
    servo_placements,
)


class FitStudyTests(unittest.TestCase):
    def test_actual_servo_datum_maps_to_scaled_joint_centres(self):
        for scale in (1.5, 1.6, 1.8):
            ts = servo_placements(scale)
            targets = [np.zeros(3), np.array([22, 35.7771, -4]) * scale]
            for t, target in zip(ts, targets):
                np.testing.assert_allclose(
                    t @ np.array([12.5, 0, 18.7, 1]), np.r_[target, 1], atol=1e-10
                )
                np.testing.assert_allclose(
                    t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-10
                )
                self.assertAlmostEqual(np.linalg.det(t[:3, :3]), 1)
                # Both motor shafts point toward the original lower joint plane.
                np.testing.assert_array_equal(t[:3, 2], [0, 0, -1])

    def test_cut_preserves_connected_volume_and_clears_pockets(self):
        original = original_upper(1.6)
        cut, cutters = make_candidate(original, servo_placements(1.6), 0.4)
        self.assertTrue(cut.is_volume)
        self.assertEqual(len(cut.split()), 1)
        self.assertGreater(cut.volume, 0)
        self.assertLess(cut.volume, original.volume)
        for cutter in cutters:
            intersection = trimesh.boolean.intersection(
                [cut, cutter], engine="manifold"
            )
            self.assertLess(abs(intersection.volume), 1e-4)

    def test_part_and_world_frames_agree(self):
        records, frames = assembly_records(1.6)
        _, _, world = next(
            r for r in records if r[0] == "upper_leg_left" and r[1] == "upper_leg_left"
        )
        local = original_upper(1.6)
        local.apply_transform(frames["upper_leg_left"])
        np.testing.assert_allclose(local.vertices, world.vertices, atol=1e-5)


if __name__ == "__main__":
    unittest.main()

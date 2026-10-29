"""Tests for the URDF chain kinematics that now backs the bridge.

The web viewer computes FK/IK from the same URDF tree in JS. These tests pin
the Python side to the numbers the viewer was measured against, so the two
cannot drift apart silently.
"""
import unittest
from pathlib import Path

from urdf_kinematics import JOINTS, UrdfKinematics

URDF = Path(__file__).resolve().parent / "arm-model" / "arm.urdf"


class UrdfKinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = UrdfKinematics(URDF)

    def test_chain_shape(self):
        self.assertEqual(len(self.kin.chain), JOINTS)
        self.assertEqual(self.kin.names,
                         ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

    def test_zero_pose_matches_viewer(self):
        """Measured on the live viewer page: (0, 181.5, 172.6) mm, Y-up."""
        pos, _ = self.kin.fk([0.0] * JOINTS)
        self.assertAlmostEqual(pos[0], 0.0, places=3)
        self.assertAlmostEqual(pos[1], 181.5, places=1)
        self.assertAlmostEqual(pos[2], 172.6, places=1)

    def test_fk_is_deterministic(self):
        a, _ = self.kin.fk([10.0, -20.0, 30.0, -5.0, 15.0, 0.0])
        b, _ = self.kin.fk([10.0, -20.0, 30.0, -5.0, 15.0, 0.0])
        self.assertEqual(a, b)

    def test_roundtrip_has_no_failures(self):
        worst, failures = self.kin.roundtrip(80)
        self.assertEqual(failures, 0)
        self.assertLess(worst, 1e-3)

    def test_ik_converges_and_verifies(self):
        target = (0.0, 250.0, 200.0)
        solved, residual, _iters, converged = self.kin.ik(target, seed_deg=[0.0] * JOINTS)
        self.assertTrue(converged)
        self.assertLess(residual, 0.01)
        back, _ = self.kin.fk(solved)
        for i in range(3):
            self.assertAlmostEqual(back[i], target[i], places=1)

    def test_ik_keeps_a_small_move_small(self):
        """Seeding from the current pose must not teleport the arm."""
        seed = [5.0, -10.0, 20.0, 0.0, 5.0, 0.0]
        start, _ = self.kin.fk(seed)
        target = (start[0] + 8.0, start[1], start[2])
        solved, _residual, _iters, converged = self.kin.ik(target, seed_deg=seed)
        self.assertTrue(converged)
        for a, b in zip(seed, solved):
            self.assertLess(abs(a - b), 25.0)

    def test_ik_reports_failure_outside_reach(self):
        _solved, residual, _iters, converged = self.kin.ik((9000.0, 9000.0, 9000.0))
        self.assertFalse(converged)
        self.assertGreater(residual, 1.0)


if __name__ == "__main__":
    unittest.main()

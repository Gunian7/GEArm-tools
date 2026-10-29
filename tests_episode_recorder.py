"""Tests for the episode recorder: schema, limits, and replay verification."""
import json
import shutil
import tempfile
import unittest
from pathlib import Path

from episode_recorder import JOINTS, STATE_DIM, EpisodeRecorder, RecordingError
from urdf_kinematics import UrdfKinematics

URDF = Path(__file__).resolve().parent / "arm-model" / "arm.urdf"


class EpisodeRecorderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gearm-ep-"))
        self.kin = UrdfKinematics(URDF)
        self.rec = EpisodeRecorder(self.kin, self.tmp)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_limits_are_read_from_urdf(self):
        """Only 3 of 6 joints declare travel; the rest must stay None.

        The URDF stores -1.5708 / 1.5708 (rounded pi/2), so compare with a
        tolerance rather than expecting an exact 90 degrees.
        """
        declared = [l for l in self.kin.limits if l is not None]
        self.assertEqual(len(declared), 3)
        self.assertAlmostEqual(self.kin.limits[1][0], -90.0, places=2)
        self.assertAlmostEqual(self.kin.limits[1][1], 90.0, places=2)
        self.assertAlmostEqual(self.kin.limits[2][0], 0.0, places=6)
        self.assertAlmostEqual(self.kin.limits[2][1], 180.0, places=2)
        self.assertAlmostEqual(self.kin.limits[4][0], -90.0, places=2)
        self.assertAlmostEqual(self.kin.limits[4][1], 0.0, places=6)
        self.assertIsNone(self.kin.limits[0])

    def test_limit_violation_is_detected(self):
        bad = self.kin.limit_violations([0.0, 0.0, 200.0, 0.0, 0.0, 0.0])
        self.assertEqual(len(bad), 1)
        self.assertEqual(bad[0][0], 2)
        self.assertEqual(self.kin.limit_violations([0.0, 0.0, 90.0, 0.0, 0.0, 0.0]), [])

    def test_start_requires_a_task(self):
        with self.assertRaises(RecordingError):
            self.rec.start("   ")

    def test_record_rejects_out_of_limit_joints(self):
        self.rec.start("pick up the cube")
        with self.assertRaises(RecordingError):
            self.rec.record([0.0, 0.0, 250.0, 0.0, 0.0, 0.0])
        self.rec.stop()

    def test_record_writes_a_complete_frame(self):
        self.rec.start("pick up the cube")
        frame = self.rec.record([0.0, 10.0, 30.0, 0.0, -10.0, 0.0], timestamp=1000.0)
        self.assertEqual(frame["frame_index"], 0)
        self.assertEqual(len(frame["observation.state"]), STATE_DIM)
        self.assertEqual(frame["action"], frame["observation.state"][:JOINTS])
        self.assertEqual(frame["task"], "pick up the cube")
        self.rec.stop()

    def test_stop_writes_meta_and_reopens(self):
        self.rec.start("move home")
        for i in range(5):
            self.rec.record([0.0, 5.0 * i, 30.0, 0.0, -5.0, 0.0], timestamp=1000.0 + i * 0.02)
        meta = self.rec.stop()
        self.assertEqual(meta["frames"], 5)
        self.assertEqual(meta["task"], "move home")
        self.assertIsNone(self.rec.episode)
        episodes = self.rec.list_episodes()
        self.assertEqual(len(episodes), 1)
        self.assertTrue(episodes[0]["complete"])

    def test_episode_index_increments(self):
        first = self.rec.start("a")
        self.rec.stop()
        second = self.rec.start("b")
        self.rec.stop()
        self.assertEqual(second, first + 1)

    def test_verify_passes_for_a_good_episode(self):
        self.rec.start("good episode")
        for i in range(10):
            self.rec.record([1.0 * i, 10.0, 30.0, 0.0, -10.0, 0.0], timestamp=1000.0 + i * 0.02)
        self.rec.stop()
        report = self.rec.verify(0)
        self.assertTrue(report["ok"], report["problems"])
        self.assertEqual(report["frames"], 10)
        self.assertLess(report["tcp_reproduce_worst_mm"], 1e-6)
        # 10 frames span 9 intervals of 20 ms -> 0.18 s, so frames/duration is
        # 55.6 Hz. mean_hz is frames/duration, not the sampling interval.
        self.assertAlmostEqual(report["duration_s"], 0.18, places=6)
        self.assertAlmostEqual(report["mean_hz"], 55.556, places=2)

    def test_verify_catches_tampered_tcp(self):
        """A recorded TCP that FK cannot reproduce must fail verification."""
        self.rec.start("tampered")
        self.rec.record([0.0, 10.0, 30.0, 0.0, -10.0, 0.0], timestamp=1000.0)
        self.rec.stop()
        path = self.tmp / "episode_000000" / "frames.jsonl"
        frame = json.loads(path.read_text(encoding="utf-8").splitlines()[0])
        frame["observation.state"][JOINTS] += 500.0
        path.write_text(json.dumps(frame) + "\n", encoding="utf-8")
        report = self.rec.verify(0)
        self.assertFalse(report["ok"])
        self.assertTrue(any("reproducible" in p for p in report["problems"]))

    def test_verify_catches_backwards_time(self):
        self.rec.start("backwards")
        self.rec.record([0.0, 10.0, 30.0, 0.0, -10.0, 0.0], timestamp=1000.0)
        self.rec.record([0.0, 11.0, 30.0, 0.0, -10.0, 0.0], timestamp=999.0)
        self.rec.stop()
        report = self.rec.verify(0)
        self.assertFalse(report["ok"])
        self.assertTrue(any("backwards" in p for p in report["problems"]))

    def test_verify_catches_frame_index_gap(self):
        self.rec.start("gap")
        self.rec.record([0.0, 10.0, 30.0, 0.0, -10.0, 0.0], timestamp=1000.0)
        self.rec.record([0.0, 11.0, 30.0, 0.0, -10.0, 0.0], timestamp=1000.02)
        self.rec.stop()
        path = self.tmp / "episode_000000" / "frames.jsonl"
        lines = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
        lines[1]["frame_index"] = 7
        path.write_text("\n".join(json.dumps(l) for l in lines) + "\n", encoding="utf-8")
        report = self.rec.verify(0)
        self.assertFalse(report["ok"])

    def test_second_episode_does_not_leak_state(self):
        self.rec.start("first")
        self.rec.record([0.0, 10.0, 30.0, 0.0, -10.0, 0.0], timestamp=1000.0)
        self.rec.stop()
        self.rec.start("second")
        frame = self.rec.record([0.0, 20.0, 30.0, 0.0, -10.0, 0.0], timestamp=2000.0)
        self.assertEqual(frame["episode_index"], 1)
        self.assertEqual(frame["frame_index"], 0)
        self.rec.stop()
        self.assertEqual(self.rec.verify(0)["ok"], True)
        self.assertEqual(self.rec.verify(1)["ok"], True)


if __name__ == "__main__":
    unittest.main()

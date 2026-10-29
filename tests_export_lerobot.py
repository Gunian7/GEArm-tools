"""Tests for the LeRobot exporter: layout, metadata, and stats."""
import json
import shutil
import tempfile
import unittest
from pathlib import Path

from export_lerobot import CHUNK, JOINT_NAMES, STATE_DIM, STATE_NAMES, LerobotExporter
from urdf_kinematics import UrdfKinematics

URDF = Path(__file__).resolve().parent / "arm-model" / "arm.urdf"


class LerobotExporterTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gearm-lerobot-"))
        self.kin = UrdfKinematics(URDF)
        from episode_recorder import EpisodeRecorder
        self.rec = EpisodeRecorder(self.kin, self.tmp / "episodes")
        self.exporter = LerobotExporter(self.kin, self.tmp / "episodes")
        # Two episodes with different tasks.
        self.rec.start("把方块搬到右边")
        for i in range(5):
            self.rec.record([1.0 * i, 10.0, 30.0, 0.0, -10.0, 0.0],
                            timestamp=1000.0 + i * 0.02)
        self.rec.stop()
        self.rec.start("把方块放回左边")
        for i in range(3):
            self.rec.record([2.0 * i, 20.0, 40.0, 0.0, -20.0, 0.0],
                            timestamp=2000.0 + i * 0.02)
        self.rec.stop()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_export_writes_the_full_layout(self):
        out = self.tmp / "dataset"
        result = self.exporter.export(out)
        self.assertTrue((out / "meta" / "info.json").exists())
        self.assertTrue((out / "meta" / "tasks.jsonl").exists())
        self.assertTrue((out / "meta" / "episodes.jsonl").exists())
        self.assertTrue((out / "stats.json").exists())
        data_dir = out / "data" / CHUNK
        self.assertEqual(len(list(data_dir.glob("episode_*"))), 2)
        self.assertEqual(result["episodes"], 2)
        self.assertEqual(result["frames"], 8)
        self.assertEqual(result["tasks"], 2)
        self.assertEqual(result["skipped"], 0)

    def test_info_json_feature_spec(self):
        out = self.tmp / "dataset"
        self.exporter.export(out)
        info = json.loads((out / "meta" / "info.json").read_text(encoding="utf-8"))
        self.assertEqual(info["fps"], 50.0)
        self.assertEqual(info["features"]["observation.state"]["shape"], [STATE_DIM])
        self.assertEqual(info["features"]["observation.state"]["names"], STATE_NAMES)
        self.assertEqual(info["features"]["action"]["shape"], [6])
        self.assertEqual(info["features"]["action"]["names"], JOINT_NAMES)

    def test_tasks_and_episodes_jsonl(self):
        out = self.tmp / "dataset"
        self.exporter.export(out)
        tasks = [json.loads(l) for l in
                 (out / "meta" / "tasks.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
        self.assertEqual(len(tasks), 2)
        self.assertEqual([t["task"] for t in tasks],
                         ["把方块搬到右边", "把方块放回左边"])
        episodes = [json.loads(l) for l in
                    (out / "meta" / "episodes.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
        self.assertEqual([e["length"] for e in episodes], [5, 3])
        self.assertEqual([e["episode_index"] for e in episodes], [0, 1])

    def test_stats_cover_state_and_action(self):
        out = self.tmp / "dataset"
        self.exporter.export(out)
        stats = json.loads((out / "stats.json").read_text(encoding="utf-8"))
        self.assertIn("observation.state", stats)
        self.assertIn("action", stats)
        for key in ("observation.state", "action"):
            self.assertEqual(len(stats[key]["min"]), STATE_DIM if key == "observation.state" else 6)
            self.assertEqual(len(stats[key]["max"]), len(stats[key]["min"]))
        # joint1 spans 0..4 in episode 0 and 0..4 in episode 1
        self.assertAlmostEqual(stats["action"]["min"][0], 0.0, places=6)
        self.assertAlmostEqual(stats["action"]["max"][0], 4.0, places=6)

    def test_export_is_idempotent(self):
        out = self.tmp / "dataset"
        first = self.exporter.export(out)
        second = self.exporter.export(out)
        self.assertEqual(first["frames"], second["frames"])
        self.assertEqual(len(list((out / "data" / CHUNK).glob("episode_*"))), 2)

    def test_incomplete_episode_is_skipped(self):
        self.rec.start("未完成的录制")
        self.rec.record([0.0, 10.0, 30.0, 0.0, -10.0, 0.0])
        # deliberately not stopped
        out = self.tmp / "dataset2"
        result = self.exporter.export(out)
        self.assertEqual(result["episodes"], 2)
        self.assertEqual(result["skipped"], 1)
        self.assertEqual(len(list((out / "data" / CHUNK).glob("episode_*"))), 2)

    def test_export_with_no_episodes_raises(self):
        empty = self.tmp / "empty"
        exporter = LerobotExporter(self.kin, empty)
        with self.assertRaises(ValueError):
            exporter.export(self.tmp / "dataset3")


if __name__ == "__main__":
    unittest.main()

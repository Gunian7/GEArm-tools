"""Export recorded episodes into a LeRobot-compatible dataset layout.

Emits the directory structure and metadata LeRobot's loader expects:

    <out>/
      meta/info.json        feature spec, fps, robot_type
      meta/tasks.jsonl      task_index -> task string
      meta/episodes.jsonl   episode_index -> tasks, length
      data/chunk-000/episode_000000.parquet   (or .csv without pyarrow)
      stats.json            per-feature min/max/mean/std

Only the metadata carries decisions (dtype, shape, names, fps); the data files are
mechanical. Writing parquet needs pyarrow, which is optional -- the CSV fallback
keeps the dataset loadable by anything that reads the same columns.
"""
from __future__ import annotations

import json
import shutil
from pathlib import Path

from episode_recorder import JOINTS, STATE_DIM, EpisodeRecorder

CODABASE_VERSION = "v2.0"
CHUNK = "chunk-000"

# Observation/action naming. observation.state carries the 6 joint angles plus
# the TCP position, so a policy can be trained on either view.
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
STATE_NAMES = JOINT_NAMES + ["tcp_x_mm", "tcp_y_mm", "tcp_z_mm"]


def _try_pyarrow():
    try:
        import pyarrow  # noqa: F401
        return True
    except ImportError:
        return False


class LerobotExporter:
    def __init__(self, kin, episode_root):
        self.rec = EpisodeRecorder(kin, episode_root)

    def export(self, out_root, dataset_name="gearm_teleop", fps=50.0, robot_type="gearm_mc02"):
        """Write every complete episode into a dataset at out_root.

        Incomplete episodes (started but never stopped) are skipped and reported,
        because their frame count and duration were never finalised.
        """
        out = Path(out_root)
        if out.exists():
            shutil.rmtree(out)
        (out / "meta").mkdir(parents=True)
        (out / "data" / CHUNK).mkdir(parents=True)

        episodes = [e for e in self.rec.list_episodes() if e["complete"]]
        skipped = [e for e in self.rec.list_episodes() if not e["complete"]]
        if not episodes:
            raise ValueError("no complete episodes to export")

        task_index = {}
        tasks_rows, episode_rows = [], []
        columns = {}
        for episode in episodes:
            index = episode["episode_index"]
            meta, frames = self.rec.load(index)
            if not frames:
                continue
            task = meta["task"]
            if task not in task_index:
                task_index[task] = len(task_index)
                tasks_rows.append({"task_index": task_index[task], "task": task})
            self._write_frames(out, index, frames, task_index[task])
            episode_rows.append({"episode_index": index,
                                 "tasks": [task],
                                 "length": len(frames)})
            for frame in frames:
                for key, value in frame.items():
                    columns.setdefault(key, []).append(value)

        self._write_jsonl(out / "meta" / "tasks.jsonl", tasks_rows)
        self._write_jsonl(out / "meta" / "episodes.jsonl", episode_rows)
        self._write_info(out, dataset_name, fps, robot_type)
        stats = self._write_stats(out, columns)
        return {"path": str(out), "episodes": len(episode_rows),
                "frames": sum(r["length"] for r in episode_rows),
                "tasks": len(tasks_rows), "skipped": len(skipped),
                "format": "parquet" if _try_pyarrow() else "csv",
                "features": sorted(stats.keys())}

    def _write_frames(self, out, index, frames, task_index):
        rows = []
        for position, frame in enumerate(frames):
            state = frame["observation.state"]
            rows.append({
                "index": position,
                "episode_index": index,
                "frame_index": frame["frame_index"],
                "timestamp": frame["t"],
                "task_index": task_index,
                "task": frame["task"],
                "observation.state": list(state),
                "action": list(frame["action"]),
            })
        target = out / "data" / CHUNK / ("episode_%06d" % index)
        if _try_pyarrow():
            self._write_parquet(target.with_suffix(".parquet"), rows)
        else:
            self._write_csv(target.with_suffix(".csv"), rows)

    @staticmethod
    def _write_parquet(path, rows):
        import pandas as pd
        frame = pd.DataFrame(rows)
        for column in ("observation.state", "action"):
            frame[column] = frame[column].apply(lambda v: [float(x) for x in v])
        frame.to_parquet(path, index=False)

    @staticmethod
    def _write_csv(path, rows):
        import csv
        flat = []
        for row in rows:
            item = dict(row)
            item["observation.state"] = json.dumps(item["observation.state"])
            item["action"] = json.dumps(item["action"])
            flat.append(item)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(flat[0].keys()))
            writer.writeheader()
            writer.writerows(flat)

    def _write_info(self, out, name, fps, robot_type):
        info = {
            "codebase_version": CODABASE_VERSION,
            "robot_type": robot_type,
            "fps": float(fps),
            "features": {
                "observation.state": {"dtype": "float32", "shape": [STATE_DIM],
                                      "names": STATE_NAMES},
                "action": {"dtype": "float32", "shape": [JOINTS],
                           "names": JOINT_NAMES},
                "timestamp": {"dtype": "float32", "shape": [1]},
                "frame_index": {"dtype": "int64", "shape": [1]},
                "episode_index": {"dtype": "int64", "shape": [1]},
                "task_index": {"dtype": "int64", "shape": [1]},
                "index": {"dtype": "int64", "shape": [1]},
            },
        }
        (out / "meta" / "info.json").write_text(
            json.dumps(info, indent=2, ensure_ascii=False), encoding="utf-8")
        self._info = info

    def _write_stats(self, out, columns):
        """Per-feature min/max/mean/std over the concatenated episodes.

        LeRobot computes these during dataset preparation; emitting them here
        keeps the exported dataset self-describing and gives an immediate sanity
        check on the recorded ranges.
        """
        stats = {}
        for key in ("observation.state", "action"):
            rows = columns.get(key) or []
            if not rows:
                continue
            flat = [list(r) for r in rows]
            width = len(flat[0])
            mins, maxs, means, stds = [], [], [], []
            for i in range(width):
                column = [row[i] for row in flat]
                n = len(column)
                mean = sum(column) / n
                variance = sum((v - mean) ** 2 for v in column) / n
                mins.append(min(column))
                maxs.append(max(column))
                means.append(mean)
                stds.append(variance ** 0.5)
            stats[key] = {"min": mins, "max": maxs, "mean": means, "std": stds}
        times = columns.get("t") or []
        if times:
            stats["timestamp"] = {"min": [min(times)], "max": [max(times)],
                                  "mean": [sum(times) / len(times)], "std": [0.0]}
        (out / "stats.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
        return stats

    @staticmethod
    def _write_jsonl(path, rows):
        with path.open("w", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row, ensure_ascii=False) + "\n")

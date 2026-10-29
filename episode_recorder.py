"""Episode recording for behaviour cloning.

One frame per line of JSONL, so a recording is readable, diffable and replayable
without any extra dependency. The schema maps straight onto LeRobot's names
(observation.state / action / episode_index / frame_index / task), so the same
files can be converted to a LeRobot dataset later without re-recording.

Decisions baked in (see SESSION_HANDOFF.md):
  * action space is 6 joint angles, never an end-effector pose -- an IK solution
    is not unique, so a recorded pose would not reproduce the same motion
  * observation.state is 6 joint angles + 3 TCP position, in the URDF Y-up frame
  * images are reserved but not recorded yet
  * nominal rate is 50 Hz; frames are timestamped on arrival, not synthesised
  * an episode is bounded by remote_enable -> remote_disable
"""
from __future__ import annotations

import json
import time
from pathlib import Path

from urdf_kinematics import JOINTS, UrdfKinematics

NOMINAL_HZ = 50.0
STATE_DIM = JOINTS + 3


class RecordingError(Exception):
    pass


class EpisodeRecorder:
    """Records obs/action pairs for one episode and replays them."""

    def __init__(self, kin: UrdfKinematics, root):
        self.kin = kin
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.episode = None
        self._frames = 0
        self._t0 = None
        self._last_t = None
        self._last_abs = None

    # ---------------------------------------------------------------- recording
    def start(self, task, nominal_hz=NOMINAL_HZ):
        if self.episode is not None:
            raise RecordingError("episode %s is still open" % self.episode)
        if not task or not str(task).strip():
            raise RecordingError("a task description is required")
        index = self._next_index()
        self.episode = index
        self._frames = 0
        self._t0 = None
        self._task = str(task).strip()
        self._hz = float(nominal_hz)
        directory = self.root / ("episode_%06d" % index)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "meta.json").write_text(json.dumps({
            "episode_index": index,
            "task": self._task,
            "nominal_hz": self._hz,
            "state_dim": STATE_DIM,
            "action_dim": JOINTS,
            "frame": "urdf-y-up",
            "joint_names": list(self.kin.names),
            "joint_limits_deg": [list(l) if l else None for l in self.kin.limits],
            "started_at": time.time(),
        }, indent=2, ensure_ascii=False), encoding="utf-8")
        return index

    def record(self, joints_deg, timestamp=None):
        """Append one frame. joints_deg is the observed state; the action is the
        commanded joint target for this frame.

        timestamp is an absolute epoch time and is converted to the gap since the
        previous frame, because record_batch works in gaps. Omit it to use now.
        """
        dt = None
        if timestamp is not None:
            stamp = float(timestamp)
            if self._last_abs is not None:
                dt = stamp - self._last_abs
            self._last_abs = stamp
        return self.record_batch([(joints_deg, dt)])[0]

    def record_batch(self, samples):
        """Append several frames in one call.

        samples is a list of (joints_deg, dt_seconds_or_None). A dt is the gap
        since the previous sample, measured on the client with its own clock, so
        the batch is not distorted by how long the HTTP round trip took. The
        first sample's absolute time comes from this machine's clock.

        Batching exists because one POST per frame caps the rate at the network
        round trip (measured ~26 Hz on the dev box, not the nominal 50 Hz).
        """
        if self.episode is None:
            raise RecordingError("no open episode")
        if not samples:
            return []
        # Continue from the last written frame so separate calls chain together
        # instead of each re-anchoring to the wall clock (which double counts).
        now = self._last_t if self._last_t is not None else time.time()
        if self._t0 is None:
            self._t0 = now
        path = self.root / ("episode_%06d" % self.episode) / "frames.jsonl"
        written = []
        first = self._last_t is None
        with path.open("a", encoding="utf-8") as handle:
            for joints_deg, dt in samples:
                joints = [float(v) for v in joints_deg]
                if len(joints) != JOINTS:
                    raise RecordingError("expected %d joint angles" % JOINTS)
                violations = self.kin.limit_violations(joints)
                if violations:
                    index, value, limit = violations[0]
                    raise RecordingError(
                        "joint %d = %.3f deg outside URDF limit %s" % (index + 1, value, limit))
                if dt is not None:
                    now = now + float(dt)
                elif not first:
                    # No gap supplied and not the episode's first frame: fall back
                    # to the wall clock. The first frame keeps the anchor we
                    # already took, so the anchor call does not leak into t.
                    now = time.time()
                first = False
                self._last_t = now
                tcp, _quat = self.kin.fk(joints)
                frame = {
                    "t": round(now - self._t0, 6),
                    "episode_index": self.episode,
                    "frame_index": self._frames,
                    "observation.state": joints + list(tcp),
                    "action": list(joints),
                    "task": self._task,
                }
                handle.write(json.dumps(frame, ensure_ascii=False) + "\n")
                self._frames += 1
                written.append(frame)
        return written

    def stop(self):
        if self.episode is None:
            raise RecordingError("no open episode")
        index = self.episode
        path = self.root / ("episode_%06d" % index) / "frames.jsonl"
        meta_path = self.root / ("episode_%06d" % index) / "meta.json"
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        duration = 0.0
        if path.exists():
            lines = [l for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
            duration = json.loads(lines[-1])["t"] if lines else 0.0
        meta["frames"] = self._frames
        meta["duration_s"] = round(duration, 6)
        meta["mean_hz"] = round(self._frames / duration, 3) if duration > 0 else 0.0
        meta["stopped_at"] = time.time()
        meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
        self.episode = None
        return meta

    # ------------------------------------------------------------------ reading
    def list_episodes(self):
        out = []
        for meta_path in sorted(self.root.glob("episode_*/meta.json")):
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
            except (ValueError, OSError):
                continue
            out.append({"episode_index": meta.get("episode_index"),
                        "task": meta.get("task"),
                        "frames": meta.get("frames"),
                        "duration_s": meta.get("duration_s"),
                        "mean_hz": meta.get("mean_hz"),
                        "complete": meta.get("frames") is not None})
        return out

    def load(self, index):
        """Return (meta, frames). Frames are validated, not trusted."""
        directory = self.root / ("episode_%06d" % index)
        meta_path = directory / "meta.json"
        if not meta_path.exists():
            raise RecordingError("episode %d not found" % index)
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        path = directory / "frames.jsonl"
        frames = []
        if path.exists():
            for line in path.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    frames.append(json.loads(line))
        return meta, frames

    # ------------------------------------------------------------------ replay
    def verify(self, index):
        """Replay an episode: every frame's recorded TCP must be reproducible
        from its recorded joints, indices must be contiguous and time must not
        run backwards. Returns a report dict; a caller must act on 'ok'."""
        meta, frames = self.load(index)
        problems = []
        if not frames:
            problems.append("episode has no frames")
        for expected, frame in enumerate(frames):
            if frame.get("frame_index") != expected:
                problems.append("frame_index gap at %d" % expected)
                break
        for previous, frame in zip(frames, frames[1:]):
            if frame["t"] < previous["t"]:
                problems.append("time runs backwards at frame %d" % frame["frame_index"])
                break
        worst = 0.0
        for frame in frames:
            state = frame.get("observation.state")
            if not isinstance(state, list) or len(state) != STATE_DIM:
                problems.append("bad state at frame %d" % frame.get("frame_index"))
                continue
            tcp, _ = self.kin.fk(state[:JOINTS])
            recorded = state[JOINTS:]
            worst = max(worst, max(abs(tcp[i] - recorded[i]) for i in range(3)))
        if worst > 1e-6:
            problems.append("recorded TCP not reproducible, worst %.6f mm" % worst)
        duration = frames[-1]["t"] if frames else 0.0
        return {
            "ok": not problems,
            "problems": problems,
            "frames": len(frames),
            "duration_s": duration,
            "mean_hz": round(len(frames) / duration, 3) if duration > 0 else 0.0,
            "tcp_reproduce_worst_mm": worst,
        }

    def _next_index(self):
        highest = -1
        for meta_path in self.root.glob("episode_*/meta.json"):
            try:
                highest = max(highest, int(meta_path.parent.name.split("_")[-1]))
            except ValueError:
                continue
        return highest + 1

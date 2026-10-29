"""Python wrapper for the host kinematics exe (same pattern as virtual_mc02)."""
import struct
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXE = HERE / 'build' / 'kinematics_host.exe'


class Kinematics:
    """FK/IK backed by the host C program using the firmware's DH table."""

    def __init__(self, exe=None):
        self.proc = subprocess.Popen([str(exe or EXE)], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    def _read_exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.proc.stdout.read(n - len(buf))
            if not chunk:
                raise EOFError('kinematics host closed')
            buf += chunk
        return buf

    def _command(self, cmd, payload=b''):
        self.proc.stdin.write(cmd.encode() + struct.pack('<I', len(payload)) + payload)
        self.proc.stdin.flush()
        header = self._read_exact(4)
        (n,) = struct.unpack('<I', header)
        return self._read_exact(n) if n else b''

    def fk(self, joints_deg):
        """Six joint angles in degrees -> {'tcp': [x,y,z] mm, 'chain': 6 joint origins}."""
        payload = struct.pack('<6d', *[float(j) for j in joints_deg])
        raw = self._command('F', payload)
        if len(raw) < 8 * 21:
            raise ValueError('short fk reply')
        values = [struct.unpack_from('<d', raw, 8 * i)[0] for i in range(21)]
        return {'tcp': values[18:21],
                'chain': [values[3 * i:3 * i + 3] for i in range(6)]}

    def ik(self, target_xyz, seed_joints_deg):
        """Target (x, y, z) in mm + seed joints in degrees -> joint solution."""
        payload = struct.pack('<3d', *[float(v) for v in target_xyz])
        payload += struct.pack('<6d', *[float(j) for j in seed_joints_deg])
        raw = self._command('I', payload)
        if len(raw) < 56:
            raise ValueError('short ik reply')
        joints = [struct.unpack_from('<d', raw, 8 * i)[0] for i in range(6)]
        residual = struct.unpack_from('<d', raw, 48)[0]
        iterations = int(struct.unpack_from('<d', raw, 56)[0])
        return {'joints': joints, 'residual': residual, 'iterations': iterations}

    def roundtrip(self, count=60):
        raw = self._command('R', struct.pack('<I', int(count)))
        if len(raw) < 24:
            raise ValueError('short roundtrip reply')
        return {'worst_mm': struct.unpack_from('<d', raw, 0)[0],
                'worst_residual': struct.unpack_from('<d', raw, 8)[0],
                'failures': int(struct.unpack_from('<d', raw, 16)[0])}

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except Exception:
            self.proc.kill()

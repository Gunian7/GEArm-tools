/* GEArm 3D viewer: URDF + STL meshes, standard orbit controls, dry-run only.
 *
 * One convention: the on-screen geometry IS the kinematics. FK is read from the
 * three.js scene graph (the URDF joint tree), so the arm, the TCP frame, the
 * target marker and the IK solution can never disagree. The DH-based host
 * service is kept only as a cross-check reference, not as the source of what
 * the page shows. */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const $ = (id) => document.getElementById(id);
const log = $('log');
const JOINTS = 6;
const MAX_V = 20.0;   // ROBOT_REMOTE_MAX_VELOCITY, mm/s (firmware robot.h)
const MAX_RPM = 5.0;  // ROBOT_REMOTE_MAX_RPM, rpm

let joints = [0, 0, 0, 0, 0, 0];
let jointFrames = [];
let jointLimits = [null, null, null, null, null, null];  // null = 未验证，不是无限制
let baseGroup = null;
let stepDeg = 1;
let jogSpeed = 10;      // deg/s, ROBOT_JOINT_DEFAULT_VELOCITY
let speedPercent = 100;

/* ---------------------------------------------------------------------------
 * 实机控制
 *
 * 这个页面就是唯一的控制台。realMode 打开时，画面里的关节角会真实下发给固件；
 * 关闭时退回干跑（只驱动 3D 模型），用于验证 IK 或调试界面，不碰设备。
 *
 * 命令用 rel_rotate <关节号> <增量>，不用 auto <x> <y> <z>：
 * 页面的坐标系是"绿轴 Y 朝上"的屏幕坐标系，而固件 robot_auto_handle() 吃的是
 * 机器人基坐标系（Z 朝上）。两者的原点和轴向对应关系没有在实物上标定过，
 * 直接发 auto 有让机械臂朝意外方向猛冲的风险。改用关节增量则完全绕开这个问题 ——
 * IK 在页面上解出 6 个关节角，我们只告诉固件"每个关节转多少度"。
 *
 * 下发节奏固定 100ms 一拍（与原版 robot_joystick.py 的 10Hz 一致）：
 * 拖动滑块会连续触发 input 事件，每个事件发一条会把桥接队列灌满，
 * 而且固件持有的是"最后一份指令"，发太快没有意义。定尺 flush 同时保证
 * 松手后一定会补发归零帧——那是明确的"停"。
 * ------------------------------------------------------------------------- */
let realMode = true;
let lastSent = [0, 0, 0, 0, 0, 0];   // 固件当前持有的关节角，用于算增量

function sendArm(text) {
  fetch('/api/command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ cmd: 'arm', arg: text })
  }).then((r) => r.json()).then((res) => {
    if (res && res.ok === false) {
      note('拒绝 ' + text + '：' + (res.error || '未知原因'), true);
      return;
    }
    note('已发送 ' + text + '，等待板子确认…');
    waitReply(text, 2500);
  }).catch(() => note('桥接无响应：' + text, true));
}

/* 等板子回显 "+OK <命令>"。拿不到就明确说没收到——
   "发送成功"只说明桥接收下了，机械臂不动时这两件事看起来一模一样。 */
function waitReply(text, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  (function poll() {
    fetch('/api/status').then((r) => r.json()).then((s) => {
      const rows = s.commands || [];
      for (let i = rows.length - 1; i >= 0; i--) {
        if (rows[i].arg === text && rows[i].device_reply) {
          note('板子已确认 ' + text + ' → ' + rows[i].device_reply);
          return;
        }
      }
      if (Date.now() > deadline) {
        note('未收到板子回显：' + text + '（检查 USB 连接）', true);
        return;
      }
      setTimeout(poll, 120);
    }).catch(() => {});
  })();
}

/* 100ms 定尺：把画面关节角与固件持有值之间的差，按关节增量发出去。
   回放期间必须锁住——replayShowFrame() 会直接覆写 joints[]，
   不锁的话"播放一段录制"就会驱动真实机械臂，那是完全不该发生的事。 */
let replaying = false;
setInterval(() => {
  if (!realMode || replaying) return;
  for (let i = 0; i < JOINTS; i++) {
    const delta = joints[i] - lastSent[i];
    if (Math.abs(delta) < 1e-4) continue;
    lastSent[i] = joints[i];
    sendArm('rel_rotate ' + i + ' ' + (Math.round(delta * 100) / 100));
  }
}, 100);

function setRealMode(on) {
  realMode = on;
  const pill = $('pill-mode');
  if (pill) {
    pill.textContent = on ? '实机控制 · 指令真实下发' : '干跑 · 不发送运动命令';
    pill.className = 'pill' + (on ? ' live' : '');
  }
  const btn = $('mode-toggle');
  if (btn) btn.textContent = on ? '切换到干跑' : '切换到实机';
  const banner = $('mode-banner');
  if (banner) banner.hidden = on;
  /* 切回实机时以当前画面角为基准重新对齐，避免把切换期间的位移一次性补发 */
  if (on) { lastSent = joints.slice(); }
  note(on ? '已切换到实机控制：关节变动会真实下发' : '已切换到干跑：只驱动模型，不发指令');
}

function note(text, bad) {
  const li = document.createElement('li');
  li.textContent = text;
  if (bad) li.className = 'bad';
  log.insertBefore(li, log.firstChild);
  while (log.children.length > 30) log.removeChild(log.lastChild);
}

/* ---------- renderer / scene ---------- */
const canvas = $('gl');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0c0d);
scene.fog = new THREE.Fog(0x0b0c0d, 1.6, 5.5);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
/* Framed for the upright arm: ~390 mm tall, reaching ~170 mm toward +Z. */
camera.position.set(0.55, 0.42, 0.62);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 0.18, 0.05);
controls.minDistance = 0.12;
controls.maxDistance = 4;

scene.add(new THREE.HemisphereLight(0xdfe6f2, 0x0b0c0d, 1.35));
const key = new THREE.DirectionalLight(0xffffff, 2.0);
key.position.set(0.7, 1.2, 0.6);
scene.add(key);
const fill = new THREE.DirectionalLight(0x9fb4ff, 0.5);
fill.position.set(-0.8, 0.5, -0.7);
scene.add(fill);

const grid = new THREE.GridHelper(1.0, 20, 0x2a2d33, 0x1a1c20);
scene.add(grid);
scene.add(new THREE.AxesHelper(0.3));

function resize() {
  const box = canvas.parentElement.getBoundingClientRect();
  renderer.setSize(box.width, box.height, false);
  camera.aspect = box.width / Math.max(1, box.height);
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

/* ---------- text labels ---------- */
function makeLabel(text) {
  const size = 128;
  const cv = document.createElement('canvas');
  cv.width = size; cv.height = size;
  const ctx = cv.getContext('2d');
  ctx.fillStyle = 'rgba(15, 16, 17, 0.88)';
  ctx.strokeStyle = 'rgba(113, 112, 255, 0.55)';
  ctx.lineWidth = 5;
  ctx.beginPath();
  ctx.arc(size / 2, size / 2, size / 2 - 8, 0, Math.PI * 2);
  ctx.fill(); ctx.stroke();
  ctx.fillStyle = '#f7f8f8';
  ctx.font = '500 52px ui-monospace, Consolas, monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, size / 2, size / 2 + 2);
  const texture = new THREE.CanvasTexture(cv);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false }));
  sprite.scale.set(0.035, 0.035, 0.035);
  return sprite;
}

/* ---------- URDF ---------- */
function parseVec(text, fallback) {
  if (!text) return fallback;
  const v = text.trim().split(/\s+/).map(Number);
  return v.length === fallback.length && v.every((n) => Number.isFinite(n)) ? v : fallback;
}

/* URDF rpy is roll-pitch-yaw about fixed axes: R = Rz(yaw) * Ry(pitch) * Rx(roll).
 * Composed explicitly so no Euler-order convention can silently change it. */
function rpyQuaternion(roll, pitch, yaw) {
  const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), roll);
  const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), pitch);
  const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), yaw);
  return qz.multiply(qy).multiply(qx);
}

/* The URDF is Z-up (its vertical is +Z, the blue axis) while three.js screens are
 * Y-up (green is up). A single root group carries the Z-up -> Y-up rotation, so
 * the URDF data stays untouched and there is exactly one place the frame
 * conversion lives. S maps (x, y, z) -> (x, z, -y), i.e. Rx(-90). */
const REMAP = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
/* URDF joint axis is +Z; it stays +Z because the root group does the conversion. */
const JOINT_AXIS = new THREE.Vector3(0, 0, 1);

/* The URDF paints every link the same grey, which hides the joint boundaries on
 * screen. A small per-link luminance step keeps the neutral machined look while
 * making each joint's motion readable. Index follows the URDF link order. */
const LINK_TONES = [0.82, 1.0, 0.93, 1.06, 0.88, 1.0, 0.72];

async function attachMesh(parent, def, loader, diagnostics, tone) {
  if (!def || !def.mesh) {
    diagnostics.push({ link: parent.name, mesh: null, status: 'no mesh' });
    return;
  }
  try {
    const geometry = await new Promise((resolve, reject) =>
      loader.load('/arm/meshes/' + def.mesh, resolve, undefined, reject));
    geometry.computeVertexNormals();
    const count = geometry.attributes.position.count;
    const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
      color: def.color.clone().multiplyScalar(tone === undefined ? 1 : tone),
      metalness: 0.45, roughness: 0.42
    }));
    parent.add(mesh);
    diagnostics.push({ link: parent.name, mesh: def.mesh, triangles: count / 3, status: 'ok' });
  } catch (e) {
    diagnostics.push({ link: parent.name, mesh: def.mesh, status: 'load failed' });
  }
}

async function loadRobot() {
  const urdfText = await (await fetch('/arm/arm.urdf')).text();
  const xml = new DOMParser().parseFromString(urdfText, 'application/xml');
  const loader = new STLLoader();
  const diagnostics = [];

  const links = {};
  let linkIndex = 0;
  for (const link of xml.querySelectorAll('link')) {
    const name = link.getAttribute('name');
    const visual = link.querySelector('visual');
    const meshNode = visual && visual.querySelector('mesh');
    const meshFile = meshNode ? meshNode.getAttribute('filename').split('/').pop() : null;
    const colorNode = visual && visual.querySelector('color');
    const color = colorNode ? new THREE.Color().setStyle(
      'rgb(' + colorNode.getAttribute('rgba').split(/\s+/).slice(0, 3)
        .map((c) => Math.round(parseFloat(c) * 255)).join(',') + ')') : new THREE.Color(0xb8b0a2);
    links[name] = { mesh: meshFile, color, tone: LINK_TONES[linkIndex] || 1 };
    linkIndex++;
  }

  /* One root group carries the Z-up -> Y-up conversion; everything under it
   * (the pedestal and the joint chain) shares that frame. */
  const robotRoot = new THREE.Group();
  robotRoot.name = 'robot_root';
  robotRoot.quaternion.copy(REMAP);
  scene.add(robotRoot);

  /* base_link has no parent joint: attach it to the robot root explicitly. */
  baseGroup = new THREE.Group();
  baseGroup.name = 'base_link';
  robotRoot.add(baseGroup);
  await attachMesh(baseGroup, links['base_link'], loader, diagnostics, links['base_link'].tone);
  const baseLabel = makeLabel('Base');
  baseLabel.position.set(0, 0.03, 0);
  baseGroup.add(baseLabel);

  const jointsDef = [];
  for (const joint of xml.querySelectorAll('joint')) {
    const type = joint.getAttribute('type');
    if (type !== 'revolute' && type !== 'continuous') continue;
    const origin = joint.querySelector('origin');
    /* Two groups per joint: the URDF origin (xyz + rpy) and the joint rotation.
     * Rotating a single group's .z would overwrite the origin's rpy Z. */
    const originGroup = new THREE.Group();
    originGroup.name = joint.getAttribute('name') + '-origin';
    originGroup.position.fromArray(parseVec(origin && origin.getAttribute('xyz'), [0, 0, 0]));
    originGroup.quaternion.copy(rpyQuaternion(...parseVec(origin && origin.getAttribute('rpy'), [0, 0, 0])));
    const rotGroup = new THREE.Group();
    rotGroup.name = joint.getAttribute('name');
    originGroup.add(rotGroup);
    jointsDef.push({
      name: rotGroup.name,
      parent: joint.querySelector('parent').getAttribute('link'),
      child: joint.querySelector('child').getAttribute('link'),
      frame: rotGroup
    });
  }

  const childToJoint = {};
  for (const j of jointsDef) childToJoint[j.child] = j;
  for (const j of jointsDef) {
    const parentJoint = childToJoint[j.parent];
    (parentJoint ? parentJoint.frame : baseGroup).add(j.frame.parent);
  }

  for (const j of jointsDef) {
    await attachMesh(j.frame, links[j.child], loader, diagnostics, links[j.child].tone);
    const axis = new THREE.ArrowHelper(
      JOINT_AXIS, new THREE.Vector3(0, 0, 0), 0.07, 0x7170ff, 0.022, 0.013);
    j.frame.add(axis);
    const label = makeLabel(j.name.replace('joint', 'J'));
    label.position.set(0, 0.03, 0);
    j.frame.add(label);
  }

  return { jointsDef, diagnostics };
}

/* ---------- TCP frame + target ---------- */
const targetMarker = new THREE.Mesh(
  new THREE.SphereGeometry(0.012, 24, 18),
  new THREE.MeshBasicMaterial({ color: 0xf2545b, wireframe: true }));
targetMarker.visible = false;
scene.add(targetMarker);

const tcpFrame = new THREE.Group();
const L = 0.07;
tcpFrame.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), L, 0xf2545b, 0.022, 0.013));
tcpFrame.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), L, 0x10b981, 0.022, 0.013));
tcpFrame.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), L, 0x4c8dff, 0.022, 0.013));
scene.add(tcpFrame);

/* ---------- FK straight from the scene graph (URDF is the single source) ---------- */
function fk() {
  scene.updateMatrixWorld(true);
  const last = jointFrames[jointFrames.length - 1];
  const p = new THREE.Vector3();
  const q = new THREE.Quaternion();
  const s = new THREE.Vector3();
  if (last) last.frame.matrixWorld.decompose(p, q, s);
  tcpFrame.position.copy(p);
  tcpFrame.quaternion.copy(q);
  return p;
}

/* ---------- numerical IK on the URDF chain ---------- */
function applyJointsSilent() {
  for (let i = 0; i < jointFrames.length; i++) {
    jointFrames[i].frame.quaternion.setFromAxisAngle(JOINT_AXIS, THREE.MathUtils.degToRad(joints[i]));
  }
}

function applyJoints() {
  applyJointsSilent();
  const p = fk();
  $('tcp-x').textContent = (p.x * 1000).toFixed(1);
  $('tcp-y').textContent = (p.y * 1000).toFixed(1);
  $('tcp-z').textContent = (p.z * 1000).toFixed(1);
}

function ikOnce(target, seed, maxIter) {
  const q = seed.slice();
  const saved = joints;
  const lambda = 1e-4;
  for (let iter = 0; iter < maxIter; iter++) {
    joints = q;
    applyJointsSilent();
    const p = fk();
    const ex = target.x - p.x, ey = target.y - p.y, ez = target.z - p.z;
    const err = Math.hypot(ex, ey, ez);
    if (err < 1e-5) { joints = saved; return { q, residual: err, iterations: iter }; }
    const J = [];
    const h = 1e-6;
    for (let j = 0; j < JOINTS; j++) {
      const save = q[j];
      q[j] = save + h;
      joints = q;
      applyJointsSilent();
      const p2 = fk();
      J.push([(p2.x - p.x) / h, (p2.y - p.y) / h, (p2.z - p.z) / h]);
      q[j] = save;
    }
    const A = [], b = [];
    for (let r = 0; r < JOINTS; r++) {
      b.push(J[r][0] * ex + J[r][1] * ey + J[r][2] * ez);
      const row = [];
      for (let c = 0; c < JOINTS; c++) {
        row.push(J[r][0] * J[c][0] + J[r][1] * J[c][1] + J[r][2] * J[c][2] + (r === c ? lambda : 0));
      }
      A.push(row);
    }
    for (let col = 0; col < JOINTS; col++) {
      let piv = col;
      for (let r = col + 1; r < JOINTS; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
      if (Math.abs(A[piv][col]) < 1e-12) { joints = saved; return { q, residual: err, iterations: -1 }; }
      if (piv !== col) { const t = A[col]; A[col] = A[piv]; A[piv] = t; const tb = b[col]; b[col] = b[piv]; b[piv] = tb; }
      for (let r = col + 1; r < JOINTS; r++) {
        const f = A[r][col] / A[col][col];
        if (!f) continue;
        for (let c = col; c < JOINTS; c++) A[r][c] -= f * A[col][c];
        b[r] -= f * b[col];
      }
    }
    const dq = new Array(JOINTS).fill(0);
    for (let r = JOINTS - 1; r >= 0; r--) {
      let sum = b[r];
      for (let c = r + 1; c < JOINTS; c++) sum -= A[r][c] * dq[c];
      dq[r] = sum / A[r][r];
    }
    let step = 0;
    for (let r = 0; r < JOINTS; r++) step += dq[r] * dq[r];
    step = Math.sqrt(step);
    const scale = step > 0.15 ? 0.15 / step : 1;
    for (let r = 0; r < JOINTS; r++) q[r] += dq[r] * scale;
  }
  joints = q;
  applyJointsSilent();
  const p = fk();
  const err = Math.hypot(target.x - p.x, target.y - p.y, target.z - p.z);
  joints = saved;
  return { q, residual: err, iterations: maxIter };
}

function solveIK(target) {
  const seeds = [joints.slice(), [0, 0, 0, 0, 0, 0],
    [0.3, 0.3, 0.3, 0, 0, 0], [-0.3, -0.3, -0.3, 0, 0, 0],
    [0.6, -0.3, 0.4, 0, 0, 0], [-0.6, 0.3, -0.4, 0, 0, 0]];
  let best = null;
  for (const seed of seeds) {
    const r = ikOnce(target, seed, 300);
    if (!best || r.residual < best.residual) best = r;
    if (best.residual < 1e-5) break;
  }
  return best;
}

/* ---------- UI ---------- */
function buildControls() {
  const host = $('sliders');
  host.innerHTML = '';
  for (let i = 0; i < JOINTS; i++) {
    (function (index) {
      const row = document.createElement('div');
      row.className = 'row';
      const label = document.createElement('label');
      label.textContent = 'J' + (index + 1);
      const minus = document.createElement('button');
      minus.type = 'button'; minus.className = 'step'; minus.textContent = '\u2212';
      const plus = document.createElement('button');
      plus.type = 'button'; plus.className = 'step'; plus.textContent = '+';
      const input = document.createElement('input');
      input.type = 'range'; input.min = '-180'; input.max = '180'; input.step = '0.5';
      input.value = String(joints[index]);
      const out = document.createElement('output');
      out.textContent = joints[index].toFixed(1);
      function set(v) {
        const limit = jointLimits[index];
        let next = v;
        if (limit) next = Math.max(limit[0], Math.min(limit[1], v));
        joints[index] = next;
        input.value = String(joints[index]);
        out.textContent = joints[index].toFixed(1) + (limit ? '' : ' *');
        applyJoints();
      }
      minus.addEventListener('click', () => set(joints[index] - stepDeg));
      plus.addEventListener('click', () => set(joints[index] + stepDeg));
      input.addEventListener('input', () => set(parseFloat(input.value)));
      row.append(label, minus, input, plus, out);
      host.appendChild(row);
    })(i);
  }
}

/* Real travel comes from the URDF via /api/model, so the page and the recorder
 * enforce the same limits. Joints the URDF leaves unbounded stay unclamped and
 * are marked with a star: unknown is not the same as unlimited. */
async function loadJointLimits() {
  try {
    const model = await (await fetch('/api/model')).json();
    if (!model.ok) return;
    jointLimits = model.joints.map((j) => j.limit_deg);
    const inputs = $('sliders').querySelectorAll('input');
    const outs = $('sliders').querySelectorAll('output');
    model.joints.forEach((j, i) => {
      if (!inputs[i]) return;
      if (j.limit_deg) {
        inputs[i].min = String(j.limit_deg[0]);
        inputs[i].max = String(j.limit_deg[1]);
        outs[i].title = 'URDF 行程 ' + j.limit_deg[0].toFixed(1) + '° ~ ' + j.limit_deg[1].toFixed(1) + '°';
      } else {
        outs[i].title = 'URDF 未定义行程（continuous 关节），当前不受限';
      }
    });
    buildControls();
    note('关节限位已按 URDF 载入；带 * 的关节未定义行程');
  } catch (e) { note('限位载入失败，滑块退回 ±180°', true); }
}

function refreshOutputs() {
  const inputs = $('sliders').querySelectorAll('input');
  const outs = $('sliders').querySelectorAll('output');
  for (let i = 0; i < JOINTS; i++) {
    inputs[i].value = String(joints[i]);
    outs[i].textContent = joints[i].toFixed(1) + (jointLimits[i] ? '' : ' *');
  }
}

function buildSpeed() {
  const host = $('speed');
  host.innerHTML =
    '<div class="kv"><span>关节点动速度</span><strong>' + jogSpeed + ' °/s</strong></div>' +
    '<div class="kv"><span>远程线速度上限</span><strong>' + (MAX_V * speedPercent / 100).toFixed(1) + ' mm/s</strong></div>' +
    '<div class="kv"><span>远程转速上限</span><strong>' + (MAX_RPM * speedPercent / 100).toFixed(2) + ' rpm</strong></div>' +
    '<div class="speedrow">' +
    '<label>点动 °/s <input id="jog" type="number" min="1" max="60" step="1" value="' + jogSpeed + '"></label>' +
    '<label>步长 ° <input id="step" type="number" min="0.1" max="30" step="0.1" value="' + stepDeg + '"></label>' +
    '</div>' +
    '<div class="btns">' +
    '<button data-p="25" type="button">25%</button>' +
    '<button data-p="50" type="button">50%</button>' +
    '<button data-p="100" type="button">100%</button>' +
    '</div>' +
    '<p class="hintbox">固件常量：ROBOT_JOINT_DEFAULT_VELOCITY=10、ROBOT_REMOTE_MAX_VELOCITY=20mm/s、ROBOT_REMOTE_MAX_RPM=5rpm。</p>';
  $('jog').addEventListener('change', (e) => {
    jogSpeed = Math.max(1, Math.min(60, parseFloat(e.target.value) || 10));
    buildSpeed();
  });
  $('step').addEventListener('change', (e) => {
    stepDeg = Math.max(0.1, Math.min(30, parseFloat(e.target.value) || 1));
    buildSpeed();
  });
  host.querySelectorAll('button[data-p]').forEach((b) => b.addEventListener('click', () => {
    speedPercent = parseInt(b.dataset.p, 10);
    buildSpeed();
    note('速度设为 ' + speedPercent + '%（远程上限 ' + (MAX_V * speedPercent / 100).toFixed(1) + ' mm/s）');
  }));
}

function showDiagnostics(list) {
  const host = $('diag');
  host.innerHTML = list.map((d) =>
    '<div class="kv"><span>' + d.link + (d.mesh ? ' · ' + d.mesh : '') + '</span><strong>' +
    (d.status === 'ok' ? Math.round(d.triangles).toLocaleString() + ' 面' : d.status) + '</strong></div>'
  ).join('');
  const ok = list.filter((d) => d.status === 'ok').length;
  const bad = list.filter((d) => d.status !== 'ok');
  $('pill-model').textContent = '模型 ' + ok + '/' + list.length + ' 已加载';
  if (bad.length) note('网格未加载：' + bad.map((d) => d.link).join(', '), true);
}

function solve() {
  const target = new THREE.Vector3(
    parseFloat($('tx').value) / 1000,
    parseFloat($('ty').value) / 1000,
    parseFloat($('tz').value) / 1000);
  targetMarker.position.copy(target);
  targetMarker.visible = true;
  const best = solveIK(target);
  if (!best || best.residual > 5e-3) {
    note('IK 未收敛：残差 ' + (best ? (best.residual * 1000).toFixed(2) : '?') + ' mm', true);
    return;
  }
  joints = best.q.map((v) => THREE.MathUtils.radToDeg(v));
  refreshOutputs();
  applyJoints();
  $('res').textContent = (best.residual * 1000).toFixed(3) + ' mm';
  note('IK 已解出并驱动模型（残差 ' + (best.residual * 1000).toFixed(3) + ' mm）');
}

function setView(kind) {
  if (kind === 'iso') camera.position.set(0.55, 0.42, 0.62);
  if (kind === 'front') camera.position.set(0, 0.2, 0.8);
  if (kind === 'top') camera.position.set(0.001, 0.9, 0.001);
  controls.target.set(0, 0.18, 0.05);
  controls.update();
}

$('solve').addEventListener('click', solve);
$('home').addEventListener('click', () => {
  joints = [0, 0, 0, 0, 0, 0];
  refreshOutputs();
  applyJoints();
});
$('view-iso').addEventListener('click', () => setView('iso'));
$('view-front').addEventListener('click', () => setView('front'));
$('view-top').addEventListener('click', () => setView('top'));

function pollStatus() {
  fetch('/api/status').then((r) => r.json()).then((s) => {
    $('conn').textContent = s.connection;
    const rows = s.commands || [];
    for (let i = rows.length - 1; i >= 0; i--) {
      if (rows[i].device_reply) {
        $('last-reply').textContent = rows[i].device_reply;
        break;
      }
    }
  }).catch(() => {});
}

/* ---------- episode recording (behaviour cloning) ---------- */
const rec = { open: false, index: null, frames: 0, auto: null, buffer: [], lastT: null };
const REC_MAX_FRAMES = 3000;    // ~60 s at 50 Hz
const REC_INTERVAL_MS = 20;     // 50 Hz nominal
const REC_FLUSH_EVERY = 25;     // frames per batch
const REC_FLUSH_MS = 400;       // or this often, whichever comes first

function recSetButtons() {
  $('rec-start').disabled = rec.open;
  $('rec-auto').disabled = !rec.open;
  $('rec-frame').disabled = !rec.open || rec.auto !== null;
  $('rec-stop').disabled = !rec.open;
  $('rec-auto').textContent = rec.auto === null ? '连续录制 50Hz' : '暂停连续录制';
  $('rec-state').textContent = rec.open
    ? (rec.auto === null ? '录制中（手动）' : '录制中（连续 50Hz）') : '空闲';
  $('rec-index').textContent = rec.index === null ? '-' : String(rec.index);
  $('rec-count').textContent = String(rec.frames);
}

async function recPost(action, payload) {
  const response = await fetch('/api/episode/' + action, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload || {})
  });
  return response.json();
}

/* One frame. Shared by the manual button and the 50 Hz timer so both paths
   enforce the same limit check and the same bookkeeping. */
async function recCapture() {
  try {
    const data = await recPost('frame', { joints: joints.slice() });
    if (!data.ok) { note('该帧被拒绝：' + data.error, true); return false; }
    rec.frames = data.frame_index + 1;
    recSetButtons();
    return true;
  } catch (e) { note('录帧异常 ' + e, true); return false; }
}

/* Continuous capture: sample locally at the nominal rate and ship a batch.
   One POST per frame caps the rate at the network round trip (measured ~26 Hz
   on the dev box); batching removes that ceiling. dt is the client-measured
   gap to the previous sample so the timestamps stay honest. */
async function recFlush() {
  if (!rec.buffer.length) return true;
  const batch = rec.buffer.splice(0, rec.buffer.length);
  try {
    const data = await recPost('frames', { samples: batch });
    if (!data.ok) { note('批量提交被拒绝：' + data.error, true); return false; }
    rec.frames = data.last_frame_index + 1;
    recSetButtons();
    return true;
  } catch (e) { note('批量提交异常 ' + e, true); return false; }
}

function recAutoStop(reason) {
  if (rec.auto === null) return;
  clearInterval(rec.auto);
  clearInterval(rec.flushTimer);
  rec.auto = null;
  rec.flushTimer = null;
  recFlush();
  recSetButtons();
  note('连续录制已' + reason);
}

$('rec-start').addEventListener('click', async () => {
  const task = $('task').value.trim();
  if (!task) { note('先填任务描述再录制', true); return; }
  try {
    const data = await recPost('start', { task });
    if (!data.ok) { note('录制启动失败：' + data.error, true); return; }
    rec.open = true;
    rec.index = data.episode_index;
    rec.frames = 0;
    recSetButtons();
    note('开始录制 episode ' + data.episode_index + '：' + task);
  } catch (e) { note('录制启动异常 ' + e, true); }
});

$('rec-frame').addEventListener('click', async () => {
  await recCapture();
});

$('rec-auto').addEventListener('click', () => {
  if (rec.auto !== null) { recAutoStop('暂停'); return; }
  if (rec.frames >= REC_MAX_FRAMES) {
    note('已达单集上限 ' + REC_MAX_FRAMES + ' 帧，请先停止再开新的一集', true);
    return;
  }
  rec.lastT = null;
  // Sample locally at the nominal rate; ship batches. setInterval drifts, which
  // is why each sample carries its own measured dt rather than assuming 20 ms.
  rec.auto = setInterval(() => {
    if (rec.frames + rec.buffer.length >= REC_MAX_FRAMES) {
      recAutoStop('达到上限自动停止');
      return;
    }
    const now = performance.now();
    const dt = rec.lastT === null ? null : (now - rec.lastT) / 1000;
    rec.lastT = now;
    rec.buffer.push({ joints: joints.slice(), dt: dt });
    rec.frames += 0;
    $('rec-count').textContent = String(rec.frames + rec.buffer.length);
    if (rec.buffer.length >= REC_FLUSH_EVERY) recFlush();
  }, REC_INTERVAL_MS);
  rec.flushTimer = setInterval(recFlush, REC_FLUSH_MS);
  recSetButtons();
  note('连续录制开始（标称 50Hz，批量提交，上限 ' + REC_MAX_FRAMES + ' 帧）');
});

$('rec-stop').addEventListener('click', async () => {
  recAutoStop('停止');
  try {
    const data = await recPost('stop', {});
    if (!data.ok) { note('停止失败：' + data.error, true); return; }
    const meta = data.meta;
    rec.open = false;
    rec.frames = meta.frames;
    recSetButtons();
    note('已停止：' + meta.frames + ' 帧 / ' + meta.duration_s.toFixed(3) + ' s / ' +
         meta.mean_hz.toFixed(1) + ' Hz');
    replayRefreshList();
  } catch (e) { note('停止异常 ' + e, true); }
});

$('rec-verify').addEventListener('click', async () => {
  try {
    const list = await (await fetch('/api/episodes')).json();
    if (!list.episodes.length) { note('还没有录制任何 episode', true); return; }
    const last = list.episodes[list.episodes.length - 1].episode_index;
    const data = await recPost('verify', { index: last });
    if (!data.ok) { note('校验失败：' + data.error, true); return; }
    const report = data.report;
    $('rec-verify-out').textContent = report.ok
      ? '通过 · ' + report.frames + ' 帧 · TCP 复现 ' + report.tcp_reproduce_worst_mm.toFixed(6) + ' mm'
      : '未通过：' + report.problems.join('；');
    note(report.ok ? 'episode ' + last + ' 校验通过' : 'episode ' + last + ' 校验未通过', !report.ok);
  } catch (e) { note('校验异常 ' + e, true); }
});

recSetButtons();

/* ---------- replay (read-only, device never touched) ---------- */
const replay = { frames: null, index: null, i: 0, timer: null };

async function replayRefreshList() {
  const select = $('replay-list');
  let list;
  try {
    list = (await (await fetch('/api/episodes')).json()).episodes;
  } catch (e) { return; }
  select.innerHTML = '';
  if (!list.length) {
    select.innerHTML = '<option value="">（还没有录制）</option>';
    return;
  }
  for (const ep of list) {
    const option = document.createElement('option');
    option.value = String(ep.episode_index);
    option.textContent = '#' + ep.episode_index + ' · ' + ep.frames + ' 帧 · ' +
                         (ep.duration_s || 0).toFixed(2) + 's · ' + (ep.task || '').slice(0, 18);
    select.appendChild(option);
  }
}

function replayStop() {
  if (replay.timer !== null) { clearTimeout(replay.timer); replay.timer = null; }
  replaying = false;
  $('replay-play').textContent = '播放';
}

function replayShowFrame() {
  const frames = replay.frames;
  if (!frames || !frames.length) return;
  const frame = frames[Math.min(replay.i, frames.length - 1)];
  joints = frame['observation.state'].slice(0, JOINTS);
  refreshOutputs();
  applyJoints();
  $('replay-pos').textContent = (replay.i + 1) + ' / ' + frames.length +
                                '（t=' + frame.t.toFixed(3) + 's）';
  const tcp = frame['observation.state'].slice(JOINTS);
  $('replay-tcp').textContent = tcp.map((v) => v.toFixed(1)).join(', ');
}

function replayTick() {
  replayShowFrame();
  replay.i++;
  if (replay.i >= replay.frames.length) { replayStop(); return; }
  const frames = replay.frames;
  const dt = (frames[replay.i].t - frames[replay.i - 1].t) * 1000 /
             parseFloat($('replay-speed').value || '1');
  replay.timer = setTimeout(replayTick, Math.max(4, dt));
}

$('replay-play').addEventListener('click', async () => {
  if (replay.timer !== null) { replayStop(); return; }
  const value = $('replay-list').value;
  if (!value) { note('没有可回放的 episode', true); return; }
  try {
    const data = await (await fetch('/api/episode/frames?index=' + encodeURIComponent(value))).json();
    if (!data.ok || !data.frames.length) { note('回放加载失败：' + (data.error || '空 episode'), true); return; }
    replay.frames = data.frames;
    replay.index = parseInt(value, 10);
    replay.i = 0;
    replaying = true;          // 锁住指令下发：回放只驱动画面
    $('replay-play').textContent = '暂停';
    note('开始回放 episode ' + replay.index + '（' + data.frames.length + ' 帧，仅驱动画面，不发指令）');
    replayTick();
  } catch (e) { note('回放异常 ' + e, true); }
});

$('replay-stop').addEventListener('click', () => {
  replayStop();
  note('回放已停止');
});

replayRefreshList();
setInterval(replayRefreshList, 5000);

/* ---------- boot ---------- */
buildControls();
buildSpeed();

/* 实机/干跑切换 + 侧栏急停。
   急停复用 estop.js 的逻辑：直接点页面上的红色大按钮效果相同，
   这里只是给侧栏一个顺手的位置。 */
$('mode-toggle').addEventListener('click', () => setRealMode(!realMode));
$('estop-quick').addEventListener('click', () => {
  const btn = document.getElementById('estop-btn');
  if (btn) btn.click();
});
setRealMode(true);

pollStatus();
loadJointLimits();
setInterval(pollStatus, 2000);

loadRobot().then(({ jointsDef, diagnostics }) => {
  jointFrames = jointsDef;
  showDiagnostics(diagnostics);
  applyJoints();
  $('loading').style.display = 'none';
  note('URDF 与 STL 加载完成；FK/IK 均取自 URDF 链，与画面同源');
}).catch((e) => {
  $('loading').textContent = '模型加载失败：' + e;
  note('模型加载失败 ' + e, true);
});

let frames = 0;
let last = performance.now();
function loop(now) {
  requestAnimationFrame(loop);
  controls.update();
  renderer.render(scene, camera);
  frames++;
  if (now - last >= 1000) {
    $('fps').textContent = frames + ' fps';
    frames = 0;
    last = now;
  }
}
requestAnimationFrame(loop);

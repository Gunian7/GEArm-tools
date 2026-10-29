/* GEArm joystick console. Mirrors the original project's control model
 * (robot_joystick.py + robot_cmd.c): a remote-enable gate, six axes mapped to
 * end-effector linear/angular velocity, and 10 Hz text lines.
 *
 * Dry-run: nothing is sent to the device. The exact text line that the original
 * firmware would receive is previewed instead. */
(function () {
  'use strict';
  var MAX_V = 20.0;   // ROBOT_REMOTE_MAX_VELOCITY, mm/s
  var MAX_RPM = 5.0;  // ROBOT_REMOTE_MAX_RPM

  var axes = [0, 0, 0, 0, 0, 0];
  var enabled = false;
  var lastLine = '';

  var $ = function (id) { return document.getElementById(id); };
  var log = $('log');

  function note(text, bad) {
    var li = document.createElement('li');
    li.textContent = text;
    if (bad) li.className = 'bad';
    log.insertBefore(li, log.firstChild);
    while (log.children.length > 30) log.removeChild(log.lastChild);
  }

  /* ---------- stick pads ---------- */
  function makePad(padId, knobId, indexX, indexY, invertY) {
    var pad = $(padId), knob = $(knobId);
    var active = false;

    function place(clientX, clientY) {
      var r = pad.getBoundingClientRect();
      var x = (clientX - r.left) / r.width * 2 - 1;
      var y = (clientY - r.top) / r.height * 2 - 1;
      var len = Math.hypot(x, y);
      if (len > 1) { x /= len; y /= len; }
      axes[indexX] = Math.round(x * 100) / 100;
      axes[indexY] = Math.round((invertY ? -y : y) * 100) / 100;
      knob.style.left = (50 + axes[indexX] * 50) + '%';
      knob.style.top = (50 + axes[indexY] * (invertY ? -50 : 50)) + '%';
      render();
    }
    function release() {
      active = false;
      axes[indexX] = 0; axes[indexY] = 0;
      knob.style.left = '50%'; knob.style.top = '50%';
      render();
    }
    pad.addEventListener('pointerdown', function (e) {
      active = true; pad.setPointerCapture(e.pointerId); place(e.clientX, e.clientY);
    });
    pad.addEventListener('pointermove', function (e) { if (active) place(e.clientX, e.clientY); });
    pad.addEventListener('pointerup', release);
    pad.addEventListener('pointercancel', release);
  }

  makePad('pad-left', 'knob-left', 0, 1, false);
  makePad('pad-right', 'knob-right', 3, 2, false);

  function bindTrigger(id, labelId, index) {
    var input = $(id);
    input.addEventListener('input', function () {
      axes[index] = parseFloat(input.value);
      $(labelId).textContent = axes[index].toFixed(2);
      render();
    });
  }
  bindTrigger('t4', 't4v', 4);
  bindTrigger('t5', 't5v', 5);

  /* ---------- derived velocities, exactly as robot_remote_event_handle ---------- */
  function derived() {
    return {
      vx: -axes[0] * MAX_V,
      vy: axes[1] * MAX_V,
      vz: (axes[4] - axes[5]) / 2 * MAX_V,
      rx: -axes[3] * MAX_RPM,
      ry: axes[2] * MAX_RPM
    };
  }

  function render() {
    var d = derived();
    var rows = [
      ['axis0 左X', axes[0], 'vx = -a0 × 20', d.vx],
      ['axis1 左Y', axes[1], 'vy = +a1 × 20', d.vy],
      ['axis2 右Y', axes[2], 'ry = +a2 × 5', d.ry],
      ['axis3 右X', axes[3], 'rx = -a3 × 5', d.rx],
      ['axis4 扳机', axes[4], 'vz = (a4-a5)/2', d.vz],
      ['axis5 扳机', axes[5], '', 0]
    ];
    var html = '';
    for (var i = 0; i < rows.length; i++) {
      var r = rows[i];
      var active = Math.abs(r[1]) > 0.001;
      html += '<tr' + (active ? ' class="hl"' : '') + '><td>' + r[0] + '</td><td>' +
        r[1].toFixed(2) + '</td><td>' + (r[2] || '') + '</td><td>' +
        (r[3] ? r[3].toFixed(2) + (i < 3 ? ' mm/s' : ' rpm') : '') + '</td></tr>';
    }
    $('axestable').innerHTML = html;

    lastLine = 'remote_event ' + axes.map(function (v) { return v.toFixed(1); }).join(' ');
    if (enabled) {
      $('proto').innerHTML = '<span class="dim">→ </span>' + lastLine + '\\n';
    } else {
      $('proto').innerHTML = '<span class="dim">远程模式未使能：原固件会忽略 remote_event。' +
        '预期文本行：</span>\\n' + lastLine;
    }
  }

  /* ---------- gate ---------- */
  function setEnabled(state) {
    enabled = state;
    var pill = $('rmode');
    pill.textContent = '远程模式：' + (state ? '已使能' : '未使能');
    pill.className = 'pill' + (state ? ' enabled' : '');
    $('enable').className = state ? '' : 'on';
    $('disable').className = state ? 'off' : '';
    note(state ? 'remote_enable（原固件会先软复位并置位远程模式）——干跑，未发送'
               : 'remote_disable（原固件会先软复位并清除远程模式）——干跑，未发送');
    render();
  }
  $('enable').addEventListener('click', function () { setEnabled(true); });
  $('disable').addEventListener('click', function () { setEnabled(false); });

  /* ---------- command buttons ---------- */
  function cmd(text) {
    note('命令预览：' + text + '（干跑，未发送）');
  }
  $('c-rel').addEventListener('click', function () { cmd('rel_rotate <joint_id> <angle>'); });
  $('c-auto').addEventListener('click', function () { cmd('auto <x> <y> <z>'); });
  $('c-soft').addEventListener('click', function () { cmd('soft_reset'); });
  $('c-hard').addEventListener('click', function () { cmd('hard_reset'); });
  $('c-zero').addEventListener('click', function () { cmd('zero'); });

  /* ---------- 10 Hz protocol cadence, like the original loop ---------- */
  setInterval(function () {
    if (!enabled) return;
    var d = derived();
    var moving = Math.abs(d.vx) + Math.abs(d.vy) + Math.abs(d.vz) + Math.abs(d.rx) + Math.abs(d.ry) > 0.01;
    if (moving) note('10Hz → ' + lastLine);
  }, 100);

  render();
  note('控制台就绪。远程模式未使能，所有命令仅预览。');
})();

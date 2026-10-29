/* GEArm TCP control page. Dry-run only: computes and visualises, never moves. */
(function () {
  'use strict';
  var JOINTS = 6;
  var joints = [0, 0, 0, 0, 0, 0];
  var solution = null;
  var target = [200, 0, 200];

  var $ = function (id) { return document.getElementById(id); };
  var log = $('log');

  function note(text, bad) {
    var li = document.createElement('li');
    li.textContent = text;
    if (bad) li.className = 'bad';
    log.insertBefore(li, log.firstChild);
    while (log.children.length > 40) log.removeChild(log.lastChild);
  }

  function post(body) {
    return fetch('/api/kinematics', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    }).then(function (r) {
      return r.json().then(function (data) { return { status: r.status, data: data }; });
    });
  }

  function buildSliders() {
    var host = $('sliders');
    host.innerHTML = '';
    for (var i = 0; i < JOINTS; i++) {
      (function (index) {
        var row = document.createElement('div');
        row.className = 'row';
        var label = document.createElement('label');
        label.textContent = '关节 ' + (index + 1);
        var input = document.createElement('input');
        input.type = 'range';
        input.min = '-180';
        input.max = '180';
        input.step = '0.5';
        input.value = String(joints[index]);
        var out = document.createElement('output');
        out.textContent = joints[index].toFixed(1);
        input.addEventListener('input', function () {
          joints[index] = parseFloat(input.value);
          out.textContent = joints[index].toFixed(1);
          refresh();
        });
        row.appendChild(label);
        row.appendChild(input);
        row.appendChild(out);
        host.appendChild(row);
      })(i);
    }
  }

  function drawGrid() {
    var g = $('grid');
    var parts = [];
    for (var x = -400; x <= 400; x += 100) {
      parts.push('<line class="gridline" x1="' + x + '" y1="-500" x2="' + x + '" y2="100"/>');
    }
    // Screen vertical is -Y now (Y-up frame), so these are Y gridlines.
    for (var y = -500; y <= 100; y += 100) {
      parts.push('<line class="gridline" x1="-400" y1="' + y + '" x2="400" y2="' + y + '"/>');
    }
    // X axis at Y=0, Y axis at X=0.
    parts.push('<line class="axis" x1="-400" y1="0" x2="400" y2="0"/>');
    parts.push('<line class="axis" x1="0" y1="-500" x2="0" y2="100"/>');
    g.innerHTML = parts.join('');
  }

  /* The chain is now Y-up (see /api/model frame), so the vertical screen axis is
     Y, negated because SVG's y axis points down. It used to be Z. */
  function render(chain) {
    var pts = [[0, 0, 0]].concat(chain);
    var poly = pts.map(function (p) { return p[0] + ',' + (-p[1]); }).join(' ');
    $('arm').setAttribute('points', poly);
    var jointsG = $('joints');
    var marks = pts.map(function (p, i) {
      return '<circle class="joint" cx="' + p[0] + '" cy="' + (-p[1]) + '" r="' + (i === 0 ? 9 : 6) + '"/>';
    }).join('');
    jointsG.innerHTML = marks;
    var tcp = pts[pts.length - 1];
    $('tcpmark').setAttribute('cx', tcp[0]);
    $('tcpmark').setAttribute('cy', -tcp[1]);
    $('targetline').setAttribute('points',
      tcp[0] + ',' + (-tcp[1]) + ' ' + target[0] + ',' + (-target[1]));
    $('tcpnote').textContent = 'TCP  X ' + tcp[0].toFixed(1) + '  Y ' + tcp[1].toFixed(1) +
      '  Z ' + tcp[2].toFixed(1) + ' mm（Y 朝上）';
  }

  function refresh() {
    post({ op: 'fk', joints: joints }).then(function (res) {
      if (res.status !== 200 || !res.data.ok) {
        note('FK 失败：' + JSON.stringify(res.data), true);
        return;
      }
      render(res.data.chain);
      var t = res.data.tcp;
      $('tcpinfo').innerHTML =
        '<div class="kv"><span>X</span><strong>' + t[0].toFixed(2) + '</strong></div>' +
        '<div class="kv"><span>Y</span><strong>' + t[1].toFixed(2) + '</strong></div>' +
        '<div class="kv"><span>Z</span><strong>' + t[2].toFixed(2) + '</strong></div>';
    }).catch(function (e) { note('FK 请求异常：' + e, true); });
  }

  function solve() {
    target = [parseFloat($('tx').value), parseFloat($('ty').value), parseFloat($('tz').value)];
    post({ op: 'ik', target: target, seed: joints }).then(function (res) {
      if (res.status !== 200 || !res.data.ok) {
        note('IK 被拒绝：' + JSON.stringify(res.data), true);
        return;
      }
      solution = res.data;
      $('ikinfo').innerHTML =
        '<div class="kv"><span>残差</span><strong>' + solution.residual.toExponential(2) + ' mm</strong></div>' +
        '<div class="kv"><span>迭代</span><strong>' + solution.iterations + '</strong></div>' +
        '<div class="kv"><span>关节解</span><strong>' +
        solution.joints.map(function (v) { return v.toFixed(1); }).join(', ') + '</strong></div>';
      $('apply').disabled = false;
      note('IK 已解出：残差 ' + solution.residual.toExponential(2) + ' mm');
    }).catch(function (e) { note('IK 请求异常：' + e, true); });
  }

  function applySolution() {
    if (!solution) return;
    joints = solution.joints.slice();
    var inputs = $('sliders').querySelectorAll('input');
    for (var i = 0; i < JOINTS; i++) {
      inputs[i].value = String(joints[i]);
      inputs[i].nextSibling.textContent = joints[i].toFixed(1);
    }
    refresh();
    note('关节解已应用到滑杆（仍为干跑，未发送命令）');
  }

  function pollStatus() {
    fetch('/api/status').then(function (r) { return r.json(); }).then(function (s) {
      var pill = $('connection');
      pill.textContent = s.connection;
      pill.className = 'pill ' + s.connection;
    }).catch(function () {});
  }

  $('solve').addEventListener('click', solve);
  $('apply').addEventListener('click', applySolution);
  $('move').addEventListener('click', function () {
    note('移动命令未放行：固件白名单拒绝运动功能码', true);
  });

  buildSliders();
  drawGrid();
  refresh();
  pollStatus();
  setInterval(pollStatus, 2000);
})();

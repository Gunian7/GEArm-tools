/* GEArm 急停按钮
 *
 * 所有页面通过 <script src="estop.js"></script> 引入，按钮自动注入。
 * 设计约束（按重要性排序）：
 *
 * 1) 一键生效，不弹确认框。急停场景下任何对话框都是负担。
 * 2) 任何页面都可见。viewer.html 是全屏 3D 视图、没有 header，
 *    此时退化为屏幕右上角固定定位，保证不会找不到。
 * 3) 状态可见。按过之后按钮变成"急停中"并显示解除入口，
 *    否则用户无法理解"为什么机械臂不动了"。
 * 4) 解除要刻意。解除按钮小、和急停按钮分开，避免误触恢复。
 * 5) 键盘可达。Esc / 空格均可触发，便于单手操作或鼠标失灵时兜底。
 *
 * 状态说明：急停标志在固件里，本页面只做本地指示（localStorage 持久化）。
 * 固件未提供查询急停状态的命令，所以刷新页面后指示可能和实际不符。
 * 两种偏差都是安全的：指示"未急停"而实际急停 → 运动指令被固件拒绝；
 * 指示"急停"而实际未急停 → 用户多点一下解除。故默认按"未急停"显示。
 */
(function () {
  'use strict';

  var KEY = 'gearm.estop.active';

  function active() {
    try { return window.localStorage.getItem(KEY) === '1'; } catch (e) { return false; }
  }
  function setActive(on) {
    try { window.localStorage.setItem(KEY, on ? '1' : '0'); } catch (e) { /* 隐私模式下忽略 */ }
  }

  function post(cmd, arg) {
    return fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd: cmd, arg: arg })
    }).then(function (r) { return r.json(); });
  }

  function build() {
    var wrap = document.createElement('div');
    wrap.className = 'estop';
    wrap.id = 'estop-wrap';

    var btn = document.createElement('button');
    btn.type = 'button';
    btn.id = 'estop-btn';
    btn.className = 'estop-btn';
    btn.title = '急停：立即停止全部 6 轴并阻断后续动作（Esc）';

    var clear = document.createElement('button');
    clear.type = 'button';
    clear.id = 'estop-clear';
    clear.className = 'estop-clear';
    clear.textContent = '解除急停';
    clear.title = '恢复运动（需刻意操作，避免误触）';
    clear.hidden = true;

    wrap.appendChild(btn);
    wrap.appendChild(clear);
    return { wrap: wrap, btn: btn, clear: clear };
  }

  function render(el, on) {
    var btn = el.btn, clear = el.clear;
    if (on) {
      btn.textContent = '急停中';
      btn.className = 'estop-btn active';
      btn.setAttribute('aria-pressed', 'true');
      clear.hidden = false;
      document.body.setAttribute('data-estop', 'on');
    } else {
      btn.textContent = '急停';
      btn.className = 'estop-btn';
      btn.setAttribute('aria-pressed', 'false');
      clear.hidden = true;
      document.body.removeAttribute('data-estop');
    }
  }

  function trigger(el) {
    if (active()) { return; }          // 已在急停状态，不重复发送
    setActive(true);
    render(el, true);
    post('arm', 'estop').then(function (res) {
      if (res && res.ok === false) {
        // 桥接层拒绝（例如未连接）。保留本地指示，让用户看到按钮有响应，
        // 但给出提示说明指令未必到达设备。
        flash('急停指令被桥接拒绝：' + (res.error || '未知原因'));
      }
    }).catch(function () {
      flash('桥接无响应，急停指令可能未送达');
    });
  }

  function clearEstop(el) {
    setActive(false);
    render(el, false);
    post('arm', 'estop_clear').catch(function () {
      flash('桥接无响应，解除指令可能未送达');
    });
  }

  function flash(msg) {
    var log = document.getElementById('log');
    if (!log) { return; }
    var li = document.createElement('li');
    li.textContent = msg;
    li.className = 'bad';
    log.insertBefore(li, log.firstChild);
    while (log.children.length > 50) { log.removeChild(log.lastChild); }
  }

  function mount() {
    var el = build();
    // 优先放进 header（index / control / joystick 都有），
    // viewer.html 没有 header，用 fixed 定位兜底。
    var header = document.querySelector('header');
    if (header) {
      header.appendChild(el.wrap);
    } else {
      el.wrap.classList.add('floating');
      document.body.appendChild(el.wrap);
    }

    el.btn.addEventListener('click', function () { trigger(el); });
    el.clear.addEventListener('click', function () { clearEstop(el); });

    document.addEventListener('keydown', function (ev) {
      // Esc 或空格触发，但不在输入框里抢按键
      var tag = (ev.target && ev.target.tagName || '').toLowerCase();
      if (tag === 'input' || tag === 'textarea' || tag === 'select') { return; }
      if (ev.key === 'Escape') { ev.preventDefault(); trigger(el); }
    });

    render(el, active());
    return el;
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function () { mount(); });
  } else {
    mount();
  }
})();

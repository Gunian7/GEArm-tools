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

  /* 急停命令走 arm 文本通道。桥接把 arm 的 arg 归一化成 null，
     所以"这条回显是不是我的"只能靠 bridge 分配的命令 id（seq）判断，
     命令名和参数都不足以区分两条 arm 命令。 */
  var ESTOP_CMD = 'arm';
  var ESTOP_ARG = 'estop';

  /* mount() 之后持有按钮元素；公开 API 不依赖调用方自己找到按钮。 */
  var mounted = null;

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

  /* 轮询命令状态，直到"这一条"命令的回显到达。
     命令通道是异步的：POST 返回只说明桥接收下了，板子要到收到回显
     "+OK <cmd>" 才算真确认。所以发完必须等 device_reply，
     不能拿"发送成功"当时效证据。

     seq 是 bridge 在接收命令时分配的单调递增整数，POST 响应里回传。
     用它做凭证而不是时间戳：时钟不可靠，而 seq 由桥接侧生成。

     匹配必须是 seq 精确相等，不能"取最近一条带 device_reply 的"：
     arm 命令的 arg 在桥接里被归一化成 null、cmd 又恒为 'arm'，
     所以除了 seq 没有任何字段能区分两条 arm 命令。放宽成"更新的 seq"
     就会把后续 rel_rotate 的回显认成急停的确认——正是要消除的缺陷。 */
  function waitDeviceReply(seq, timeoutMs) {
    var deadline = Date.now() + (timeoutMs || 2500);
    return new Promise(function (resolve) {
      (function poll() {
        fetch('/api/status').then(function (r) { return r.json(); }).then(function (st) {
          var rows = st.commands || [];
          for (var i = rows.length - 1; i >= 0; i--) {
            var row = rows[i];
            if (row.seq !== seq) { continue; }
            if (row.cmd !== ESTOP_CMD) { continue; }
            if (row.device_reply) { resolve(row.device_reply); return; }
          }
          if (Date.now() > deadline) { resolve(null); return; }
          setTimeout(poll, 120);
        }).catch(function () { resolve(null); });
      })();
    });
  }

  /* 在按钮上显示一行临时反馈，让用户知道这一下到底成没成。 */
  function flash(msg, kind) {
    var el = document.getElementById('estop-feedback');
    if (!el) { return; }
    el.textContent = msg;
    el.className = 'estop-feedback ' + (kind || '');
    if (msg) {
      setTimeout(function () {
        if (el.textContent === msg) { el.textContent = ''; el.className = 'estop-feedback'; }
      }, 4000);
    }
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

    /* 反馈行：显示"已发送/板子已确认/未送达"。没有它，用户按了急停
       在机械臂不动的情况下完全看不出这一下起没起作用。 */
    var fb = document.createElement('div');
    fb.id = 'estop-feedback';
    fb.className = 'estop-feedback';
    wrap.appendChild(fb);

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

  /* 触发急停，并把"到底发出去没发出去"以明确的字符串状态返回给调用方：
       'confirmed'   板子的回显已确认这一条命令
       'already'     本地已在急停态，未重复发送
       'unconfirmed' 桥接已接收，但没等到板子回显
       'rejected'    桥接拒绝（例如未连接）
       'unreachable' 桥接无响应，或没回传命令 id，无法确认
     返回状态而不是 undefined，是给自动急停（limit_alert.js）用的：
     它不能靠"点了一下没报错"当成急停成功。 */
  function trigger(el) {
    if (active()) { return Promise.resolve('already'); }  // 已在急停状态，不重复发送
    setActive(true);
    if (el) { render(el, true); }
    flash('急停指令已发送，等待板子确认…', 'pending');
    logLine('急停指令已发送', true);
    return post(ESTOP_CMD, ESTOP_ARG).then(function (res) {
      if (!res || res.ok === false) {
        // 桥接层拒绝（例如未连接）。保留本地指示，让用户看到按钮有响应，
        // 但给出提示说明指令未必到达设备。
        var why = (res && res.error) || '未知原因';
        flash('急停指令被桥接拒绝：' + why, 'bad');
        logLine('急停指令被桥接拒绝：' + why, true);
        return 'rejected';
      }
      if (typeof res.seq !== 'number') {
        // 没有命令 id 就无法把回显归属到这一条命令。宁可报"无法确认"，
        // 也不能抓一条别人的回显当成自己的确认。
        flash('桥接未返回命令 id，无法确认板子回显', 'bad');
        logLine('桥接未返回命令 id，无法确认板子回显', true);
        return 'unreachable';
      }
      return waitDeviceReply(res.seq, 2500).then(function (reply) {
        if (reply) {
          flash('板子已确认急停：' + reply, 'good');
          logLine('板子已确认急停：' + reply, false);
          return 'confirmed';
        }
        flash('桥接已接收，但未收到板子回显（可能未连接或已掉线）', 'bad');
        logLine('未收到板子回显：急停是否生效未知', true);
        return 'unconfirmed';
      });
    }).catch(function () {
      flash('桥接无响应，急停指令可能未送达', 'bad');
      logLine('桥接无响应，急停指令可能未送达', true);
      return 'unreachable';
    });
  }

  function clearEstop(el) {
    setActive(false);
    render(el, false);
    post(ESTOP_CMD, 'estop_clear').catch(function () {
      flash('桥接无响应，解除指令可能未送达');
      logLine('桥接无响应，解除指令可能未送达', true);
    });
  }

  /* 追加一条到页面日志。
     与 flash 分开：flash 写按钮下方那一行反馈（build() 里的 #estop-feedback），
     logLine 留一条可追溯的记录。两者原先同名，后一个函数声明覆盖前一个，
     结果是反馈行从来收不到消息、所有提示都进了日志。 */
  function logLine(msg, bad) {
    var log = document.getElementById('log');
    if (!log) { return; }
    var li = document.createElement('li');
    li.textContent = msg;
    if (bad) { li.className = 'bad'; }
    log.insertBefore(li, log.firstChild);
    while (log.children.length > 50) { log.removeChild(log.lastChild); }
  }

  function mount() {
    var el = build();
    mounted = el;
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

  /* 公开 API：自动急停（limit_alert.js）走这里，而不是去 click 那个按钮。
     按钮由本文件注入，一旦加载顺序变化或页面变体没有它，
     "点击"就会静默失效——限位已经触发，急停却没发出去，且没有任何提示。
     trigger() 返回明确的处置状态，null 元素也能照常把命令发出去。 */
  window.GEArmEstop = {
    trigger: function () { return trigger(mounted); },
    isActive: active
  };
})();

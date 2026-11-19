/* GEArm 限位触发警报
 *
 * 通过 WebSocket 监听设备的限位触发消息，弹窗警告并自动触发急停。
 * 固件发送 "!LIMIT_TRIGGERED jX" 时，bridge 会推送 limit_alert 消息。
 */
(function () {
  'use strict';

  let ws = null;
  let reconnectTimer = null;
  const RECONNECT_DELAY = 2000;

  // 当前弹窗的 ESC 监听器。始终只保留一个：开新弹窗前先摘掉上一个，
  // 关闭时（确定 / 点背景 / ESC 三条路径）也一定摘掉。否则监听器会随
  // 告警次数累加，每次按 ESC 都要把陈年闭包跑一遍。
  let escHandler = null;

  function connect() {
    if (ws && ws.readyState === WebSocket.OPEN) return;

    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const url = proto + '//' + window.location.host + '/ws';

    try {
      ws = new WebSocket(url);

      ws.onopen = function () {
        console.log('[limit_alert] WebSocket connected');
        if (reconnectTimer) {
          clearTimeout(reconnectTimer);
          reconnectTimer = null;
        }
      };

      ws.onmessage = function (event) {
        try {
          const msg = JSON.parse(event.data);

          // 处理限位触发消息
          if (msg.kind === 'limit_alert') {
            handleLimitAlert(msg);
          }
        } catch (e) {
          console.error('[limit_alert] Failed to parse message:', e);
        }
      };

      ws.onerror = function (err) {
        console.error('[limit_alert] WebSocket error:', err);
      };

      ws.onclose = function () {
        console.log('[limit_alert] WebSocket closed, will reconnect...');
        ws = null;
        reconnectTimer = setTimeout(connect, RECONNECT_DELAY);
      };
    } catch (e) {
      console.error('[limit_alert] Failed to create WebSocket:', e);
      reconnectTimer = setTimeout(connect, RECONNECT_DELAY);
    }
  }

  /* 关节号必须是纯整数。它会被拼进 innerHTML，虽然 bridge 侧已用正则
     归一化，这里再强制一次作为深度防御：归一之后不可能带 HTML 元字符。 */
  function normalizeJointId(raw) {
    const n = parseInt(raw, 10);
    return Number.isInteger(n) ? n : null;
  }

  function describeEstop(status) {
    if (status === 'confirmed') { return { ok: true, text: '自动急停已确认：板子已回显' }; }
    if (status === 'already') { return { ok: true, text: '急停此前已处于生效状态，未重复发送' }; }
    if (status === 'unconfirmed') {
      return { ok: false, text: '自动急停已发出，但未收到板子回显（是否生效未知）' };
    }
    if (status === 'rejected') { return { ok: false, text: '自动急停未能发送：桥接拒绝' }; }
    return { ok: false, text: '自动急停未能发送：桥接无响应或未返回命令 id' };
  }

  /* estop.js 的公开 API 不存在时的兜底：直接发命令，不再依赖那个按钮节点。
     POST 响应里的 device_reply 是 bridge 对"这一条"命令的返回，归属明确。 */
  function directEstop() {
    return fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd: 'arm', arg: 'estop' })
    }).then(function (r) { return r.json(); }).then(function (res) {
      if (!res || res.ok === false) {
        return { ok: false, text: '自动急停未能发送：桥接拒绝（' +
          ((res && res.error) || '未知原因') + '）' };
      }
      if (res.device_reply) {
        return { ok: true, text: '自动急停已确认：' + res.device_reply };
      }
      return { ok: false, text: '自动急停已发出，但未收到板子回显（是否生效未知）' };
    }).catch(function () {
      return { ok: false, text: '自动急停未能发送：桥接无响应' };
    });
  }

  /* 自动急停。优先调用 estop.js 暴露的 API；该 API 不存在时直接 POST 兜底。
     两条路的失败都必须返回一句写明"自动急停未能发送"的文字，
     绝不允许静默：限位已经触发而急停没发出去，是必须让人看见的事。 */
  function autoEstop() {
    const api = window.GEArmEstop;
    if (api && typeof api.trigger === 'function') {
      return new Promise(function (resolve) {
        let out;
        try {
          out = api.trigger();
        } catch (e) {
          resolve({ ok: false, text: '自动急停未能发送：estop API 调用异常（' + e + '）' });
          return;
        }
        Promise.resolve(out).then(function (status) {
          resolve(describeEstop(status));
        }, function (e) {
          resolve({ ok: false, text: '自动急停未能发送：estop API 调用失败（' + e + '）' });
        });
      });
    }
    return directEstop();
  }

  function handleLimitAlert(msg) {
    const jointId = normalizeJointId(msg.joint_id);
    const label = jointId === null ? '未知关节' : ('关节' + jointId);
    const message = (typeof msg.message === 'string' && msg.message)
      ? msg.message : (label + '限位触发');

    console.warn('[limit_alert]', message);

    // 先弹窗：安全警告不等异步结果。急停结论随后补进弹窗那一行。
    const setEstopNote = showAlertModal(message, jointId);

    autoEstop().then(function (result) {
      console.warn('[limit_alert]', result.text);
      setEstopNote(result.text);
      addLog(result.text, !result.ok);
    });

    // 在日志中添加警告
    addLog(message, true);
  }

  function showAlertModal(message, jointId) {
    // 移除旧的弹窗，并先摘掉它遗留的 keydown 监听器。
    // 只 remove() 节点不会摘监听器——监听器挂在 document 上。
    const oldModal = document.getElementById('limit-alert-modal');
    if (oldModal) {
      closeModal(oldModal);
    } else if (escHandler) {
      document.removeEventListener('keydown', escHandler);
      escHandler = null;
    }

    // jointId 已被 normalizeJointId() 强制成整数或 null，
    // 拼进 innerHTML 的只可能是数字和固定文案。
    const jointText = jointId === null ? '未知' : ('J' + jointId + ' (关节' + jointId + ')');

    // 创建弹窗
    const modal = document.createElement('div');
    modal.id = 'limit-alert-modal';
    modal.className = 'limit-alert-modal';
    modal.innerHTML = `
      <div class="limit-alert-content">
        <div class="limit-alert-icon">⚠️</div>
        <h2 class="limit-alert-title">限位触发警告</h2>
        <p class="limit-alert-message">${escapeHtml(message)}</p>
        <div class="limit-alert-details">
          <strong>触发关节：</strong>${jointText}
          <div id="limit-alert-estop">自动急停：发送中…</div>
        </div>
        <div class="limit-alert-actions">
          <button id="limit-alert-close" class="limit-alert-btn-primary">确定</button>
        </div>
      </div>
    `;

    // 添加样式
    if (!document.getElementById('limit-alert-styles')) {
      const style = document.createElement('style');
      style.id = 'limit-alert-styles';
      style.textContent = `
        .limit-alert-modal {
          position: fixed;
          inset: 0;
          background: rgba(0, 0, 0, 0.75);
          display: flex;
          align-items: center;
          justify-content: center;
          z-index: 10000;
          animation: fadeIn 0.2s ease-out;
        }
        @keyframes fadeIn {
          from { opacity: 0; }
          to { opacity: 1; }
        }
        .limit-alert-content {
          background: #1a1b1e;
          border: 2px solid #e74c3c;
          border-radius: 12px;
          padding: 28px 32px;
          max-width: 480px;
          box-shadow: 0 8px 32px rgba(231, 76, 60, 0.4);
          animation: slideIn 0.3s ease-out;
        }
        @keyframes slideIn {
          from { transform: translateY(-20px); opacity: 0; }
          to { transform: translateY(0); opacity: 1; }
        }
        .limit-alert-icon {
          font-size: 48px;
          text-align: center;
          margin-bottom: 16px;
        }
        .limit-alert-title {
          font-size: 22px;
          font-weight: 600;
          color: #e74c3c;
          text-align: center;
          margin: 0 0 16px 0;
        }
        .limit-alert-message {
          font-size: 16px;
          color: #e8e9eb;
          text-align: center;
          line-height: 1.6;
          margin: 0 0 20px 0;
        }
        .limit-alert-details {
          background: rgba(231, 76, 60, 0.1);
          border: 1px solid rgba(231, 76, 60, 0.3);
          border-radius: 8px;
          padding: 12px 16px;
          font-size: 14px;
          color: #e8e9eb;
          margin-bottom: 24px;
        }
        .limit-alert-details strong {
          color: #e74c3c;
        }
        .limit-alert-actions {
          display: flex;
          justify-content: center;
          gap: 12px;
        }
        .limit-alert-btn-primary {
          background: #e74c3c;
          color: white;
          border: none;
          border-radius: 8px;
          padding: 10px 32px;
          font-size: 15px;
          font-weight: 500;
          cursor: pointer;
          transition: background 0.2s;
        }
        .limit-alert-btn-primary:hover {
          background: #c0392b;
        }
      `;
      document.head.appendChild(style);
    }

    document.body.appendChild(modal);

    // 关闭按钮
    const closeBtn = document.getElementById('limit-alert-close');
    if (closeBtn) {
      closeBtn.addEventListener('click', function () {
        closeModal(modal);
      });
    }

    // 点击背景关闭
    modal.addEventListener('click', function (e) {
      if (e.target === modal) {
        closeModal(modal);
      }
    });

    // ESC 键关闭。三条关闭路径统一走 closeModal()，都会摘掉监听器。
    escHandler = function (e) {
      if (e.key === 'Escape') {
        closeModal(modal);
      }
    };
    document.addEventListener('keydown', escHandler);

    // 返回一个"补写急停结论"的句柄：急停结果要等异步回来才确定。
    const estopNote = document.getElementById('limit-alert-estop');
    return function (text) {
      if (estopNote) { estopNote.textContent = text; }
    };
  }

  /* 唯一的关闭出口：先摘监听器，再删节点。
     确定按钮 / 点背景 / ESC 三条路径都走这里，所以不会有残留。 */
  function closeModal(modal) {
    if (escHandler) {
      document.removeEventListener('keydown', escHandler);
      escHandler = null;
    }
    if (modal) { modal.remove(); }
  }

  function addLog(message, isBad) {
    const log = document.getElementById('log');
    if (!log) return;

    const li = document.createElement('li');
    li.textContent = `[限位] ${message}`;
    if (isBad) {
      li.className = 'bad';
    }
    log.insertBefore(li, log.firstChild);

    // 保持日志不超过 50 条
    while (log.children.length > 50) {
      log.removeChild(log.lastChild);
    }
  }

  function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
  }

  // 页面加载完成后连接 WebSocket
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', connect);
  } else {
    connect();
  }

  // 页面卸载时断开连接
  window.addEventListener('beforeunload', function () {
    if (ws) {
      ws.close();
    }
  });
})();

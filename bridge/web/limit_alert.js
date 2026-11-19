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

  function handleLimitAlert(msg) {
    const jointId = msg.joint_id;
    const message = msg.message || `关节${jointId}限位触发，已急停`;

    console.warn('[limit_alert]', message);

    // 显示模态弹窗
    showAlertModal(message, jointId);

    // 触发急停按钮（如果存在）
    const estopBtn = document.getElementById('estop-btn');
    if (estopBtn) {
      estopBtn.click();
    }

    // 在日志中添加警告
    addLog(message, true);
  }

  function showAlertModal(message, jointId) {
    // 移除旧的弹窗（如果存在）
    const oldModal = document.getElementById('limit-alert-modal');
    if (oldModal) {
      oldModal.remove();
    }

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
          <strong>触发关节：</strong>J${jointId} (关节${jointId})
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
        modal.remove();
      });
    }

    // 点击背景关闭
    modal.addEventListener('click', function (e) {
      if (e.target === modal) {
        modal.remove();
      }
    });

    // ESC 键关闭
    const handleEsc = function (e) {
      if (e.key === 'Escape') {
        modal.remove();
        document.removeEventListener('keydown', handleEsc);
      }
    };
    document.addEventListener('keydown', handleEsc);
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

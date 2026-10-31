"use strict";

const nodes = {
  connection: document.querySelector("#connection"),
  retry: document.querySelector("#retry"),
  heartbeat: document.querySelector("#heartbeat"),
  initStage: document.querySelector("#init-stage"),
  txEnqueued: document.querySelector("#tx-enqueued"),
  txComplete: document.querySelector("#tx-complete"),
  rxCount: document.querySelector("#rx-count"),
  matched: document.querySelector("#matched"),
  timeouts: document.querySelector("#timeouts"),
  lastFn: document.querySelector("#last-fn"),
  lastRaw: document.querySelector("#last-raw"),
  log: document.querySelector("#log")
};

const seen = new Set();

function text(node, value) {
  node.textContent = value === undefined || value === null || value === "" ? "-" : String(value);
}

function hexByte(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) {
    return "-";
  }
  return "0x" + number.toString(16).toUpperCase().padStart(2, "0");
}

function formatRaw(value) {
  if (!value) {
    return "-";
  }
  return String(value).replace(/(.{2})/g, "$1 ").trim();
}

function renderTelemetry(telem) {
  const data = telem || {};
  text(nodes.heartbeat, data.heartbeat);
  text(nodes.initStage, data.init_stage);
  text(nodes.txEnqueued, data.tx_enqueued);
  text(nodes.txComplete, data.tx_complete);
  text(nodes.rxCount, data.rx_count);
  text(nodes.matched, data.matched);
  text(nodes.timeouts, data.timeouts);
  text(nodes.lastFn, data.last_fn === undefined || data.last_fn === null ? "-" : hexByte(data.last_fn));
  text(nodes.lastRaw, formatRaw(data.last_raw));
}

function commandLine(row) {
  const arg = row.arg === undefined || row.arg === null ? "" : " arg=" + row.arg;
  let detail = "";
  if (row.response && row.response.kind) {
    detail = " " + row.response.kind;
    if (row.response.kind === "ack") {
      detail += " type=" + hexByte(row.response.type) + " arg=" + row.response.arg;
    }
    if (row.response.kind === "error") {
      detail += " type=" + hexByte(row.response.type) + " code=" + row.response.code;
    }
  }
  if (row.device_reply) {
    /* 板子亲口回了 "+OK <命令>"，说明指令真的到达并被收下了。
       这是用户在机械臂不动时唯一能确认"急停确实生效了"的依据，
       所以单独标出来，不要混在普通的 state 里。 */
    detail += " 板子已确认: " + row.device_reply;
  }
  if (row.error) {
    detail += " " + row.error;
  }
  return "#" + row.seq + " " + row.cmd + arg + " " + row.state + detail;
}

function addLog(key, message, bad) {
  if (seen.has(key)) {
    const existing = nodes.log.querySelector("[data-key='" + key + "']");
    if (existing) {
      existing.textContent = message;
      existing.classList.toggle("bad", !!bad);
      return;
    }
  }
  seen.add(key);
  const item = document.createElement("li");
  item.dataset.key = key;
  item.textContent = message;
  if (bad) {
    item.classList.add("bad");
  }
  nodes.log.prepend(item);
}

function renderCommands(rows) {
  (rows || []).forEach((row) => {
    const bad = row.state === "device_error" || row.state === "timeout" || row.state === "transport_lost" || row.state === "cancelled";
    addLog("cmd-" + row.seq + "-" + row.state, commandLine(row), bad);
  });
}

function renderStatus(data) {
  text(nodes.connection, data.connection);
  nodes.connection.className = "pill " + (data.connection || "");
  text(nodes.retry, data.retry_count);
  renderTelemetry(data.telemetry);
  renderCommands(data.commands);
  (data.rejections || []).forEach((row, index) => {
    addLog("rej-" + index + "-" + row.cmd, "拒绝 " + row.cmd + (row.detail ? " " + row.detail : ""), true);
  });
}

async function refresh() {
  try {
    const response = await fetch("/api/status");
    if (!response.ok) {
      return;
    }
    renderStatus(await response.json());
  } catch (err) {
    text(nodes.connection, "bridge unreachable");
  }
}

async function sendCommand(cmd, arg) {
  const body = {cmd: cmd};
  if (arg !== undefined) {
    body.arg = arg;
  }
  try {
    const response = await fetch("/api/command", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(body)
    });
    const data = await response.json();
    if (!response.ok || data.ok === false) {
      addLog("post-" + Date.now(), "拒绝 " + (data.cmd || cmd), true);
    }
    await refresh();
  } catch (err) {
    addLog("post-" + Date.now(), "桥接没有接受 " + cmd, true);
  }
}

function connectSocket() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const socket = new WebSocket(proto + "//" + location.host + "/ws");
  socket.onmessage = (event) => {
    let data;
    try {
      data = JSON.parse(event.data);
    } catch (err) {
      return;
    }
    if (data.kind === "telemetry") {
      renderTelemetry(data.telemetry);
      if (data.connection) {
        text(nodes.connection, data.connection);
        nodes.connection.className = "pill " + data.connection;
      }
    } else if (data.kind === "status") {
      text(nodes.connection, data.connection);
      nodes.connection.className = "pill " + (data.connection || "");
      text(nodes.retry, data.retry_count);
    } else if (data.kind === "command" && data.command) {
      renderCommands([data.command]);
    }
  };
  socket.onclose = () => {
    window.setTimeout(connectSocket, 1000);
  };
}

document.querySelector("#ping").addEventListener("click", () => {
  sendCommand("ping");
});
document.querySelector("#query-version").addEventListener("click", () => {
  sendCommand("query", 0);
});
document.querySelector("#query-voltage").addEventListener("click", () => {
  sendCommand("query", 1);
});
document.querySelector("#query-state").addEventListener("click", () => {
  sendCommand("query", 2);
});
document.querySelector("#rate-btn").addEventListener("click", () => {
  const value = Number(document.querySelector("#rate").value);
  if (!Number.isInteger(value) || value < 1 || value > 100) {
    addLog("rate-bad-" + Date.now(), "拒绝 遥测速率超出 1..100", true);
    return;
  }
  sendCommand("rate", value);
});
document.querySelector("#stop").addEventListener("click", () => {
  sendCommand("stop");
});

refresh();
window.setInterval(refresh, 500);
connectSocket();

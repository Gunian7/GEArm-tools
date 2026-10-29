#include "protocol.h"

static void wipe(void *dst, size_t n) {
  unsigned char *p = dst;
  while (n--) *p++ = 0;
}
static void put_u32(uint8_t *d, uint32_t v) {
  d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8); d[2] = (uint8_t)(v >> 16); d[3] = (uint8_t)(v >> 24);
}
static uint32_t get_u32(const uint8_t *d) {
  return (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}
static int motion_code(uint8_t type) {
  switch (type) {
    case 0x0A: case 0x0E: case 0x46: case 0x4C: case 0x93:
    case 0x9A: case 0x9C: case 0xF3: case 0xF6: case 0xFD:
    case 0xFE: case 0xFF:
      return 1;
    default:
      return 0;
  }
}
static uint32_t period_ms(uint8_t hz) {
  uint32_t period = 0, acc = 0;
  if (!hz) return 0;
  while (acc + hz <= 1000u) { acc += hz; period++; }
  return period;
}

uint8_t ser_crc8(const uint8_t *data, size_t len) {
  uint8_t crc = SER_CRC8_INIT;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80u) crc = (uint8_t)((crc << 1) ^ SER_CRC8_POLY);
      else crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}
int ser_encode(uint8_t type, const uint8_t *payload, uint8_t payload_len, uint8_t *out, size_t cap, size_t *out_len) {
  if (out_len) *out_len = 0;
  if (!out || payload_len > SER_MAX_PAYLOAD || (payload_len && !payload)) return 0;
  if (cap < (size_t)payload_len + 4u) return 0;
  out[0] = SER_SOF;
  out[1] = payload_len;
  out[2] = type;
  for (uint8_t i = 0; i < payload_len; i++) out[3u + i] = payload[i];
  out[3u + payload_len] = ser_crc8(out, 3u + payload_len);
  if (out_len) *out_len = 4u + payload_len;
  return 1;
}
void ser_parser_init(SerParser *p) { if (p) wipe(p, sizeof *p); }
int ser_parse_byte(SerParser *p, uint8_t byte, SerFrame *out, uint8_t *err) {
  if (!p || !out || !err) return -1;
  switch (p->state) {
    case 0:
      if (byte == SER_SOF) { p->state = 1; p->body[0] = SER_SOF; p->body_n = 1; }
      return 0;
    case 1:
      if (byte > SER_MAX_PAYLOAD) {
        *err = SER_ERR_BAD_LEN;
        if (byte == SER_SOF) { p->state = 1; p->body[0] = SER_SOF; p->body_n = 1; }
        else { p->state = 0; p->body_n = 0; }
        return -1;
      }
      p->len = byte; p->body[1] = byte; p->body_n = 2; p->state = 2;
      return 0;
    case 2:
      p->type = byte; p->body[2] = byte; p->body_n = 3; p->got = 0;
      p->state = p->len ? 3 : 4;
      return 0;
    case 3:
      if (p->got >= SER_MAX_PAYLOAD || p->body_n >= sizeof p->body) {
        *err = SER_ERR_BAD_LEN; p->state = 0; p->body_n = 0; return -1;
      }
      p->body[p->body_n++] = byte;
      if (++p->got == p->len) p->state = 4;
      return 0;
    case 4: {
      uint8_t expect = ser_crc8(p->body, p->body_n);
      if (byte != expect) {
        *err = SER_ERR_BAD_CRC;
        if (byte == SER_SOF) { p->state = 1; p->body[0] = SER_SOF; p->body_n = 1; }
        else { p->state = 0; p->body_n = 0; }
        return -1;
      }
      out->type = p->type; out->len = p->len;
      for (uint8_t i = 0; i < p->len; i++) out->payload[i] = p->body[3u + i];
      p->state = 0; p->body_n = 0;
      return 1;
    }
    default:
      p->state = 0; p->body_n = 0; return 0;
  }
}
static int emit_error(uint8_t type, uint8_t code, uint8_t *tx, size_t cap, size_t *tx_len) {
  uint8_t payload[2]; payload[0] = type; payload[1] = code;
  return ser_encode(SER_ERROR, payload, 2, tx, cap, tx_len);
}
static int emit_ack(uint8_t type, uint8_t arg, uint8_t *tx, size_t cap, size_t *tx_len) {
  uint8_t payload[2]; payload[0] = type; payload[1] = arg;
  return ser_encode(SER_ACK, payload, 2, tx, cap, tx_len);
}
void ser_link_init(SerLink *l) {
  if (!l) return;
  wipe(l, sizeof *l);
  l->rate_hz = 10;
}
static int finish_can(SerLink *l, const CanIo *io) {
  if (!io || !io->stop || !io->stop()) {
    l->error = SER_ERR_TRANSPORT;
    l->can_state = SER_CAN_FAULT;
    return 0;
  }
  l->can_state = SER_CAN_IDLE;
  l->tx_seen = 0;
  return 1;
}
int ser_link_command(SerLink *l, const SerFrame *frame, uint32_t now, const CanIo *io, uint8_t *tx, size_t cap, size_t *tx_len) {
  if (tx_len) *tx_len = 0;
  if (!l || !frame || !tx || !tx_len) return -1;
  uint8_t type = frame->type;
  uint8_t len = frame->len;
  if (type == SER_PING) {
    if (len != 0) { l->error = SER_ERR_BAD_PARAM; emit_error(type, SER_ERR_BAD_PARAM, tx, cap, tx_len); return -1; }
    l->error = SER_OK;
    if (!ser_encode(SER_PONG, 0, 0, tx, cap, tx_len)) return -1;
    return 1;
  }
  if (type == SER_QUERY) {
    if (len != 1 || frame->payload[0] > SER_QUERY_STATE) {
      l->error = SER_ERR_BAD_PARAM; emit_error(type, SER_ERR_BAD_PARAM, tx, cap, tx_len); return -1;
    }
    if (l->can_state == SER_CAN_FAULT) {
      l->error = SER_ERR_STATE; emit_error(type, SER_ERR_STATE, tx, cap, tx_len); return -1;
    }
    if (l->can_state == SER_CAN_WAIT || l->can_state == SER_CAN_MATCHED) {
      l->error = SER_ERR_BUSY; emit_error(type, SER_ERR_BUSY, tx, cap, tx_len); return -1;
    }
    uint8_t fn = frame->payload[0] == SER_QUERY_VERSION ? SER_CAN_FN_VERSION :
                 frame->payload[0] == SER_QUERY_VOLTAGE ? SER_CAN_FN_VOLTAGE : SER_CAN_FN_STATE;
    uint8_t bytes[2];
    if (!can_whitelist_bytes(fn, bytes) || !io || !io->send || !io->send(bytes)) {
      l->error = SER_ERR_ENQUEUE;
      l->can_state = SER_CAN_FAULT;
      if (io) (void)finish_can(l, io);
      l->can_state = SER_CAN_FAULT;
      l->error = SER_ERR_ENQUEUE;
      emit_error(type, SER_ERR_ENQUEUE, tx, cap, tx_len);
      return -1;
    }
    l->tx_enqueued++;
    l->last_fn = fn;
    l->pending_fn = fn;
    l->last_raw_len = 0;
    l->can_start = now;
    l->can_state = SER_CAN_WAIT;
    l->tx_seen = 0;
    l->error = SER_OK;
    if (!emit_ack(type, frame->payload[0], tx, cap, tx_len)) return -1;
    return 1;
  }
  if (type == SER_TELEMETRY_RATE) {
    if (len != 1 || frame->payload[0] < 1 || frame->payload[0] > 100) {
      l->error = SER_ERR_BAD_PARAM; emit_error(type, SER_ERR_BAD_PARAM, tx, cap, tx_len); return -1;
    }
    l->rate_hz = frame->payload[0];
    l->error = SER_OK;
    if (!emit_ack(type, l->rate_hz, tx, cap, tx_len)) return -1;
    return 1;
  }
  if (type == SER_STOP) {
    if (len != 0) { l->error = SER_ERR_BAD_PARAM; emit_error(type, SER_ERR_BAD_PARAM, tx, cap, tx_len); return -1; }
    l->rate_hz = 0;
    if (l->can_state == SER_CAN_WAIT || l->can_state == SER_CAN_MATCHED) {
      if (!finish_can(l, io)) { emit_error(type, (uint8_t)l->error, tx, cap, tx_len); return -1; }
    }
    l->error = SER_OK;
    if (!emit_ack(type, 0, tx, cap, tx_len)) return -1;
    return 1;
  }
  if (motion_code(type)) {
    l->error = SER_ERR_MOTION; emit_error(type, SER_ERR_MOTION, tx, cap, tx_len); return -1;
  }
  l->error = SER_ERR_UNKNOWN; emit_error(type, SER_ERR_UNKNOWN, tx, cap, tx_len); return -1;
}
int ser_ingest(SerParser *p, SerLink *l, uint8_t byte, uint32_t now, const CanIo *io, uint8_t *tx, size_t cap, size_t *tx_len) {
  if (tx_len) *tx_len = 0;
  if (!p || !l || !tx || !tx_len) return -1;
  SerFrame frame; uint8_t err = 0;
  int rc = ser_parse_byte(p, byte, &frame, &err);
  if (rc == 0) return 0;
  if (rc < 0) {
    l->error = err;
    emit_error(0, err, tx, cap, tx_len);
    return -1;
  }
  return ser_link_command(l, &frame, now, io, tx, cap, tx_len);
}
void ser_link_step(SerLink *l, uint32_t now, const CanIo *io) {
  if (!l) return;
  l->heartbeat++;
  if (l->can_state == SER_CAN_WAIT || l->can_state == SER_CAN_MATCHED) {
    if (io && io->complete && io->complete() && !l->tx_seen) { l->tx_complete++; l->tx_seen = 1; }
  }
  if (l->can_state == SER_CAN_MATCHED) (void)finish_can(l, io);
  else if (l->can_state == SER_CAN_WAIT && (uint32_t)(now - l->can_start) >= 200u) {
    l->timeouts++;
    l->error = SER_ERR_TIMEOUT;
    (void)finish_can(l, io);
  }
}
void ser_link_on_can(SerLink *l, uint32_t now, uint32_t id, uint32_t extended, uint32_t remote, uint32_t fd, uint32_t brs, uint32_t dlc, const uint8_t *data) {
  if (!l) return;
  l->rx_count++;
  if (l->can_state != SER_CAN_WAIT || (uint32_t)(now - l->can_start) >= 200u) return;
  if (!data || id != SER_CAN_ID || extended != 1 || remote || fd || brs || dlc < 1 || dlc > 8) return;
  if (data[dlc - 1] != SER_CAN_CHECKSUM || data[0] != l->pending_fn) return;
  int dlc_ok = (l->pending_fn == SER_CAN_FN_VERSION && (dlc == 4 || dlc == 6)) ||
               (l->pending_fn == SER_CAN_FN_VOLTAGE && dlc == 4) ||
               (l->pending_fn == SER_CAN_FN_STATE && dlc == 3);
  if (!dlc_ok) return;
  l->last_raw_len = (uint8_t)dlc;
  for (uint32_t i = 0; i < dlc; i++) l->last_raw[i] = data[i];
  l->matched++;
  l->can_state = SER_CAN_MATCHED;
}
int ser_link_telemetry_due(const SerLink *l, uint32_t now) {
  if (!l || !l->rate_hz) return 0;
  if (!l->telem_sent) return 1;
  return (uint32_t)(now - l->last_telem) >= period_ms(l->rate_hz);
}
void ser_link_telemetry_mark(SerLink *l, uint32_t now) {
  if (!l) return;
  l->telem_sent = 1;
  l->last_telem = now;
}
int ser_link_pack_telemetry(const SerLink *l, uint8_t *out, size_t cap, size_t *out_len) {
  if (out_len) *out_len = 0;
  if (!l || !out) return 0;
  uint8_t raw[SER_TELEM_PAYLOAD];
  wipe(raw, sizeof raw);
  put_u32(raw + 0, l->heartbeat);
  put_u32(raw + 4, l->init_stage);
  put_u32(raw + 8, l->tx_enqueued);
  put_u32(raw + 12, l->tx_complete);
  put_u32(raw + 16, l->rx_count);
  put_u32(raw + 20, l->matched);
  put_u32(raw + 24, l->timeouts);
  put_u32(raw + 28, l->error);
  raw[32] = l->last_fn;
  raw[33] = l->last_raw_len;
  for (int i = 0; i < 8; i++) raw[34 + i] = l->last_raw[i];
  return ser_encode(SER_TELEMETRY, raw, SER_TELEM_PAYLOAD, out, cap, out_len);
}
int ser_telem_unpack(const uint8_t *payload, uint8_t len, SerTelem *out) {
  if (!payload || !out || len != SER_TELEM_PAYLOAD) return 0;
  wipe(out, sizeof *out);
  out->heartbeat = get_u32(payload + 0);
  out->init_stage = get_u32(payload + 4);
  out->tx_enqueued = get_u32(payload + 8);
  out->tx_complete = get_u32(payload + 12);
  out->rx_count = get_u32(payload + 16);
  out->matched = get_u32(payload + 20);
  out->timeouts = get_u32(payload + 24);
  out->error = get_u32(payload + 28);
  out->last_fn = payload[32];
  out->last_raw_len = payload[33];
  for (int i = 0; i < 8; i++) out->last_raw[i] = payload[34 + i];
  return 1;
}
int can_whitelist_bytes(uint8_t fn, uint8_t out[2]) {
  if (!out) return 0;
  if (fn != SER_CAN_FN_VERSION && fn != SER_CAN_FN_VOLTAGE && fn != SER_CAN_FN_STATE) return 0;
  out[0] = fn; out[1] = SER_CAN_CHECKSUM;
  return 1;
}
/* Host-side stand-in for the MC02 serial firmware (stdio driven, no hardware).
 *
 * The protocol state machine is the firmware's own protocol.c. This file only
 * adds: a virtual CAN driver that answers the three recorded read-only replies,
 * and a length-prefixed command channel on stdin/stdout so the Python side owns
 * the clock and the transport:
 *
 *   'F' u32 len + bytes  -> feed received serial bytes; reply: u32 len + tx bytes
 *   'T' u32 now_ms       -> one main-loop step at that clock; reply: u32 len + tx bytes
 *   'S'                  -> status (refused motion requests, accepted CAN requests)
 *   'Q'                  -> exit
 *
 * The virtual driver refuses anything outside the read-only whitelist and counts
 * it, so a motion request can never turn into a CAN frame here.
 */
#include "protocol.h"
#include <stddef.h>

__declspec(dllimport) int __cdecl _read(int, void *, unsigned int);
__declspec(dllimport) int __cdecl _write(int, const void *, unsigned int);
__declspec(dllimport) int __cdecl _setmode(int, int);
__declspec(dllimport) void __cdecl exit(int);

#define O_BINARY 0x8000

static SerLink link;
static SerParser parser;
static uint8_t txbuf[96];
static uint32_t now_ms;
static uint8_t pending_fn;
static uint32_t reply_at, tx_done_at;
static int reply_pending;
static int tx_done;
static unsigned long motion_refused;
static unsigned long can_requests;

static int can_send(const uint8_t b[2]) {
  uint8_t allow[2];
  if (!b || !can_whitelist_bytes(b[0], allow) || allow[0] != b[0] || allow[1] != b[1]) {
    motion_refused++;
    return 0;
  }
  can_requests++;
  pending_fn = b[0];
  reply_pending = 1;
  tx_done = 0;
  reply_at = now_ms + 5u;
  tx_done_at = now_ms + 1u;
  return 1;
}
static int can_stop(void) { return 1; }
static int can_complete(void) { return tx_done; }
static CanIo io = { can_send, can_stop, can_complete };

static const uint8_t *reply_for(uint8_t fn, uint32_t *len) {
  static const uint8_t version[6] = {0x1F, 0x00, 0xCA, 0x23, 0x14, 0x6B};
  static const uint8_t voltage[4] = {0x24, 0x5F, 0x00, 0x6B};
  static const uint8_t state[3] = {0x3A, 0x03, 0x6B};
  if (fn == 0x1F) { *len = 6; return version; }
  if (fn == 0x24) { *len = 4; return voltage; }
  if (fn == 0x3A) { *len = 3; return state; }
  *len = 0;
  return 0;
}

static int read_exact(void *dst, unsigned n) {
  unsigned char *d = dst;
  unsigned got = 0;
  while (got < n) {
    int r = _read(0, d + got, n - got);
    if (r <= 0) return 0;
    got += (unsigned)r;
  }
  return 1;
}
static void write_all(const void *src, unsigned n) {
  const unsigned char *s = src;
  unsigned put = 0;
  while (put < n) {
    int r = _write(1, s + put, n - put);
    if (r <= 0) exit(3);
    put += (unsigned)r;
  }
}
static void emit(const uint8_t *data, unsigned n) {
  uint8_t hdr[4];
  hdr[0] = (uint8_t)n; hdr[1] = (uint8_t)(n >> 8);
  hdr[2] = (uint8_t)(n >> 16); hdr[3] = (uint8_t)(n >> 24);
  write_all(hdr, 4);
  if (n) write_all(data, n);
}
static void put_u32(uint8_t *d, uint32_t v) {
  d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8); d[2] = (uint8_t)(v >> 16); d[3] = (uint8_t)(v >> 24);
}

int main(void) {
  /* The CRT defaults stdin/stdout to text mode, where 0x1A is treated as EOF.
     Clock and frame bytes routinely contain 0x1A, so both pipes must be binary. */
  _setmode(0, O_BINARY);
  _setmode(1, O_BINARY);
  ser_link_init(&link);
  ser_parser_init(&parser);
  for (;;) {
    uint8_t cmd;
    int r = _read(0, &cmd, 1);
    if (r <= 0) exit(0);
    if (cmd == 'Q') {
      uint8_t hdr[4];
      (void)read_exact(hdr, 4);
      exit(0);
    }
    if (cmd == 'F') {
      uint8_t hdr[4];
      if (!read_exact(hdr, 4)) exit(0);
      uint32_t n = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
      if (n > 4096u) exit(4);
      static uint8_t buf[4096];
      if (n && !read_exact(buf, n)) exit(0);
      static uint8_t out[8192];
      unsigned out_n = 0;
      for (uint32_t i = 0; i < n; i++) {
        size_t produced = 0;
        (void)ser_ingest(&parser, &link, buf[i], now_ms, &io, txbuf, sizeof txbuf, &produced);
        if (produced && out_n + produced <= sizeof out) {
          for (size_t k = 0; k < produced; k++) out[out_n++] = txbuf[k];
        }
      }
      emit(out, out_n);
      continue;
    }
    if (cmd == 'T') {
      uint8_t hdr[4];
      if (!read_exact(hdr, 4)) exit(0);
      uint32_t plen = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
      if (plen != 4u) exit(6);
      uint8_t now_bytes[4];
      if (!read_exact(now_bytes, 4)) exit(0);
      now_ms = (uint32_t)now_bytes[0] | ((uint32_t)now_bytes[1] << 8) | ((uint32_t)now_bytes[2] << 16) | ((uint32_t)now_bytes[3] << 24);
      static uint8_t out[8192];
      unsigned out_n = 0;
      if (reply_pending && now_ms >= tx_done_at) tx_done = 1;
      if (reply_pending && now_ms >= reply_at) {
        uint32_t len = 0;
        const uint8_t *reply = reply_for(pending_fn, &len);
        if (reply && len) ser_link_on_can(&link, now_ms, 0x100u, 1u, 0u, 0u, 0u, len, reply);
        reply_pending = 0;
      }
      ser_link_step(&link, now_ms, &io);
      if (ser_link_telemetry_due(&link, now_ms)) {
        size_t produced = 0;
        if (ser_link_pack_telemetry(&link, txbuf, sizeof txbuf, &produced) && produced) {
          for (size_t k = 0; k < produced && out_n < sizeof out; k++) out[out_n++] = txbuf[k];
          ser_link_telemetry_mark(&link, now_ms);
        }
      }
      emit(out, out_n);
      continue;
    }
    if (cmd == 'S') {
      uint8_t hdr[4];
      if (!read_exact(hdr, 4)) exit(0);
      uint8_t payload[8];
      put_u32(payload, (uint32_t)motion_refused);
      put_u32(payload + 4, (uint32_t)can_requests);
      emit(payload, 8);
      continue;
    }
    exit(5);
  }
}

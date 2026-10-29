#include "protocol.h"
__declspec(dllimport) int __cdecl printf(const char *fmt, ...);
static int g_passes, g_fails, g_reports;
#define EXPECT(c) do { if (c) g_passes++; else { g_fails++; if (g_reports < 16) { printf("FAIL %u %s\n", (unsigned)__LINE__, #c); g_reports++; } } } while (0)

static unsigned sends, stops;
static int send_ok = 1, stop_ok = 1, done = 0;
static uint8_t sent0, sent1;

static uint8_t test_crc8(const uint8_t *data, size_t len) {
  uint8_t crc = SER_CRC8_INIT;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) crc = (uint8_t)((crc & 0x80u) ? (uint8_t)((crc << 1) ^ SER_CRC8_POLY) : (uint8_t)(crc << 1));
  }
  return crc;
}
static void zero(void *p, size_t n) { unsigned char *d = p; while (n--) *d++ = 0; }
static int send_fake(const uint8_t b[2]) { sent0 = b[0]; sent1 = b[1]; sends++; return send_ok; }
static int stop_fake(void) { stops++; return stop_ok; }
static int complete_fake(void) { return done; }
static const CanIo io = { send_fake, stop_fake, complete_fake };
static void reset(SerLink *l, SerParser *p) {
  zero(l, sizeof *l); zero(p, sizeof *p); ser_link_init(l); ser_parser_init(p);
  sends = stops = 0; send_ok = stop_ok = 1; done = 0; sent0 = sent1 = 0;
}
static size_t put_frame(uint8_t *dst, size_t used, uint8_t type, const uint8_t *payload, uint8_t plen) {
  dst[used] = SER_SOF; dst[used + 1] = plen; dst[used + 2] = type;
  for (uint8_t i = 0; i < plen; i++) dst[used + 3u + i] = payload[i];
  dst[used + 3u + plen] = test_crc8(dst + used, 3u + plen);
  return used + 4u + plen;
}
static size_t feed(SerParser *p, SerLink *l, const uint8_t *in, size_t n, uint32_t now, uint8_t *out, size_t cap) {
  size_t used = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t tmp[96]; size_t tn = 0;
    ser_ingest(p, l, in[i], now, &io, tmp, sizeof tmp, &tn);
    if (tn && used + tn <= cap) { for (size_t k = 0; k < tn; k++) out[used++] = tmp[k]; }
  }
  return used;
}
static unsigned collect(const uint8_t *buf, size_t n, SerFrame *frames, unsigned max) {
  SerParser p; ser_parser_init(&p); unsigned count = 0;
  for (size_t i = 0; i < n; i++) {
    SerFrame f; uint8_t err = 0; zero(&f, sizeof f);
    if (ser_parse_byte(&p, buf[i], &f, &err) == 1 && count < max) frames[count++] = f;
  }
  return count;
}
static int parse_one(const uint8_t *buf, size_t n, SerFrame *out, uint8_t *err) {
  SerParser p; ser_parser_init(&p); int saw = 0; *err = 0; zero(out, sizeof *out);
  for (size_t i = 0; i < n; i++) {
    SerFrame f; uint8_t e = 0; int r = ser_parse_byte(&p, buf[i], &f, &e);
    if (r < 0) { *err = e; return -1; }
    if (r == 1) { *out = f; saw++; }
  }
  return saw;
}

int main(void) {
  _Static_assert(SER_CRC8_POLY == 0x07, "crc poly");
  _Static_assert(SER_TELEM_PAYLOAD == 42, "telemetry size");
  static const uint8_t digits[] = {'1','2','3','4','5','6','7','8','9'};
  EXPECT(test_crc8(digits, 9) == 0xF4);
  EXPECT(ser_crc8(digits, 9) == 0xF4);
  EXPECT(ser_crc8(0, 0) == 0);

  static const uint8_t ping[] = {0xA5, 0x00, 0x01, 0x8F};
  SerFrame frame; uint8_t err = 0;
  EXPECT(parse_one(ping, sizeof ping, &frame, &err) == 1);
  EXPECT(frame.type == SER_PING && frame.len == 0 && err == 0);
  uint8_t encoded[16]; size_t elen = 0;
  EXPECT(ser_encode(SER_PING, 0, 0, encoded, sizeof encoded, &elen));
  EXPECT(elen == 4 && encoded[0] == 0xA5 && encoded[1] == 0x00 && encoded[2] == 0x01 && encoded[3] == 0x8F);

  uint8_t big[49]; for (int i = 0; i < 49; i++) big[i] = (uint8_t)i;
  EXPECT(!ser_encode(SER_QUERY, big, 49, encoded, sizeof encoded, &elen));
  uint8_t wide[80]; size_t wlen = 0;
  EXPECT(ser_encode(SER_TELEMETRY, big, 48, wide, sizeof wide, &wlen));
  EXPECT(parse_one(wide, wlen, &frame, &err) == 1 && frame.len == 48 && frame.payload[47] == 47);

  SerParser parser; uint8_t e = 0; SerFrame got; zero(&got, sizeof got);
  ser_parser_init(&parser);
  EXPECT(ser_parse_byte(&parser, 0xA5, &got, &e) == 0);
  EXPECT(ser_parse_byte(&parser, 49, &got, &e) < 0 && e == SER_ERR_BAD_LEN);
  static const uint8_t query_a5[] = {0xA5, 0x01, 0x02, 0xA5, 0x82};
  EXPECT(parse_one(query_a5, sizeof query_a5, &frame, &err) == 1);
  EXPECT(frame.type == SER_QUERY && frame.len == 1 && frame.payload[0] == 0xA5);  uint8_t stream[320]; size_t n = 0;
  stream[n++] = 0x00; stream[n++] = 0x11; stream[n++] = 0xFF;
  for (size_t i = 0; i < sizeof ping; i++) stream[n++] = ping[i];
  stream[n++] = 0xA5; stream[n++] = 0x00; stream[n++] = 0x01; stream[n++] = 0x00;
  for (size_t i = 0; i < sizeof ping; i++) stream[n++] = ping[i];
  stream[n++] = 0xA5; stream[n++] = 0xA5; stream[n++] = 0x00; stream[n++] = 0x01; stream[n++] = 0x8F;
  SerLink link; reset(&link, &parser);
  uint8_t rx[512]; size_t rn = feed(&parser, &link, stream, n, 0, rx, sizeof rx);
  SerFrame rs[8]; unsigned rc = collect(rx, rn, rs, 8);
  EXPECT(rc == 5);
  EXPECT(rc > 0 && rs[0].type == SER_PONG && rs[0].len == 0);
  EXPECT(rc > 1 && rs[1].type == SER_ERROR && rs[1].len == 2 && rs[1].payload[1] == SER_ERR_BAD_CRC);
  EXPECT(rc > 2 && rs[2].type == SER_PONG);
  EXPECT(rc > 3 && rs[3].type == SER_ERROR && rs[3].payload[1] == SER_ERR_BAD_LEN);
  EXPECT(rc > 4 && rs[4].type == SER_PONG && rs[4].len == 0);
  EXPECT(sends == 0);

  uint8_t qpay = 0;
  n = 0; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  qpay = 1; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  qpay = 2; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  qpay = 3; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  uint8_t evilpay[] = {0xF3, 0xAB, 0x01, 0x00, 0x6B};
  n = put_frame(stream, n, SER_QUERY, evilpay, 5);
  uint8_t rate = 0; n = put_frame(stream, n, SER_TELEMETRY_RATE, &rate, 1);
  rate = 101; n = put_frame(stream, n, SER_TELEMETRY_RATE, &rate, 1);
  rate = 255; n = put_frame(stream, n, SER_TELEMETRY_RATE, &rate, 1);
  rate = 1; n = put_frame(stream, n, SER_TELEMETRY_RATE, &rate, 1);
  rate = 100; n = put_frame(stream, n, SER_TELEMETRY_RATE, &rate, 1);
  n = put_frame(stream, n, SER_STOP, 0, 0);
  uint8_t extra = 0x00; n = put_frame(stream, n, SER_PING, &extra, 1);
  n = put_frame(stream, n, SER_STOP, &extra, 1);
  n = put_frame(stream, n, 0x00, 0, 0);
  n = put_frame(stream, n, 0x05, 0, 0);
  n = put_frame(stream, n, 0x20, 0, 0);
  n = put_frame(stream, n, 0x33, 0, 0);
  n = put_frame(stream, n, 0x35, 0, 0);
  n = put_frame(stream, n, 0x36, 0, 0);
  static const uint8_t motion[] = {0x0A, 0x0E, 0x46, 0x4C, 0x93, 0x9A, 0x9C, 0xF3, 0xF6, 0xFD, 0xFE, 0xFF};
  for (unsigned i = 0; i < sizeof motion; i++) n = put_frame(stream, n, motion[i], &extra, 1);
  reset(&link, &parser);
  rn = feed(&parser, &link, stream, n, 1000, rx, sizeof rx);
  SerFrame all[40]; unsigned ac = collect(rx, rn, all, 40);
  EXPECT(sends == 1 && stops == 1);
  EXPECT(sent0 == SER_CAN_FN_VERSION && sent1 == SER_CAN_CHECKSUM);
  EXPECT(link.tx_enqueued == 1 && link.last_fn == SER_CAN_FN_VERSION && link.can_state == SER_CAN_IDLE);
  EXPECT(link.rate_hz == 0);
  EXPECT(ac == 31);
  EXPECT(all[0].type == SER_ACK && all[0].payload[0] == SER_QUERY && all[0].payload[1] == 0);
  EXPECT(all[1].type == SER_ERROR && all[1].payload[1] == SER_ERR_BUSY);
  EXPECT(all[2].type == SER_ERROR && all[2].payload[1] == SER_ERR_BUSY);
  EXPECT(all[3].type == SER_ERROR && all[3].payload[0] == SER_QUERY && all[3].payload[1] == SER_ERR_BAD_PARAM);
  EXPECT(all[4].type == SER_ERROR && all[4].payload[0] == SER_QUERY && all[4].payload[1] == SER_ERR_BAD_PARAM);
  EXPECT(all[5].type == SER_ERROR && all[5].payload[0] == SER_TELEMETRY_RATE && all[5].payload[1] == SER_ERR_BAD_PARAM);
  EXPECT(all[6].type == SER_ERROR && all[6].payload[1] == SER_ERR_BAD_PARAM);
  EXPECT(all[7].type == SER_ERROR && all[7].payload[1] == SER_ERR_BAD_PARAM);
  EXPECT(all[8].type == SER_ACK && all[8].payload[0] == SER_TELEMETRY_RATE && all[8].payload[1] == 1);
  EXPECT(all[9].type == SER_ACK && all[9].payload[0] == SER_TELEMETRY_RATE && all[9].payload[1] == 100);
  EXPECT(all[10].type == SER_ACK && all[10].payload[0] == SER_STOP && all[10].payload[1] == 0);
  EXPECT(all[11].type == SER_ERROR && all[11].payload[0] == SER_PING && all[11].payload[1] == SER_ERR_BAD_PARAM);
  EXPECT(all[12].type == SER_ERROR && all[12].payload[0] == SER_STOP && all[12].payload[1] == SER_ERR_BAD_PARAM);
  EXPECT(all[13].type == SER_ERROR && all[13].payload[0] == 0x00 && all[13].payload[1] == SER_ERR_UNKNOWN);
  EXPECT(all[18].type == SER_ERROR && all[18].payload[0] == 0x36 && all[18].payload[1] == SER_ERR_UNKNOWN);
  EXPECT(all[19].type == SER_ERROR && all[19].payload[0] == 0x0A && all[19].payload[1] == SER_ERR_MOTION);
  EXPECT(all[30].type == SER_ERROR && all[30].payload[0] == 0xFF && all[30].payload[1] == SER_ERR_MOTION);
  uint8_t b[2] = {0xFF, 0xFF};
  static const uint8_t banned[] = {0x0A,0x0E,0x20,0x21,0x27,0x31,0x33,0x35,0x36,0x37,0x3B,0x42,0x43,0x46,0x4C,0x93,0x9A,0x9C,0xF3,0xF6,0xFD,0xFE,0xFF};
  for (unsigned i = 0; i < sizeof banned; i++) EXPECT(!can_whitelist_bytes(banned[i], b));
  EXPECT(can_whitelist_bytes(SER_CAN_FN_VERSION, b) && b[0] == 0x1F && b[1] == 0x6B);
  EXPECT(can_whitelist_bytes(SER_CAN_FN_VOLTAGE, b) && b[0] == 0x24 && b[1] == 0x6B);
  EXPECT(can_whitelist_bytes(SER_CAN_FN_STATE, b) && b[0] == 0x3A && b[1] == 0x6B);

  reset(&link, &parser);
  EXPECT(link.rate_hz == 10);
  EXPECT(ser_link_telemetry_due(&link, 0) == 1);
  ser_link_telemetry_mark(&link, 0);
  EXPECT(ser_link_telemetry_due(&link, 99) == 0);
  EXPECT(ser_link_telemetry_due(&link, 100) == 1);
  n = 0; rate = 100; n = put_frame(stream, n, SER_TELEMETRY_RATE, &rate, 1);
  feed(&parser, &link, stream, n, 1000, rx, sizeof rx);
  EXPECT(link.rate_hz == 100 && sends == 0);
  ser_link_telemetry_mark(&link, 1000);
  EXPECT(ser_link_telemetry_due(&link, 1009) == 0);
  EXPECT(ser_link_telemetry_due(&link, 1010) == 1);
  n = 0; rate = 1; n = put_frame(stream, n, SER_TELEMETRY_RATE, &rate, 1);
  reset(&link, &parser);
  feed(&parser, &link, stream, n, 50, rx, sizeof rx);
  ser_link_telemetry_mark(&link, 50);
  EXPECT(link.rate_hz == 1);
  EXPECT(ser_link_telemetry_due(&link, 1049) == 0);
  EXPECT(ser_link_telemetry_due(&link, 1050) == 1);
  n = 0; n = put_frame(stream, n, SER_STOP, 0, 0);
  feed(&parser, &link, stream, n, 1050, rx, sizeof rx);
  EXPECT(link.rate_hz == 0 && ser_link_telemetry_due(&link, 99999) == 0);

  reset(&link, &parser);
  for (unsigned i = 0; i < 1000; i++) ser_link_step(&link, i, &io);
  EXPECT(sends == 0 && link.heartbeat == 1000);

  reset(&link, &parser);
  qpay = 0; n = 0; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  feed(&parser, &link, stream, n, 1000, rx, sizeof rx);
  EXPECT(sends == 1 && sent0 == 0x1F && sent1 == 0x6B && link.can_state == SER_CAN_WAIT);
  qpay = 1; n = 0; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  rn = feed(&parser, &link, stream, n, 1001, rx, sizeof rx);
  rc = collect(rx, rn, rs, 4);
  EXPECT(sends == 1 && rc == 1 && rs[0].type == SER_ERROR && rs[0].payload[1] == SER_ERR_BUSY);
  ser_link_step(&link, 1199, &io);
  EXPECT(link.timeouts == 0 && link.can_state == SER_CAN_WAIT);
  ser_link_step(&link, 1200, &io);
  EXPECT(link.timeouts == 1 && link.error == SER_ERR_TIMEOUT && link.can_state == SER_CAN_IDLE && stops == 1);

  reset(&link, &parser);
  qpay = 1; n = 0; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  feed(&parser, &link, stream, n, 10, rx, sizeof rx);
  uint8_t reply[6]; reply[0] = 0x24; reply[1] = 0x12; reply[2] = 0x34; reply[3] = 0x6B;
  ser_link_on_can(&link, 20, SER_CAN_ID, 1, 0, 0, 0, 4, reply);
  EXPECT(link.matched == 1 && link.can_state == SER_CAN_MATCHED && link.last_fn == 0x24 && link.last_raw_len == 4);
  EXPECT(link.last_raw[0] == 0x24 && link.last_raw[3] == 0x6B && link.rx_count == 1);
  done = 1; ser_link_step(&link, 20, &io); ser_link_step(&link, 21, &io);
  EXPECT(link.tx_complete == 1 && link.can_state == SER_CAN_IDLE && stops == 1);  reset(&link, &parser);
  qpay = 0; n = 0; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  feed(&parser, &link, stream, n, 100, rx, sizeof rx);
  uint8_t ver6[6] = {0x1F, 1, 2, 3, 4, 0x6B};
  ser_link_on_can(&link, 110, SER_CAN_ID, 1, 0, 0, 0, 6, ver6);
  EXPECT(link.matched == 1 && link.last_raw_len == 6 && link.last_raw[5] == 0x6B);
  uint8_t bad[8] = {0x1F, 0, 0, 0, 0, 0, 0, 0x6B};
  ser_link_on_can(&link, 111, 0x200, 1, 0, 0, 0, 4, bad);
  ser_link_on_can(&link, 111, 0x101, 1, 0, 0, 0, 4, bad);
  ser_link_on_can(&link, 111, SER_CAN_ID, 0, 0, 0, 0, 4, bad);
  ser_link_on_can(&link, 111, SER_CAN_ID, 1, 1, 0, 0, 4, bad);
  ser_link_on_can(&link, 111, SER_CAN_ID, 1, 0, 1, 0, 4, bad);
  ser_link_on_can(&link, 111, SER_CAN_ID, 1, 0, 0, 1, 4, bad);
  ser_link_on_can(&link, 111, SER_CAN_ID, 1, 0, 0, 0, 3, bad);
  ser_link_on_can(&link, 111, SER_CAN_ID, 1, 0, 0, 0, 8, bad);
  bad[3] = 0x00; ser_link_on_can(&link, 111, SER_CAN_ID, 1, 0, 0, 0, 4, bad);
  bad[0] = 0x24; bad[3] = 0x6B; ser_link_on_can(&link, 111, SER_CAN_ID, 1, 0, 0, 0, 4, bad);
  EXPECT(link.matched == 1 && link.rx_count == 11);

  reset(&link, &parser);
  qpay = 2; n = 0; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  feed(&parser, &link, stream, n, 0, rx, sizeof rx);
  uint8_t st[3] = {0x3A, 0x01, 0x6B};
  ser_link_on_can(&link, 1, SER_CAN_ID, 1, 0, 0, 0, 3, st);
  EXPECT(link.matched == 1 && link.last_raw_len == 3);
  uint8_t stbad[4] = {0x3A, 0, 0, 0x6B};
  reset(&link, &parser);
  feed(&parser, &link, stream, n, 0, rx, sizeof rx);
  ser_link_on_can(&link, 1, SER_CAN_ID, 1, 0, 0, 0, 4, stbad);
  EXPECT(link.matched == 0 && link.rx_count == 1);

  reset(&link, &parser);
  send_ok = 0; qpay = 0; n = 0; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  rn = feed(&parser, &link, stream, n, 5, rx, sizeof rx);
  rc = collect(rx, rn, rs, 2);
  EXPECT(sends == 1 && stops == 1 && link.can_state == SER_CAN_FAULT && link.error == SER_ERR_ENQUEUE);
  EXPECT(rc == 1 && rs[0].type == SER_ERROR && rs[0].payload[1] == SER_ERR_ENQUEUE);
  send_ok = 1; rn = feed(&parser, &link, stream, n, 6, rx, sizeof rx);
  EXPECT(sends == 1 && link.can_state == SER_CAN_FAULT);

  reset(&link, &parser);
  stop_ok = 0; qpay = 0; n = 0; n = put_frame(stream, n, SER_QUERY, &qpay, 1);
  feed(&parser, &link, stream, n, 0xfffffff0u, rx, sizeof rx);
  ser_link_step(&link, 0xfffffff0u + 199u, &io);
  EXPECT(link.timeouts == 0);
  ser_link_step(&link, 0xfffffff0u + 200u, &io);
  EXPECT(link.timeouts == 1 && link.error == SER_ERR_TRANSPORT && link.can_state == SER_CAN_FAULT && stops == 1);

  reset(&link, &parser);
  link.heartbeat = 0x11223344u; link.init_stage = 4; link.tx_enqueued = 3; link.tx_complete = 2;
  link.rx_count = 9; link.matched = 2; link.timeouts = 1; link.error = 8;
  link.last_fn = 0x1F; link.last_raw_len = 4;
  link.last_raw[0] = 0x1F; link.last_raw[1] = 0xA5; link.last_raw[2] = 0x00; link.last_raw[3] = 0x6B;
  uint8_t telem[80]; size_t tn = 0;
  EXPECT(ser_link_pack_telemetry(&link, telem, sizeof telem, &tn));
  EXPECT(parse_one(telem, tn, &frame, &err) == 1 && frame.type == SER_TELEMETRY && frame.len == SER_TELEM_PAYLOAD);
  SerTelem view; zero(&view, sizeof view);
  EXPECT(ser_telem_unpack(frame.payload, frame.len, &view));
  EXPECT(view.heartbeat == 0x11223344u && view.init_stage == 4 && view.tx_enqueued == 3 && view.tx_complete == 2);
  EXPECT(view.rx_count == 9 && view.matched == 2 && view.timeouts == 1 && view.error == 8);
  EXPECT(view.last_fn == 0x1F && view.last_raw_len == 4 && view.last_raw[1] == 0xA5 && view.last_raw[3] == 0x6B);
  EXPECT(!ser_telem_unpack(frame.payload, 41, &view));

  printf("HOST_TESTS pass=%u fail=%u\n", (unsigned)g_passes, (unsigned)g_fails);
  return g_fails ? 1 : 0;
}

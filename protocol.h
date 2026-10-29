#ifndef SER_PROTOCOL_H
#define SER_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

#define SER_SOF 0xA5u
#define SER_MAX_PAYLOAD 48u
#define SER_TELEM_PAYLOAD 42u
#define SER_CRC8_POLY 0x07u
#define SER_CRC8_INIT 0x00u

enum {
  SER_PING = 0x01,
  SER_QUERY = 0x02,
  SER_TELEMETRY_RATE = 0x03,
  SER_STOP = 0x04,
  SER_TELEMETRY = 0x10,
  SER_PONG = 0x11,
  SER_ACK = 0x12,
  SER_ERROR = 0x7F
};

enum {
  SER_QUERY_VERSION = 0,
  SER_QUERY_VOLTAGE = 1,
  SER_QUERY_STATE = 2
};

enum {
  SER_OK = 0,
  SER_ERR_BAD_CRC = 1,
  SER_ERR_BAD_LEN = 2,
  SER_ERR_UNKNOWN = 3,
  SER_ERR_BAD_PARAM = 4,
  SER_ERR_MOTION = 5,
  SER_ERR_BUSY = 6,
  SER_ERR_STATE = 7,
  SER_ERR_TIMEOUT = 8,
  SER_ERR_TRANSPORT = 90,
  SER_ERR_ENQUEUE = 91
};

enum {
  SER_CAN_FN_VERSION = 0x1F,
  SER_CAN_FN_VOLTAGE = 0x24,
  SER_CAN_FN_STATE = 0x3A,
  SER_CAN_CHECKSUM = 0x6B,
  SER_CAN_ID = 0x100
};

enum {
  SER_CAN_IDLE = 0,
  SER_CAN_WAIT = 1,
  SER_CAN_FAULT = 4,
  SER_CAN_MATCHED = 5
};

typedef struct {
  uint8_t type;
  uint8_t len;
  uint8_t payload[SER_MAX_PAYLOAD];
} SerFrame;

typedef struct {
  uint8_t state;
  uint8_t len;
  uint8_t type;
  uint8_t got;
  uint8_t body[3u + SER_MAX_PAYLOAD];
  uint8_t body_n;
} SerParser;

typedef struct {
  uint32_t heartbeat;
  uint32_t init_stage;
  uint32_t tx_enqueued;
  uint32_t tx_complete;
  uint32_t rx_count;
  uint32_t matched;
  uint32_t timeouts;
  uint32_t error;
  uint8_t last_fn;
  uint8_t last_raw_len;
  uint8_t last_raw[8];
  uint8_t rate_hz;
  uint8_t telem_sent;
  uint8_t can_state;
  uint8_t pending_fn;
  uint8_t tx_seen;
  uint32_t can_start;
  uint32_t last_telem;
} SerLink;

typedef struct {
  int (*send)(const uint8_t bytes[2]);
  int (*stop)(void);
  int (*complete)(void);
} CanIo;

typedef struct {
  uint32_t heartbeat;
  uint32_t init_stage;
  uint32_t tx_enqueued;
  uint32_t tx_complete;
  uint32_t rx_count;
  uint32_t matched;
  uint32_t timeouts;
  uint32_t error;
  uint8_t last_fn;
  uint8_t last_raw_len;
  uint8_t last_raw[8];
} SerTelem;

uint8_t ser_crc8(const uint8_t *data, size_t len);
int ser_encode(uint8_t type, const uint8_t *payload, uint8_t payload_len, uint8_t *out, size_t cap, size_t *out_len);
void ser_parser_init(SerParser *p);
int ser_parse_byte(SerParser *p, uint8_t byte, SerFrame *out, uint8_t *err);
void ser_link_init(SerLink *l);
int ser_link_command(SerLink *l, const SerFrame *frame, uint32_t now, const CanIo *io, uint8_t *tx, size_t cap, size_t *tx_len);
int ser_ingest(SerParser *p, SerLink *l, uint8_t byte, uint32_t now, const CanIo *io, uint8_t *tx, size_t cap, size_t *tx_len);
void ser_link_step(SerLink *l, uint32_t now, const CanIo *io);
void ser_link_on_can(SerLink *l, uint32_t now, uint32_t id, uint32_t extended, uint32_t remote, uint32_t fd, uint32_t brs, uint32_t dlc, const uint8_t *data);
int ser_link_telemetry_due(const SerLink *l, uint32_t now);
void ser_link_telemetry_mark(SerLink *l, uint32_t now);
int ser_link_pack_telemetry(const SerLink *l, uint8_t *out, size_t cap, size_t *out_len);
int ser_telem_unpack(const uint8_t *payload, uint8_t len, SerTelem *out);
int can_whitelist_bytes(uint8_t fn, uint8_t out[2]);
#endif

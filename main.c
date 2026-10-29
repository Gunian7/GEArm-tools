#include "stm32h7xx_hal.h"
#include "protocol.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

static FDCAN_HandleTypeDef can;
static UART_HandleTypeDef uart;
static uint32_t txmask;
static uint8_t rx_byte;
static volatile uint8_t rx_ring[256];
static volatile uint32_t rx_head, rx_tail, rx_drop;
SerLink link;

/* USB CDC protocol channel: an independent parser+link instance from the UART
   one, so a command on one channel cannot corrupt the other's CAN state
   machine. USB is the primary control link; UART stays as a fallback.
   CAN responses are routed to whichever link is waiting (see drain()). */
static SerLink link_usb;
static SerParser parser_usb;
static volatile uint8_t usb_rx_ring[512];
static volatile uint32_t usb_rx_head, usb_rx_tail, usb_rx_drop;
volatile uint32_t g_usb_rx_bytes;

static void fail(uint32_t e) {
  __HAL_RCC_FDCAN_FORCE_RESET();
  link.error = e;
  link.can_state = SER_CAN_FAULT;
  for (;;) link.heartbeat++;
}
static void check(HAL_StatusTypeDef s, uint32_t e) { if (s != HAL_OK) fail(e); }

/* Referenced by the ST USB device stack (usb_device.c / usbd_conf.c). */
void Error_Handler(void) { fail(200); }

void SysTick_Handler(void) { HAL_IncTick(); }
/* The startup file only provides a weak default (infinite loop) for this
   vector. Without a real handler, the first USB interrupt after USBD_Start()
   parks the CPU there and the main loop never runs. */
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
void OTG_HS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS); }
void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&can); }
void FDCAN1_IT1_IRQHandler(void) { HAL_FDCAN_IRQHandler(&can); }
void HardFault_Handler(void) { fail(100); }
void MemManage_Handler(void) { fail(101); }
void BusFault_Handler(void) { fail(102); }
void UsageFault_Handler(void) { fail(103); }

static void clock_init(void) {
  RCC_OscInitTypeDef o = {0};
  RCC_ClkInitTypeDef c = {0};
  RCC_PeriphCLKInitTypeDef p = {0};
  check(HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY), 10);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  uint32_t start = HAL_GetTick();
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) { if (HAL_GetTick() - start > 200) fail(11); }
  o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  o.HSEState = RCC_HSE_ON;
  o.PLL.PLLState = RCC_PLL_ON;
  o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  o.PLL.PLLM = 3;
  o.PLL.PLLN = 50;
  o.PLL.PLLP = 2;
  o.PLL.PLLQ = 4;
  o.PLL.PLLR = 2;
  o.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  o.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  o.PLL.PLLFRACN = 0;
  check(HAL_RCC_OscConfig(&o), 12);
  c.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1;
  c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  c.SYSCLKDivider = RCC_SYSCLK_DIV1;
  c.AHBCLKDivider = RCC_HCLK_DIV2;
  c.APB3CLKDivider = RCC_APB3_DIV2;
  c.APB1CLKDivider = RCC_APB1_DIV2;
  c.APB2CLKDivider = RCC_APB2_DIV2;
  c.APB4CLKDivider = RCC_APB4_DIV2;
  check(HAL_RCC_ClockConfig(&c, FLASH_LATENCY_4), 13);
  p.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  p.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  check(HAL_RCCEx_PeriphCLKConfig(&p), 14);
  if (HAL_RCC_GetSysClockFreq() != 200000000u || HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) != 100000000u) fail(15);
  RCC_PeriphCLKInitTypeDef u = {0};
  u.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  u.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  check(HAL_RCCEx_PeriphCLKConfig(&u), 16);
  if (HAL_RCC_GetPCLK2Freq() != 50000000u) fail(17);
  if (__HAL_RCC_GET_USART16_SOURCE() != RCC_USART16CLKSOURCE_D2PCLK2) fail(18);
  /* HSI48 must be running before HAL_RCCEx_PeriphCLKConfig(RCC_USBCLKSOURCE_HSI48)
     is called from HAL_PCD_MspInit, otherwise that call fails and lands in
     Error_Handler before the main loop ever starts. */
  RCC_OscInitTypeDef o48 = {0};
  o48.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  o48.HSI48State = RCC_HSI48_ON;
  o48.PLL.PLLState = RCC_PLL_NONE;
  check(HAL_RCC_OscConfig(&o48), 19);
}

static void can_init(uint32_t mode) {
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_FDCAN_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOD, &g);
  can.Instance = FDCAN1;
  can.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  can.Init.Mode = mode;
  can.Init.AutoRetransmission = DISABLE;
  can.Init.TransmitPause = DISABLE;
  can.Init.ProtocolException = DISABLE;
  can.Init.NominalPrescaler = 10;
  can.Init.NominalSyncJumpWidth = 4;
  can.Init.NominalTimeSeg1 = 15;
  can.Init.NominalTimeSeg2 = 4;
  can.Init.DataPrescaler = 10;
  can.Init.DataSyncJumpWidth = 4;
  can.Init.DataTimeSeg1 = 15;
  can.Init.DataTimeSeg2 = 4;
  can.Init.MessageRAMOffset = 0;
  can.Init.StdFiltersNbr = 0;
  can.Init.ExtFiltersNbr = 1;
  can.Init.RxFifo0ElmtsNbr = 16;
  can.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  can.Init.TxFifoQueueElmtsNbr = 1;
  can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  can.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  check(HAL_FDCAN_Init(&can), 20);
  FDCAN_FilterTypeDef f = {0};
  f.IdType = FDCAN_EXTENDED_ID;
  f.FilterIndex = 0;
  f.FilterType = FDCAN_FILTER_MASK;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1 = 0x100;
  f.FilterID2 = 0x1fffffff;
  check(HAL_FDCAN_ConfigFilter(&can, &f), 21);
  check(HAL_FDCAN_ConfigGlobalFilter(&can, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE), 22);
  check(HAL_FDCAN_Start(&can), 23);
}

static void drain(void) {
  uint32_t budget = 64;
  while (HAL_FDCAN_GetRxFifoFillLevel(&can, FDCAN_RX_FIFO0)) {
    if (!budget--) { link.error = 31; link.can_state = SER_CAN_FAULT; return; }
    FDCAN_RxHeaderTypeDef h = {0};
    uint8_t raw[64] = {0};
    if (HAL_FDCAN_GetRxMessage(&can, FDCAN_RX_FIFO0, &h, raw) != HAL_OK) {
      link.error = 30;
      link.can_state = SER_CAN_FAULT;
      return;
    }
    /* Route the CAN response to whichever channel is waiting for it, so a
       QUERY issued over USB still gets its reply (the CAN bus is shared). */
    if (link.can_state == SER_CAN_WAIT || link.can_state == SER_CAN_MATCHED) {
      ser_link_on_can(&link, HAL_GetTick(), h.Identifier, h.IdType == FDCAN_EXTENDED_ID,
                      h.RxFrameType == FDCAN_REMOTE_FRAME, h.FDFormat == FDCAN_FD_CAN,
                      h.BitRateSwitch == FDCAN_BRS_ON, h.DataLength, raw);
    } else {
      ser_link_on_can(&link_usb, HAL_GetTick(), h.Identifier, h.IdType == FDCAN_EXTENDED_ID,
                      h.RxFrameType == FDCAN_REMOTE_FRAME, h.FDFormat == FDCAN_FD_CAN,
                      h.BitRateSwitch == FDCAN_BRS_ON, h.DataLength, raw);
    }
  }
  if (can.Instance->IR & FDCAN_IR_RF0L) can.Instance->IR = FDCAN_IR_RF0L;
}

static int stop_capture(void) {
  if (HAL_FDCAN_Stop(&can) != HAL_OK) return 0;
  SET_BIT(can.Instance->CCCR, FDCAN_CCCR_MON);
  if (HAL_FDCAN_Start(&can) != HAL_OK) return 0;
  drain();
  if (HAL_FDCAN_Stop(&can) != HAL_OK) return 0;
  return link.can_state != SER_CAN_FAULT;
}

static int send_can(const uint8_t b[2]) {
  uint8_t allow[2];
  if (!b || !can_whitelist_bytes(b[0], allow) || allow[0] != b[0] || allow[1] != b[1]) return 0;
  if (!stop_capture() || HAL_FDCAN_DeInit(&can) != HAL_OK) return 0;
  __HAL_RCC_FDCAN_FORCE_RESET();
  __HAL_RCC_FDCAN_RELEASE_RESET();
  can_init(FDCAN_MODE_NORMAL);
  FDCAN_TxHeaderTypeDef h = {0};
  h.Identifier = SER_CAN_ID;
  h.IdType = FDCAN_EXTENDED_ID;
  h.TxFrameType = FDCAN_DATA_FRAME;
  h.DataLength = FDCAN_DLC_BYTES_2;
  h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  h.BitRateSwitch = FDCAN_BRS_OFF;
  h.FDFormat = FDCAN_CLASSIC_CAN;
  h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&can, &h, allow) != HAL_OK) return 0;
  txmask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&can);
  return 1;
}

static int can_done(void) { return txmask && (can.Instance->TXBTO & txmask) != 0; }

static int stop_can(void) {
  if (txmask && HAL_FDCAN_IsTxBufferMessagePending(&can, txmask)) {
    if (HAL_FDCAN_AbortTxRequest(&can, txmask) != HAL_OK) return 0;
  }
  if (!stop_capture()) return 0;
  if (HAL_FDCAN_DeInit(&can) != HAL_OK) return 0;
  __HAL_RCC_FDCAN_FORCE_RESET();
  __HAL_RCC_FDCAN_RELEASE_RESET();
  txmask = 0;
  can_init(FDCAN_MODE_BUS_MONITORING);
  return 1;
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1) return;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &g);
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&uart); }

volatile uint32_t g_uart_rx;   /* total bytes received on USART1 (every ISR byte) */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1) return;
  g_uart_rx++;
  uint32_t next = (rx_head + 1u) & 255u;
  if (next != rx_tail) {
    rx_ring[rx_head] = rx_byte;
    rx_head = next;
  } else rx_drop++;
  if (HAL_UART_Receive_IT(huart, &rx_byte, 1) != HAL_OK) rx_drop++;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1) return;
  rx_drop++;
  (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

static int rx_pop(uint8_t *b) {
  uint32_t tail = rx_tail;
  if (tail == rx_head) return 0;
  *b = rx_ring[tail];
  rx_tail = (tail + 1u) & 255u;
  return 1;
}

/* --- USB CDC RX ring -> protocol (see declarations at top of file) -------- */

/* Called from the USB interrupt context (CDC_Receive_HS). */
void usb_cdc_push(const uint8_t *data, uint32_t n) {
  if (!data) return;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t next = (usb_rx_head + 1u) & 511u;
    if (next != usb_rx_tail) { usb_rx_ring[usb_rx_head] = data[i]; usb_rx_head = next; }
    else usb_rx_drop++;
  }
  g_usb_rx_bytes += n;
}

static int usb_pop(uint8_t *b) {
  uint32_t tail = usb_rx_tail;
  if (tail == usb_rx_head) return 0;
  *b = usb_rx_ring[tail];
  usb_rx_tail = (tail + 1u) & 511u;
  return 1;
}

static int usb_write(const uint8_t *data, size_t n) {
  if (!data || !n || n > 512u) return 0;
  return CDC_Transmit_HS((uint8_t *)data, (uint16_t)n) == USBD_OK;
}

volatile uint32_t g_uart_tx_ok, g_uart_tx_fail;

static int uart_write(const uint8_t *data, size_t n) {
  if (!data || !n || n > 0xffffu) return 0;
  int ok = HAL_UART_Transmit(&uart, (uint8_t *)data, (uint16_t)n, 20) == HAL_OK;
  if (ok) g_uart_tx_ok++; else g_uart_tx_fail++;
  return ok;
}

static void uart_init(void) {
  uart.Instance = USART1;
  uart.Init.BaudRate = 115200;
  uart.Init.WordLength = UART_WORDLENGTH_8B;
  uart.Init.StopBits = UART_STOPBITS_1;
  uart.Init.Parity = UART_PARITY_NONE;
  uart.Init.Mode = UART_MODE_TX_RX;
  uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart.Init.OverSampling = UART_OVERSAMPLING_16;
  uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  check(HAL_UART_Init(&uart), 50);
  check(HAL_UART_Receive_IT(&uart, &rx_byte, 1), 51);
}

/* --- WS2812 on PA7 = SPI6_MOSI (AF8) -------------------------------------
   Uses SPI6 as a bit encoder, exactly as Damiao's official ws2812.c does:
   one SPI byte per WS2812 bit, 0xF0 = '1', 0xC0 = '0'. SPI6 runs at
   HSE(24MHz)/4 = 6MHz, which makes those two patterns produce the right
   T0H/T1H widths. Bit-banging produced a fixed white LED, i.e. all bits
   latched as '1', so the peripheral does the timing instead. */
static SPI_HandleTypeDef hspi6 __attribute__((unused));

static void __attribute__((unused)) ws2812_spi_init(void) {
  RCC_PeriphCLKInitTypeDef pc = {0};
  pc.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
  pc.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE;
  check(HAL_RCCEx_PeriphCLKConfig(&pc), 30);

  __HAL_RCC_SPI6_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_7;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF8_SPI6;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &g);

  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  check(HAL_SPI_Init(&hspi6), 31);
}

static void __attribute__((unused)) ws2812_send(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t txbuf[24];
  uint8_t res = 0;
  for (int i = 0; i < 8; i++) {
    txbuf[7 - i]  = (uint8_t)((((g >> i) & 1u) ? 0xF0 : 0xC0) >> 1);
    txbuf[15 - i] = (uint8_t)((((r >> i) & 1u) ? 0xF0 : 0xC0) >> 1);
    txbuf[23 - i] = (uint8_t)((((b >> i) & 1u) ? 0xF0 : 0xC0) >> 1);
  }
  (void)HAL_SPI_Transmit(&hspi6, &res, 0, 0xFFFF);
  while (hspi6.State != HAL_SPI_STATE_READY) { }
  (void)HAL_SPI_Transmit(&hspi6, txbuf, 24, 0xFFFF);
  for (int i = 0; i < 100; i++) (void)HAL_SPI_Transmit(&hspi6, &res, 1, 0xFFFF);
}

/* USB_OTG_HS device pins: PA11 = USB_OTG_HS_DM, PA12 = USB_OTG_HS_DP, AF10.
   The CubeMX .ioc declares them (Device_Only_FS) but the generated .c files
   in the official example never actually configure the GPIO, and our reset
   state left PA11/PA12 as ANALOG -> the USB PHY was not connected to the pins,
   so the device could never enumerate. Configure them explicitly here. */
static void usb_pin_init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = 0x0AU;   /* AF10 = USB_OTG_HS (DM/DP) on PA11/PA12 */
  HAL_GPIO_Init(GPIOA, &g);
}

int main(void) {
  ser_link_init(&link);
  link.init_stage = 1;
  check(HAL_Init(), 1);
  clock_init();
  link.init_stage = 2;
  can_init(FDCAN_MODE_BUS_MONITORING);
  link.init_stage = 3;
  uart_init();
  link.init_stage = 4;
  /* NOTE (2026-10-27): WS2812 over SPI6 hangs the loop. SPI6's TXP flag never
     sets, so every HAL_SPI_Transmit blocks for its full 65s timeout and
     ws2812_send does 100+ of them per call -> main loop frozen for ~14min per
     pass, which is why heartbeat=0 and USART1 emitted one line then went
     silent. The LED is cosmetic; take it OFF the critical path until the SPI6
     timing is fixed. See HANDOFF section 九. */
  /* ws2812_spi_init(); */
  /* ws2812_send(0, 40, 0); */
  usb_pin_init();
  MX_USB_DEVICE_Init();  /* USB CDC: enumerate as a virtual COM port (follow official CtrBoard-H7_USB) */
  SerParser parser;
  ser_parser_init(&parser);
  ser_link_init(&link_usb);
  /* Stage 5 = fully initialised (UART + USB CDC both up). Must be set AFTER
     ser_link_init(), which zeroes the struct -- otherwise the USB link's
     telemetry reports init_stage=0 forever. */
  link_usb.init_stage = 5;
  ser_parser_init(&parser_usb);
  const CanIo can_io = { send_can, stop_can, can_done };
  for (;;) {
    if (can.Instance->PSR & FDCAN_PSR_BO) {
      link.error = 40;
      link.can_state = SER_CAN_FAULT;
      if (!stop_can()) fail(41);
      fail(40);
    }
    if (link.can_state != SER_CAN_FAULT) drain();
    uint8_t byte = 0;
    while (rx_pop(&byte)) {
      uint8_t tx[96];
      size_t n = 0;
      ser_ingest(&parser, &link, byte, HAL_GetTick(), &can_io, tx, sizeof tx, &n);
      if (n) (void)uart_write(tx, n);
    }
    uint8_t ubyte = 0;
    while (usb_pop(&ubyte)) {
      uint8_t tx[96];
      size_t n = 0;
      ser_ingest(&parser_usb, &link_usb, ubyte, HAL_GetTick(), &can_io, tx, sizeof tx, &n);
      if (n) (void)usb_write(tx, n);
    }
    /* NOTE: no plain-text heartbeat here on purpose. Both channels now carry
       the framed protocol, and "GEArm-...-OK" strings would be parsed as
       garbage and trigger spurious error frames. Liveness is the protocol's
       own TELEMETRY frames (rate_hz defaults to 10 -> one frame every 100ms);
       an on-demand check is PING -> PONG. */
    uint32_t now = HAL_GetTick();
    ser_link_step(&link, now, &can_io);
    ser_link_step(&link_usb, now, &can_io);
    if (ser_link_telemetry_due(&link, now)) {
      uint8_t tx[96];
      size_t n = 0;
      if (ser_link_pack_telemetry(&link, tx, sizeof tx, &n) && n && uart_write(tx, n)) ser_link_telemetry_mark(&link, now);
    }
    if (ser_link_telemetry_due(&link_usb, now)) {
      uint8_t tx[96];
      size_t n = 0;
      if (ser_link_pack_telemetry(&link_usb, tx, sizeof tx, &n) && n && usb_write(tx, n)) ser_link_telemetry_mark(&link_usb, now);
    }
    (void)rx_drop;
    (void)usb_rx_drop;
  }
}
#pragma once

#include <main.hpp>
#include "stm32f1xx_hal.h"


namespace mcp2515
{
/*
 * Register masks and bit offsets
 */

#define RXBNCTRL_RXM_MASK   (0x60U)
#define RXBNCTRL_RXM        (5)
#define RXBNCTRL_BUKT_MASK  (0x4U)
#define RXBNCTRL_BUKT       (2)

#define CNF3_SOF_MASK       (0x80U)
#define CNF3_SOF            (7)
#define CNF3_WAKFIL_MASK    (0x40U)
#define CNF3_WAKFIL         (6)
#define CNF3_PHSEG2_MASK    (0x7U)
#define CNF3_PHSEG2         (0)

#define CNF2_BLTMODE_MASK   (0x80U)
#define CNF2_BLTMODE        (7)
#define CNF2_SAM_MASK       (0x40U)
#define CNF2_SAM            (6)
#define CNF2_PHSEG1_MASK    (0x38U)
#define CNF2_PHSEG1         (3)
#define CNF2_PRSEG_MASK     (0x7U)
#define CNF2_PRSEG          (0)

#define CNF1_SJW_MASK       (0xC0U)
#define CNF1_SJW            (6)
#define CNF1_BRP_MASK       (0x3FU)
#define CNF1_BRP            (0)

#define CANCTRL_REQOP_MASK  (0xE0U)
#define CANCTRL_REQOP       (5)

#define CANSTAT_OPMOD_MASK  (0xE0U)
#define CANSTAT_OPMOD       (5)
#define CANSTAT_ICOD_MASK   (0x0EU)
#define CANSTAT_ICOD        (1)

/*
 * CAN speed timings
 */

#define MCP_TIMING_SJW      (0U)
#define MCP_TIMING_SAM      (0U)
#define MCP_TIMING_BTLMODE  (1U)

#define MCP_500KBPS_BRP     (0U)
#define MCP_500KBPS_PRSEG   (1U)
#define MCP_500KBPS_PHSEG1  (1U)
#define MCP_500KBPS_PHSEG2  (2U)
#define MCP_500KBPS_SAM     (0U)

#define MCP_250KBPS_BRP     (0U)
#define MCP_250KBPS_PRSEG   (4U)
#define MCP_250KBPS_PHSEG1  (4U)
#define MCP_250KBPS_PHSEG2  (4U)
#define MCP_250KBPS_SAM     (1U)

#define MCP_125KBPS_BRP     (1U)
#define MCP_125KBPS_PRSEG   (1U)
#define MCP_125KBPS_PHSEG1  (6U)
#define MCP_125KBPS_PHSEG2  (5U)
#define MCP_125KBPS_SAM     (1U)

enum class Error {
  OK = 0U,
  ERROR,
  TIMEOUT
};

enum class Command : uint8_t {
  RESET          = 0xC0,
  READ           = 0x03,
  READ_RX_BUFFER = 0x90,
  WRITE          = 0x02,
  LOAD_TX_BUFFER = 0x40,
  RTS            = 0x80,
  READ_STATUS    = 0xA0,
  RX_STATUS      = 0xB0,
  BIT_MODIFY     = 0x05
};

enum class Speed {
  MCP_500KBPS,
  MCP_250KBPS,
  MCP_125KBPS
};

enum class Mode : uint8_t {
  NORMAL = 0U,
  SLEEP = 1U,
  LOOPBACK = 2U,
  LISTENONLY = 3U,
  CONFIGURATION = 4U,
  UNKNOWN = 7U,
};

enum class Register : uint8_t {
  RXF0SIDH = 0x00,
  RXF0SIDL = 0x01,
  RXF0EID8 = 0x02,
  RXF0EID0 = 0x03,
  RXF1SIDH = 0x04,
  RXF1SIDL = 0x05,
  RXF1EID8 = 0x06,
  RXF1EID0 = 0x07,
  RXF2SIDH = 0x08,
  RXF2SIDL = 0x09,
  RXF2EID8 = 0x0A,
  RXF2EID0 = 0x0B,
  BFPCTRL  = 0x0C,
  TXRTSCTRL= 0x0D,
  CANSTAT  = 0x0E,
  CANCTRL  = 0x0F,
  RXF3SIDH = 0x10,
  RXF3SIDL = 0x11,
  RXF3EID8 = 0x12,
  RXF3EID0 = 0x13,
  RXF4SIDH = 0x14,
  RXF4SIDL = 0x15,
  RXF4EID8 = 0x16,
  RXF4EID0 = 0x17,
  RXF5SIDH = 0x18,
  RXF5SIDL = 0x19,
  RXF5EID8 = 0x1A,
  RXF5EID0 = 0x1B,
  TEC      = 0x1C,
  REC      = 0x1D,
  RXM0SIDH = 0x20,
  RXM0SIDL = 0x21,
  RXM0EID8 = 0x22,
  RXM0EID0 = 0x23,
  RXM1SIDH = 0x24,
  RXM1SIDL = 0x25,
  RXM1EID8 = 0x26,
  RXM1EID0 = 0x27,
  CNF3     = 0x28,
  CNF2     = 0x29,
  CNF1     = 0x2A,
  CANINTE  = 0x2B,
  CANINTF  = 0x2C,
  EFLG     = 0x2D,
  TXB0CTRL = 0x30,
  TXB0SIDH = 0x31,
  TXB0SIDL = 0x32,
  TXB0EID8 = 0x33,
  TXB0EID0 = 0x34,
  TXB0DLC  = 0x35,
  TXB0DATA = 0x36,
  TXB1CTRL = 0x40,
  TXB1SIDH = 0x41,
  TXB1SIDL = 0x42,
  TXB1EID8 = 0x43,
  TXB1EID0 = 0x44,
  TXB1DLC  = 0x45,
  TXB1DATA = 0x46,
  TXB2CTRL = 0x50,
  TXB2SIDH = 0x51,
  TXB2SIDL = 0x52,
  TXB2EID8 = 0x53,
  TXB2EID0 = 0x54,
  TXB2DLC  = 0x55,
  TXB2DATA = 0x56,
  RXB0CTRL = 0x60,
  RXB0SIDH = 0x61,
  RXB0SIDL = 0x62,
  RXB0EID8 = 0x63,
  RXB0EID0 = 0x64,
  RXB0DLC  = 0x65,
  RXB0DATA = 0x66,
  RXB1CTRL = 0x70,
  RXB1SIDH = 0x71,
  RXB1SIDL = 0x72,
  RXB1EID8 = 0x73,
  RXB1EID0 = 0x74,
  RXB1DLC  = 0x75,
  RXB1DATA = 0x76
};

class Driver
{
public:
  Driver(SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio, uint16_t gpioCsPin);

  Error init();
  void reset();
  Error setSpeed(Speed speed);
  Error setMode(Mode mode);
  Mode getMode();


  void interruptHandler();

  uint8_t isConnected();

private:
  void selectChip();
  void deselectChip();
  void readRegister(Register reg, uint8_t *buf);
  void readRegisters(Register reg, uint8_t *buf, uint8_t count);
  void writeRegister(Register reg, uint8_t val);
  void writeRegisters(Register reg, uint8_t *vals, uint8_t count);
  void bitSetRegister(Register reg, uint8_t mask, uint8_t val);

private:
  SPI_HandleTypeDef *d_hspi;
  GPIO_TypeDef *d_gpio;
  uint16_t d_gpioCsPin;

  bool d_connected;
};

}

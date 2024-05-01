#pragma once

#include <main.hpp>
#include "stm32f1xx_hal.h"
#include "mcp2515registers.hpp"


namespace mcp2515
{
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
  OK = 0U,            // Function succeeded
  ERROR,              // Function failed
  TIMEOUT,
  INCORRECT_MODE,     // Device was in mode that doesn't support such operation
  BUFFER_FULL,        // e.g. all TX buffers occupied
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

enum class TxPriority {
  LOWEST = 0x00,
  LOW = 0x01,
  HIGH = 0x02,
  HIGHEST = 0x03
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

struct CanFrame
{
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
  bool extended;
  bool rtr;
};

class Driver
{
public:
  Driver(SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio, uint16_t gpioCsPin, std::function<void(bool)> isrSwitchFunc);

  Error init();
  void reset();
  Error setSpeed(Speed speed);
  Error setMode(Mode mode);
  Mode getMode();

  Error sendFrame(uint16_t id, uint8_t dlc, uint8_t data[], TxPriority pri = TxPriority::LOW);
  Error sendFrameExt(uint32_t id, uint8_t dlc, uint8_t data[], TxPriority pri = TxPriority::LOW);

  Error setFilter(uint8_t idx, uint32_t mask, uint32_t val, bool extended);
  bool frameAvailable();
  bool readFrame(CanFrame &frame, uint8_t &filterId, bool &overflow);

  void interruptHandler();

private:
  void selectChip();
  void deselectChip();
  void readRegister(Register reg, uint8_t *buf);
  void readRegisters(Register reg, uint8_t *buf, uint8_t count);
  void writeRegister(Register reg, uint8_t val);
  void writeRegisters(Register reg, uint8_t *vals, uint8_t count);
  Error sendFrame(uint32_t id, uint8_t dlc, uint8_t data[], bool extId, TxPriority pri = TxPriority::LOW);
  void writeTxBuffer(uint8_t idx, uint32_t id, uint8_t dlc, uint8_t data[], bool extId, TxPriority pri = TxPriority::LOW);
  void bitSetRegister(Register reg, uint8_t mask, uint8_t val);
  uint8_t readStatus();
  void readRxBuffer(uint8_t idx);
  uint8_t readRxStatus();
  void prepareFrameData(uint32_t id, bool extended, uint8_t output[]);

private:
  SPI_HandleTypeDef *d_hspi;
  GPIO_TypeDef *d_gpio;
  uint16_t d_gpioCsPin;
  std::function<void(bool)> d_switchIsr;

  Mode d_mode;
  uint8_t d_freeTxBufMask;

  uint8_t d_recvFramesBufMask;
  uint8_t d_recvFramesBufOvf;
  uint8_t d_matchedFilter[2];
  CanFrame d_receivedFrames[2];
};

}

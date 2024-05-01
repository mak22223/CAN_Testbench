#include "mcp2515.hpp"

#include "mcp2515registers.hpp"

#include "string.h"

namespace mcp2515
{

Driver::Driver(SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio, uint16_t gpioCsPin)
: d_hspi(hspi), d_gpio(gpio), d_gpioCsPin(gpioCsPin) {}

Error Driver::init()
{
  // Controller reset
  deselectChip();
  reset();
  HAL_Delay(2);

  // Check if there is MCP2515 connected
  uint8_t readBuf[1] = { 0 };
  readRegister(Register::CANCTRL, readBuf);

  if (readBuf[0] != 0x87) {
    return Error::ERROR;
  }

  // Allow to receive all messages, regardless filters, and rollover
  bitSetRegister(Register::RXB0CTRL, RXBNCTRL_RXM_MASK | RXBNCTRL_BUKT_MASK,
      (0b11 << RXBNCTRL_RXM) | (1 << RXBNCTRL_BUKT));
  bitSetRegister(Register::RXB1CTRL, RXBNCTRL_RXM_MASK, 0b11 << RXBNCTRL_RXM);

  // Enable TXC interrupts for all TX buffers
  bitSetRegister(
    Register::CANINTE, 
    CANINTE_TX2IE_MASK | CANINTE_TX1IE_MASK | CANINTE_TX0IE_MASK,
    (1 << CANINTE_TX2IE) | (1 << CANINTE_TX1IE) | (1 << CANINTE_TX0IE));
  d_freeTxBufMask = 1 << 0 | 1 << 1 | 1 << 2;;
  if (setSpeed(Speed::MCP_500KBPS) == Error::ERROR) {
    return Error::ERROR;
  }

  return Error::OK;
}

void Driver::reset()
{
  Command cmd = Command::RESET;
  selectChip();
  HAL_SPI_Transmit(d_hspi, (uint8_t*)&cmd, 1, 1);
  deselectChip();
}

void Driver::selectChip()
{
  HAL_GPIO_WritePin(d_gpio, d_gpioCsPin, GPIO_PIN_RESET);
}

void Driver::deselectChip()
{
  HAL_GPIO_WritePin(d_gpio, d_gpioCsPin, GPIO_PIN_SET);
}

void Driver::readRegister(Register reg, uint8_t *buf)
{
  readRegisters(reg, buf, 1);
}

void Driver::readRegisters(Register reg, uint8_t *buf, uint8_t count)
{
  selectChip();
  uint8_t sendBuf[] = { (uint8_t)Command::READ, (uint8_t)reg };
  HAL_SPI_Transmit(d_hspi, sendBuf, sizeof(sendBuf), 1);
  HAL_SPI_Receive(d_hspi, buf, count, 1);
  deselectChip();
}

void Driver::writeRegister(Register reg, uint8_t val)
{
  writeRegisters(reg, &val, 1);
}

void Driver::writeRegisters(Register reg, uint8_t *vals, uint8_t count)
{
  selectChip();
  uint8_t sendBuf[] = { (uint8_t)Command::WRITE, (uint8_t)reg };
  HAL_SPI_Transmit(d_hspi, sendBuf, sizeof(sendBuf), 1);
  HAL_SPI_Transmit(d_hspi, vals, count, 2);
  deselectChip();
}

void Driver::bitSetRegister(Register reg, uint8_t mask, uint8_t val)
{
  selectChip();
  uint8_t sendBuf[] = { (uint8_t)Command::BIT_MODIFY, (uint8_t)reg, mask, val };
  HAL_SPI_Transmit(d_hspi, sendBuf, sizeof(sendBuf), 1);
  deselectChip();
}

uint8_t Driver::readStatus()
{
  uint8_t result;

  selectChip();
  uint8_t sendBuf[] = { (uint8_t)Command::READ_STATUS };
  HAL_SPI_Transmit(d_hspi, sendBuf, sizeof(sendBuf), 1);
  HAL_SPI_Receive(d_hspi, &result, 1, 1);
  deselectChip();

  return result;
}

Error Driver::setSpeed(Speed speed)
{
  uint8_t sendBuf[3];

  switch (speed) {
    case Speed::MCP_500KBPS:
      sendBuf[0] = 0 << CNF3_SOF | 0 << CNF3_WAKFIL | MCP_500KBPS_PHSEG2 << CNF3_PHSEG2;  // CNF3
      sendBuf[1] =                                                                        // CNF2
          1 << CNF2_BLTMODE |
          MCP_500KBPS_SAM << CNF2_SAM |
          MCP_500KBPS_PHSEG1 << CNF2_PHSEG1 |
          MCP_500KBPS_PRSEG << CNF2_PRSEG;
      sendBuf[2] = 0 << CNF1_SJW | MCP_500KBPS_BRP << CNF1_BRP;                           // CNF1
      break;

    case Speed::MCP_250KBPS:
      sendBuf[0] = 0 << CNF3_SOF | 0 << CNF3_WAKFIL | MCP_250KBPS_PHSEG2 << CNF3_PHSEG2;  // CNF3
      sendBuf[1] =                                                                        // CNF2
          1 << CNF2_BLTMODE |
          MCP_250KBPS_SAM << CNF2_SAM |
          MCP_250KBPS_PHSEG1 << CNF2_PHSEG1 |
          MCP_250KBPS_PRSEG << CNF2_PRSEG;
      sendBuf[2] = 0 << CNF1_SJW | MCP_250KBPS_BRP << CNF1_BRP;                           // CNF1
      break;

    case Speed::MCP_125KBPS:
      sendBuf[0] = 0 << CNF3_SOF | 0 << CNF3_WAKFIL | MCP_125KBPS_PHSEG2 << CNF3_PHSEG2;  // CNF3
      sendBuf[1] =                                                                        // CNF2
          1 << CNF2_BLTMODE |
          MCP_125KBPS_SAM << CNF2_SAM |
          MCP_125KBPS_PHSEG1 << CNF2_PHSEG1 |
          MCP_125KBPS_PRSEG << CNF2_PRSEG;
      sendBuf[2] = 0 << CNF1_SJW | MCP_125KBPS_BRP << CNF1_BRP;                           // CNF1
      break;

    default:
      return Error::ERROR;
  }

  writeRegisters(Register::CNF3, sendBuf, sizeof(sendBuf));

  uint8_t read[sizeof(sendBuf)];
  readRegisters(Register::CNF3, read, sizeof(sendBuf));

  if (memcmp(sendBuf, read, sizeof(sendBuf)) == 0) {
    return Error::OK;
  }
  return Error::ERROR;
}

Mode Driver::getMode()
{
  uint8_t result;
  readRegister(Register::CANSTAT, &result);
  result = (result & CANSTAT_OPMOD_MASK) >> CANSTAT_OPMOD;

  switch ((Mode)result) {
    case Mode::NORMAL:
      d_mode = Mode::NORMAL;
      break;

    case Mode::SLEEP:
      d_mode = Mode::SLEEP;
      break;

    case Mode::LOOPBACK:
      d_mode = Mode::LOOPBACK;
      break;

    case Mode::LISTENONLY:
      d_mode = Mode::LISTENONLY;
      break;

    case Mode::CONFIGURATION:
      d_mode = Mode::CONFIGURATION;
      break;

    default:
      d_mode = Mode::UNKNOWN;
      break;
  }
  return d_mode;
}

Error Driver::setMode(Mode mode)
{
  bitSetRegister(Register::CANCTRL, CANCTRL_REQOP_MASK, (uint8_t)mode << CANCTRL_REQOP);

  uint8_t canstat;
  readRegister(Register::CANSTAT, &canstat);

  if ((canstat & CANSTAT_OPMOD_MASK) >> CANSTAT_OPMOD == (uint8_t)mode) {
    return Error::OK;
  }

  return Error::ERROR;
}

Error Driver::sendFrame(uint16_t id, uint8_t dlc, uint8_t data[], TxPriority pri)
{
  return sendFrame(id, dlc, data, false);
}

Error Driver::sendFrameExt(uint32_t id, uint8_t dlc, uint8_t data[], TxPriority pri)
{
  return sendFrame(id, dlc, data, true);
}

Error Driver::sendFrame(uint32_t id, uint8_t dlc, uint8_t data[], bool extId, TxPriority pri)
{
  if (dlc > 8) {
    return Error::ERROR;
  }

  if (d_mode != Mode::NORMAL && d_mode != Mode::LOOPBACK) {
    return Error::INCORRECT_MODE;
  }

  if ((d_freeTxBufMask & 0x07) == 0) {
    return Error::BUFFER_FULL;
  }

  uint8_t txBufIdx;
  if (d_freeTxBufMask & 1 << 0) {
    txBufIdx = 0;
  } else if (d_freeTxBufMask & 1 << 1) {
    txBufIdx = 1;
  } else if (d_freeTxBufMask & 1 << 2) {
    txBufIdx = 2;
  } else {
    return Error::ERROR;
  }

  writeTxBuffer(txBufIdx, id, dlc, data, extId, pri);

  return Error::OK;
}

void Driver::writeTxBuffer(uint8_t idx, uint32_t id, uint8_t dlc, uint8_t data[], bool extId, TxPriority pri)
{
  // загрузка данных в регистры буфера
  const uint8_t bufferSelector[] = { 0x00, 0x02, 0x04 };
  uint8_t sendBuf[6] = {(uint8_t)((uint8_t)Command::LOAD_TX_BUFFER | bufferSelector[idx]), };

  sendBuf[1] = id >> 3;
  sendBuf[2] = 0 | (id << 5);

  if (extId) {
    sendBuf[2] |= (1 << 3) | id >> 27;
    sendBuf[3] = id >> 19;
    sendBuf[4] = id >> 11;
  }
  
  sendBuf[5] = dlc;

  selectChip();
  HAL_SPI_Transmit(d_hspi, sendBuf, sizeof(sendBuf), 2);
  HAL_SPI_Transmit(d_hspi, data, dlc, 2);
  deselectChip();

  d_freeTxBufMask &= ~(1 << idx);

  /// TODO: заменить на команду SPI, если не нужно менять приоритет
  bitSetRegister(
    (Register)((uint8_t)Register::TXB0CTRL + (idx << 4)),
    TXBNCTRL_TXREQ_MASK | TXBNCTRL_TXP_MASK,
    (1 << TXBNCTRL_TXREQ) | ((uint8_t)pri << TXBNCTRL_TXP));
}

// RXC, ERR
void Driver::interruptHandler()
{
  uint8_t status;
  status = readStatus();

  if (status & 0x80) {
    // TX2IF
    bitSetRegister(Register::CANINTF, CANINTF_TX2IF_MASK, ~(1 << CANINTF_TX2IF));
    d_freeTxBufMask |= 1 << 2;
  } else if (status & 0x20) {
    // TX1IF
    bitSetRegister(Register::CANINTF, CANINTF_TX1IF_MASK, ~(1 << CANINTF_TX1IF));
    d_freeTxBufMask |= 1 << 1;
  } else if (status & 0x08) {
    // TX0IF
    bitSetRegister(Register::CANINTF, CANINTF_TX0IF_MASK, ~(1 << CANINTF_TX0IF));
    d_freeTxBufMask |= 1 << 0;
  } else if (status & 0x02) {
    // RX1IF
    /// TODO: реализовать обработку прерываний
    Error_Handler(0x22);
  } else if (status & 0x01) {
    // RX0IF
    Error_Handler(0x21);
  } else {
    // если ни один из флагов не подошел
    uint8_t canstat;
    readRegister(Register::CANSTAT, &canstat);
    Error_Handler(0x23);
  }
}

}

#include "mcp2515.hpp"

#include "mcp2515registers.hpp"

#include "string.h"

namespace mcp2515
{

constexpr Register filterMaskRegAddr[2] = {
  Register::RXM0SIDH,
  Register::RXM1SIDH
};
constexpr Register filterRegAddr[6] = {
  Register::RXF0SIDH,
  Register::RXF1SIDH,
  Register::RXF2SIDH,
  Register::RXF3SIDH,
  Register::RXF4SIDH,
  Register::RXF5SIDH
};

Driver::Driver(SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio, uint16_t gpioCsPin, std::function<void(bool)> isrSwitchFunc)
: d_hspi(hspi), d_gpio(gpio), d_gpioCsPin(gpioCsPin), d_switchIsr(isrSwitchFunc) {}

Error Driver::init()
{
  // Controller reset
  d_switchIsr(false);
  deselectChip();
  reset();
  HAL_Delay(2);

  // Check if there is MCP2515 connected
  uint8_t readBuf[1] = { 0 };
  readRegister(Register::CANCTRL, readBuf);

  if (readBuf[0] != 0x87) {
    /// TODO: заменить на объект с освобождением ресурса при выходе из области видимости
    d_switchIsr(true);
    return Error::ERROR;
  }

  d_mode = Mode::CONFIGURATION;

  // Enable TXC, RXC interrupts for all buffers
  bitSetRegister(
    Register::CANINTE, 
    CANINTE_TX2IE_MASK | CANINTE_TX1IE_MASK | CANINTE_TX0IE_MASK | CANINTE_RX1IE_MASK | CANINTE_RX0IE_MASK,
    (1 << CANINTE_TX2IE) | (1 << CANINTE_TX1IE) | (1 << CANINTE_TX0IE) | (1 << CANINTE_RX1IE) | (1 << CANINTE_RX0IE));

  d_freeTxBufMask = (1 << 2) | (1 << 1) | (1 << 0);
  d_recvFramesBufMask = 0;

  d_switchIsr(true);
  return Error::OK;
}

void Driver::reset()
{
  Command cmd = Command::RESET;
  d_switchIsr(false);
  selectChip();
  HAL_SPI_Transmit(d_hspi, (uint8_t*)&cmd, 1, 1);
  deselectChip();
  d_switchIsr(true);
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

  if (d_mode != Mode::CONFIGURATION) {
    return Error::INCORRECT_MODE;
  }

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
  d_switchIsr(false);
  readRegister(Register::CANSTAT, &result);
  result = (result & CANSTAT_OPMOD_MASK) >> CANSTAT_OPMOD;
  d_switchIsr(true);

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
  d_switchIsr(false);
  bitSetRegister(Register::CANCTRL, CANCTRL_REQOP_MASK, (uint8_t)mode << CANCTRL_REQOP);

  uint8_t canstat;
  readRegister(Register::CANSTAT, &canstat);
  d_switchIsr(true);

  if ((canstat & CANSTAT_OPMOD_MASK) >> CANSTAT_OPMOD == (uint8_t)mode) {
    d_mode = mode;
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

  d_switchIsr(false);

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
  d_switchIsr(true);

  return Error::OK;
}

void Driver::writeTxBuffer(uint8_t idx, uint32_t id, uint8_t dlc, uint8_t data[], bool extId, TxPriority pri)
{
  // загрузка данных в регистры буфера
  const uint8_t bufferSelector[] = { 0x00, 0x02, 0x04 };
  uint8_t sendBuf[6] = {(uint8_t)((uint8_t)Command::LOAD_TX_BUFFER | bufferSelector[idx]), };
  prepareFrameData(id, extId, &sendBuf[1]);
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

/// TODO: обработка ошибок
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
  } else if (status & 0x01) {
    // RX0IF (check before RX1IF as it has higher prioity in RX_STATUS data)
    uint8_t rxStatus = readRxStatus();
    d_matchedFilter[0] = rxStatus & 0x7;
    readRxBuffer(0);
  } else if (status & 0x02) {
    // RX1IF
    uint8_t rxStatus = readRxStatus();

    if ((rxStatus & 0xC0) == 0x80) {
      // only RXB1 present therefore data valid for RXB1
      d_matchedFilter[1] = rxStatus & 0x7;
      readRxBuffer(1);
    }
  } else {
    // если ни один из флагов не подошел
    uint8_t canstat;
    readRegister(Register::CANSTAT, &canstat);
    Error_Handler(0x21);
  }
}

uint8_t Driver::readRxStatus()
{
  uint8_t rxStatus;
  uint8_t sendData = (uint8_t)Command::RX_STATUS;
  selectChip();
  HAL_SPI_Transmit(d_hspi, &sendData, 1, 1);
  HAL_SPI_Receive(d_hspi, &rxStatus, 1, 1);
  deselectChip();
  return rxStatus;
}

void Driver::readRxBuffer(uint8_t idx)
{
  const uint8_t bufferSelector[] = { 0x0, 0x4 };
  uint8_t frameData[13];
  uint8_t sendData = (uint8_t)Command::READ_RX_BUFFER | bufferSelector[idx];
  selectChip();
  HAL_SPI_Transmit(d_hspi, &sendData, 1, 1);
  HAL_SPI_Receive(d_hspi, &frameData[0], 5, 2);

  uint8_t dataLen = frameData[4] & 0x0F; // DLC field
  HAL_SPI_Receive(d_hspi, &frameData[5], dataLen, 2);
  deselectChip();

  if (d_recvFramesBufMask & (1 << idx)) {
    d_recvFramesBufOvf |= 1 << idx;
  }

  d_recvFramesBufMask |= 1 << idx;

  d_receivedFrames[idx].extended = false;
  d_receivedFrames[idx].id = frameData[0] << 3 | frameData[1] >> 5;

  if (frameData[1] & (1 << 3)) {
    // extended id
    d_receivedFrames[idx].rtr = ((frameData[4] & (1 << 6)) != 0);
    d_receivedFrames[idx].extended = true;
    d_receivedFrames[idx].id |= (frameData[1] & 0x3) << 27 | frameData[2] << 19 | frameData[3] << 11;
  } else {
    d_receivedFrames[idx].rtr = ((frameData[1] & (1 << 4)) != 0);
  }

  d_receivedFrames[idx].dlc = dataLen;
  for (int i = 0; i < dataLen; ++i) {
    d_receivedFrames[idx].data[i] = frameData[5 + i];
  }
}

bool Driver::frameAvailable()
{
  return d_recvFramesBufMask != 0;
}

bool Driver::readFrame(CanFrame &frame, uint8_t &filterId, bool &overflow)
{
  if (d_recvFramesBufMask & (1 << 0)) {
    d_recvFramesBufMask &= ~(1 << 0);
    overflow = false;

    if (d_recvFramesBufOvf & (1 << 0)) {
      overflow = true;
      d_recvFramesBufOvf &= ~(1 << 0);
    }
    filterId = d_matchedFilter[0];
    frame = d_receivedFrames[0];
  } else if (d_recvFramesBufMask & (1 << 1)) {
    d_recvFramesBufMask &= ~(1 << 1);
    overflow = false;

    if (d_recvFramesBufOvf & (1 << 1)) {
      overflow = true;
      d_recvFramesBufOvf &= ~(1 << 1);
    }
    filterId = d_matchedFilter[1];
    frame = d_receivedFrames[1];
  } else {
    return false;
  }
  return true;
}

// Possible filter idx [5..0].
// Filters [1..0] and [5..2] have two separate masks.
// When using standard frame filters, 
// high part of mask and filter are applied to data (counting from 0)
// [18:11] to second data byte, [26:19] to first data byte
// two most significant bits [28:27] are not used.
Error Driver::setFilter(uint8_t idx, uint32_t mask, uint32_t val, bool extended)
{
  if (d_mode != Mode::CONFIGURATION) {
    return Error::INCORRECT_MODE;
  }

  if (idx > 5) {
    return Error::ERROR;
  }

  uint8_t maskIdx = 0;
  if (idx > 1) {
    maskIdx = 1;
  }

  uint8_t sendData[4];
  prepareFrameData(val, true, sendData);
  if (!extended) {
    sendData[1] &= ~(1 << 3);
  }

  d_switchIsr(false);
  writeRegisters(filterRegAddr[idx], sendData, 4);
  d_switchIsr(true);

  prepareFrameData(mask, true, sendData);

  d_switchIsr(false);
  writeRegisters(filterMaskRegAddr[maskIdx], sendData, 4);
  d_switchIsr(true);
  
  return Error::OK;
}

void Driver::prepareFrameData(uint32_t id, bool extended, uint8_t output[])
{
  output[0] = id >> 3;
  output[1] = id << 5;

  if (extended) {
    output[1] |= 1 << 3 | (0x03 & (id >> 27));
    output[2] = id >> 19;
    output[3] = id >> 11;
  } else {
    output[2] = 0;
    output[3] = 0;
  }
}

}

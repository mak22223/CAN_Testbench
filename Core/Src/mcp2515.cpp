#include "mcp2515.hpp"

#include "string.h"

namespace mcp2515
{

Driver::Driver(SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio, uint16_t gpioCsPin)
: d_hspi(hspi), d_gpio(gpio), d_gpioCsPin(gpioCsPin), d_connected(false) {}

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
      return Mode::NORMAL;

    case Mode::SLEEP:
      return Mode::SLEEP;

    case Mode::LOOPBACK:
      return Mode::LOOPBACK;

    case Mode::LISTENONLY:
      return Mode::LISTENONLY;

    case Mode::CONFIGURATION:
      return Mode::CONFIGURATION;

    default:
      return Mode::UNKNOWN;
  }
  return Mode::UNKNOWN;
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

void Driver::interruptHandler()
{
  uint8_t result[1];
  readRegister(Register::CANINTF, result);

  // TODO: реализовать обработку прерываний
}

}

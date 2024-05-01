#pragma once

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

#define CANINTE_MERRE_MASK  (0x80U)
#define CANINTE_MERRE       (7)
#define CANINTE_WAKIE_MASK  (0x40U)
#define CANINTE_WAKIE       (6)
#define CANINTE_ERRIE_MASK  (0x20U)
#define CANINTE_ERRIE       (5)
#define CANINTE_TX2IE_MASK  (0x10U)
#define CANINTE_TX2IE       (4)
#define CANINTE_TX1IE_MASK  (0x08U)
#define CANINTE_TX1IE       (3)
#define CANINTE_TX0IE_MASK  (0x04U)
#define CANINTE_TX0IE       (2)
#define CANINTE_RX1IE_MASK  (0x02U)
#define CANINTE_RX1IE       (1)
#define CANINTE_RX0IE_MASK  (0x01U)
#define CANINTE_RX0IE       (0)

#define CANINTF_MERRF_MASK  (0x80U)
#define CANINTF_MERRF       (7)
#define CANINTF_WAKIF_MASK  (0x40U)
#define CANINTF_WAKIF       (6)
#define CANINTF_ERRIF_MASK  (0x20U)
#define CANINTF_ERRIF       (5)
#define CANINTF_TX2IF_MASK  (0x10U)
#define CANINTF_TX2IF       (4)
#define CANINTF_TX1IF_MASK  (0x08U)
#define CANINTF_TX1IF       (3)
#define CANINTF_TX0IF_MASK  (0x04U)
#define CANINTF_TX0IF       (2)
#define CANINTF_RX1IF_MASK  (0x02U)
#define CANINTF_RX1IF       (1)
#define CANINTF_RX0IF_MASK  (0x01U)
#define CANINTF_RX0IF       (0)

#define TXBNCTRL_ABTF_MASK  (0x40U)
#define TXBNCTRL_ABTF       (6)
#define TXBNCTRL_MLOA_MASK  (0x20U)
#define TXBNCTRL_MLOA       (5)
#define TXBNCTRL_TXERR_MASK (0x10U)
#define TXBNCTRL_TXERR      (4)
#define TXBNCTRL_TXREQ_MASK (0x08U)
#define TXBNCTRL_TXREQ      (3)
#define TXBNCTRL_TXP_MASK   (0x03U)
#define TXBNCTRL_TXP        (0)

}
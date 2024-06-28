/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/**
 * @file cy_eth_phy_dp83867ir.h
 * @brief This file provides enums, macros related to ethernet phy for PHY hardware part DP83867IR.
 */

#ifndef ETH_PHY_DP83867IR_H_
#define ETH_PHY_DP83867IR_H_

#include "cy_gpio.h"
#include "cy_eth_phy_user_config.h"

#include "cycfg.h"

/********************************************************/
/******************EMAC configuration********************/
/********************************************************/
#define EMAC_MII             0
#define EMAC_RMII            1
#define EMAC_GMII            2
#define EMAC_RGMII           3

/** PHY related constants    */
#define PHY_ADDR             (0) /* Value depends on PHY and its hardware configurations */

/********************************************************/
/** PHY Mode Selection       */
#if (defined (eth_1_ENABLED) && (eth_1_ENABLED == 1u))
#define EMAC_INTERFACE       eth_1_PHY_INTERFACE
#endif

#if (defined (eth_0_ENABLED) && (eth_0_ENABLED == 1u))
#define EMAC_INTERFACE       eth_0_PHY_INTERFACE
#endif

/********************************************************/
#define ETH_INTR_SRC         (CY_GIG_ETH_IRQN0)
#define ETH_INTR_SRC_Q1      (CY_GIG_ETH_IRQN1)
#define ETH_INTR_SRC_Q2      (CY_GIG_ETH_IRQN2)

#define ETHx_TD0_PORT        CY_GIG_ETH_TD0_PORT
#define ETHx_TD0_PIN         CY_GIG_ETH_TD0_PIN
#define ETHx_TD0_PIN_MUX     CY_GIG_ETH_TD0_PIN_MUX

#define ETHx_TD1_PORT        CY_GIG_ETH_TD1_PORT
#define ETHx_TD1_PIN         CY_GIG_ETH_TD1_PIN
#define ETHx_TD1_PIN_MUX     CY_GIG_ETH_TD1_PIN_MUX

#define ETHx_TD2_PORT        CY_GIG_ETH_TD2_PORT
#define ETHx_TD2_PIN         CY_GIG_ETH_TD2_PIN
#define ETHx_TD2_PIN_MUX     CY_GIG_ETH_TD2_PIN_MUX

#define ETHx_TD3_PORT        CY_GIG_ETH_TD3_PORT
#define ETHx_TD3_PIN         CY_GIG_ETH_TD3_PIN
#define ETHx_TD3_PIN_MUX     CY_GIG_ETH_TD3_PIN_MUX

#if (EMAC_INTERFACE != EMAC_RGMII)

#define ETHx_TD4_PORT        CY_GIG_ETH_TD4_PORT
#define ETHx_TD4_PIN         CY_GIG_ETH_TD4_PIN
#define ETHx_TD4_PIN_MUX     CY_GIG_ETH_TD4_PIN_MUX

#define ETHx_TD5_PORT        CY_GIG_ETH_TD5_PORT
#define ETHx_TD5_PIN         CY_GIG_ETH_TD5_PIN
#define ETHx_TD5_PIN_MUX     CY_GIG_ETH_TD5_PIN_MUX

#define ETHx_TD6_PORT        CY_GIG_ETH_TD6_PORT
#define ETHx_TD6_PIN         CY_GIG_ETH_TD6_PIN
#define ETHx_TD6_PIN_MUX     CY_GIG_ETH_TD6_PIN_MUX

#define ETHx_TD7_PORT        CY_GIG_ETH_TD7_PORT
#define ETHx_TD7_PIN         CY_GIG_ETH_TD7_PIN
#define ETHx_TD7_PIN_MUX     CY_GIG_ETH_TD7_PIN_MUX

#define ETHx_TXER_PORT       CY_GIG_ETH_TXER_PORT
#define ETHx_TXER_PIN        CY_GIG_ETH_TXER_PIN
#define ETHx_TXER_PIN_MUX    CY_GIG_ETH_TXER_PIN_MUX

#endif /* (EMAC_INTERFACE != EMAC_RGMII) */

#define ETHx_TX_CTL_PORT     CY_GIG_ETH_TX_CLK_PORT
#define ETHx_TX_CTL_PIN      CY_GIG_ETH_TX_CTL_PIN
#define ETHx_TX_CTL_PIN_MUX  CY_GIG_ETH_TX_CTL_PIN_MUX

#define ETHx_RD0_PORT        CY_GIG_ETH_RD0_PORT
#define ETHx_RD0_PIN         CY_GIG_ETH_RD0_PIN
#define ETHx_RD0_PIN_MUX     CY_GIG_ETH_RD0_PIN_MUX

#define ETHx_RD1_PORT        CY_GIG_ETH_RD1_PORT
#define ETHx_RD1_PIN         CY_GIG_ETH_RD1_PIN
#define ETHx_RD1_PIN_MUX     CY_GIG_ETH_RD1_PIN_MUX

#define ETHx_RD2_PORT        CY_GIG_ETH_RD2_PORT
#define ETHx_RD2_PIN         CY_GIG_ETH_RD2_PIN
#define ETHx_RD2_PIN_MUX     CY_GIG_ETH_RD2_PIN_MUX

#define ETHx_RD3_PORT        CY_GIG_ETH_RD3_PORT
#define ETHx_RD3_PIN         CY_GIG_ETH_RD3_PIN
#define ETHx_RD3_PIN_MUX     CY_GIG_ETH_RD3_PIN_MUX

#if (EMAC_INTERFACE != EMAC_RGMII)

#define ETHx_RD4_PORT        CY_GIG_ETH_RD4_PORT
#define ETHx_RD4_PIN         CY_GIG_ETH_RD4_PIN
#define ETHx_RD4_PIN_MUX     CY_GIG_ETH_RD4_PIN_MUX

#define ETHx_RD5_PORT        CY_GIG_ETH_RD5_PORT
#define ETHx_RD5_PIN         CY_GIG_ETH_RD5_PIN
#define ETHx_RD5_PIN_MUX     CY_GIG_ETH_RD5_PIN_MUX

#define ETHx_RD6_PORT        CY_GIG_ETH_RD6_PORT
#define ETHx_RD6_PIN         CY_GIG_ETH_RD6_PIN
#define ETHx_RD6_PIN_MUX     CY_GIG_ETH_RD6_PIN_MUX

#define ETHx_RD7_PORT        CY_GIG_ETH_RD7_PORT
#define ETHx_RD7_PIN         CY_GIG_ETH_RD7_PIN
#define ETHx_RD7_PIN_MUX     CY_GIG_ETH_RD7_PIN_MUX

#define ETHx_RX_ER_PORT      CY_GIG_ETH_RX_ER_PORT
#define ETHx_RX_ER_PIN       CY_GIG_ETH_RX_ER_PIN
#define ETHx_RX_ER_PIN_MUX   CY_GIG_ETH_RX_ER_PIN_MUX

#endif /* #if (EMAC_INTERFACE != EMAC_RGMII) */

#define ETHx_RX_CTL_PORT     CY_GIG_ETH_RX_CTL_PORT
#define ETHx_RX_CTL_PIN      CY_GIG_ETH_RX_CTL_PIN
#define ETHx_RX_CTL_PIN_MUX  CY_GIG_ETH_RX_CTL_PIN_MUX

#define ETHx_TX_CLK_PORT     CY_GIG_ETH_TX_CLK_PORT
#define ETHx_TX_CLK_PIN      CY_GIG_ETH_TX_CLK_PIN
#define ETHx_TX_CLK_PIN_MUX  CY_GIG_ETH_TX_CLK_PIN_MUX

#define ETHx_RX_CLK_PORT     CY_GIG_ETH_RX_CLK_PORT
#define ETHx_RX_CLK_PIN      CY_GIG_ETH_RX_CLK_PIN
#define ETHx_RX_CLK_PIN_MUX  CY_GIG_ETH_RX_CLK_PIN_MUX

#define ETHx_REF_CLK_PORT    CY_GIG_ETH_REF_CLK_PORT
#define ETHx_REF_CLK_PIN     CY_GIG_ETH_REF_CLK_PIN
#define ETHx_REF_CLK_PIN_MUX CY_GIG_ETH_REF_CLK_PIN_MUX

#define ETHx_MDC_PORT        CY_GIG_ETH_MDC_PORT
#define ETHx_MDC_PIN         CY_GIG_ETH_MDC_PIN
#define ETHx_MDC_PIN_MUX     CY_GIG_ETH_MDC_PIN_MUX

#define ETHx_MDIO_PORT       CY_GIG_ETH_MDIO_PORT
#define ETHx_MDIO_PIN        CY_GIG_ETH_MDIO_PIN
#define ETHx_MDIO_PIN_MUX    CY_GIG_ETH_MDIO_PIN_MUX

/* Bits masks to verify auto negotiation configured speed */
#define ANLPAR_10_Msk                           (0x00000020UL)  /**< 10BASE-Te Support */
#define ANLPAR_10_Pos                           (5UL)           /**< 10BASE-Te bit position */
#define ANLPAR_10FD_Msk                         (0x00000040UL)  /**< 10BASE-Te Full Duplex Support */
#define ANLPAR_10FD_Pos                         (6UL)           /**< 10BASE-Te Full Duplex bit position */

#define ANLPAR_TX_Msk                           (0x00000080UL)  /**< 100BASE-TX Support */
#define ANLPAR_TX_Pos                           (7UL)           /**< 100BASE-TX bit position */
#define ANLPAR_TXFD_Msk                         (0x00000100UL)  /**< 100BASE-TX Full Duplex Support */
#define ANLPAR_TXFD_Pos                         (8UL)           /**< 100BASE-TX Full Duplex bit position */
#define ANLPAR_T4_Msk                           (0x00000200UL)  /**< 100BASE-T4 Support */
#define ANLPAR_T4_Pos                           (9UL)           /**< 100BASE-T4 bit position */

#define STS1_1000BASE_T_HALFDUPLEX_Msk          (0x00000400UL)  /**< 1000BASE-T Half-Duplex Capable */
#define STS1_1000BASE_T_HALFDUPLEX_Pos          (10UL)          /**< 1000BASE-T Half-Duplex bit position */
#define STS1_1000BASE_T_FULLDUPLEX_Msk          (0x00000800UL)  /**< 1000BASE-T Full-Duplex Capable */
#define STS1_1000BASE_T_FULLDUPLEX_Pos          (11UL)          /**< 1000BASE-T Full-Duplex bit position */

#define REGISTER_ADDRESS_PHY_REG_BMCR       PHYREG_00_BMCR          /* BMCR register (0x0000) to read the speed and duplex mode */
#define REGISTER_PHY_REG_DUPLEX_MASK        PHYBMCR_FULL_DUPLEX_Msk /* Bit 8 of BMCR register to read the duplex mode */
#define REGISTER_PHY_REG_SPEED_MASK         (0x2040)                /* Bit 6, 13: BMCR register to read the speed */
#define REGISTER_PHY_REG_SPEED_MASK_10M     (0x0000)                /* Bit 6, 13: Both are set to 0 for 10M speed */
#define REGISTER_PHY_REG_SPEED_MASK_100M    (0x2000)                /* Bit 6, 13: Set to 0 and 1 respectively for 100M speed */
#define REGISTER_PHY_REG_SPEED_MASK_1000M   (0x0040)                /* Bit 6, 13: Set to 1 and 0 respectively for 1000M speed */

#endif /* ETH_PHY_DP83867IR_H_ */

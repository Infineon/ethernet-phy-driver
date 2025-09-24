/*
 * Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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
* @file cy_eth_phy_dp83867ir.c
* @brief Provides implementation of PHY operation callbacks required by ECM. 
* This implementation is valid only for PHY hardware part DP83867IR.
*/
#include "cycfg.h"

#if defined(ETH_PHY_DP83867IR)

#include "cy_eth_phy_driver.h"
#include "cy_result.h"
#include "cyabs_rtos.h"

/********************************************************/
/******************EMAC configuration********************/
/********************************************************/
#define EMAC_MII             0
#define EMAC_RMII            1
#define EMAC_GMII            2
#define EMAC_RGMII           3

/********************************************************/
/** PHY Mode Selection       */
#if (defined (eth_1_ENABLED) && (eth_1_ENABLED == 1u))
#define EMAC_INTERFACE       eth_1_PHY_INTERFACE
#define PHY_ADDR             eth_1_PHY_ADDR
#endif

#if (defined (eth_0_ENABLED) && (eth_0_ENABLED == 1u))
#define EMAC_INTERFACE       eth_0_PHY_INTERFACE
#define PHY_ADDR             eth_0_PHY_ADDR
#endif

/********************************************************/
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

/********************************************************/

cy_stc_ephy_t phyObj[2];

cy_rslt_t enable_phy_DP83867IR_extended_reg(ETH_Type *reg_base, uint32_t phy_speed)
{
    cy_rslt_t    result = CY_RSLT_SUCCESS;

    if(phy_speed == CY_EPHY_SPEED_100)
    {
        Cy_ETHIF_PhyRegWrite(reg_base, 0x10, 0x5028, PHY_ADDR); /** Disable auto negotiation for MDI/MDI-X **/
    }
    else if(phy_speed == CY_EPHY_SPEED_1000 || phy_speed == CY_EPHY_SPEED_AUTO)
    {
        uint32_t    u32ReadData;
        Cy_ETHIF_PhyRegWrite(reg_base, 0x0D, 0x001F, PHY_ADDR);         /** Begin write access to the extended register     */
        Cy_ETHIF_PhyRegWrite(reg_base, 0x0E, 0x0170, PHY_ADDR);
        Cy_ETHIF_PhyRegWrite(reg_base, 0x0D, 0x401F, PHY_ADDR);
        u32ReadData = Cy_ETHIF_PhyRegRead(reg_base, (uint8_t)0x0E, PHY_ADDR);
        u32ReadData = u32ReadData & 0x0000;                                 /** Change the I/O impedance on the PHY    */
        u32ReadData = u32ReadData | 0x010C;
        Cy_ETHIF_PhyRegWrite(reg_base, 0x0E, u32ReadData, PHY_ADDR);         /** Enable clock from the PHY -> Route it to the MCU    */
        u32ReadData = Cy_ETHIF_PhyRegRead(reg_base, (uint8_t)0x0E, PHY_ADDR);
    }
    else
    {
        /* Do nothing */
    }

    /** Disable RGMII by accessing the extended register set || Please read datasheet section 8.4.2.1 for the procedure in detail */
    Cy_ETHIF_PhyRegWrite(reg_base, 0x0D, 0x001F, PHY_ADDR);                     /** REGCR  */
    Cy_ETHIF_PhyRegWrite(reg_base, 0x0E, 0x0032, PHY_ADDR);                     /** ADDAR, 0x0032 RGMII config register  */
    Cy_ETHIF_PhyRegWrite(reg_base, 0x0D, 0x401F, PHY_ADDR);                     /** REGCR; will force the next write/read access non-incremental  */

#if EMAC_INTERFACE == EMAC_RGMII
    Cy_ETHIF_PhyRegWrite(reg_base, 0x0E, 0x00D3, PHY_ADDR);                     /** Enable Tx and RX clock delay in the RGMII configuration register  */
    Cy_ETHIF_PhyRegWrite(reg_base, 0x0D, 0x001F, PHY_ADDR);                     /** REGCR  */
    Cy_ETHIF_PhyRegWrite(reg_base, 0x0E, 0x0086, PHY_ADDR);                     /** ADDAR; 0x0086 delay config register  */
    Cy_ETHIF_PhyRegWrite(reg_base, 0x0D, 0x401F, PHY_ADDR);                     /** REGCR; will force the next write/read access non-incremental  */
    Cy_ETHIF_PhyRegWrite(reg_base, 0x0E, 0x0066, PHY_ADDR);                     /** Adjust Tx and Rx clock delays in the PHY  */
#else
    Cy_ETHIF_PhyRegWrite(reg_base, 0x0E, 0x0000, PHY_ADDR);                     /** Disable RGMII  */
    Cy_ETHIF_PhyRegRead(reg_base, (uint8_t)0x0E, PHY_ADDR);                     /** Read RGMII mode status  */
#endif

    Cy_ETHIF_PhyRegWrite(reg_base, 0x1F, 0x4000, PHY_ADDR);                     /** CTRL   */
    cy_rtos_delay_milliseconds(30);                                             /** Some more delay to get the PHY adapted to new interface */
    Cy_ETHIF_PhyRegRead(reg_base, (uint8_t)0x11, PHY_ADDR);
    return result;
}

void cy_eth0_phy_read(uint32_t phyId, uint32_t regAddress, uint32_t *value)
{
    *value = Cy_ETHIF_PhyRegRead(ETH0, regAddress, phyId);
}

void cy_eth0_phy_write(uint32_t phyId, uint32_t regAddress, uint32_t value)
{
    Cy_ETHIF_PhyRegWrite(ETH0, regAddress, value, phyId);
}

void cy_eth1_phy_read(uint32_t phyId, uint32_t regAddress, uint32_t *value)
{
    *value = Cy_ETHIF_PhyRegRead(ETH1, regAddress, phyId);
}

void cy_eth1_phy_write(uint32_t phyId, uint32_t regAddress, uint32_t value)
{
    Cy_ETHIF_PhyRegWrite(ETH1, regAddress, value, phyId);
}

cy_rslt_t cy_eth_phy_driver_init(void)
{
    cy_rslt_t    result = CY_RSLT_SUCCESS;

    /*
     * This API is a place holder for Ethernet PHY layer initialization.
     * This needs to be called from Application.
     * All mutex, eth phy object initialization must be done here.
     *
     */

    return result;
}

cy_rslt_t cy_eth_phy_init(uint8_t eth_idx, ETH_Type *reg_base)
{
    cy_rslt_t    result = CY_RSLT_SUCCESS;
    cy_en_ephy_status_t    phy_res = CY_EPHY_SUCCESS;
    cy_eth_interface_t eth_iface = (cy_eth_interface_t)eth_idx;

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    if((eth_iface == CY_INTERFACE_ETH0) || (eth_iface == CY_INTERFACE_ETH1))
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Initializing Ethernet port pin...\n");
        if(eth_iface == CY_INTERFACE_ETH0)
        {
            cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Initializing ETH0 phy object...\n");
            phy_res = Cy_EPHY_Init(&phyObj[0], cy_eth0_phy_read, cy_eth0_phy_write);
        }
        else
        {
            cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Initializing ETH1 phy object...\n");
            phy_res = Cy_EPHY_Init(&phyObj[1], cy_eth1_phy_read, cy_eth1_phy_write);
        }
    }
    else
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid ETH interface...\n");
        return ((cy_rslt_t)(-1));
    }

    if(phy_res != CY_EPHY_SUCCESS)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Cy_EPHY_Init failed with error :  %d \n", (int)phy_res);
    }

    result = (cy_rslt_t)phy_res;
    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return result;
}

cy_rslt_t cy_eth_phy_configure(uint8_t eth_idx, uint32_t duplex, uint32_t speed)
{
    cy_rslt_t    result = CY_RSLT_SUCCESS;
    cy_en_ephy_status_t    phy_res = CY_EPHY_SUCCESS;
    cy_stc_ephy_config_t    phyConfig;

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    /* Configure PHY */
    phyConfig.speed = speed;
    phyConfig.duplex = duplex;
    phy_res = Cy_EPHY_Configure( &phyObj[eth_idx], &phyConfig );
    if(phy_res != CY_EPHY_SUCCESS)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Cy_EPHY_Configure failed with error :  %d \n", (int)phy_res);
    }

    result = (cy_rslt_t)phy_res;
    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return result;
}

cy_rslt_t cy_eth_phy_reset(uint8_t eth_idx, ETH_Type *reg_base)
{
    cy_rslt_t    result = CY_RSLT_SUCCESS;
    cy_en_ephy_status_t    phy_res = CY_EPHY_SUCCESS;

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    /* Reset the PHY */
    phy_res = Cy_EPHY_Reset(&phyObj[eth_idx]);
    phy_res |= Cy_ETHIF_PhyRegWrite(reg_base, 0x1F, 0x8000, PHY_ADDR); /* Ext-Reg CTRl: Perform a full reset, including all registers  */
    cy_rtos_delay_milliseconds(30);    /* Required delay of 30 ms to get PHY back to Run state after reset */
    if(phy_res != CY_EPHY_SUCCESS)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Cy_EPHY_Reset failed with error :  %d \n", (int)phy_res);
    }

    result = (cy_rslt_t)phy_res;
    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return result;
}

cy_rslt_t cy_eth_phy_discover(uint8_t eth_idx)
{
    cy_rslt_t    result = CY_RSLT_SUCCESS;
    cy_en_ephy_status_t    phy_res = CY_EPHY_SUCCESS;

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    phy_res = Cy_EPHY_Discover(&phyObj[eth_idx]);
    if(phy_res != CY_EPHY_SUCCESS)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Cy_EPHY_Discover failed with error :  %d \n", (int)phy_res);
    }

    result = (cy_rslt_t)phy_res;
    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return result;
}

cy_rslt_t cy_eth_phy_enable_ext_reg(ETH_Type *reg_base, uint32_t phy_speed)
{
    cy_rslt_t    result = CY_RSLT_SUCCESS;

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    result = enable_phy_DP83867IR_extended_reg(reg_base, phy_speed);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "enable_phy_DP83867IR_extended_reg failed with error :  %d \n", (int)result);
    }

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return result;
}

cy_rslt_t cy_eth_phy_get_linkstatus(uint8_t eth_idx, uint32_t *link_status)
{
    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    *link_status = Cy_EPHY_GetLinkStatus( &phyObj[eth_idx] );
    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_eth_phy_get_linkspeed(uint8_t eth_idx, uint32_t *duplex, uint32_t *speed)
{
    uint16_t    configured_hw_speed = 0;
    uint32_t    value = 0;
    cy_eth_interface_t eth_iface = (cy_eth_interface_t)eth_idx;

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    if(eth_iface == CY_INTERFACE_ETH0)
    {
        cy_eth0_phy_read( 0, REGISTER_ADDRESS_PHY_REG_BMCR, &value );
    }
    else if(eth_iface == CY_INTERFACE_ETH1)
    {
        cy_eth1_phy_read( 0, REGISTER_ADDRESS_PHY_REG_BMCR, &value );
    }
    else
    {
        return ((cy_rslt_t)(-1));
    }

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG,  "REGISTER_ADDRESS_PHY_REG_BMCR = 0x%X\n", (unsigned long)value );
    *duplex = ((value & (REGISTER_PHY_REG_DUPLEX_MASK)) == 0) ? CY_EPHY_DUPLEX_HALF : CY_EPHY_DUPLEX_FULL;
    configured_hw_speed = value & (REGISTER_PHY_REG_SPEED_MASK);

    if(configured_hw_speed == REGISTER_PHY_REG_SPEED_MASK_10M)
    {
        *speed = CY_EPHY_SPEED_10;
    }
    else if (configured_hw_speed == REGISTER_PHY_REG_SPEED_MASK_100M)
    {
        *speed = CY_EPHY_SPEED_100;
    }
    else if(configured_hw_speed ==  REGISTER_PHY_REG_SPEED_MASK_1000M)
    {
        *speed = CY_EPHY_SPEED_1000;
    }
    else
    {

    }

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_eth_phy_get_auto_neg_status(uint8_t eth_idx, uint32_t *neg_status)
{
    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    *neg_status = Cy_EPHY_GetAutoNegotiationStatus(&phyObj[eth_idx]);
    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_eth_phy_get_link_partner_cap(uint8_t eth_idx, uint32_t *duplex, uint32_t *speed)
{
    cy_rslt_t    result = CY_RSLT_SUCCESS;
    cy_en_ephy_status_t    phy_res = CY_EPHY_SUCCESS;
    cy_stc_ephy_config_t    phyConfig;

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    result = cy_eth_phy_get_linkspeed(eth_idx, duplex, speed);
    if(result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Start auto negotiation */
    phyConfig.speed = *speed;
    phyConfig.duplex = *duplex;
    phy_res = Cy_EPHY_getLinkPartnerCapabilities(&phyObj[eth_idx], &phyConfig);
    if(phy_res != CY_EPHY_SUCCESS)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Cy_EPHY_Init failed with error :  %d \n", (int)phy_res);
    }
    else
    {
        *speed = phyConfig.speed;
        *duplex = phyConfig.duplex;
    }

    result = (cy_rslt_t)phy_res;
    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return result;
}

cy_rslt_t cy_eth_phy_driver_deinit(void)
{
    cy_rslt_t    result = CY_RSLT_SUCCESS;

    /*
     * This API is a place holder for Ethernet PHY layer deinitialization.
     * This needs to be called from Application.
     * All mutex, eth phy object deinitialization must be done here.
     *
     */

    return result;
}
#endif // PHY_DEVICE_NAME == DP83867IR

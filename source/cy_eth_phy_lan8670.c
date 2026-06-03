/*
 * (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 */

/**
* @file cy_eth_phy_lan8670.c
* @brief Provides implementation of PHY operation callbacks required by ECM.
* This implementation is valid only for PHY hardware part lan8670.
*/
#include "cycfg.h"

#if defined(ETH_PHY_LAN8670)

#include "cy_eth_phy_driver.h"
#include "cy_result.h"
#include "cyabs_rtos.h"

/********************************************************/
/**************** LAN8670 Registers Map *****************/
/********************************************************/

/* MMD device addresses (Clause 45) */
#define MMD_ACCESS_CONTROL_REG      (0x0DU)
#define MMD_ACCESS_ADDR_DATA_REG    (0x0EU)
#define MMD_DATA_NO_INCR            (0x4000U)
#define MMD_PMA                     (0x01U)
#define MMD_VEND_SPECIFIC_2         (0x1FU)

/* Pad Control 3 Register */
#define PADCTRL3_REG                (0x00CBU)
#define PADCTRL3_PDRV1_POS          (8U)
#define PADCTRL3_PDRV1_MSK          (0x0300U)
#define PADCTRL3_PDRV2_POS          (10U)
#define PADCTRL3_PDRV2_MSK          (0x0C00U)
#define PADCTRL3_PDRV3_POS          (12U)
#define PADCTRL3_PDRV3_MSK          (0x3000U)
#define PADCTRL3_PDRV4_POS          (14U)
#define PADCTRL3_PDRV4_MSK          (0xC000U)
#define PADCTRL3_RESERVED_MSK       (0x00FFU)

/* 10BASE-T1S PMA Control Register */
#define T1SPMACTL_REG               (0x08F9U)
#define T1SPMACTL_MULTIDROP_EN_BIT  (1U << 10U)

/* PLCA Control 0 Register */
#define PLCA_CTRL0_REG              (0xCA01U)
#define PLCA_CTRL0_PLCA_EN_BIT      (1U << 15U)

/* PLCA Control 1 Register */
#define PLCA_CTRL1_REG              (0xCA02U)
#define PLCA_CTRL1_NODE_ID_POS      (0U)
#define PLCA_CTRL1_NODE_ID_MSK      (0x00FFU)
#define PLCA_CTRL1_NODE_COUNT_POS   (8U)
#define PLCA_CTRL1_NODE_COUNT_MSK   (0xFF00U)

/* PLCA Transmit Opportunity Timer Register */
#define PLCA_TOTMR_REG              (0xCA04U)
#define PLCA_TOTMR_POS              (0U)
#define PLCA_TOTMR_MSK              (0x00FFU)

/* PLCA Burst Mode Register */
#define PLCA_BURST_REG              (0xCA05U)
#define PLCA_BURST_BTMR_POS         (0U)
#define PLCA_BURST_BTMR_MSK         (0x00FFU)
#define PLCA_BURST_MAXBC_POS        (8U)
#define PLCA_BURST_MAXBC_MSK        (0xFF00U)


/********************************************************/
/**************** LAN8670 Config Options ****************/
/********************************************************/

#if (defined (eth_0_ENABLED) && (eth_0_ENABLED == 1u))
/** PHY Mode Selection */
#define EMAC_INTERFACE       eth_0_PHY_INTERFACE
#define PHY_ADDR             eth_0_PHY_ADDR

/** PLCA Mode Selection */
#define MULTIDROP_ENABLE                        eth_0_MULTIDROP_ENABLE
#define PLCA_MODE                               eth_0_PLCA_MODE
#define PLCA_MODE_CSMA_CD                       (0U)
#define PLCA_MODE_FOLLOWER                      (1U)
#define PLCA_MODE_COORDINATOR                   (2U)
#define PLCA_NODES_COUNT                        eth_0_PLCA_NODES_COUNT
#define PLCA_LOCAL_ID                           eth_0_PLCA_LOCAL_ID
#define PLCA_TRANSMIT_OPPORTUNITY_TIMER_100NS   eth_0_PLCA_TRANSMIT_OPPORTUNITY_TIMER_100NS
#define PLCA_MAX_BURST                          eth_0_PLCA_MAX_BURST
#define PLCA_BURST_TIMER_100NS                  eth_0_PLCA_BURST_TIMER_100NS

/** Digital Output Pad Drive Strength Selection */
#define OUTPUT_STRENGTH_GROUP1                  eth_0_OUT_STRENGTH_GROUP1
#define OUTPUT_STRENGTH_GROUP2                  eth_0_OUT_STRENGTH_GROUP2
#define OUTPUT_STRENGTH_GROUP3                  eth_0_OUT_STRENGTH_GROUP3
#define OUTPUT_STRENGTH_GROUP4                  eth_0_OUT_STRENGTH_GROUP4

#endif /* (defined (eth_0_ENABLED) && (eth_0_ENABLED == 1u)) */


/********************************************************/

static cy_stc_ephy_t phyObj;

static void LAN8670_Ext_SetAddr(ETH_Type *reg_base, uint16_t mmd, uint16_t addr)
{
    Cy_ETHIF_PhyRegWrite(reg_base, MMD_ACCESS_CONTROL_REG, mmd, PHY_ADDR);
    Cy_ETHIF_PhyRegWrite(reg_base, MMD_ACCESS_ADDR_DATA_REG, addr, PHY_ADDR);
    Cy_ETHIF_PhyRegWrite(reg_base, MMD_ACCESS_CONTROL_REG, (MMD_DATA_NO_INCR | mmd), PHY_ADDR);
}

static uint16_t LAN8670_Ext_Read(ETH_Type *reg_base, uint16_t mmd, uint16_t addr)
{
    LAN8670_Ext_SetAddr(reg_base, mmd, addr);
    return (uint16_t)Cy_ETHIF_PhyRegRead(reg_base, MMD_ACCESS_ADDR_DATA_REG, PHY_ADDR);
}

static void LAN8670_Ext_Write(ETH_Type *reg_base, uint16_t mmd, uint16_t addr, uint16_t data)
{
    LAN8670_Ext_SetAddr(reg_base, mmd, addr);
    Cy_ETHIF_PhyRegWrite(reg_base, MMD_ACCESS_ADDR_DATA_REG, data, PHY_ADDR);
}

cy_rslt_t enable_phy_lan8670_extended_reg(ETH_Type *reg_base)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Set Digital Output Pad Drive Strength for Groups 1-4 */
    uint16_t padctrl3_reserved = LAN8670_Ext_Read(reg_base, MMD_VEND_SPECIFIC_2, PADCTRL3_REG) & PADCTRL3_RESERVED_MSK;
    LAN8670_Ext_Write(reg_base, MMD_VEND_SPECIFIC_2, PADCTRL3_REG, padctrl3_reserved |
            ((OUTPUT_STRENGTH_GROUP1 << PADCTRL3_PDRV1_POS) & PADCTRL3_PDRV1_MSK) |
            ((OUTPUT_STRENGTH_GROUP2 << PADCTRL3_PDRV2_POS) & PADCTRL3_PDRV2_MSK) |
            ((OUTPUT_STRENGTH_GROUP3 << PADCTRL3_PDRV3_POS) & PADCTRL3_PDRV3_MSK) |
            ((OUTPUT_STRENGTH_GROUP4 << PADCTRL3_PDRV4_POS) & PADCTRL3_PDRV4_MSK)
        );

#if MULTIDROP_ENABLE
    /* Enable Multidrop */
    LAN8670_Ext_Write(reg_base, MMD_PMA, T1SPMACTL_REG, T1SPMACTL_MULTIDROP_EN_BIT);

    #if (PLCA_MODE == PLCA_MODE_FOLLOWER)
        /* Set Node ID (follower only) */
        LAN8670_Ext_Write(reg_base, MMD_VEND_SPECIFIC_2, PLCA_CTRL1_REG,
                (PLCA_LOCAL_ID << PLCA_CTRL1_NODE_ID_POS) & PLCA_CTRL1_NODE_ID_MSK);
    #elif (PLCA_MODE == PLCA_MODE_COORDINATOR)
        /* Set Node Count (coordinator only) */
        LAN8670_Ext_Write(reg_base, MMD_VEND_SPECIFIC_2, PLCA_CTRL1_REG,
                (PLCA_NODES_COUNT << PLCA_CTRL1_NODE_COUNT_POS) & PLCA_CTRL1_NODE_COUNT_MSK);
    #endif

    #if ((PLCA_MODE == PLCA_MODE_FOLLOWER) || (PLCA_MODE == PLCA_MODE_COORDINATOR))
        #if PLCA_MAX_BURST
            /* Set Burst Count and Burst Timer */
            LAN8670_Ext_Write(reg_base, MMD_VEND_SPECIFIC_2, PLCA_BURST_REG,
                    ((PLCA_MAX_BURST << PLCA_BURST_MAXBC_POS) & PLCA_BURST_MAXBC_MSK) |
                    ((PLCA_BURST_TIMER_100NS << PLCA_BURST_BTMR_POS) & PLCA_BURST_BTMR_MSK)
                );
        #endif
        /* Set Transmit Opportunity Timer */
        LAN8670_Ext_Write(reg_base, MMD_VEND_SPECIFIC_2, PLCA_TOTMR_REG,
                (PLCA_TRANSMIT_OPPORTUNITY_TIMER_100NS << PLCA_TOTMR_POS) & PLCA_TOTMR_MSK);
        /* Enable PLCA */
        LAN8670_Ext_Write(reg_base, MMD_VEND_SPECIFIC_2, PLCA_CTRL0_REG, PLCA_CTRL0_PLCA_EN_BIT);
    #endif
#endif

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
    CY_UNUSED_PARAMETER( reg_base );

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    if(eth_iface == CY_INTERFACE_ETH0)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Initializing Ethernet port pin...\n");
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Initializing ETH0 phy object...\n");
        phy_res = Cy_EPHY_Init(&phyObj, cy_eth0_phy_read, cy_eth0_phy_write);
        if(phy_res == CY_EPHY_SUCCESS)
        {
            Cy_EPHY_SetPhyAddr(&phyObj, PHY_ADDR);
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
    CY_UNUSED_PARAMETER( eth_idx );
    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    if (duplex != CY_EPHY_DUPLEX_HALF)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid duplex mode configuration for LAN8670 PHY. Only half duplex is supported.\n");
        return (cy_rslt_t)CY_EPHY_INVALID_DUPLEX;
    }
    else if (speed != CY_EPHY_SPEED_10)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid speed configuration for LAN8670 PHY. Only 10 Mbps is supported.\n");
        return (cy_rslt_t)CY_EPHY_INVALID_SPEED;
    }

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_eth_phy_reset(uint8_t eth_idx, ETH_Type *reg_base)
{
    cy_rslt_t    result = CY_RSLT_SUCCESS;
    cy_en_ephy_status_t    phy_res = CY_EPHY_SUCCESS;
    CY_UNUSED_PARAMETER( eth_idx );
    CY_UNUSED_PARAMETER( reg_base );

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    /* Reset the PHY */
    phy_res = Cy_EPHY_Reset(&phyObj);

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
    CY_UNUSED_PARAMETER( eth_idx );

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    phy_res = Cy_EPHY_Discover(&phyObj);
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
    CY_UNUSED_PARAMETER( phy_speed );

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    result = enable_phy_lan8670_extended_reg(reg_base);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "enable_phy_lan8670_extended_reg failed with error :  %d \n", (int)result);
    }

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return result;
}

cy_rslt_t cy_eth_phy_get_linkstatus(uint8_t eth_idx, uint32_t *link_status)
{
    CY_UNUSED_PARAMETER( eth_idx );
    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    /* The Link Status bit is always 1, but still read it to check the communication. */
    *link_status = Cy_EPHY_GetLinkStatus( &phyObj );

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_eth_phy_get_linkspeed(uint8_t eth_idx, uint32_t *duplex, uint32_t *speed)
{
    CY_UNUSED_PARAMETER( eth_idx );
    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    /* 10BASE-T1S supports only 10 Mbit/sec and half duplex */
    *speed = CY_EPHY_SPEED_10;
    *duplex = CY_EPHY_DUPLEX_HALF;

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_eth_phy_get_auto_neg_status(uint8_t eth_idx, uint32_t *neg_status)
{
    CY_UNUSED_PARAMETER( eth_idx );
    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    /* Auto-Negotiation is not supported by LAN8670. This bit is always 0. */
    *neg_status = 0;

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_eth_phy_get_link_partner_cap(uint8_t eth_idx, uint32_t *duplex, uint32_t *speed)
{
    CY_UNUSED_PARAMETER( eth_idx );
    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    /* 10BASE-T1S supports only 10 Mbit/sec and half duplex */
    *speed = CY_EPHY_SPEED_10;
    *duplex = CY_EPHY_DUPLEX_HALF;

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
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
#endif // PHY_DEVICE_NAME == LAN8670

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
* @file cy_eth_phy_rtl8211fi.c
* @brief Provides implementation of PHY operation callbacks required by ECM.
* This implementation is valid only for PHY hardware part RTL8211FI.
*/
#include "cycfg.h"

#if defined(ETH_PHY_RTL8211FI)

#include "cy_eth_phy_driver.h"
#include "cy_result.h"
#include "cyabs_rtos.h"

/********************************************************/
/************** RTL8211FI Registers Map *****************/
/********************************************************/

/* Common page-selection register. */
#define RTL8211F_REG_PAGSR              (0x1FU)     /* Page Select Register. */

/* Standard IEEE registers (page 0x0000). */
#define RTL8211F_PAGE_NORMAL            (0x0000U)
#define RTL8211F_REG_BMCR               (0x00U)     /* Basic Mode Control Register. */
#define RTL8211F_BMCR_POWER_DOWN_Msk    (1U << 11)  /* IEEE power-down mode. */

/* RGMII clock-delay controls (page 0x0D08). */
#define RTL8211F_PAGE_RGMII_DELAY       (0x0D08U)
#define RTL8211F_REG_TXDLY              (0x11U)     /* RGMII transmit clock-delay control. */
#define RTL8211F_TXDLY_EN_Msk           (0x0100U)   /* Enable PHY-side TX clock delay. */

/* PHY-specific status and control registers (page 0x0A43). */
#define RTL8211F_PAGE_PHY_CONTROL       (0x0A43U)
#define RTL8211F_REG_PHYSR              (0x1AU)     /* PHY Specific Status Register. */
#define RTL8211F_PHYSR_LINK_Msk         (1U << 2)   /* Real-time link status. */
#define RTL8211F_PHYSR_DUPLEX_Msk       (1U << 3)   /* Resolved duplex: 0 = half, 1 = full. */
#define RTL8211F_PHYSR_SPEED_Pos        (4U)        /* Resolved speed field position. */
#define RTL8211F_PHYSR_SPEED_Msk        (3U << RTL8211F_PHYSR_SPEED_Pos)
#define RTL8211F_PHYSR_SPEED_10M        (0U)        /* PHYSR speed encoding: 10 Mbps. */
#define RTL8211F_PHYSR_SPEED_100M       (1U)        /* PHYSR speed encoding: 100 Mbps. */
#define RTL8211F_PHYSR_SPEED_1000M      (2U)        /* PHYSR speed encoding: 1000 Mbps. */


/********************************************************/
/**************** RTL8211FI Config Options ***************/
/********************************************************/

#if (defined (eth_0_ENABLED) && (eth_0_ENABLED == 1u))
#define PHY_ADDR             eth_0_PHY_ADDR
#endif

/********************************************************/
static cy_stc_ephy_t phyObj;

cy_rslt_t enable_phy_rtl8211fi_extended_reg(ETH_Type *reg_base)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint32_t bmcr = 0;
    uint32_t txdly = 0;

    /* Leave IEEE power-down mode before accessing vendor-specific pages. */
    result = (cy_rslt_t)Cy_ETHIF_PhyRegWrite(reg_base, RTL8211F_REG_PAGSR, RTL8211F_PAGE_NORMAL, PHY_ADDR);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }
    bmcr = Cy_ETHIF_PhyRegRead(reg_base, RTL8211F_REG_BMCR, PHY_ADDR);
    if (bmcr == PHYREG_INVALID_VALUE)
    {
        return (cy_rslt_t)CY_EPHY_ERROR;
    }
    result = (cy_rslt_t)Cy_ETHIF_PhyRegWrite(reg_base, RTL8211F_REG_BMCR,
            (bmcr & ~RTL8211F_BMCR_POWER_DOWN_Msk), PHY_ADDR);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Enable PHY TX delay; preserve the strap-configured RX delay. */
    result = (cy_rslt_t)Cy_ETHIF_PhyRegWrite(reg_base, RTL8211F_REG_PAGSR, RTL8211F_PAGE_RGMII_DELAY, PHY_ADDR);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }
    txdly = Cy_ETHIF_PhyRegRead(reg_base, RTL8211F_REG_TXDLY, PHY_ADDR);
    if (txdly == PHYREG_INVALID_VALUE)
    {
        return (cy_rslt_t)CY_EPHY_ERROR;
    }
    result = (cy_rslt_t)Cy_ETHIF_PhyRegWrite(reg_base, RTL8211F_REG_TXDLY,
            (txdly | RTL8211F_TXDLY_EN_Msk), PHY_ADDR);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Set default page back. */
    result = (cy_rslt_t)Cy_ETHIF_PhyRegWrite(reg_base, RTL8211F_REG_PAGSR, RTL8211F_PAGE_NORMAL, PHY_ADDR);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

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
    cy_rslt_t    result = CY_RSLT_SUCCESS;
    cy_en_ephy_status_t    phy_res = CY_EPHY_SUCCESS;
    cy_stc_ephy_config_t    phyConfig;
    CY_UNUSED_PARAMETER( eth_idx );

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    /* Configure PHY */
    phyConfig.speed = speed;
    phyConfig.duplex = duplex;
    phy_res = Cy_EPHY_Configure( &phyObj, &phyConfig );
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
    CY_UNUSED_PARAMETER(phy_speed);

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);
    result = enable_phy_rtl8211fi_extended_reg(reg_base);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "enable_phy_rtl8211fi_extended_reg failed with error :  %d \n", (int)result);
    }

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return result;
}

cy_rslt_t cy_eth_phy_get_linkstatus(uint8_t eth_idx, uint32_t *link_status)
{
    uint32_t physr = 0;
    CY_UNUSED_PARAMETER( eth_idx );
    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    cy_eth0_phy_write(PHY_ADDR, RTL8211F_REG_PAGSR, RTL8211F_PAGE_PHY_CONTROL);
    cy_eth0_phy_read(PHY_ADDR, RTL8211F_REG_PHYSR, &physr);
    cy_eth0_phy_write(PHY_ADDR, RTL8211F_REG_PAGSR, RTL8211F_PAGE_NORMAL);
    *link_status = ((physr & RTL8211F_PHYSR_LINK_Msk) != 0U) ? 1U : 0U;

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_eth_phy_get_linkspeed(uint8_t eth_idx, uint32_t *duplex, uint32_t *speed)
{
    uint32_t    physr = 0;
    uint32_t    resolved_speed;
    CY_UNUSED_PARAMETER( eth_idx );

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    /* BMCR does not reflect the auto-negotiated result on this PHY - PHYSR does. */
    cy_eth0_phy_write(PHY_ADDR, RTL8211F_REG_PAGSR, RTL8211F_PAGE_PHY_CONTROL);
    cy_eth0_phy_read(PHY_ADDR, RTL8211F_REG_PHYSR, &physr);
    cy_eth0_phy_write(PHY_ADDR, RTL8211F_REG_PAGSR, RTL8211F_PAGE_NORMAL);
    *duplex = ((physr & RTL8211F_PHYSR_DUPLEX_Msk) == 0U) ? CY_EPHY_DUPLEX_HALF : CY_EPHY_DUPLEX_FULL;

    resolved_speed = (physr & RTL8211F_PHYSR_SPEED_Msk) >> RTL8211F_PHYSR_SPEED_Pos;
    if (resolved_speed == RTL8211F_PHYSR_SPEED_1000M)
    {
        *speed = CY_EPHY_SPEED_1000;
    }
    else if (resolved_speed == RTL8211F_PHYSR_SPEED_100M)
    {
        *speed = CY_EPHY_SPEED_100;
    }
    else
    {
        *speed = CY_EPHY_SPEED_10;
    }

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_eth_phy_get_auto_neg_status(uint8_t eth_idx, uint32_t *neg_status)
{
    CY_UNUSED_PARAMETER( eth_idx );
    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    *neg_status = Cy_EPHY_GetAutoNegotiationStatus(&phyObj);

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
    phy_res = Cy_EPHY_getLinkPartnerCapabilities(&phyObj, &phyConfig);
    if(phy_res != CY_EPHY_SUCCESS)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Cy_EPHY_getLinkPartnerCapabilities failed with error :  %d \n", (int)phy_res);
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
#endif // PHY_DEVICE_NAME == RTL8211FI

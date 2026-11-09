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
* @file cy_eth_phy_dp83848vv.c
* @brief Provides implementation of PHY operation callbacks required by ECM.
* This implementation is valid only for PHY hardware part DP83848VV.
*/
#include "cycfg.h"

#if defined(ETH_PHY_DP83848VV)

#include "cy_eth_phy_driver.h"
#include "cy_result.h"
#include "cyabs_rtos.h"

/********************************************************/
/**************** DP83848VV Registers Map *****************/
/********************************************************/

/* Standard IEEE registers. */
#define DP83848VV_REG_BMCR               (0x00U)     /* Basic Mode Control Register. */
#define DP83848VV_BMCR_POWER_DOWN_Msk    (1U << 11)  /* IEEE power-down mode. */

/* PHY-specific link status. */
#define DP83848VV_REG_PHYSTS             (0x10U)     /* PHY Status Register. */
#define DP83848VV_PHYSTS_LINK_Msk        (1U << 0)   /* Link status: 0 = down, 1 = up. */
#define DP83848VV_PHYSTS_DUPLEX_Msk      (1U << 2)   /* Duplex status: 0 = half, 1 = full. */
#define DP83848VV_PHYSTS_SPEED_Msk       (1U << 1)   /* Speed status: 0 = 100 Mbps, 1 = 10 Mbps. */

/* MII/RMII interface controls. */
#define DP83848VV_REG_RBR                (0x17U)     /* RMII and Bypass Register. */
#define DP83848VV_RBR_RMII_MODE_Msk      (1U << 5)   /* Interface mode: 0 = MII, 1 = RMII. */
#define DP83848VV_RBR_RMII_REV1_0_Msk    (1U << 4)   /* RMII revision: 0 = 1.2, 1 = 1.0. */

/* PHY-specific crossover control. */
#define DP83848VV_REG_PHYCR              (0x19U)     /* PHY Control Register. */
#define DP83848VV_PHYCR_MDIX_EN_Msk      (1U << 15)  /* Enable automatic MDI/MDIX crossover. */
#define DP83848VV_PHYCR_FORCE_MDIX_Msk   (1U << 14)  /* Force MDI pairs to cross. */


/********************************************************/
/**************** DP83848VV Config Options ****************/
/********************************************************/

#define EMAC_MII                     (0U)
#define EMAC_RMII                    (1U)

#if (defined (eth_0_ENABLED) && (eth_0_ENABLED == 1u))
#define PHY_ADDR             eth_0_PHY_ADDR
#define EMAC_INTERFACE       eth_0_PHY_INTERFACE
#endif

/********************************************************/
static cy_stc_ephy_t phyObj;

cy_rslt_t enable_phy_dp83848vv_extended_reg(ETH_Type *reg_base)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint32_t bmcr = 0;
    uint32_t rbr = 0;

    /* Exit PHY power-down mode through BMCR. */
    bmcr = Cy_ETHIF_PhyRegRead(reg_base, DP83848VV_REG_BMCR, PHY_ADDR);
    if (bmcr == PHYREG_INVALID_VALUE)
    {
        return (cy_rslt_t)CY_EPHY_ERROR;
    }

    result = (cy_rslt_t)Cy_ETHIF_PhyRegWrite(reg_base, DP83848VV_REG_BMCR,
            (bmcr & ~DP83848VV_BMCR_POWER_DOWN_Msk), PHY_ADDR);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Select MII or RMII through RBR, using revision 1.2 for RMII. */
    rbr = Cy_ETHIF_PhyRegRead(reg_base, DP83848VV_REG_RBR, PHY_ADDR);
    if (rbr == PHYREG_INVALID_VALUE)
    {
        return (cy_rslt_t)CY_EPHY_ERROR;
    }

    rbr &= ~(DP83848VV_RBR_RMII_MODE_Msk | DP83848VV_RBR_RMII_REV1_0_Msk);
    if (EMAC_INTERFACE == EMAC_RMII)
    {
        rbr |= DP83848VV_RBR_RMII_MODE_Msk;
    }

    result = (cy_rslt_t)Cy_ETHIF_PhyRegWrite(reg_base, DP83848VV_REG_RBR, rbr, PHY_ADDR);

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

    if ((EMAC_INTERFACE != EMAC_MII) && (EMAC_INTERFACE != EMAC_RMII))
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid PHY Interconnect...\n");
        return ((cy_rslt_t)(-1));
    }

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
    uint32_t    phycr = 0;
    CY_UNUSED_PARAMETER( eth_idx );

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    cy_eth0_phy_read(PHY_ADDR, DP83848VV_REG_PHYCR, &phycr);
    if (phycr == PHYREG_INVALID_VALUE)
    {
        return (cy_rslt_t)CY_EPHY_ERROR;
    }

    if ((speed == CY_EPHY_SPEED_AUTO) || (duplex == CY_EPHY_DUPLEX_AUTO))
    {
        phycr &= ~DP83848VV_PHYCR_FORCE_MDIX_Msk;
        phycr |= DP83848VV_PHYCR_MDIX_EN_Msk;
    }
    else
    {
        phycr &= ~DP83848VV_PHYCR_MDIX_EN_Msk;
    }

    result = (cy_rslt_t)Cy_ETHIF_PhyRegWrite(ETH0, DP83848VV_REG_PHYCR, phycr, PHY_ADDR);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

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
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_en_ephy_status_t phy_res = CY_EPHY_SUCCESS;
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
    result = enable_phy_dp83848vv_extended_reg(reg_base);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "enable_phy_dp83848vv_extended_reg failed with error :  %d \n", (int)result);
    }

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return result;
}

cy_rslt_t cy_eth_phy_get_linkstatus(uint8_t eth_idx, uint32_t *link_status)
{
    uint32_t physr = 0;
    CY_UNUSED_PARAMETER( eth_idx );
    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    cy_eth0_phy_read(PHY_ADDR, DP83848VV_REG_PHYSTS, &physr);
    if (physr == PHYREG_INVALID_VALUE)
    {
        return (cy_rslt_t)CY_EPHY_ERROR;
    }

    *link_status = ((physr & DP83848VV_PHYSTS_LINK_Msk) != 0U) ? 1U : 0U;

    cy_eth_phy_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): END \n", __FUNCTION__ );
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_eth_phy_get_linkspeed(uint8_t eth_idx, uint32_t *duplex, uint32_t *speed)
{
    uint32_t    physr = 0;
    CY_UNUSED_PARAMETER( eth_idx );

    cy_eth_phy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG1, "%s(): START \n", __FUNCTION__);

    cy_eth0_phy_read(PHY_ADDR, DP83848VV_REG_PHYSTS, &physr);
    if (physr == PHYREG_INVALID_VALUE)
    {
        return (cy_rslt_t)CY_EPHY_ERROR;
    }
    *duplex = ((physr & DP83848VV_PHYSTS_DUPLEX_Msk) == 0U) ? CY_EPHY_DUPLEX_HALF : CY_EPHY_DUPLEX_FULL;
    *speed = ((physr & DP83848VV_PHYSTS_SPEED_Msk) == 0U) ? CY_EPHY_SPEED_100 : CY_EPHY_SPEED_10;

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
#endif // PHY_DEVICE_NAME == DP83848VV

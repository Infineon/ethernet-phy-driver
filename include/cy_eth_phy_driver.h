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
 * @file cy_eth_phy_driver.h
 * @brief This file provides function declaration for phy related callbacks for different ethrnet PHY hardwares.
 */

#ifndef CY_ETH_PHY_DRIVER_H_
#define CY_ETH_PHY_DRIVER_H_

#include "cy_ethif.h"
#include "cy_ephy.h"
#include "cy_log.h"

#ifdef ENABLE_ETH_PHY_DRIVER_LOGS
#define cy_eth_phy_log_msg cy_log_msg
#else
#define cy_eth_phy_log_msg(a,b,c,...)
#endif

/**
 * \addtogroup group_eth_phy_driver_enums
 * \{
 */

/******************************************************
 *            Enumerations
 ******************************************************/

/**
 * Enumeration of Ethernet interfaces types
 */
typedef enum
{
    CY_INTERFACE_ETH0 = 0,    /**< Interface for Ethernet port 0 */
    CY_INTERFACE_ETH1,        /**< Interface for Ethernet port 1 */
    CY_INTERFACE_INVALID      /**< Invalid interface */
} cy_eth_interface_t;

/** \} group_eth_phy_driver_enums */

/**
 * \addtogroup group_eth_phy_driver_functions
 * \{
 * * The library provides the Ethernet PHY-related interface APIs as required by Ethernet connection manager library to enable the completion of Ethernet-based applications on supported platforms.
 * * All APIs are blocking APIs.
 */

/**
 * Does general allocation and initialization of resources needed for the library.
 * This API function must be called before using any other Ethernet PHY Driver API.
 *
 * \note \ref cy_eth_phy_driver_init and \ref cy_eth_phy_driver_deinit API functions are not thread-safe. The caller
 *       must ensure that these two API functions are not invoked simultaneously from different threads.
 *
 * @return CY_RSLT_SUCCESS if Ethernet PHY Driver library initialization was successful; an error code on failure.
 *
 */
cy_rslt_t cy_eth_phy_driver_init(void);

/**
 * Ethernet PHY driver Initialization. API initializes the required ethernet ports of the given PHY hardware and creates a PHY object. Subsequently, this same PHY object will be utilized to perform other ethernet PHY operations.
 *
 * @param[in]  eth_idx      : Ethernet interface to be initialized
 * @param[in]  reg_base     : Register base address of the ethernet interface
 *
 * @return CY_RSLT_SUCCESS if ethernet PHY driver initialization was successful; an error code on failure.
 *
 */
cy_rslt_t cy_eth_phy_init(uint8_t eth_idx, ETH_Type *reg_base);

/**
 * Ethernet PHY driver Configure.
 * Configures ethernet PHY with given duplex mode and PHY speed.
 *
 * @param[in]  eth_idx      : Ethernet port to be initialized
 * @param[in]  duplex       : PHY duplex mode
 * @param[in]  speed        : PHY speed
 *
 * @return CY_RSLT_SUCCESS if ethernet PHY Configure was successful; an error code on failure.
 *
 */
cy_rslt_t cy_eth_phy_configure(uint8_t eth_idx, uint32_t duplex, uint32_t speed);

/**
 * Ethernet PHY driver reset.
 * Performs a full reset of ethernet PHY, including all ethernet PHY registers.
 *
 * @param[in]  eth_idx      : Ethernet interface number
 * @param[in]  reg_base     : Register base address of the ethernet interface
 *
 * @return CY_RSLT_SUCCESS if ethernet PHY driver reset was successful; an error code on failure.
 *
 */
cy_rslt_t cy_eth_phy_reset(uint8_t eth_idx, ETH_Type *reg_base);

/**
 * Ethernet PHY driver discover.
 * Discovers connected ethernet PHY at address zero.
 *
 * @param[in]  eth_idx      : Ethernet interface number
 *
 * @return CY_RSLT_SUCCESS if ethernet PHY driver discover was successful; an error code on failure.
 *
 */
cy_rslt_t cy_eth_phy_discover(uint8_t eth_idx);

/**
 * Enable extended Ethernet PHY driver Registers.
 * For the selected ethernet PHY chip, this API configures and enables extended registers according to the type of PHY interface and the PHY speed.
 *
 * @param[in]  reg_base     : Register base address of the ethernet interface
 * @param[in]  phy_speed    : PHY speed
 *
 * @return CY_RSLT_SUCCESS if ethernet PHY driver enabling extended registors was successful; an error code on failure.
 *
 */
cy_rslt_t cy_eth_phy_enable_ext_reg(ETH_Type *reg_base, uint32_t phy_speed);

/**
 * Ethernet PHY driver get link speed.
 * Gets the configured PHY duplex mode and PHY speed of selected ethernet interface.
 *
 * @param[in]  eth_idx      : Ethernet interface number
 * @param[out]  duplex      : PHY duplex mode
 * @param[out]  speed       : PHY speed
 *
 * @return CY_RSLT_SUCCESS if ethernet PHY driver get link speed was successful; an error code on failure.
 *
 */
cy_rslt_t cy_eth_phy_get_linkspeed(uint8_t eth_idx, uint32_t *duplex, uint32_t *speed);

/**
 * Ethernet PHY driver get link status.
 * Gets the ethernet PHY link status of selected ethernet interface.
 *
 * @param[in]  eth_idx      : Ethernet interface number
 * @param[out]  link_status : PHY duplex mode
 *
 * @return CY_RSLT_SUCCESS if ethernet PHY driver get link status was successful; an error code on failure.
 *
 */
cy_rslt_t cy_eth_phy_get_linkstatus(uint8_t eth_idx, uint32_t *link_status);

/**
 * Get Ethernet PHY Auto Negotiation Status.
 * Gets the current Auto-Negotiation status (completed or In-progress) of selected ethernet interface.
 *
 * @param[in]  eth_idx      : Ethernet interface number
 * @param[out]  neg_status  : PHY duplex mode
 *
 * @return CY_RSLT_SUCCESS if ethernet PHY driver reset was successful; an error code on failure.
 *
 */
cy_rslt_t cy_eth_phy_get_auto_neg_status(uint8_t eth_idx, uint32_t *neg_status);

/**
 * Get Ethernet PHY Link Partner Capabilities.
 * Gets the Link partner capabilities (speed and duplex) of selected ethernet interface.
 *
 * @param[in]  eth_idx      : Ethernet interface number
 * @param[out]  duplex      : PHY duplex mode
 * @param[out]  speed       : PHY speed
 *
 * @return CY_RSLT_SUCCESS if ethernet PHY driver get link partner capabilities was successful; an error code on failure.
 *
 */
cy_rslt_t cy_eth_phy_get_link_partner_cap(uint8_t eth_idx, uint32_t *duplex, uint32_t *speed);

/**
 * Releases the resources allocated in the \ref cy_eth_phy_driver_init function.
 *
 * \note \ref cy_eth_phy_driver_init and \ref cy_eth_phy_driver_deinit API functions are not thread-safe. The caller
 *       must ensure that these two API functions are not invoked simultaneously from different threads.
 *
 * @return CY_RSLT_SUCCESS if Ethernet PHY Driver library deinitialization was successful; an error code on failure.
 */
cy_rslt_t cy_eth_phy_driver_deinit(void);

/** \} group_eth_phy_driver_functions */

#endif /* CY_ETH_PHY_DRIVER_H_ */

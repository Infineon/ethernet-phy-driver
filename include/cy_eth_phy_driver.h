/*
 * (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
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

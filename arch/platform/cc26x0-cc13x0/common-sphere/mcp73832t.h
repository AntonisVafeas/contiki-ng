/*
 * mcp73832t.h
 *
 *  Created on: May 2, 2019
 *      Author: antonisvafeas
 */

#ifndef ARCH_PLATFORM_SRF06_CC26XX_COMMON_SPHERE_MCP73832T_H_
#define ARCH_PLATFORM_SRF06_CC26XX_COMMON_SPHERE_MCP73832T_H_

//
#include <stdio.h>
#include "dev/gpio-hal.h"
/**
 * \brief Initialises the battery charging  status pin
 *
 */
void mcp73832t_init(void);
/**
 * \brief Configures the interrupt handler to the status pin
 * \param *conf   : configuration provided by main file.
 * \return True when successful.
 * Mode                 	| Pin status
 * SHUTDOWN MODE 			| Hi-Z
 * PRECONDITIONING MODE		| LOW
 * FAST CHARGE MODE			| LOW
 * CONSTANT VOLTAGE MODE	| LOW
 * CHARGE COMPLETE MODE		| HIGH (MCP73831) Hi-Z (MCP73832)
 * Thus MCP73832 must be used not to pull the pin to VBAT
 * The interrupt will trigger at the start of charging
 *
 */
void mcp73832t_configure_interrupt(gpio_hal_callback_t callback_ptr);
/**
 * \brief return charging condition
 *
 */
uint8_t mcp73832t_is_charging(void);


#endif /* ARCH_PLATFORM_SRF06_CC26XX_COMMON_SPHERE_MCP73832T_H_ */

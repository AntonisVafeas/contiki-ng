/*
 * Copyright (c) 2017, University of Bristol - http://www.bristol.ac.uk/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup sphere
 * @{
 *
 * Defines related to the SPHERE SPG-2
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 *
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the SPES-2
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_H_
#define BOARD_H_
/*---------------------------------------------------------------------------*/
#include "ioc.h"
/*---------------------------------------------------------------------------*/
extern uint8_t is_g_side;
#define BOARD_IOID_FG          IOID_15
/*---------------------------------------------------------------------------*/
extern uint8_t *ble_address;
/*---------------------------------------------------------------------------*/
/**
 * \name LED HAL configuration
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define LEDS_CONF_COUNT                 2
#define LEDS_CONF_RED                   1
#define LEDS_CONF_GREEN                 2
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_4
#define BOARD_IOID_LED_2          IOID_7
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name UART IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_UART_RX        IOID_2
#define BOARD_IOID_UART_TX        IOID_3

#define BOARD_IOID_UART_CTS       IOID_UNUSED
#define BOARD_IOID_UART_RTS       IOID_UNUSED
#define BOARD_UART_RX             (1 << BOARD_IOID_UART_RX)
#define BOARD_UART_TX             (1 << BOARD_IOID_UART_TX)
#define BOARD_UART_CTS            (1 << BOARD_IOID_UART_CTS)
#define BOARD_UART_RTS            (1 << BOARD_IOID_UART_RTS)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Button IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_BUTTON       IOID_23
#ifdef BOARD_CONF_IOID_KEY_SELECT
#define BOARD_IOID_KEY_SELECT BOARD_CONF_IOID_KEY_SELECT
#else
#define BOARD_IOID_KEY_SELECT	BOARD_IOID_BUTTON
#endif
#define BOARD_BUTTON            (1 << BOARD_IOID_BUTTON)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name External flash IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define EXT_FLASH_SPI_CONTROLLER    SPI_CONTROLLER_SPI1

#define EXT_FLASH_SPI_PIN_SCK       IOID_20
#define EXT_FLASH_SPI_PIN_MOSI      IOID_19
#define EXT_FLASH_SPI_PIN_MISO      IOID_18
#define EXT_FLASH_SPI_PIN_CS        IOID_14

#define EXT_FLASH_DEVICE_ID         0x17
#define EXT_FLASH_MID               0xC2

#define EXT_FLASH_PROGRAM_PAGE_SIZE 256
#define EXT_FLASH_ERASE_SECTOR_SIZE 4096
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief I2C IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_SDA            IOID_5 /**< Interface 0 SDA: All sensors bar MPU */
#define BOARD_IOID_SCL            IOID_6 /**< Interface 0 SCL: All sensors bar MPU */
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Board audio sensor (microphone) IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_AUDIO_PWR      IOID_13
#define BOARD_IOID_AUDIO_CLK      IOID_24
#define BOARD_IOID_AUDIO_DI       IOID_1
#define BOARD_AUDIO_PWR           (1 << BOARD_IOID_AUDIO_PWR)
#define BOARD_AUDIO_CLK           (1 << BOARD_IOID_AUDIO_CLK)
#define BOARD_AUDIO_DI            (1 << BOARD_IOID_AUDIO_DI)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Board hardware watchdog IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_HAS_HW_WATCHDOG          1
#define BOARD_IOID_HW_WATCHDOG_WAKE    IOID_22
#define BOARD_IOID_HW_WATCHDOG_DONE    IOID_21
#define BOARD_HW_WATCHDOG_WAKE         (1 << BOARD_IOID_HW_WATCHDOG_WAKE)
#define BOARD_HW_WATCHDOG_DONE         (1 << BOARD_IOID_HW_WATCHDOG_DONE)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief ROM bootloader configuration
 *
 * Change CCXXWARE_CONF_BL_PIN_NUMBER to BOARD_IOID_KEY_xyz to select which
 * button triggers the bootloader on reset. Use CCXXWARE_CONF_BL_LEVEL to
 * control the pin level that enables the bootloader (0: low, 1: high). It is
 * also possible to use any other externally-controlled DIO.
 * @{
 */
#define CCXXWARE_CONF_BL_PIN_NUMBER   BOARD_IOID_KEY_SELECT
#define CCXXWARE_CONF_BL_LEVEL        0
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "SPHERE SPG-2"

/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */

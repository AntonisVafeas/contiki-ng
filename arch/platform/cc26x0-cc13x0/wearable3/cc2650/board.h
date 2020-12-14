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
 * Defines related to the SPHERE Wearable 3.
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 *
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the
 * SPHERE Wearable 3.
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
/**
 * \name Custom BLE Address
 * @{
 */
/*extern uint8_t ble_address[]; */
extern uint8_t *ble_address;
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
/* Board PCB version multiplied by 10 */
#define BOARD_VERSION   14

#define BOARD_STRING          "Wearable3 v2.1"
/*---------------------------------------------------------------------------*/
/**
 * \name LED HAL configuration
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define LEDS_CONF_COUNT                 1
#define LEDS_CONF_RED                   1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name UART IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_UART_RX_USB   IOID_2
#define BOARD_IOID_UART_RX        IOID_11 /*IOID_2 */
#define BOARD_IOID_UART_TX        IOID_3
#define BOARD_IOID_UART_CTS       IOID_UNUSED
#define BOARD_IOID_UART_RTS       IOID_UNUSED
#define BOARD_UART_RX             (1 << BOARD_IOID_UART_RX)
#define BOARD_UART_TX             (1 << BOARD_IOID_UART_TX)
#define BOARD_UART_CTS            (1 << BOARD_IOID_UART_CTS)
#define BOARD_UART_RTS            (1 << BOARD_IOID_UART_RTS)
/** @} */
/*---------------------------------------------------------------------------*/
/*Requires hardware revision */
#define BOARD_IOID_CHARGE_STATUS       IOID_4
/*---------------------------------------------------------------------------*/
/**
 * \name Button IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_BUTTON          IOID_24
#ifdef BOARD_CONF_IOID_KEY_SELECT
#define BOARD_IOID_KEY_SELECT BOARD_CONF_IOID_KEY_SELECT
#else
#define BOARD_IOID_KEY_SELECT      BOARD_IOID_BUTTON
#endif
#define BOARD_BUTTON               (1 << BOARD_IOID_BUTTON)
/*issue with cc26x0 demos */
#define BOARD_BUTTON_HAL_INDEX_KEY_LEFT BOARD_BUTTON
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name MC3672 IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_MC3672_SPI              SPI_CONTROLLER_SPI0

#define BOARD_IOID_MC3672_INT         IOID_25
#define BOARD_IOID_MC3672_SCK         IOID_27
#define BOARD_IOID_MC3672_MOSI        IOID_26
#define BOARD_IOID_MC3672_MISO        IOID_29
#define BOARD_MC3672_INT              (1 << BOARD_IOID_MC3672_INT)
#define BOARD_MC3672_SCK              (1 << BOARD_IOID_MC3672_SCK)
#define BOARD_MC3672_MOSI             (1 << BOARD_IOID_MC3672_MOSI)
#define BOARD_MC3672_MISO             (1 << BOARD_IOID_MC3672_MISO)

#define BOARD_IOID_MC3672_CS          IOID_28
#define BOARD_MC3672_CS               (1 << BOARD_IOID_MC3672_CS)

#define BOARD_IOID_MC3672_POWER       IOID_22
#define BOARD_MC3672_POWER            (1 << BOARD_IOID_MC3672_POWER)

/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name External flash IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define EXT_FLASH_SPI_CONTROLLER    SPI_CONTROLLER_SPI1

#define EXT_FLASH_SPI_PIN_SCK       IOID_7
#define EXT_FLASH_SPI_PIN_MOSI      IOID_8
#define EXT_FLASH_SPI_PIN_MISO      IOID_14
#define EXT_FLASH_SPI_PIN_CS        IOID_15

#define EXT_FLASH_DEVICE_ID         0x17
#define EXT_FLASH_MID               0xC2

#define EXT_FLASH_PROGRAM_PAGE_SIZE 256
#define EXT_FLASH_ERASE_SECTOR_SIZE 4096
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name TPS62746 IOD mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_TPS62746_ADC_CHANNEL  ADC_COMPB_IN_AUXIO7
#define BOARD_IOID_TPS62746_CTRL    IOID_21
#define BOARD_IOID_TPS62746_VINSW   IOID_23

#define BOARD_TPS62746_CTRL         (1 << BOARD_IOID_TPS62746_CTRL)
#define BOARD_TPS62746_VINSW        (1 << BOARD_IOID_TPS62746_VINSW)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name I2C IOD mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_SDA              IOID_17 /**< Interface 0 SDA: All sensors bar MPU */
#define BOARD_IOID_SCL              IOID_18 /**< Interface 0 SCL: All sensors bar MPU */
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name ICM20948 IOD mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_ICM20948_POWER   IOID_10
#define BOARD_IOID_ICM20948_INT1    IOID_16
#define BOARD_IOID_ICM20948_FSYNC   IOID_13
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name HRMonitor IOD mapping (? HR monitor)
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_HR_POWER         IOID_9
#define BOARD_IOID_HR_TX            IOID_11
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name OLED IOD mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_OLED_POWER       IOID_20
#define BOARD_IOID_OLED_RESET       IOID_19
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Buzzer IOD mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_BUZZER           IOID_12
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Status Pin of mcp73832t lipo charger IC
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_MCP73832T_STATUS     IOID_4
#define BOARD_MCP73832T_STATUS          (1 << BOARD_IOID_MCP73832T_STATUS)
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
#define CCXXWARE_CONF_BL_PIN_NUMBER   0x03
#define CCXXWARE_CONF_BL_LEVEL        0
/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */

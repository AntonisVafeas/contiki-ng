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
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_
/*---------------------------------------------------------------------------*/
/* Enable the ROM bootloader */
#define ROM_BOOTLOADER_ENABLE           1
/*---------------------------------------------------------------------------*/
#define CC26XX_UART_CONF_BAUD_RATE      115200
/*---------------------------------------------------------------------------*/
/* Used by te BLE stack */
#define BLE_ADDRESS_PTR                 ble_address
/*---------------------------------------------------------------------------*/
#define NETSTACK_CONF_RADIO             ble_cc2650_driver
#define LOG_CONF_LEVEL_MAC              LOG_LEVEL_ERR
/*---------------------------------------------------------------------------*/
#define CC26XX_BLE_ADDRESS_MODE         1
/*---------------------------------------------------------------------------*/
/* Do not receive any advertisements */
#define RF_BLE_CONF_ENABLED             1
#define BLE_CONF_WITH_ADV_RX            1
#define BLE_WITH_SCANNER                1
#define BLE_CONF_WITH_CALLBACK          0
/*---------------------------------------------------------------------------*/
#endif /* PROJECT_CONF_H_ */
/*---------------------------------------------------------------------------*/
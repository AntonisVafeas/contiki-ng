/*
 * Copyright (c) 2018, University of Bristol - http://www.bristol.ac.uk/
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
/**
 *
 * \author Atis Elsts,Antonis Vafeas
 * \brief Demo Example of a BLE Scanner.
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "dev/ble-hal.h"
#include "net/netstack.h"

#include "ble-addr.h"

#include "cc26xx-aes.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include <math.h>
/*---------------------------------------------------------------------------*/
#define SPHERE_LOOP           (CLOCK_SECOND * 10000)
/*---------------------------------------------------------------------------*/
#define SCAN_INTERVAL_MS     0
#define SCAN_WINDOW_MS       15000
/*---------------------------------------------------------------------------*/
#define FILTER_BLE_ADDRESS_0 { 0xa0, 0xe6, 0xf8, 0x00, 0x00, 0xC0 }
#define FILTER_BLE_ADDRESS_1 { 0xa0, 0xe6, 0xf8, 0x00, 0x00, 0xC1 }
#define FILTER_BLE_ADDRESS_2 { 0xa0, 0xe6, 0xf8, 0x00, 0x00, 0xC2 }
#define FILTER_BLE_ADDRESS_3 { 0xa0, 0xe6, 0xf8, 0x00, 0x00, 0xC3 }
#define FILTER_BLE_ADDRESS_4 { 0xa0, 0xe6, 0xf8, 0x00, 0x00, 0xC4 }
#define FILTER_BLE_ADDRESS_5 { 0xa0, 0xe6, 0xf8, 0x00, 0x00, 0xC5 }
#define FILTER_BLE_ADDRESS_6 { 0xa0, 0xe6, 0xf8, 0x00, 0x00, 0xC6 }
#define FILTER_BLE_ADDRESS_7 { 0xa0, 0xe6, 0xf8, 0x00, 0x00, 0xC7 }
#define ADDRESS_WHITELIST_CNT 8
/* By default, load the key of HID 1234 (Playground NUC) */
static uint8_t address_whitelist[ADDRESS_WHITELIST_CNT][6] = {
  FILTER_BLE_ADDRESS_0, FILTER_BLE_ADDRESS_0,
  FILTER_BLE_ADDRESS_1, FILTER_BLE_ADDRESS_2,
  FILTER_BLE_ADDRESS_3, FILTER_BLE_ADDRESS_4,
  FILTER_BLE_ADDRESS_5, FILTER_BLE_ADDRESS_6,
};
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* This is a globally exported symbol */
uint8_t *ble_address = address_whitelist[0];
/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS(sphere_process, "sphere_process");
AUTOSTART_PROCESSES(&sphere_process);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t
init_scan_req_data(uint8_t *buf)
{
  memset(buf, 0x00, BLE_SCAN_REQ_DATA_LEN);
  uint8_t ble_addr[BLE_ADDR_SIZE];
  ble_addr_cpy_to(ble_addr);
  /* own address */
  memcpy(buf, ble_addr, BLE_ADDR_SIZE);
  /* remote address */
  memcpy(buf + BLE_ADDR_SIZE, address_whitelist[1], BLE_ADDR_SIZE);
  return BLE_SCAN_REQ_DATA_LEN;
}
/*---------------------------------------------------------------------------*/
static void
start_scanner(void)
{
  int i, j;
  uint8_t scan_req_data[BLE_SCAN_REQ_DATA_LEN];
  uint8_t scan_req_data_len;
  /* set parameters */
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_SCAN_INTERVAL, SCAN_INTERVAL_MS);
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_SCAN_WINDOW, SCAN_WINDOW_MS);
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_SCAN_TYPE, BLE_SCAN_ACTIVE);
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_SCAN_OWN_ADDR_TYPE, BLE_ADDR_TYPE_PUBLIC);

  scan_req_data_len = init_scan_req_data(scan_req_data);

  NETSTACK_RADIO.set_object(RADIO_PARAM_BLE_SCAN_REQUEST, scan_req_data, scan_req_data_len);

  for(i = 0; i < ADDRESS_WHITELIST_CNT; i++) {
    for(j = 0; j < 6; ++j) {
      printf(" 0x%02x", address_whitelist[i][j]);
    }
    printf("\n");
    NETSTACK_RADIO.set_object(RADIO_PARAM_BLE_SCAN_FLTR, address_whitelist[i], 6);
  }

  /* enable scanner */
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_SCAN_ENABLE, 1);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sphere_process, ev, data)
{
  PROCESS_BEGIN();

  printf("[SPG-2] SPHERE BLE Scanner Example\n");
  /* Initialize the BLE controller */
  start_scanner();
  printf("BLE scanner started\n");

  etimer_set(&et, SPHERE_LOOP);

  while(1) {
    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER && data == &et) {
      printf("got timer event\n");
      etimer_reset(&et);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if BLE_CONF_WITH_SCANNER_CALLBACK
#define ADV_DATA_LEN 18
typedef struct {
  uint8_t header_length; /* Header Length (should be 2 including flags) */
  uint8_t gap_adtype; /* BLE GAP Advertisement Data Types */
  uint8_t gap_mode; /* BLE GAP Flags Discovery Modes */
  uint8_t uuid_length; /* UUID Length (should be 3 including flags) */
  uint8_t uuid_flags; /* UUID Flags */
  uint8_t uuid_lo; /* UUID Lower Byte */
  uint8_t uuid_hi; /* UUID Higher Byte */
  uint8_t monitor[4]; /* Monitoring Data */
  uint8_t mc; /* ADV Counter (LSB) */
  uint8_t data[ADV_DATA_LEN]; /* Acceleration Data */
  uint8_t mc_msb; /* ADV Counter (MSB) */
} ble_frame_adv_t;
void
ble_scanner_callback(const uint8_t *rx_data, uint16_t length, int rssi,
                     uint8_t channel, uint32_t timestamp)
{
  int8_t x, y, z;
  uint8_t offset = 2;
  ble_frame_adv_t *adv = (ble_frame_adv_t *)(rx_data + offset + BLE_ADDR_SIZE);
  printf("length:%d rssi:%d channel:%d data: 0x", length, rssi, channel);
  printf("Addr:");
  for(size_t i = 0; i < BLE_ADDR_SIZE; i++) {
    printf("%02x", rx_data[offset + i]);
  }
  cc26xx_aes_decrypt(adv->data + 2);
  cc26xx_aes_decrypt(adv->data);
  printf(" data:");
  for(int j = 0; j < 6; ++j) {
    x = adv->data[j * 3];
    y = adv->data[1 + (j * 3)];
    z = adv->data[2 + (j * 3)];
    printf("g:%d ", (uint16_t)(sqrt(x * x + y * y + z * z)) << 5);
  }

  printf(" mc_msb:%d mc:%d", adv->mc_msb, adv->mc);
  printf("\n");
}
#endif

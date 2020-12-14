/*
 * Copyright (c) 2020, University of Bristol - http://www.bristol.ac.uk/
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
 * \brief
 *        This example replicates the functionality of the SPHERE Wearable 3
 *        deployments for Wearable3 with some minor additions:
 *        bidirectional communication, screen, wall-clock time, etc.
 * \author
 *        Xenofon (Fontas) Fafoutis <xenofon.fafoutis@bristol.ac.uk>
 *        Alex Hamilton <ah14128@my.bristol.ac.uk>
 *        Atis Elsts <atis.elsts@bristol.ac.uk>
 *        Antonis Vafeas <antonis.vafeas@bristol.ac.uk>
 */
#include "board-peripherals.h"
#include "cc26xx-aes.h"
#include "contiki.h"
#include "dev/ble-hal.h"
#include "dev/button-hal.h"
#include "dev/cc26xx-uart.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "rf-core/rf-ble.h"
#include "sys/energest.h"
#include "ti-lib.h"

#include "sys/int-master.h"
#include "lib/random.h"

#include <stdio.h>

#if BOARD_WEARABLE3
/* Definitions for intellisense */
#include "mc3672.h"
#include "ble-adv-def.h"
#endif /* BOARD_WEARABLE3 */

#if 1
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* Monitoring data: software version */
#define W3_SOFTWARE_VERSION 1
/*---------------------------------------------------------------------------*/
#define SPHERE_LOOP CLOCK_SECOND * 100
#define CHARGING_LOOP CLOCK_SECOND * 86400 * 10
/*---------------------------------------------------------------------------*/
#define W3_ADV_PAYLOAD_SIZE 24
/*---------------------------------------------------------------------------*/
/* Timer to reboot after error detected */
#define W3_ERRTIME_REBOOT_SEC 120
/* Error Codes in monitoring data */
#define W3_ERRCODE_NOERROR 0x00     /* No error */
#define W3_ERRCODE_MC3672_INIT 0x01 /* Accelerometer failed to init */
#define W3_ERRCODE_MC3672_OPEN 0x02 /* Accelerometer failed to open SPI */
#define W3_ERRCODE_FLASH 0x04       /* Flash failed to init */
/*---------------------------------------------------------------------------*/
static struct etimer et;
static struct etimer et_charger;
/*---------------------------------------------------------------------------*/
#define SCAN_INTERVAL_MS 200
#define SCAN_WINDOW_MS 200

/*---------------------------------------------------------------------------*/
/*static uint32_t last_time_clock_updated_sec; */

/* assume the clock stays within bounds for 10 days */
#define CLOCK_FRESH_TIME_SEC (3600 * 24 * 10)

/* try to update the clock and config at least once per 12 hours */
#define CLOCK_RECENT_TIME_SEC (3600 * 12)

/*---------------------------------------------------------------------------*/

/* In beats per minute. Requires 9600 serial baudrate to work. */
/*static unsigned char heartrate_sensor_value; */
/* Status 0: ok, status nonzero: a problem */
static uint8_t accelerometer_status = 1;
/*---------------------------------------------------------------------------*/

static bool print_monitoring_data = true;

/* BLE related variables*/
#define DEFAULT_BLE_ADDRESS \
  { \
    0xa0, 0xe6, 0xf8, 0x00, 0x00, 0x0c0 \
  }
#define FILTER_BLE_ADDRESS_0 \
  { \
    0xa0, 0xe6, 0xf8, 0x00, 0x00, 0x0c0 \
  }
#define ADDRESS_WHITELIST_CNT 1
/*
   static uint8_t address_whitelist[ADDRESS_WHITELIST_CNT][6] = {
   FILTER_BLE_ADDRESS_0
   };
 */
#define BLE_DEVICE_NAME "SPHERE W3 "

static uint8_t custom_ble_address[6] = DEFAULT_BLE_ADDRESS;

/* This is a globally exported symbol */
uint8_t *ble_address = custom_ble_address;

static uint16_t num_accel_interrupts;

/*---------------------------------------------------------------------------*/

/* BLE Advertisement-related macros */
#define BLE_ADV_TYPE_DEVINFO 0x01
#define BLE_ADV_PAYLOAD_BUF_LEN 31
#define BLE_ADV_PAYLOAD_DATA_LEN 24

#define BLE_HEADER_SIZE 14
#define BLE_FOOTER_SIZE 3 /* CRC */

/* Scan request */
/*BLE scanning address + advertising address + payload + footer crc */

#define BLE_EXPECTED_GW_PAYLOAD_SIZE sizeof(sphere_single_wearable_config_t)
#define BLE_EXPECTED_GW_PACKET_SIZE (BLE_HEADER_SIZE + BLE_EXPECTED_GW_PAYLOAD_SIZE + BLE_FOOTER_SIZE)

/* For SPHERE use 0xFFF0 */
#define BLE_ADV_UUID_HI 0xFF
#define BLE_ADV_UUID_LO 0xF0

#define BLE_ADV_MAC_CHANNEL BLE_ADV_CHANNEL_ALL

/*---------------------------------------------------------------------------*/
/*static uint8_t adv_payload[W3_ADV_PAYLOAD_SIZE]; */

/* Static variables for monitoring data */
static uint8_t errcode = W3_ERRCODE_NOERROR; /* Error code */
static uint16_t vbat;                        /* Battery voltage */
static uint16_t vdd;                         /* CPU voltage */
static uint8_t temp;                         /* CPU temperature */
static uint8_t accel_status;                 /* Accelerometer status register */
static uint8_t accel_filter;                 /* Accelerometer filter register (sampling frequency) */

static uint32_t errtime; /* Time when the error code was first set */

/*---------------------------------------------------------------------------*/
PROCESS(sphere_process, "sphere_process ");

#if BOARD_WEARABLE3
PROCESS(charging_process, "Charging Events ");
#endif /* BOARD_WEARABLE3 */

AUTOSTART_PROCESSES(&sphere_process);

#if BOARD_WEARABLE3

static void accelerometer_interrupt_handler(gpio_hal_pin_mask_t mask);

/*---------------------------------------------------------------------------*/
/* Configuration options for the MC3672, see README for more info on each option*/
static mc3672_configuration_t
  mc3672_config =
{
  .g_range = MC3672_RANGE_4G,
  .data_resolution = MC3672_RESOLUTION_8BIT,
  .modes = MC3672_MODE_CWAKE,

  /*FIFO Settings*/
  .fifo_enable = ENABLE,
  .fifo_control = MC3672_FIFO_CONTROL_ENABLE,
  .fifo_thres = 6,

  /*SNIFF settings*/
  .sniff_power = MC3672_ULTRALOWPOWER,
  .sniff_sr = MC3672_SNIFF_SR_7Hz,
  .sniff_thres_x = 10,
  .sniff_thres_y = 1,
  .sniff_thres_z = 1,
  .sniff_count_x = 3,
  .sniff_count_y = 1,
  .sniff_count_z = 1,
  .sniff_mode = MC3672_SNIFF_C2B,
  .sniff_and_or = MC3672_SNIFF_AND,

  /*Wake settings*/
  .wake_sr = MC3672_CWAKE_SR_28Hz,
  .wake_power = MC3672_LOWPOWER,

  /*Interrupt settings*/
  .int_wake = DISABLE,
  .int_ACQ = DISABLE,
  .int_fifo_empty = DISABLE,
  .int_fifo_full = DISABLE,
  .int_fifo_thres = ENABLE,
  .int_swake = DISABLE,
  .interrupt_handler = accelerometer_interrupt_handler,

  /*Offset settings*/
  .offset_x = 0,
  .offset_y = 0,
  .offset_z = 0
};
/*---------------------------------------------------------------------------*/
static void
mcp73832t_interrupt_handler(gpio_hal_pin_mask_t mask)
{
  if(mask & BOARD_MCP73832T_STATUS) {
    process_poll(&charging_process);
  }
}
/*---------------------------------------------------------------------------*/
static void
accelerometer_interrupt_handler(gpio_hal_pin_mask_t mask)
{
  if(mask & BOARD_MC3672_INT) {
    process_poll(&sphere_process);
  }
}
/*---------------------------------------------------------------------------*/
static void
set_errcode(uint8_t error)
{
  errcode |= error;
  if(errtime == 0) {
    errtime = clock_seconds();
  }
}
/*---------------------------------------------------------------------------*/
static void
update_monitoring_data(void)
{
  vbat = tps62746_value();
  vdd = (uint16_t)batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
  temp = (uint8_t)batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
  accel_status = mc3672_get_register(MC3672_REG_STATUS_1);
  accel_filter = mc3672_config.wake_sr;
}
/*---------------------------------------------------------------------------*/
static void
print_monitoring_report(void)
{
#if 1
  PRINTF("Monitoring Data %s \n ", BOARD_STRING);
  PRINTF("  version = %d \n ", W3_SOFTWARE_VERSION);
  PRINTF("  uptime = %lu seconds \n ", clock_seconds());
  PRINTF("  filter = 0x %02x \n ", accel_filter);
  PRINTF("  status = 0x %02x \n ", accel_status);
  PRINTF("  errcode = 0x %02x \n ", errcode);
  PRINTF("  temp = %d oC \n ", temp);
  PRINTF("  vdd = %d mV \n ", vdd);
  PRINTF("  vbat = %d mv charging: %u ", vbat, mcp73832t_is_charging());
#endif
}
/*---------------------------------------------------------------------------*/
#endif /* BOARD_WEARABLE3 */

void
ble_adv_callback(uint8_t *rx_data, uint16_t length, int rssi)
{
  int i;
  PRINTF("rx %u bytes rssi = %d ", length, rssi);
  for(i = 0; i < BLE_HEADER_SIZE; ++i) {
    PRINTF(" %02x ", rx_data[i]);
  }
  for(i = BLE_HEADER_SIZE; i < length - BLE_FOOTER_SIZE; ++i) {
    PRINTF(" %02x ", rx_data[i]);
  }
  putchar('\n');
}
/*---------------------------------------------------------------------------*/
/* public device address of BLE controller */
static uint8_t ble_addr[BLE_ADDR_SIZE];
#define BLE_ADV_INTERVAL 200
/*---------------------------------------------------------------------------*/
static uint8_t
init_scan_resp_data(char *scan_resp_data)
{
  uint8_t scan_resp_data_len = 0;
  memset(scan_resp_data, 0x00, BLE_SCAN_RESP_DATA_LEN);
  /* complete device name */
  scan_resp_data[scan_resp_data_len++] = 1 + strlen(BLE_DEVICE_NAME);
  scan_resp_data[scan_resp_data_len++] = 0x09;
  memcpy(&scan_resp_data[scan_resp_data_len],
         BLE_DEVICE_NAME, strlen(BLE_DEVICE_NAME));
  scan_resp_data_len += strlen(BLE_DEVICE_NAME);

  return scan_resp_data_len;
}
/*---------------------------------------------------------------------------*/
static uint8_t
set_adv_packet_data(char *adv_data)
{
  memset(adv_data, 0x00, BLE_ADV_DATA_LEN);
  ble_frame_adv_t *adv;
  adv = (ble_frame_adv_t *)adv_data;
  adv->header_length = sizeof(adv->gap_adtype) + sizeof(adv->gap_mode);
  adv->gap_adtype = BLE_ADV_TYPE_DEVINFO;
  adv->gap_mode = 0x06; /* LE general discoverable + BR/EDR not supported */
  adv->uuid_length = sizeof(adv->uuid_flags) + sizeof(adv->uuid_lo) + sizeof(adv->uuid_hi);
  adv->uuid_flags = 0x02;
  adv->uuid_lo = BLE_ADV_UUID_LO;
  adv->uuid_hi = BLE_ADV_UUID_HI;
  return 7;
}
/*---------------------------------------------------------------------------*/
#if BOARD_WEARABLE3
static uint8_t
update_adv_packet_data(char *adv_data)
{
  ble_frame_adv_t *adv;
  adv = (ble_frame_adv_t *)adv_data;
  adv->mc++;
  /*fix message generated on linux hci */
  adv->uuid_length = 27;
  if(adv->mc == 0) {
    adv->mc_msb++;
  }
  update_monitoring_data();
  switch(adv->mc % 4) {
  case 0: /* Uptime */
  {
    uint32_t seconds = clock_seconds();
    adv->monitor[0] = (seconds >> 24) & 0xFF;
    adv->monitor[1] = (seconds >> 16) & 0xFF;
    adv->monitor[2] = (seconds >> 8) & 0xFF;
    adv->monitor[3] = (seconds) & 0xFF;
  }
  break;

  case 1: /* Chip temperature */
    adv->monitor[0] = accel_filter;
    adv->monitor[1] = accel_status;
    adv->monitor[2] = errcode;
    adv->monitor[3] = temp;
    break;

  case 2: /* VBAT and VDD */
    adv->monitor[0] = (vdd >> 8) & 0xFF;
    adv->monitor[1] = (vdd) & 0xFF;
    adv->monitor[2] = (vbat >> 8) & 0xFF;
    adv->monitor[3] = (vbat) & 0xFF;
    break;

  case 3: /* HW version and SW version */
    adv->monitor[0] = BOARD_VERSION;
    adv->monitor[1] = (W3_SOFTWARE_VERSION >> 16) & 0xFF;
    adv->monitor[2] = (W3_SOFTWARE_VERSION >> 8) & 0xFF;
    adv->monitor[3] = (W3_SOFTWARE_VERSION) & 0xFF;
    break;
  }
  return sizeof(ble_frame_adv_t);
}
#endif /* BOARD_WEARABLE3 */
/*---------------------------------------------------------------------------*/
static void
ble_adv_init(void)
{
  NETSTACK_RADIO.init();
  NETSTACK_RADIO.get_object(RADIO_CONST_BLE_BD_ADDR, &ble_addr,
                            BLE_ADDR_SIZE);
  PRINTF("ble_address : ");
  uint8_t i;
  for(i = 0; i < BLE_ADDR_SIZE; ++i) {
    PRINTF("0x %02x ", ble_addr[i]);
  }
  PRINTF("\n");

  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_TYPE, BLE_ADV_SCAN_IND);

  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_OWN_ADDR_TYPE,
                           BLE_ADDR_TYPE_PUBLIC);
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_OWN_ADDR_TYPE,
                           BLE_ADDR_TYPE_PUBLIC);
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_CHANNEL_MAP,
                           BLE_ADV_CHANNEL_1_MASK | BLE_ADV_CHANNEL_2_MASK | BLE_ADV_CHANNEL_3_MASK);
}
/*---------------------------------------------------------------------------*/
#if BOARD_WEARABLE3
PROCESS_THREAD(charging_process, ev, data)
{
  static uint32_t last_charge_interrupt_sec;
  static bool charging;
  PROCESS_BEGIN();
  last_charge_interrupt_sec = clock_seconds();
  charging = false;
  etimer_set(&et_charger, CHARGING_LOOP);
  while(1) {
    PROCESS_YIELD();
    if(ev == PROCESS_EVENT_TIMER) {
      if(etimer_expired(&et_charger)) {
        PRINTF("[W3] Charging Required \n ");
        last_charge_interrupt_sec = clock_seconds();
        etimer_reset(&et_charger);
      }
    }
    if(ev == PROCESS_EVENT_POLL) {
      /*Check charging every 60 seconds */
      int32_t delta = clock_seconds() - last_charge_interrupt_sec;
      if(delta < 30) {
        /*PRINTF("Interval not passed %ld \n ",delta); */
        if(mcp73832t_is_charging()) {
          charging = true;
        }
        continue;
      }
      if(charging) {
        /*charging event received */
        PRINTF("[W3] Started Charging \n ");
        mc3672_power_down();
      } else {
        /*Charging pin toggling too fast */
        PRINTF("[W3] Charging Ended \n ");
        if(mc3672_init(&mc3672_config) == false) {
          set_errcode(W3_ERRCODE_MC3672_INIT);
          PRINTF("[W3] ERROR: MC3672 initialisation failed \n ");
        } else {
          PRINTF("[W3] MC3672 initialisation successful \n ");
        }
        process_poll(&sphere_process);
      }
      last_charge_interrupt_sec = clock_seconds();
      charging = false;
    }
  }
  PROCESS_END();
}
#endif /* BOARD_WEARABLE3 */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sphere_process, ev, data)
{
  static uint32_t last_accel_interrupt_sec;

  PROCESS_BEGIN();
#if BOARD_WEARABLE3
  process_start(&charging_process, NULL);
  PRINTF("[W3] Wearable3 \n ");

  SENSORS_ACTIVATE(batmon_sensor);
  tps62746_init();
#endif /* BOARD_WEARABLE3 */

  /*Radio */
  rf_core_set_modesel();
  ble_adv_init();

  size_t scan_resp_data_len;
  static char adv_data[BLE_ADV_DATA_LEN];             /*"dev / ble - hal.h " */
  static char scan_resp_data[BLE_SCAN_RESP_DATA_LEN]; /*"dev / ble - hal.h " */

  set_adv_packet_data(adv_data);
  scan_resp_data_len = init_scan_resp_data(scan_resp_data);

  /* set advertisement packet pointer & scan response */
  NETSTACK_RADIO.set_object(RADIO_PARAM_BLE_ADV_SCAN_RESPONSE, &scan_resp_data, scan_resp_data_len);

#if BOARD_WEARABLE3
  NETSTACK_RADIO.set_object(RADIO_PARAM_BLE_ADV_PAYLOAD, &adv_data, sizeof(ble_frame_adv_t));

  if(mc3672_init(&mc3672_config) == false) {
    set_errcode(W3_ERRCODE_MC3672_INIT);
    PRINTF("[W3] ERROR: MC3672 initialisation failed \n ");
  } else {
    PRINTF("[W3] MC3672 initialisation successful \n ");
    accelerometer_status = 0;
  }
  uint8_t i = 0;
  do {
    mc3672_read_fifo(mc3672_config.data_resolution);
    if(i > 32) {
      PRINTF("FIFO cant be cleaned ");
      break;
    }
  } while(mc3672_fifo_empty() != MC3672_RESULT_OK);
  mcp73832t_init();
  mcp73832t_configure_interrupt(mcp73832t_interrupt_handler);
#else
  NETSTACK_RADIO.set_object(RADIO_PARAM_BLE_ADV_PAYLOAD, &adv_data, BLE_ADDR_SIZE + BLE_ADV_DATA_LEN);
#endif /* BOARD_WEARABLE3 */
  etimer_set(&et, SPHERE_LOOP);

  while(1) {
    PROCESS_YIELD();
    if(ev == PROCESS_EVENT_TIMER) {
      if(etimer_expired(&et)) {
        if(print_monitoring_data) {
          print_monitoring_report();
        }
        int32_t delta = clock_seconds() - last_accel_interrupt_sec;
        if((delta > 60) && !(mcp73832t_is_charging())) {
          /* the accelerometer is not giving data, reboot */
          PRINTF("rebooting the node: no accel interrupts \n ");
          watchdog_reboot();
        }
        etimer_reset(&et);
      }
    } else if(ev == PROCESS_EVENT_POLL) {
#if BOARD_WEARABLE3
      uint8_t interrupt_reg = mc3672_get_register(0x09);
      ble_frame_adv_t *adv;
      if((interrupt_reg >> 6) & 0x01) { /* MC3672_FIFO_THRES_INT */
        adv = (ble_frame_adv_t *)adv_data;
        /* Read FIFO samples until the FIFO is empty */

        if(mc3672_fill_payload(adv->data) != MC3672_RESULT_OK) {
          PRINTF("FIFO Error ");
          /*FIFO ERROR */
          set_errcode(0x02);
        }
        /*for (int var = 0; var < 18; ++var) { */
        /*  printf(" %02x ",adv->data[var]); */
        /*} */
        int i = 0;

        while(mc3672_fifo_empty() != MC3672_RESULT_OK) {
          mc3672_read_fifo(mc3672_config.data_resolution);
          if(i > 32) {
            PRINTF("FIFO cant be cleaned ");
            break;
          }
          i++;
        }
        if(i > 0) {
          PRINTF("FIFO cleaned at %d ", i);
          i = 0;
        }
      }
      if(!mc3672_clear_interrupt()) {
        set_errcode(0x02);
        PRINTF("clear interrupt failed ");
        mc3672_spi_close();
        break;
      }
      mc3672_spi_close();
      /* Add accelerometer data and monitoring to packet */
      update_adv_packet_data(adv_data);
      last_accel_interrupt_sec = clock_seconds();
      num_accel_interrupts++;
      int_master_status_t status = int_master_read_and_disable();
      /*advertising sequence with command sequence */
      NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_ENABLE, 0);
      int_master_status_set(status);
    } else if(ev == button_hal_press_event) {
      /* This runs if the button is pressed */
      PRINTF("[W3] Button pressed.\n ");
    } else if(ev == button_hal_release_event) {
      /* This runs if the button is pressed */
      PRINTF("[W3] Button released.\n ");
    } else if(ev == button_hal_periodic_event) {
      /* This runs if the button is pressed */
      PRINTF("[W3] Button still pressed.\n ");
#endif /* BOARD_WEARABLE3 */
    }
  }
  watchdog_reboot();
  PROCESS_END();
}
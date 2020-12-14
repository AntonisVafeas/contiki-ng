/*
 * Copyright (c) 2017, Graz University of Technology
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

/**
 * \file
 *    BLE radio hardware abstraction implementation for the TI CC26XX controller
 *
 * \author
 *    Michael Spoerk <michael.spoerk@tugraz.at>
 *    Jinyan BAI <onefreebjy@outlook.com>
 *    Atis Elsts <atis.elsts@bristol.ac.uk>
 *    Antonis Vafeas <antonis.vafeas@bristol.ac.uk>
 */
/*---------------------------------------------------------------------------*/

#include "lpm.h"

#include "sys/rtimer.h"
#include "sys/process.h"

#include "os/dev/ble-hal.h"
#include "dev/oscillators.h"

#include "ble-addr.h"

#include "net/netstack.h"
#include "net/packetbuf.h"

#include "rf_data_entry.h"
#include "rf-core/rf-core.h"
#include "rf_ble_cmd.h"
#include "lib/random.h"

#include "ioc.h"
#include "ti-lib.h"
#include "inc/hw_types.h"
#include "inc/hw_rfc_dbell.h"
#include "dev/watchdog.h"

#include <string.h>

#include "rf-core/ble-hal/rf-ble-cmd.h"
#if RADIO_CONF_BLE5
#include "rf_patches/rf_patch_cpe_bt5.h"
#endif
/*---------------------------------------------------------------------------*/
#include "sys/log.h"
#define LOG_MODULE "BLE-RADIO"
/*#define LOG_LEVEL LOG_LEVEL_INFO */
#ifdef LOG_CONF_LEVEL_MAC
#define LOG_LEVEL LOG_CONF_LEVEL_MAC
#else
#define LOG_LEVEL LOG_LEVEL_DBG
#endif
/*---------------------------------------------------------------------------*/
#ifdef BLE_CONF_WITH_ADV
#define BLE_WITH_ADV BLE_CONF_WITH_ADV
#else
#define BLE_WITH_ADV 1
#endif
#ifdef BLE_CONF_WITH_CALLBACK
#define BLE_WITH_ADV_CALLBACK BLE_CONF_WITH_CALLBACK
#else
#define BLE_WITH_ADV_CALLBACK 0
#endif

#ifdef BLE_CONF_WITH_ADV_RX
#define BLE_WITH_ADV_RX BLE_CONF_WITH_ADV_RX
#else
#define BLE_WITH_ADV_RX 1
#endif

#ifdef BLE_CONF_WITH_ADV_RX
#define BLE_WITH_ADV_RX BLE_CONF_WITH_ADV_RX
#else
#define BLE_WITH_ADV_RX 1
#endif

#ifdef BLE_CONF_WITH_SCAN_RESPONSE
#define BLE_WITH_SCAN_RESPONSE BLE_CONF_WITH_SCAN_RESPONSE
#else
#define BLE_WITH_SCAN_RESPONSE 1
#endif

#ifdef BLE_CONF_WITH_SCANNER
#define BLE_WITH_SCANNER BLE_CONF_WITH_SCANNER
#else
#define BLE_WITH_SCANNER 1
#endif

#ifdef BLE_CONF_WITH_SCANNER_CALLBACK
#define BLE_WITH_SCANNER_CALLBACK BLE_CONF_WITH_SCANNER_CALLBACK
#else
#define BLE_WITH_SCANNER_CALLBACK 0
#endif

#ifdef BLE_CONF_WITH_CONN
#define BLE_WITH_CONN BLE_CONF_WITH_CONN
#else
#define BLE_WITH_CONN 1
#endif
/*---------------------------------------------------------------------------*/
#define CMD_GET_STATUS(X)         (((rfc_radioOp_t *)X)->status)
#define RX_ENTRY_STATUS(X)        (((rfc_dataEntry_t *)X)->status)
#define RX_ENTRY_LENGTH(X)        (((rfc_dataEntry_t *)X)->length)
#define RX_ENTRY_TYPE(X)        (((rfc_dataEntry_t *)X)->config.type)
#define RX_ENTRY_NEXT_ENTRY(X)      (((rfc_dataEntry_t *)X)->pNextEntry)
#define RX_ENTRY_DATA_LENGTH(X)     ((X)[8])
#define RX_ENTRY_DATA_PTR(X)      (&(X)[9])
#define TX_ENTRY_STATUS(X)        RX_ENTRY_STATUS(X)
#define TX_ENTRY_LENGTH(X)        RX_ENTRY_LENGTH(X)
#define TX_ENTRY_TYPE(X)        RX_ENTRY_TYPE(X)
#define TX_ENTRY_NEXT_ENTRY(X)      RX_ENTRY_NEXT_ENTRY(X)
#define TX_ENTRY_FRAME_TYPE(X)      ((X)[8])
#define TX_ENTRY_DATA_PTR(X)      (&(X)[9])
/*---------------------------------------------------------------------------*/
/* LPM                                                                       */
/*---------------------------------------------------------------------------*/
static uint8_t
request(void)
{
  if(rf_core_is_accessible()) {
    return LPM_MODE_SLEEP;
  }

  return LPM_MODE_MAX_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
LPM_MODULE(cc26xx_ble_lpm_module, request, NULL, NULL, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*/
/* timing utilities                                */
#define TIME_UNIT_MS               1000  /* 1000 times per second */
#define TIME_UNIT_0_625_MS         1600  /* 1600 times per second */
#define TIME_UNIT_1_25_MS           800  /* 800 times per second */
#define TIME_UNIT_10_MS             100  /* 100 times per second */
#define TIME_UNIT_RF_CORE       4000000  /* runs at 4 MHz */
#define TIME_UNIT_RTIMER  RTIMER_SECOND  /* 65536 Hz */

static inline rtimer_clock_t
ticks_from_unit(uint32_t value, uint32_t unit)
{
  double temp = (((double)value) / unit) * RTIMER_SECOND;
  return (rtimer_clock_t)temp;
}
static inline uint32_t
ticks_to_unit(rtimer_clock_t value, uint32_t unit)
{
  double temp = (((double)value) / RTIMER_SECOND) * unit;
  return (uint32_t)temp;
}
/*---------------------------------------------------------------------------*/
static inline int
safe_set_rtimer(struct rtimer *task, rtimer_clock_t time,
                rtimer_clock_t duration, rtimer_callback_t func, void *ptr)
{
  rtimer_clock_t now = RTIMER_NOW();
  if(RTIMER_CLOCK_LT(time, now + RTIMER_GUARD_TIME)) {
    time = now + RTIMER_GUARD_TIME;
  }
  return rtimer_set(task, time, duration, func, ptr);
}
/*---------------------------------------------------------------------------*/
#if RADIO_CONF_BLE5
#define CMD_BUFFER_SIZE         28
#define PARAM_BUFFER_SIZE       48
#define OUTPUT_BUFFER_SIZE      24
#else
#define CMD_BUFFER_SIZE         24
#define PARAM_BUFFER_SIZE       36
#define OUTPUT_BUFFER_SIZE      24
#endif
/*---------------------------------------------------------------------------*/
/* ADVERTISING data structures												 */
/*maybe the radio timestamp 4 bytes */
/*#define ADV_RX_BUFFERS_OVERHEAD     8 */
#define ADV_RX_BUFFERS_OVERHEAD     12
#define ADV_RX_BUFFERS_DATA_LEN     60
#define ADV_RX_BUFFERS_LEN        (ADV_RX_BUFFERS_OVERHEAD + ADV_RX_BUFFERS_DATA_LEN)
#define ADV_RX_BUFFERS_NUM        4

#define ADV_PREPROCESSING_TIME_TICKS  65

/* Whitelist definitions */
#define WHITELIST_BUFFERS_NUM        8
#define WHITELIST_BUFFERS_BUFFERS_LEN 8

typedef struct {
  /* PARAMETER */
  uint16_t adv_interval;
  ble_adv_type_t adv_type;
  ble_addr_type_t own_addr_type;
  uint8_t channel_map;
  uint8_t adv_data_len;
  uint8_t *adv_data_ptr;
  uint8_t adv_data[BLE_ADV_DATA_LEN];
#if BLE_WITH_SCAN_RESPONSE
  uint8_t scan_rsp_data_len;
  uint8_t scan_rsp_data[BLE_ADV_DATA_LEN];
#endif
  /* STATE information */
  uint8_t active;
  rtimer_clock_t start_rt;
  struct rtimer timer;
  /* utility */
  uint8_t cmd_buf[CMD_BUFFER_SIZE];
  uint8_t param_buf[PARAM_BUFFER_SIZE];
  uint8_t output_buf[OUTPUT_BUFFER_SIZE];
  uint8_t whitelist_buffers[WHITELIST_BUFFERS_NUM][WHITELIST_BUFFERS_BUFFERS_LEN];
#if BLE_WITH_ADV_RX
  dataQueue_t rx_queue;

  uint8_t rx_buffers[ADV_RX_BUFFERS_NUM][ADV_RX_BUFFERS_LEN];
  uint8_t *rx_queue_current;
#endif
} ble_adv_param_t;

#if BLE_WITH_ADV
static ble_adv_param_t adv_param;
static uint8_t scanResponseNum;
#endif
#if BLE_WITH_ADV_RX
static void advertising_event(struct rtimer *t, void *ptr);

#endif

/*---------------------------------------------------------------------------*/
/* SCANNER data structures												 */
#define SCANNER_RX_BUFFERS_NUM        32
#define SCANNER_RX_BUFFERS_OVERHEAD    8
#define SCANNER_RX_BUFFERS_DATA_LEN   60
#define SCANNER_RX_BUFFERS_LEN       (SCANNER_RX_BUFFERS_OVERHEAD + SCANNER_RX_BUFFERS_DATA_LEN)

#define SCANNER_PREPROCESSING_TIME_TICKS  65

typedef struct {
  /* PARAMETER */
  uint16_t scan_interval;
  uint16_t scan_window;
  ble_scan_type_t scan_type;
  ble_addr_type_t own_addr_type;
  uint8_t scan_channel;
  uint8_t scan_req_data_len;
  uint8_t scan_req_data[BLE_ADV_DATA_LEN];
  /* STATE information */
  uint8_t active;
  rtimer_clock_t start_rt;
  struct rtimer timer;
  /* utility */
  uint8_t cmd_buf[CMD_BUFFER_SIZE];
  uint8_t param_buf[PARAM_BUFFER_SIZE];
  uint8_t output_buf[OUTPUT_BUFFER_SIZE];
  dataQueue_t rx_queue;
  uint8_t rx_buffers[SCANNER_RX_BUFFERS_NUM][ADV_RX_BUFFERS_LEN];
  uint8_t whitelist_buffers[WHITELIST_BUFFERS_NUM][WHITELIST_BUFFERS_BUFFERS_LEN];
  uint8_t *rx_queue_current;
  /* Peer address for filtering */
  uint8_t peer_address[BLE_ADDR_SIZE];
} ble_scanner_param_t;

#if BLE_WITH_SCANNER
static ble_scanner_param_t scanner_param;
#if BLE_WITH_SCANNER_CALLBACK
static void
scanner_rx_packet_process(uint8_t *dataEnrty);
#endif
static void scan_event_start(struct rtimer *t, void *ptr);
static void scan_event_end(struct rtimer *t, void *ptr);
static void scanner_rx(ble_scanner_param_t *param);
#endif
/*---------------------------------------------------------------------------*/
/* CONNECTION data structures                          */

#ifdef BLE_CONF_MODE_MAX_CONNECTIONS
#define BLE_MODE_MAX_CONNECTIONS BLE_CONF_MODE_MAX_CONNECTIONS
#else
#define BLE_MODE_MAX_CONNECTIONS    1
#endif

/* maximum packet length that is transmitted during a single connection event*/
#ifdef BLE_MODE_CONF_CONN_MAX_PACKET_SIZE
#define BLE_MODE_CONN_MAX_PACKET_SIZE   BLE_MODE_CONF_CONN_MAX_PACKET_SIZE
#else
#define BLE_MODE_CONN_MAX_PACKET_SIZE   256
#endif

#define CONN_BLE_BUFFER_SIZE        27  /* maximum size of the data buffer */

#define CONN_RX_BUFFERS_OVERHEAD    8
#define CONN_RX_BUFFERS_DATA_LEN    60
#define CONN_RX_BUFFERS_LEN       (CONN_RX_BUFFERS_OVERHEAD + CONN_RX_BUFFERS_DATA_LEN)
#define CONN_RX_BUFFERS_NUM       12

/* custom status used for tx buffers */
#define DATA_ENTRY_FREE         5
#define DATA_ENTRY_QUEUED       6

#define CONN_TX_BUFFERS_OVERHEAD    9
#define CONN_TX_BUFFERS_DATA_LEN    27
#define CONN_TX_BUFFERS_LEN       (CONN_TX_BUFFERS_OVERHEAD + CONN_TX_BUFFERS_DATA_LEN)
#define CONN_TX_BUFFERS_NUM       12

#define CONN_WIN_SIZE           1
#define CONN_WIN_OFFSET          20

#define CONN_EVENT_LATENCY_THRESHOLD     10
#define CONN_WINDOW_WIDENING_TICKS     30   /* appr. 0.46 ms */
#define CONN_PREPROCESSING_TIME_TICKS 100   /* 1.5 ms */

#define CONN_UPDATE_DELAY         6

typedef struct {
  /* PARAMETER */
  uint8_t peer_address[BLE_ADDR_SIZE];
  uint32_t access_address;
  uint8_t crc_init_0;
  uint8_t crc_init_1;
  uint8_t crc_init_2;
  uint8_t win_size;
  uint16_t win_offset;
  uint16_t interval;
  uint16_t latency;
  uint16_t timeout;
  uint64_t channel_map;
  uint8_t num_used_channels;
  uint8_t hop;
  uint8_t sca;
  rtimer_clock_t timestamp_rt;
  /* STATE information */
  uint8_t active;
  uint16_t counter;
  uint8_t unmapped_channel;
  uint8_t mapped_channel;
  rtimer_clock_t start_rt;
  uint16_t conn_handle;
  struct rtimer timer;
  /* utility */
  uint8_t cmd_buf[CMD_BUFFER_SIZE];
  uint8_t param_buf[PARAM_BUFFER_SIZE];
  uint8_t output_buf[OUTPUT_BUFFER_SIZE];
  dataQueue_t rx_queue;
  uint8_t rx_buffers[CONN_RX_BUFFERS_NUM][CONN_RX_BUFFERS_LEN];
  uint8_t *rx_queue_current;
  dataQueue_t tx_queue;
  uint8_t tx_buffers[CONN_TX_BUFFERS_NUM][CONN_TX_BUFFERS_LEN];
  uint8_t tx_buffers_sent;
  uint16_t skipped_events;
  /* channel map update */
  uint64_t channel_update_channel_map;
  uint16_t channel_update_counter;
  uint8_t channel_update_num_used_channels;
  /* connection parameter update */
  uint8_t conn_update_win_size;
  uint16_t conn_update_win_offset;
  uint16_t conn_update_interval;
  uint16_t conn_update_latency;
  uint16_t conn_update_timeout;
  uint16_t conn_update_counter;
} ble_conn_param_t;

#if BLE_WITH_CONN
static ble_conn_param_t conn_param[BLE_MODE_MAX_CONNECTIONS];
#endif

static uint16_t conn_counter = 0;

#if BLE_WITH_ADV_RX
static void connection_event_slave(struct rtimer *t, void *ptr);
#endif
/*---------------------------------------------------------------------------*/
PROCESS(ble_hal_rx_process, "BLE/CC26xx RX process");
#if BLE_WITH_SCANNER
PROCESS(ble_hal_interrupt_handler, "BLE/CC26xx RX process");
#endif
process_event_t conn_rx_data_event;
process_event_t scan_rx_data_event;
/*---------------------------------------------------------------------------*/
void
ble_hal_setup_buffers(void)
{
  uint8_t conn_count;
  ble_conn_param_t *conn;
  uint8_t i;
  rfc_dataEntry_t *entry;

#if BLE_WITH_ADV
  /* setup advertisement RX buffer (circular buffer) */
  memset(&adv_param, 0x00, sizeof(ble_adv_param_t));
  scanResponseNum = 0;
#if BLE_WITH_ADV_RX
  memset(&adv_param.rx_queue, 0x00, sizeof(adv_param.rx_queue));
  adv_param.rx_queue.pCurrEntry = adv_param.rx_buffers[0];
  adv_param.rx_queue.pLastEntry = NULL;
  adv_param.rx_queue_current = adv_param.rx_buffers[0];
  for(i = 0; i < ADV_RX_BUFFERS_NUM; i++) {
    memset(&adv_param.rx_buffers[i], 0x00, ADV_RX_BUFFERS_LEN);
    entry = (rfc_dataEntry_t *)adv_param.rx_buffers[i];
    entry->pNextEntry = adv_param.rx_buffers[(i + 1) % ADV_RX_BUFFERS_NUM];
    entry->config.lenSz = 1;
    entry->length = ADV_RX_BUFFERS_DATA_LEN;
  }
  rfc_bleWhiteListEntry_t *adv_buffer;
  for(i = 0; i < WHITELIST_BUFFERS_NUM; i++) {
    memset(&adv_param.whitelist_buffers[i], 0x00,
           WHITELIST_BUFFERS_BUFFERS_LEN);
    adv_buffer = (rfc_bleWhiteListEntry_t *)adv_param.whitelist_buffers[i];
    adv_buffer->size = 0;
    adv_buffer->conf.bEnable = 0;
    adv_buffer->conf.addrType = 0;
    adv_buffer->conf.bWlIgn = 0;
    adv_buffer->conf.bIrkValid = 0;
  }
  /*
     rfc_CMD_BLE_ADV_NC_t adv_cmd_buff[3];
     rfc_bleAdvOutput_t adv_output[3];
     rfc_CMD_BLE_ADV_NC_t *cmd_buff;
     for(size_t i = 0; i < 3; ++i) {
      cmd_buff = (rfc_CMD_BLE_ADV_NC_t *)adv_param.adv_cmd_buff[i];
      cmd_buff->commandNo = CMD_BLE_ADV_NC;
      cmd_buff->whitening.bOverride = 0;
      cmd_buff->channel = BLE_ADV_CHANNEL_1+i;
      cmd_buff->pParams = (rfc_bleAdvPar_t *)adv_param.param_buf;
      cmd_buff->startTrigger.triggerType = TRIG_NOW;
      cmd_buff->pOutput = &adv_output[i];
     if(i==2){
     cmd_buff->condition.rule = COND_NEVER;
     }else{
     cmd_buff->pNextOp = (rfc_radioOp_t *)adv_param.adv_cmd_buff[i+1];
     cmd_buff->condition.rule = COND_STOP_ON_FALSE;
     }
     }
   */
#endif /* BLE_WITH_ADV_RX */
#endif /* BLE_WITH_ADV */

#if BLE_WITH_SCANNER
  /* setup scanner RX buffer (circular buffer) */
  memset(&scanner_param, 0x00, sizeof(ble_scanner_param_t));
  memset(&scanner_param.rx_queue, 0x00, sizeof(scanner_param.rx_queue));
  scanner_param.rx_queue.pCurrEntry = scanner_param.rx_buffers[0];
  scanner_param.rx_queue.pLastEntry = NULL;
  scanner_param.rx_queue_current = scanner_param.rx_buffers[0];
  for(i = 0; i < SCANNER_RX_BUFFERS_NUM; i++) {
    memset(&scanner_param.rx_buffers[i], 0x00, SCANNER_RX_BUFFERS_LEN);
    entry = (rfc_dataEntry_t *)scanner_param.rx_buffers[i];
    entry->pNextEntry = scanner_param.rx_buffers[(i + 1)
                                                 % SCANNER_RX_BUFFERS_NUM];
    entry->config.lenSz = 1;
    entry->length = SCANNER_RX_BUFFERS_DATA_LEN;
  }
  rfc_bleWhiteListEntry_t *buffer;
  for(i = 0; i < WHITELIST_BUFFERS_NUM; i++) {
    memset(&scanner_param.whitelist_buffers[i], 0x00,
           WHITELIST_BUFFERS_BUFFERS_LEN);
    buffer = (rfc_bleWhiteListEntry_t *)scanner_param.whitelist_buffers[i];
    buffer->size = 0;
    buffer->conf.bEnable = 0;
    buffer->conf.addrType = 0;
    buffer->conf.bWlIgn = 0;
    buffer->conf.bIrkValid = 0;
  }
#endif

#if BLE_WITH_CONN
  memset(conn_param, 0x00,
         sizeof(ble_conn_param_t) * BLE_MODE_MAX_CONNECTIONS);
  for(conn_count = 0; conn_count < BLE_MODE_MAX_CONNECTIONS; conn_count++) {
    /* setup connection RX buffer (circular buffer) */
    conn = &conn_param[conn_count];
    memset(&conn->rx_queue, 0x00, sizeof(conn->rx_queue));
    conn->rx_queue.pCurrEntry = conn->rx_buffers[0];
    conn->rx_queue.pLastEntry = NULL;
    conn->rx_queue_current = conn->rx_buffers[0];

    for(i = 0; i < CONN_RX_BUFFERS_NUM; i++) {
      memset(&conn->rx_buffers[i], 0x00, CONN_RX_BUFFERS_LEN);
      entry = (rfc_dataEntry_t *)conn->rx_buffers[i];
      entry->pNextEntry = conn->rx_buffers[(i + 1) % CONN_RX_BUFFERS_NUM];
      entry->config.lenSz = 1;
      entry->length = CONN_RX_BUFFERS_DATA_LEN;
    }

    /* setup connection TX buffer (buffers are added on demand to the queue) */
    memset(&conn->tx_queue, 0x00, sizeof(conn->tx_queue));
    conn->tx_queue.pCurrEntry = NULL;
    conn->tx_queue.pLastEntry = NULL;

    for(i = 0; i < CONN_TX_BUFFERS_NUM; i++) {
      memset(&conn->tx_buffers[i], 0x00, CONN_TX_BUFFERS_LEN);
      entry = (rfc_dataEntry_t *)conn->tx_buffers[i];
      entry->config.lenSz = 1;
      entry->status = DATA_ENTRY_FREE;
    }
  }
#endif
}
/*---------------------------------------------------------------------------*/
static ble_conn_param_t *
get_connection_for_handle(uint8_t conn_handle)
{
#if BLE_WITH_CONN
  uint8_t i;
  for(i = 0; i < BLE_MODE_MAX_CONNECTIONS; i++) {
    if(conn_param[i].conn_handle == conn_handle) {
      return &conn_param[i];
    }
  }
#endif
  return NULL;
}
/*---------------------------------------------------------------------------*/
static uint8_t *
tx_queue_get_buffer(ble_conn_param_t *param)
{
  uint8_t i;
  rfc_dataEntry_t *entry;
  for(i = 0; i < CONN_TX_BUFFERS_NUM; i++) {
    entry = (rfc_dataEntry_t *)param->tx_buffers[i];
    if(entry->status == DATA_ENTRY_FREE) {
      return (uint8_t *)entry;
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
static uint16_t
tx_queue_count_free_buffers(ble_conn_param_t *param)
{
  uint16_t i;
  uint16_t free_bufs = 0;
  for(i = 0; i < CONN_TX_BUFFERS_NUM; i++) {
    if(TX_ENTRY_STATUS(param->tx_buffers[i]) == DATA_ENTRY_FREE) {
      free_bufs++;
    }
  }
  return free_bufs;
}
/*---------------------------------------------------------------------------*/
static inline uint8_t
tx_queue_data_to_transmit(ble_conn_param_t *param)
{
  uint16_t i;
  for(i = 0; i < CONN_TX_BUFFERS_NUM; i++) {
    if(TX_ENTRY_STATUS(param->tx_buffers[i]) == DATA_ENTRY_QUEUED) {
      return 1;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
ble_result_t
on(void)
{
  oscillators_request_hf_xosc();
  if(!rf_core_is_accessible()) {
    /* boot the rf core */

    /*    boot and apply Bluetooth 5 Patch    */
    if(rf_core_power_up() != RF_CORE_CMD_OK) {
      LOG_ERR("rf_core_boot: rf_core_power_up() failed\n");
      rf_core_power_down();
      return RF_CORE_CMD_ERROR;
    }

#if RADIO_CONF_BLE5
    /*  Apply Bluetooth 5 patch, if applicable  */
    rf_patch_cpe_bt5();
#endif
    if(rf_core_start_rat() != RF_CORE_CMD_OK) {
      LOG_ERR("rf_core_boot: rf_core_start_rat() failed\n");
      rf_core_power_down();
      return RF_CORE_CMD_ERROR;
    }
    rf_core_setup_interrupts();
    oscillators_switch_to_hf_xosc();

    if(rf_ble_cmd_setup_ble_mode() != RF_BLE_CMD_OK) {
      LOG_ERR("could not setup rf-core to BLE mode\n");
      return BLE_RESULT_ERROR;
    }
  }
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
void
off(void)
{
  rf_core_power_down();
  oscillators_switch_to_hf_rc();
}
/*---------------------------------------------------------------------------*/
static ble_result_t
reset(void)
{
  LOG_INFO("maximum connections: %4d\n", BLE_MODE_MAX_CONNECTIONS);
  LOG_INFO("max. packet length:  %4d\n", BLE_MODE_CONN_MAX_PACKET_SIZE);
  lpm_register_module(&cc26xx_ble_lpm_module);
  rf_core_set_modesel();
  ble_hal_setup_buffers();
  if(on() != BLE_RESULT_OK) {
    return BLE_RESULT_ERROR;
  }
  off();
  if(!process_is_running(&ble_hal_rx_process)) {
    conn_rx_data_event = process_alloc_event();
    scan_rx_data_event = process_alloc_event();
    process_start(&ble_hal_rx_process, NULL);
#if BLE_WITH_SCANNER
    process_start(&ble_hal_interrupt_handler, NULL);
#endif
  }
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
read_bd_addr(uint8_t *addr)
{
  ble_addr_cpy_to(addr);
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
read_buffer_size(unsigned int *buf_len, unsigned int *num_buf)
{
  uint16_t i;
  uint16_t ll_buffers = CONN_TX_BUFFERS_NUM;
  uint16_t packet_buffers;
  uint16_t buffer_size;
  for(i = 0; i < conn_counter; i++) {
    ll_buffers = MIN(ll_buffers,
                     tx_queue_count_free_buffers(&conn_param[i]));
  }
  packet_buffers = ll_buffers
    / (BLE_MODE_CONN_MAX_PACKET_SIZE / CONN_BLE_BUFFER_SIZE);
  buffer_size = BLE_MODE_CONN_MAX_PACKET_SIZE;
  memcpy(buf_len, &buffer_size, 2);
  memcpy(num_buf, &packet_buffers, 2);
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
set_adv_param(unsigned int adv_int, ble_adv_type_t type,
              ble_addr_type_t own_type, unsigned short adv_map)
{
  adv_param.adv_interval = adv_int;
  adv_param.adv_type = type;
  adv_param.own_addr_type = own_type;
  adv_param.channel_map = adv_map;

  LOG_INFO("advertising parameter: interval: %4d, channels: %2d\n",
           adv_param.adv_interval, adv_param.channel_map);

  LOG_DBG("interval: %16u (ms)\n", adv_param.adv_interval);
  LOG_DBG("type:     %16u\n", adv_param.adv_type);
  LOG_DBG("addr_type:%16u\n", adv_param.own_addr_type);
  LOG_DBG("channels: %16u\n", adv_param.channel_map);

  return BLE_RESULT_OK;
}
static ble_result_t
send_adv_chain(ble_adv_type_t adv_type, ble_addr_type_t own_type)
{
  static rfc_CMD_BLE_ADV_NC_t cmd_buff[3];
  static uint8_t initialised = 0;
  ble_adv_param_t *param = (ble_adv_param_t *)&adv_param;
  rfc_bleAdvPar_t *p = (rfc_bleAdvPar_t *)adv_param.param_buf;
  if(!initialised) {
    initialised = 1;
    p->advConfig.deviceAddrType = adv_type;
    p->pDeviceAddress = (uint16_t *)(uint8_t *)BLE_ADDR_LOCATION;
    p->endTrigger.triggerType = TRIG_NEVER;
    p->advLen = param->adv_data_len;
    p->pAdvData = param->adv_data_ptr;

    for(size_t i = 0; i < 3; ++i) {
      cmd_buff[i].commandNo = CMD_BLE_ADV_NC;
      cmd_buff[i].channel = BLE_ADV_CHANNEL_1 + i;
      cmd_buff[i].pParams = (rfc_bleAdvPar_t *)param->param_buf;
      cmd_buff[i].startTrigger.triggerType = TRIG_NOW;
      if(i == 2) {
        cmd_buff[i].condition.rule = COND_NEVER;
      } else {
        cmd_buff[i].pNextOp = (rfc_radioOp_t *)&cmd_buff[i + 1];
        cmd_buff[i].condition.rule = COND_STOP_ON_FALSE;
      }
    }
  }
  for(size_t i = 0; i < 3; ++i) {
    cmd_buff[i].status = 0;
  }
  if(on() != BLE_RESULT_OK) {
    LOG_ERR("on: could not enable rf core\n");
    return BLE_RESULT_ERROR;
  }
  if(rf_ble_cmd_send((uint8_t *)&cmd_buff) != RF_CORE_CMD_OK) {
    LOG_ERR("rf_ble_cmd_send: could not enable rf core\n");
    return RF_BLE_CMD_ERROR;
  }
  if(rf_ble_cmd_wait((uint8_t *)&cmd_buff[2]) != RF_CORE_CMD_OK) {
    LOG_ERR("rf_ble_cmd_wait: could not enable rf core\n");
    return RF_BLE_CMD_ERROR;
  }
  off();
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
send_single_adv(ble_adv_type_t adv_type, ble_addr_type_t own_type,
                unsigned short adv_map)
{
  ble_adv_param_t *param = &adv_param;
  const uint8_t masks[3] =
  { BLE_ADV_CHANNEL_1_MASK, BLE_ADV_CHANNEL_2_MASK,
    BLE_ADV_CHANNEL_3_MASK };
  const uint8_t channels[3] =
  { BLE_ADV_CHANNEL_1, BLE_ADV_CHANNEL_2,
    BLE_ADV_CHANNEL_3 };
  uint8_t indexes[3];
  uint16_t r;
  int i;

  if(adv_param.active) {
    LOG_ERR("send_single_adv: already active\n");
    return BLE_RESULT_ERROR;
  }

  if(on() != BLE_RESULT_OK) {
    LOG_ERR("send_single_adv: could not enable rf core\n");
    return BLE_RESULT_ERROR;
  }

  adv_param.adv_interval = 0;
  adv_param.adv_type = adv_type;
  adv_param.own_addr_type = own_type;
  adv_param.channel_map = adv_map;

  dataQueue_t *rx_queue = NULL;
#if BLE_WITH_ADV_RX
  rx_queue = &param->rx_queue;
  volatile rfc_dataEntry_t *entry =
    (rfc_dataEntry_t *)param->rx_queue.pCurrEntry;
  do {
    entry->status = DATA_ENTRY_PENDING;
    entry = (rfc_dataEntry_t *)entry->pNextEntry;
  } while(entry != (rfc_dataEntry_t *)param->rx_queue.pCurrEntry);
#endif
  uint16_t scan_rsp_data_len = 0;
  uint8_t *scan_rsp_data = NULL;
#if BLE_WITH_SCAN_RESPONSE
  scan_rsp_data_len = param->scan_rsp_data_len;
  scan_rsp_data = param->scan_rsp_data;
#endif
  rfc_bleAdvPar_t *p = (rfc_bleAdvPar_t *)param->param_buf;
  rf_ble_cmd_create_adv_params(param->param_buf, rx_queue, param->adv_data_len,
                               param->adv_data, scan_rsp_data_len,
                               scan_rsp_data, param->own_addr_type,
                               (uint8_t *)BLE_ADDR_LOCATION);
  /*
     rf_ble_cmd_create_adv_params_fltr(param->param_buf, rx_queue,
     param->adv_data_len, param->adv_data,
     scan_rsp_data_len, scan_rsp_data,
     param->own_addr_type, (uint8_t *)BLE_ADDR_LOCATION,
     (rfc_bleWhiteListEntry_t *)param->whitelist_buffers[0]);
   */

  /* randomize the channel order */
  r = random_rand();
  if(r < RANDOM_RAND_MAX / 6) {
    indexes[0] = 0;
    indexes[1] = 1;
    indexes[2] = 2;
  } else if(r < RANDOM_RAND_MAX / 3) {
    indexes[0] = 0;
    indexes[1] = 2;
    indexes[2] = 1;
  } else if(r < RANDOM_RAND_MAX / 2) {
    indexes[0] = 1;
    indexes[1] = 0;
    indexes[2] = 2;
  } else if(r < 2 * RANDOM_RAND_MAX / 3) {
    indexes[0] = 1;
    indexes[1] = 2;
    indexes[2] = 0;
  } else if(r < 5 * RANDOM_RAND_MAX / 6) {
    indexes[0] = 2;
    indexes[1] = 0;
    indexes[2] = 1;
  } else {
    indexes[0] = 2;
    indexes[1] = 1;
    indexes[2] = 0;
  }

  for(i = 0; i < 3; ++i) {
    if(param->channel_map & masks[indexes[i]]) {

      rf_ble_cmd_create_adv_cmd(param->cmd_buf, channels[indexes[i]],
                                param->param_buf, param->output_buf);
      rf_ble_cmd_send(param->cmd_buf);

      LOG_INFO("send_single_adv channel:%u data_len:%d\n",
               channels[indexes[i]], p->advLen);
      /* delay a bit (XXX) */
      rtimer_clock_t end = RTIMER_NOW() + 10;
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), end))
        ;

      rf_ble_cmd_wait(param->cmd_buf);
#if BLE_WITH_ADV_RX
      const volatile rfc_bleAdvOutput_t *o =
        (rfc_bleAdvOutput_t *)param->output_buf;

      if(CMD_GET_STATUS(
           param->cmd_buf) == RF_CORE_RADIO_OP_STATUS_BLE_DONE_OK) {
        if(o->nTxScanRsp - scanResponseNum) {
#if BLE_WITH_ADV_CALLBACK
          uint8_t *rx_queue_buffer = (uint8_t *)entry;
          uint8_t *rx_data = RX_ENTRY_DATA_PTR(rx_queue_buffer);
          uint16_t length = RX_ENTRY_DATA_LENGTH(rx_queue_buffer);
          int8_t rssi = (int8_t)*(rx_data + length - 2);

          /* call the user's callback */
          extern void ble_adv_callback(const uint8_t *rx_data, uint16_t length, int rssi);
          /* add channel information */
          ble_adv_callback(rx_data, length, rssi);
#endif
        }
        /* free current entry (clear BLE data length & reset status) */
        RX_ENTRY_DATA_LENGTH(param->rx_queue_current) = 0;
        RX_ENTRY_STATUS(param->rx_queue_current) = DATA_ENTRY_PENDING;
        param->rx_queue_current = RX_ENTRY_NEXT_ENTRY(
          param->rx_queue_current);
      }
      scanResponseNum = (uint8_t)o->nTxScanRsp;
      /*LOG_DBG("rx_queue_current:%p pCurrEntry:%p entry:%p status:%x nRxAdvOk:%d\n", */
      /*        param->rx_queue_current, param->rx_queue.pCurrEntry, entry, entry->status, o->nRxAdvOk);# */
      LOG_DBG(
        "cmd_status:0x%04X responses:%d entry:%p adv params: rx=%u ig=%u crc=%u rssi=%d\n",
        CMD_GET_STATUS(param->cmd_buf), o->nTxScanRsp, entry,
        o->nRxScanReq, o->nRxNok, o->nRxIgnored, o->lastRssi);
#endif
    }
  }

  off();
  /*advertising_rx(param); */
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
read_adv_channel_tx_power(short *power)
{
  return BLE_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
set_adv_data(unsigned short data_len, char *data)
{
  if(data_len > BLE_ADV_DATA_LEN) {
    LOG_WARN("BLE-HAL: adv_data too long\n");
    return BLE_RESULT_INVALID_PARAM;
  }
  adv_param.adv_data_len = data_len;
  adv_param.adv_data_ptr = (uint8_t *)data;
  memcpy(adv_param.adv_data, data, data_len);
  /*LOG_DBG("set_adv_data len:%d\n", data_len); */
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
set_scan_resp_data(unsigned short data_len, char *data)
{
#if BLE_WITH_SCAN_RESPONSE
  if(data_len > BLE_SCAN_RESP_DATA_LEN) {
    LOG_WARN("BLE-HAL: scan_resp_data too long\n");
    return BLE_RESULT_INVALID_PARAM;
  }
  adv_param.scan_rsp_data_len = data_len;
  memcpy(adv_param.scan_rsp_data, data, data_len);
  return BLE_RESULT_OK;
#else
  return BLE_RESULT_INVALID_PARAM;
#endif
}
/*---------------------------------------------------------------------------*/
static ble_result_t
set_adv_enable(unsigned short enable)
{
#if BLE_WITH_ADV_RX
  uint32_t now = RTIMER_NOW();
  if((enable) && (!adv_param.active)) {
    adv_param.start_rt = now + ticks_from_unit(adv_param.adv_interval,
                                               TIME_UNIT_1_25_MS);
    rtimer_set(&adv_param.timer, adv_param.start_rt, 0, advertising_event,
               (void *)&adv_param);
  }
  return BLE_RESULT_OK;
#else
  return BLE_RESULT_ERROR;
#endif
}
/*---------------------------------------------------------------------------*/
static ble_result_t
set_scan_param(ble_scan_type_t type, unsigned int scan_interval,
               unsigned int scan_window, ble_addr_type_t own_addr_type)
{
  LOG_INFO("set_scan_param\n");

  LOG_INFO("interval: %16u (ms)\n", scan_interval);
  LOG_INFO("window:   %16u (ms)\n", scan_window);
  LOG_INFO("type:     %16u\n", type);
  LOG_INFO("addr_type:%16u\n", own_addr_type);
  LOG_INFO("addr_type:%16u\n", own_addr_type);

#if BLE_WITH_SCANNER
  scanner_param.scan_interval = scan_interval;
  scanner_param.scan_window = scan_window;
  scanner_param.scan_type = type;
  scanner_param.own_addr_type = own_addr_type;
#endif

  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
set_scan_param_fltr(ble_scan_type_t type, unsigned int scan_interval,
                    unsigned int scan_window, ble_addr_type_t own_addr_type,
                    uint8_t *peer_address)
{
  LOG_INFO("set_scan_param\n");

  LOG_INFO("interval: %16u (ms)\n", scan_interval);
  LOG_INFO("window:   %16u (ms)\n", scan_window);
  LOG_INFO("type:     %16u\n", type);
  LOG_INFO("addr_type:%16u\n", own_addr_type);
  printf("peer_address (0x%02X) created\n", peer_address[0]);

#if BLE_WITH_SCANNER
  scanner_param.scan_interval = scan_interval;
  scanner_param.scan_window = scan_window;
  scanner_param.scan_type = type;
  scanner_param.own_addr_type = own_addr_type;

#endif

  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
set_scan_req_data(unsigned short data_len, char *data)
{
  if(data_len > BLE_ADV_DATA_LEN) {
    LOG_WARN("BLE-HAL: scan_req_data too long\n");
    return BLE_RESULT_INVALID_PARAM;
  }
  printf("set scan req data: %u bytes\n", data_len);
#if BLE_WITH_SCANNER
  scanner_param.scan_req_data_len = data_len;
  memcpy(scanner_param.scan_req_data, data, data_len);
#endif
  return BLE_RESULT_OK;
}
static ble_result_t
set_scan_whitelist(unsigned short data_len, char *data)
{
  /* check if the address is ble address size */
  if(data_len != BLE_ADDR_SIZE) {
    LOG_WARN("BLE-HAL: whitelist address size invalid\n");
    return BLE_RESULT_INVALID_PARAM;
  }
#if BLE_WITH_SCANNER

  /*uint8_t peer_address[BLE_ADDR_SIZE] = { 0 }; */
  uint8_t exist = false;
  uint8_t i;
  uint8_t whitelist_size;
  rfc_bleWhiteListEntry_t *entry;
  uint32_t addressHi;
  uint16_t address;
  /*most significant 32 bits */
  /*memcpy(scanner_param.peer_address, data, data_len); */

  addressHi = (uint32_t)((data[0] << 24) + (data[1] << 16) + (data[2] << 8)
                         + data[3]);
  address = (uint16_t)((data[4] << 8) + data[5]);
  entry = (rfc_bleWhiteListEntry_t *)scanner_param.whitelist_buffers[0];
  for(i = 0; i < WHITELIST_BUFFERS_NUM; i++) {
    entry = (rfc_bleWhiteListEntry_t *)scanner_param.whitelist_buffers[i];
    /*check if whitelist entry exist */
    if((entry->address == address) && (entry->addressHi == addressHi)) {
      exist = true;
      break;
    }
  }
  entry = (rfc_bleWhiteListEntry_t *)scanner_param.whitelist_buffers[0];
  whitelist_size = entry->size;
  /*printf("entry current size:%d", whitelist_size); */
  if((!exist) || (whitelist_size == 0)) {
    entry->size += 1;
    /*printf("entry new size:%d", entry->size); */
    entry =
      (rfc_bleWhiteListEntry_t *)scanner_param.whitelist_buffers[whitelist_size];
    entry->address = address;
    entry->addressHi = addressHi;
    entry->conf.addrType = scanner_param.own_addr_type;
    entry->conf.bEnable = 1;
    return BLE_RESULT_OK;
  } else {
    printf("BLE-HAL: whitelist address exist\n");
    return BLE_RESULT_INVALID_PARAM;
  }
#endif
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
read_scanner_output(uint8_t *ptr)
{
#if BLE_WITH_SCANNER
  ble_scanner_param_t *param = (ble_scanner_param_t *)&scanner_param;
  if(CMD_GET_STATUS(param->cmd_buf) & RF_CORE_RADIO_OP_STATUS_ACTIVE) {
    memcpy(ptr, param->output_buf, sizeof(rfc_bleScannerOutput_t));
    return BLE_RESULT_OK;
  }
#endif
  return BLE_RESULT_ERROR;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
set_adv_whitelist(unsigned short data_len, char *data)
{
  /* check if the address is ble address size */
  if(data_len != BLE_ADDR_SIZE) {
    LOG_WARN("BLE-HAL: whitelist address size invalid\n");
    return BLE_RESULT_INVALID_PARAM;
  }
#if BLE_WITH_SCANNER

  /*uint8_t peer_address[BLE_ADDR_SIZE] = { 0 }; */
  uint8_t exist = false;
  uint8_t i;
  uint8_t whitelist_size;
  rfc_bleWhiteListEntry_t *entry;
  uint32_t addressHi;
  uint16_t address;
  /*most significant 32 bits */
  /*memcpy(scanner_param.peer_address, data, data_len); */
  printf("Address ");
  for(i = 0; i < data_len; ++i) {
    printf("%02x ", data[i]);
  }
  putchar('\n');
  addressHi = (uint32_t)((data[0] << 24) + (data[1] << 16) + (data[2] << 8)
                         + data[3]);
  address = (uint16_t)((data[4] << 8) + data[5]);
  entry = (rfc_bleWhiteListEntry_t *)adv_param.whitelist_buffers[0];
  for(i = 0; i < WHITELIST_BUFFERS_NUM; i++) {
    entry = (rfc_bleWhiteListEntry_t *)adv_param.whitelist_buffers[i];
    /*check if whitelist entry exist */
    if((entry->address == address) && (entry->addressHi == addressHi)) {
      exist = true;
      break;
    }
  }
  entry = (rfc_bleWhiteListEntry_t *)adv_param.whitelist_buffers[0];
  whitelist_size = entry->size;
  if((!exist) || (whitelist_size == 0)) {
    entry->size += 1;
    entry =
      (rfc_bleWhiteListEntry_t *)adv_param.whitelist_buffers[whitelist_size];
    entry->address = address;
    entry->addressHi = addressHi;
    entry->conf.addrType = scanner_param.own_addr_type;
    entry->conf.bEnable = 1;
    return BLE_RESULT_OK;
  } else {
    printf("BLE-HAL: whitelist address exist\n");
    return BLE_RESULT_INVALID_PARAM;
  }
#endif
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
#if BLE_WITH_SCANNER
/*---------------------------------------------------------------------------*/
volatile bool in_scan_mode;
/*---------------------------------------------------------------------------*/
#if BLE_WITH_SCANNER_CALLBACK
static void
scanner_rx_packet_process(uint8_t *dataEnrty)
{
  uint8_t index = 0;
  int8_t rssi = 0;
  uint8_t channel = 0;
  uint8_t dataOut[38] =
  { 0 };
  uint16_t length = 0;

  rfc_bleScannerPar_t *p = (rfc_bleScannerPar_t *)scanner_param.param_buf;
  rfc_dataEntry_t *entry = (rfc_dataEntry_t *)dataEnrty;
  /*radio timestamp */
  uint32_t rat_timestamp = 0;

  index += sizeof(rfc_dataEntry_t); /*8 */
  if(entry->config.lenSz == 1) {
    if(p->rxConfig.bIncludeLenByte == 1) {
      length = dataEnrty[index]
        - (p->rxConfig.bAppendRssi + p->rxConfig.bAppendStatus
           + 4 * p->rxConfig.bAppendTimestamp);
      index += 1;
    } else {
      LOG_ERR("bIncludeLenByte=0");
      return;
    }
  } else {
    LOG_ERR("Empty packet invalid");
    return;
  }
  if(length > 50) {
    LOG_ERR("Length bigger than max ble-adv length");
  }
  memcpy(dataOut, dataEnrty + index, length);
  index += length;
  if(p->rxConfig.bIncludeCrc == 1) {
  }
  if(p->rxConfig.bAppendRssi == 1) {
    rssi = (int8_t)dataEnrty[index];
    index += 1;
  }
  if(p->rxConfig.bAppendStatus == 1) {
    rfc_bleRxStatus_t *rx_status =
      (rfc_bleRxStatus_t *)&dataEnrty[index];
    channel = rx_status->status.channel;
    if(channel == 0x3F) {
      LOG_ERR("RX channel invalid");
      return;
    }
    index += 1;
  }
  if(p->rxConfig.bAppendTimestamp == 1) {
    /* get the timestamp */
    memcpy(&rat_timestamp, dataEnrty + index, 4);
    /*do time synchronisation at some point using radio timestamp */
    LOG_DBG("Timestamp appended %lu", rat_timestamp);
  }
  extern void ble_scanner_callback(const uint8_t *rx_data, uint16_t length, int rssi,
                                   uint8_t channel, uint32_t timestamp);
  ble_scanner_callback((uint8_t *)&dataOut[0], length, rssi, channel, rat_timestamp);
  LOG_DBG("length:%d rssi:%d channel:%d", length - 2, rssi, channel);
  return;
}
#endif
/*---------------------------------------------------------------------------*/
static void
scan_event_start(struct rtimer *t, void *ptr)
{
  ble_scanner_param_t *param = (ble_scanner_param_t *)ptr;
  rtimer_clock_t wakeup;
  rtimer_clock_t scan_window_ticks;

  bool interrupts_disabled;
  /*interrupts_disabled = ti_lib_int_master_disable(); */
  interrupts_disabled = false;
  /*rf_core_cmd_tx_done_en(true); */
  param->start_rt = t->time; /* reset the starting time in case we're too slow */

  if(on() != BLE_RESULT_OK) {
    LOG_ERR("BLE-HAL: scan event: could not enable rf core\n");
    param->start_rt = param->start_rt
      + ticks_from_unit(param->scan_interval, TIME_UNIT_MS);
    wakeup = param->start_rt - SCANNER_PREPROCESSING_TIME_TICKS;
    safe_set_rtimer(&param->timer, wakeup, 0, scan_event_start,
                    (void *)param);

    /* Re-enable interrupts */
    if(!interrupts_disabled) {
      ti_lib_int_master_enable();
    }
    return;
  }

  in_scan_mode = true;

  scan_window_ticks = ticks_from_unit(param->scan_window, TIME_UNIT_MS);

  if(param->scan_channel < BLE_ADV_CHANNEL_1) {
    /* Initialize the channel */
    param->scan_channel = BLE_ADV_CHANNEL_1;
  } else {
    /* Go through all ADV channels in order */
    param->scan_channel++;
    if(param->scan_channel > BLE_ADV_CHANNEL_3) {
      param->scan_channel = BLE_ADV_CHANNEL_1;
    }
  }
  rf_ble_cmd_create_scanner_params_fltr(
    param->param_buf, &param->rx_queue, param->scan_type,
    ticks_to_unit(scan_window_ticks, TIME_UNIT_RF_CORE),
    param->scan_req_data_len, param->scan_req_data, param->own_addr_type,
    (uint8_t *)BLE_ADDR_LOCATION,
    (rfc_bleWhiteListEntry_t *)param->whitelist_buffers[0]);

  rf_ble_cmd_create_scanner_cmd(param->cmd_buf, param->scan_channel,
                                param->param_buf, param->output_buf);
  volatile rfc_dataEntry_t *entry =
    (rfc_dataEntry_t *)param->rx_queue.pCurrEntry;
  do {
    entry->status = DATA_ENTRY_PENDING;
    entry = (rfc_dataEntry_t *)entry->pNextEntry;
  } while(entry != (rfc_dataEntry_t *)param->rx_queue.pCurrEntry);
  LOG_DBG("Start scanning  ");
  rf_ble_cmd_send(param->cmd_buf);

  wakeup = param->start_rt
    + scan_window_ticks - SCANNER_PREPROCESSING_TIME_TICKS;
  safe_set_rtimer(&param->timer, wakeup, 0, scan_event_end, (void *)param);

  /* Re-enable interrupts */
  if(!interrupts_disabled) {
    ti_lib_int_master_enable();
  }
}
/*---------------------------------------------------------------------------*/
static void
scan_event_end(struct rtimer *t, void *ptr)
{
  ble_scanner_param_t *param = (ble_scanner_param_t *)ptr;
  rtimer_clock_t wakeup;

  bool interrupts_disabled;
  interrupts_disabled = ti_lib_int_master_disable();
  /*rf_core_cmd_tx_done_en(true); */

  scanner_rx((ble_scanner_param_t *)&scanner_param);
  LOG_INFO("End scanning  ");
  rf_ble_cmd_wait(param->cmd_buf);

  off();

  /*process_post(&ble_hal_rx_process, scan_rx_data_event, param); */

  if(!param->active) {
    LOG_INFO("stop scanning\n");
    in_scan_mode = false;
    /* Re-enable interrupts */
    if(!interrupts_disabled) {
      ti_lib_int_master_enable();
    }
    return;
  }

  param->start_rt = param->start_rt
    + ticks_from_unit(param->scan_interval, TIME_UNIT_MS);
  wakeup = scanner_param.start_rt - SCANNER_PREPROCESSING_TIME_TICKS;

  safe_set_rtimer(&param->timer, wakeup, 0, scan_event_start, (void *)param);

  /* Re-enable interrupts */
  if(!interrupts_disabled) {
    ti_lib_int_master_enable();
  }
}
#endif
/*---------------------------------------------------------------------------*/
static ble_result_t
set_scan_enable(unsigned short enable, unsigned short filter_duplicates)
{
#if BLE_WITH_SCANNER
  ble_scanner_param_t *param;

  param = &scanner_param;

  LOG_INFO("set_scan_enable %u\n", enable);

  /* TODO: what to do with filter_duplicates? */

  if(enable && !param->active) {
    in_scan_mode = true;
    scanner_param.start_rt = RTIMER_NOW()
      + ticks_from_unit(scanner_param.scan_interval,
                        TIME_UNIT_1_25_MS);
    safe_set_rtimer(&scanner_param.timer, scanner_param.start_rt, 0,
                    scan_event_start, (void *)&scanner_param);
    param->active = 1;
  } else if(!enable && param->active) {
    param->active = 0;

    /* busy wait until the scan operation is finished */
    while(in_scan_mode) {
      watchdog_periodic();
    }
  }

#endif

  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
send_frame(ble_conn_param_t *conn, uint8_t *data, uint8_t data_len,
           uint8_t frame_type)
{
  uint8_t *tx_buffer = tx_queue_get_buffer(conn);
  if(tx_buffer == NULL) {
    LOG_WARN(
      "BLE-HAL: send_frame: no TX buffer available (conn_handle: 0x%04X)\n",
      conn->conn_handle);
    return BLE_RESULT_ERROR;
  }
  if(data_len > CONN_BLE_BUFFER_SIZE) {
    LOG_WARN("BLE-HAL: send_frame: data too long (%d bytes)\n", data_len);
    return BLE_RESULT_ERROR;
  }

  memcpy(TX_ENTRY_DATA_PTR(tx_buffer), data, data_len);
  TX_ENTRY_LENGTH(tx_buffer) = data_len + 1;
  TX_ENTRY_STATUS(tx_buffer) = DATA_ENTRY_QUEUED;
  TX_ENTRY_FRAME_TYPE(tx_buffer) = frame_type;
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
connection_update(unsigned int connection_handle, unsigned int conn_interval,
                  unsigned int conn_latency, unsigned int supervision_timeout)
{
  uint8_t len = 0;
  uint8_t data[24];
  ble_conn_param_t *conn = get_connection_for_handle(connection_handle);

  if(conn == NULL) {
    return BLE_RESULT_ERROR;
  }

  LOG_INFO(
    "connection_update: handle: 0x%04X, interval: %4d, latency: %2d, timeout: %4d\n",
    connection_handle, conn_interval, conn_latency, supervision_timeout);
#if UIP_CONF_ROUTER
  uint16_t instant = conn->counter + CONN_UPDATE_DELAY;
  /* prepare connection update packet */
  data[0] = BLE_LL_CONN_UPDATE_REQ;
  data[1] = conn->win_size;
  data[2] = 0;
  data[3] = 0;
  memcpy(&data[4], &conn_interval, 2);
  memcpy(&data[6], &conn_latency, 2);
  memcpy(&data[8], &supervision_timeout, 2);
  memcpy(&data[10], &instant, 2);
  len = 12;
  /* set new connection */
  conn->conn_update_win_size = conn->win_size;
  conn->conn_update_interval = conn_interval;
  conn->conn_update_latency = conn_latency;
  conn->conn_update_timeout = supervision_timeout;
  conn->conn_update_counter = instant;

  if(send_frame(conn, data, len, BLE_DATA_PDU_LLID_CONTROL) != BLE_RESULT_OK) {
    LOG_ERR("connection_update: send frame was NOT successful\n");
    return BLE_RESULT_ERROR;
  }
#else
  data[0] = BLE_LL_CONN_PARAM_REQ;
  memcpy(&data[1], &conn_interval, 2);  /* interval min */
  memcpy(&data[3], &conn_interval, 2);  /* interval max */
  memcpy(&data[5], &conn_latency, 2);   /* latency */
  memcpy(&data[7], &supervision_timeout, 2);    /* supervision timeout */
  memcpy(&data[9], &conn_interval, 1);  /* preferred periodicity */
  memcpy(&data[10], &conn->counter, 2); /* referenc conn event count */
  memset(&data[12], 0xFF, 12);      /* offset 0 to 5 */
  len = 24;

  if(send_frame(conn, data, len, BLE_DATA_PDU_LLID_CONTROL) != BLE_RESULT_OK) {
    LOG_ERR("connection_update: send frame was NOT successful\n");
    return BLE_RESULT_ERROR;
  }
#endif
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
send(void *buf, unsigned short buf_len)
{
  uint16_t loop_data;
  uint16_t loop_conn;
  ble_conn_param_t *conn;
  uint8_t *data;
  uint16_t data_len;
  linkaddr_t dest_addr;
  linkaddr_t conn_addr;
  uint8_t result;

  linkaddr_copy(&dest_addr, packetbuf_addr(PACKETBUF_ADDR_RECEIVER));

  LOG_DBG("ble-hal: sending %d bytes\n", buf_len);

  for(loop_conn = 0; loop_conn < conn_counter; loop_conn++) {
    conn = &conn_param[loop_conn];
    ble_addr_to_eui64(conn_addr.u8, conn->peer_address);
    if((linkaddr_cmp(&dest_addr, &linkaddr_null) != 0)
       || (linkaddr_cmp(&dest_addr, &conn_addr) != 0)) {
      for(loop_data = 0; loop_data < buf_len; loop_data +=
            CONN_BLE_BUFFER_SIZE) {
        data = &((uint8_t *)buf)[loop_data];
        data_len = MIN((buf_len - loop_data), CONN_BLE_BUFFER_SIZE);
        if(loop_data == 0) {
          result = send_frame(conn, data, data_len,
                              BLE_DATA_PDU_LLID_DATA_MESSAGE);
        } else {
          result = send_frame(conn, data, data_len,
                              BLE_DATA_PDU_LLID_DATA_FRAGMENT);
        }
        if(result != BLE_RESULT_OK) {
          LOG_WARN("ble-hal: send was unsuccessful\n");
          return result;
        }
      }
    }
  }
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static ble_result_t
read_connection_interval(unsigned int conn_handle, unsigned int *conn_interval)
{
  ble_conn_param_t *conn = get_connection_for_handle(conn_handle);
  if(conn == NULL) {
    memset(conn_interval, 0x00, sizeof(uint16_t));
    return BLE_RESULT_ERROR;
  }
  memcpy(conn_interval, &conn->interval, sizeof(uint16_t));
  return BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
const struct ble_hal_driver ble_hal =
{ reset, read_bd_addr, read_scanner_output, read_buffer_size, set_adv_param,
  read_adv_channel_tx_power, set_adv_data, set_scan_resp_data,
  send_adv_chain, send_single_adv, set_adv_enable, set_scan_param,
  set_scan_param_fltr, set_scan_whitelist, set_adv_whitelist,
  set_scan_req_data, set_scan_enable,
  NULL,
  NULL, connection_update,
  NULL, send,
  NULL, read_connection_interval };
/*---------------------------------------------------------------------------*/
#if BLE_WITH_ADV_RX
/*---------------------------------------------------------------------------*/
static void
print_advertisment(const uint8_t *rx_data, uint16_t length)
{
  static const char digits[] = "0123456789ABCDEF";
  int i;
  for(i = 0; i < length; ++i) {
    putchar(digits[rx_data[i] >> 4]);
    putchar(digits[rx_data[i] & 0xF]);
    putchar(' ');
  }
  putchar('\n');
}
/*---------------------------------------------------------------------------*/
#endif
/*---------------------------------------------------------------------------*/
#if BLE_WITH_SCANNER
/*---------------------------------------------------------------------------*/
unsigned num_packets[255];
unsigned num_ok_packets[255];

static void
scanner_rx(ble_scanner_param_t *param)
{

  while(RX_ENTRY_STATUS(param->rx_queue_current) == DATA_ENTRY_FINISHED) {
#if BLE_WITH_SCANNER_CALLBACK
    scanner_rx_packet_process(RX_ENTRY_DATA_PTR(param->rx_queue_current));
#endif
#if BLE_WITH_ADV_CALLBACK
    uint8_t *rx_data = RX_ENTRY_DATA_PTR(param->rx_queue_current);
    uint16_t length = RX_ENTRY_DATA_LENGTH(param->rx_queue_current);

    int8_t rssi = (int8_t)*(rx_data + length - 2);
    uint8_t status = *(rx_data + length - 1);

    /* LOG_DBG("scanner_rx an adv\n"); */
    if(length >= 10) {
      if(!(status & 0x80)) { /* good packet? */

        /* call the user's callback */
        extern void ble_adv_callback(const uint8_t *rx_data, uint16_t length, int rssi);
        ble_adv_callback(rx_data, length, rssi);
      }
    }
    num_packets[-rssi]++;
    if(!(status & 0x80)) {
      /* bad crc flag is not set */
      num_ok_packets[-rssi]++;
    }

    memset(rx_data, 0x0, length); /* XXX */
#endif

    /* free current entry (clear BLE data length & reset status) */
    RX_ENTRY_DATA_LENGTH(param->rx_queue_current) = 0;
    RX_ENTRY_STATUS(param->rx_queue_current) = DATA_ENTRY_PENDING;
    param->rx_queue_current = RX_ENTRY_NEXT_ENTRY(param->rx_queue_current);
  }

  rfc_bleScannerOutput_t o;
  memcpy(&o, scanner_param.output_buf, sizeof(o));
  memset(scanner_param.output_buf, 0x0, sizeof(scanner_param.output_buf));
}
/*---------------------------------------------------------------------------*/
#endif
/*---------------------------------------------------------------------------*/
#if BLE_WITH_ADV_RX
/*---------------------------------------------------------------------------*/
static void
advertising_rx(ble_adv_param_t *param)
{
  uint8_t i;
  uint8_t offset = 14;
  uint8_t *rx_data;
  ble_conn_param_t *c_param = &conn_param[0];
  rtimer_clock_t wakeup;

  while(RX_ENTRY_STATUS(param->rx_queue_current) == DATA_ENTRY_FINISHED) {
    rx_data = RX_ENTRY_DATA_PTR(param->rx_queue_current);

    if(CMD_GET_STATUS(
         param->cmd_buf) == RF_CORE_RADIO_OP_STATUS_BLE_DONE_CONNECT) {
      /* parsing connection parameter */
      for(i = 0; i < BLE_ADDR_SIZE; i++) {
        c_param->peer_address[i] = rx_data[BLE_ADDR_SIZE + 1 - i];
      }
      memcpy(&c_param->access_address, &rx_data[offset], 4);
      memcpy(&c_param->crc_init_0, &rx_data[offset + 4], 1);
      memcpy(&c_param->crc_init_1, &rx_data[offset + 5], 1);
      memcpy(&c_param->crc_init_2, &rx_data[offset + 6], 1);
      memcpy(&c_param->win_size, &rx_data[offset + 7], 1);
      memcpy(&c_param->win_offset, &rx_data[offset + 8], 2);
      memcpy(&c_param->interval, &rx_data[offset + 10], 2);
      memcpy(&c_param->latency, &rx_data[offset + 12], 2);
      memcpy(&c_param->timeout, &rx_data[offset + 14], 2);
      memcpy(&c_param->channel_map, &rx_data[offset + 16], 5);
      memcpy(&c_param->hop, &rx_data[offset + 21], 1);
      memcpy(&c_param->sca, &rx_data[offset + 21], 1);
      memcpy(&c_param->timestamp_rt, &rx_data[offset + 24], 4);

      /* convert all received timing values to rtimer ticks */

      c_param->timestamp_rt = ticks_from_unit(c_param->timestamp_rt,
                                              TIME_UNIT_RF_CORE);
      c_param->hop = c_param->hop & 0x1F;
      c_param->sca = (c_param->sca >> 5) & 0x07;

      LOG_INFO(
        "connection created: conn_int: %4u, latency: %3u, channel_map: %8llX\n",
        c_param->interval, c_param->latency, c_param->channel_map);

      LOG_DBG("access address: 0x%08lX\n", c_param->access_address);
      LOG_DBG("crc0: 0x%02X\n", c_param->crc_init_0);
      LOG_DBG("crc1: 0x%02X\n", c_param->crc_init_1);
      LOG_DBG("crc2: 0x%02X\n", c_param->crc_init_2);
      LOG_DBG("win_size:       %4u\n", c_param->win_size);
      LOG_DBG("win_offset:     %4u\n", c_param->win_offset);
      LOG_DBG("interval:       %4u\n", c_param->interval);
      LOG_DBG("latency:        %4u\n", c_param->latency);
      LOG_DBG("timeout:        %4u\n", c_param->timeout);
      LOG_DBG("channel_map:    %llX\n", c_param->channel_map);

      /* calculate the first anchor point
       * (add an interval, because we skip the first connection event ) */
      wakeup = c_param->timestamp_rt + ticks_from_unit(c_param->win_offset,
                                                       TIME_UNIT_1_25_MS) - CONN_WINDOW_WIDENING_TICKS;
      wakeup += ticks_from_unit(c_param->interval,
                                TIME_UNIT_1_25_MS) - CONN_PREPROCESSING_TIME_TICKS;
      rtimer_set(&c_param->timer, wakeup, 0, connection_event_slave,
                 (void *)c_param);

      /* initialization for the connection */
      c_param->counter = 0;
      c_param->unmapped_channel = 0;
      c_param->conn_handle = conn_counter;
      c_param->active = 1;
      conn_counter++;
      LOG_INFO("BLE-HAL: connection (0x%04X) created\n",
               c_param->conn_handle);
    } else if(CMD_GET_STATUS(
                param->cmd_buf) == RF_CORE_RADIO_OP_STATUS_BLE_DONE_OK) {

      print_advertisment(rx_data,
                         RX_ENTRY_LENGTH(param->rx_queue_current));
    }

    /* free current entry (clear BLE data length & reset status) */
    RX_ENTRY_DATA_LENGTH(param->rx_queue_current) = 0;
    RX_ENTRY_STATUS(param->rx_queue_current) = DATA_ENTRY_PENDING;
    param->rx_queue_current = RX_ENTRY_NEXT_ENTRY(param->rx_queue_current);
  }

  rfc_bleAdvOutput_t o;
  memcpy(&o, adv_param.output_buf, sizeof(o));
  memset(adv_param.output_buf, 0x0, sizeof(adv_param.output_buf));

  printf(" adv params: rx=%u ig=%u crc=%u rssi=%d\n", o.nRxScanReq, o.nRxNok,
         o.nRxIgnored, o.lastRssi);
}
/*---------------------------------------------------------------------------*/
static void
advertising_event(struct rtimer *t, void *ptr)
{
  ble_adv_param_t *param = (ble_adv_param_t *)ptr;
  uint32_t wakeup;

  if(on() != BLE_RESULT_OK) {
    LOG_ERR("BLE-HAL: advertising event: could not enable rf core\n");
    return;
  }

  rf_ble_cmd_create_adv_params(param->param_buf, &param->rx_queue,
                               param->adv_data_len, param->adv_data,
                               param->scan_rsp_data_len, param->scan_rsp_data,
                               param->own_addr_type,
                               (uint8_t *)BLE_ADDR_LOCATION);

  /* advertising on advertisement channel 1*/
  /*if(param->channel_map & BLE_ADV_CHANNEL_1_MASK) { */
  rf_ble_cmd_create_adv_cmd(param->cmd_buf, BLE_ADV_CHANNEL_1,
                            param->param_buf, param->output_buf);

  rf_ble_cmd_send(param->cmd_buf);
  rf_ble_cmd_wait(param->cmd_buf);
  /*} */

  /* TODO: need to add more channels! */

  off();
  advertising_rx(param);

  if(conn_param[0].active == 1) {
    LOG_INFO("stop advertising\n");
    return;
  }

  param->start_rt = param->start_rt
    + ticks_from_unit(param->adv_interval, TIME_UNIT_MS);
  wakeup = adv_param.start_rt - ADV_PREPROCESSING_TIME_TICKS;
  rtimer_set(&param->timer, wakeup, 0, advertising_event, (void *)param);
}
/*---------------------------------------------------------------------------*/
static void
update_data_channel(ble_conn_param_t *param)
{
  uint8_t i;
  uint8_t j;
  uint8_t remap_index;
  /* perform the data channel selection according to BLE standard */

  /* calculate unmapped channel*/
  param->unmapped_channel = (param->unmapped_channel + param->hop)
    % (BLE_DATA_CHANNEL_MAX + 1);

  /* map the calculated channel */
  if(param->channel_map & (1ULL << param->unmapped_channel)) {
    /* channel is marked as used */
    param->mapped_channel = param->unmapped_channel;
  } else {
    remap_index = param->unmapped_channel % param->num_used_channels;
    j = 0;
    for(i = 0; i < (BLE_DATA_CHANNEL_MAX + 1); i++) {
      if(param->channel_map & (1ULL << i)) {
        if(j == remap_index) {
          param->mapped_channel = i;
        }
        j++;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
#endif /* BLE_WITH_ADV_RX */
/*---------------------------------------------------------------------------*/
static void
process_ll_ctrl_msg(ble_conn_param_t *conn, uint8_t input_len, uint8_t *input,
                    uint8_t *output_len, uint8_t *output)
{
  uint8_t op_code = input[0];
  uint16_t interval;
  uint16_t latency;
  uint16_t timeout;
  uint64_t channel_map = 0;
  uint16_t instant = 0;
  uint8_t i;

  if(op_code == BLE_LL_CONN_UPDATE_REQ) {
    LOG_INFO("BLE-HAL: connection update request received\n");
    memcpy(&conn->conn_update_win_size, &input[1], 1);
    memcpy(&conn->conn_update_win_offset, &input[2], 2);
    memcpy(&conn->conn_update_interval, &input[4], 2);
    memcpy(&conn->conn_update_latency, &input[6], 2);
    memcpy(&conn->conn_update_timeout, &input[8], 2);
    memcpy(&conn->conn_update_counter, &input[10], 2);
  } else if(op_code == BLE_LL_CHANNEL_MAP_REQ) {
    LOG_INFO("BLE-HAL: channel map update received\n");
    memcpy(&channel_map, &input[1], 5);
    memcpy(&instant, &input[6], 2);

    conn->channel_update_channel_map = channel_map;
    conn->channel_update_counter = instant;
    conn->channel_update_num_used_channels = 0;
    for(i = 0; i <= BLE_DATA_CHANNEL_MAX; i++) {
      if(channel_map & (1ULL << i)) {
        conn->channel_update_num_used_channels++;
      }
    }
  } else if(op_code == BLE_LL_FEATURE_REQ) {
    LOG_INFO("BLE-HAL: feature request received\n");
    output[0] = BLE_LL_FEATURE_RSP;
    memset(&output[1], 0x00, 8);
    *output_len = 9;
  } else if(op_code == BLE_LL_VERSION_IND) {
    LOG_INFO("BLE-HAL: version request received\n");
    output[0] = BLE_LL_VERSION_IND;
    output[1] = 7;
    memset(&output[2], 0xAA, 4);
    *output_len = 6;
  } else if(op_code == BLE_LL_CONN_PARAM_REQ) {
    LOG_INFO("BLE-HAL: connection parameter request received\n");
    memcpy(&interval, &input[1], 2);    /* use interval min */
    memcpy(&latency, &input[5], 2);
    memcpy(&timeout, &input[7], 2);
    connection_update(conn->conn_handle, interval, latency, timeout);
  } else {
    LOG_WARN("BLE-HAL: unknown LL control code: %02X\n", op_code);
  }
}
/*---------------------------------------------------------------------------*/
static void
connection_rx(ble_conn_param_t *param)
{
  uint8_t header_offset = 2;
  uint8_t *rx_data;
  uint16_t len;
  uint8_t channel;
  uint8_t frame_type;
  uint8_t more_data;
  uint8_t rssi;
  linkaddr_t sender_addr;
  rfc_bleMasterSlaveOutput_t *out_buf =
    (rfc_bleMasterSlaveOutput_t *)param->output_buf;

  uint8_t output_len = 0;
  uint8_t output[26];

  while(RX_ENTRY_STATUS(param->rx_queue_current) == DATA_ENTRY_FINISHED) {
    rx_data = RX_ENTRY_DATA_PTR(param->rx_queue_current);
#if RADIO_CONF_BLE5
    len = RX_ENTRY_DATA_LENGTH(param->rx_queue_current) - 7 - 2;  /* last 9 bytes are status, timestamp, ... */
#else
    len = RX_ENTRY_DATA_LENGTH(param->rx_queue_current) - 6 - 2;   /* last 8 bytes are status, timestamp, ... */
#endif
    channel = (rx_data[len + 3] & 0x3F);
    frame_type = rx_data[0] & 0x03;
    more_data = (rx_data[0] & 0x10) >> 4;

    if(frame_type == BLE_DATA_PDU_LLID_CONTROL) {
      process_ll_ctrl_msg(param, (len - header_offset),
                          &rx_data[header_offset], &output_len, output);
      if(output_len > 0) {
        send_frame(param, output, output_len,
                   BLE_DATA_PDU_LLID_CONTROL);
      }
    } else if(frame_type == BLE_DATA_PDU_LLID_DATA_MESSAGE) {
      packetbuf_clear();
      memcpy(packetbuf_dataptr(), &rx_data[header_offset], len);
      packetbuf_set_datalen(len);
      rssi = out_buf->lastRssi;
      ble_addr_to_eui64(sender_addr.u8, param->peer_address);
      packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);
      packetbuf_set_attr(PACKETBUF_ATTR_CHANNEL, channel);
      packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &linkaddr_node_addr);
      packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &sender_addr);
      packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME_BLE_RX_EVENT);
      if((!more_data) || (len < CONN_BLE_BUFFER_SIZE)) {
        NETSTACK_MAC.input();
      }
    } else if(frame_type == BLE_DATA_PDU_LLID_DATA_FRAGMENT) {
      memcpy((packetbuf_dataptr() + packetbuf_datalen()),
             &rx_data[header_offset], len);
      packetbuf_set_datalen(packetbuf_datalen() + len);
      if((!more_data) || (len < CONN_BLE_BUFFER_SIZE)) {
        NETSTACK_MAC.input();
      }
    }

    /* free current entry (clear BLE data length & reset status) */
    RX_ENTRY_DATA_LENGTH(param->rx_queue_current) = 0;
    RX_ENTRY_STATUS(param->rx_queue_current) = DATA_ENTRY_PENDING;
    param->rx_queue_current = RX_ENTRY_NEXT_ENTRY(param->rx_queue_current);
  }
}
/*---------------------------------------------------------------------------*/
#if BLE_WITH_ADV_RX
/*---------------------------------------------------------------------------*/
static void
connection_event_slave(struct rtimer *t, void *ptr)
{

  ble_conn_param_t *conn = (ble_conn_param_t *)ptr;
  rfc_bleMasterSlaveOutput_t *output =
    (rfc_bleMasterSlaveOutput_t *)conn->output_buf;
  uint8_t first_packet = 0;
  rtimer_clock_t wakeup;
  uint8_t i;
  uint8_t tx_data = tx_queue_data_to_transmit(conn);

  if(conn->counter == 0) {
    /* the slave skips connection event 0, because it is usually too early */
    conn->start_rt = conn->timestamp_rt + ticks_from_unit(conn->win_offset,
                                                          TIME_UNIT_1_25_MS) - CONN_WINDOW_WIDENING_TICKS;
    update_data_channel(conn);
    first_packet = 1;
  }
  conn->counter++;

  /* connection timing */
  if(conn->counter == conn->conn_update_counter) {
    conn->start_rt += ticks_from_unit(
      conn->interval + conn->conn_update_win_offset,
      TIME_UNIT_1_25_MS);

    conn->win_size = conn->conn_update_win_size;
    conn->win_offset = conn->conn_update_win_offset;
    conn->interval = conn->conn_update_interval;
    conn->latency = conn->conn_update_latency;
    conn->timeout = conn->conn_update_timeout;
    conn->conn_update_win_size = 0;
    conn->conn_update_win_offset = 0;
    conn->conn_update_interval = 0;
    conn->conn_update_latency = 0;
    conn->conn_update_timeout = 0;
  } else if(output->pktStatus.bTimeStampValid) {
    conn->start_rt = ticks_from_unit(output->timeStamp, TIME_UNIT_RF_CORE)
      + ticks_from_unit(conn->interval,
                        TIME_UNIT_1_25_MS) - CONN_WINDOW_WIDENING_TICKS;
  } else {
    conn->start_rt += ticks_from_unit(conn->interval, TIME_UNIT_1_25_MS);
  }

  /* connection channel */
  if(conn->channel_update_counter == conn->counter) {
    conn->channel_map = conn->channel_update_channel_map;
    conn->num_used_channels = conn->channel_update_num_used_channels;
    conn->channel_update_counter = 0;
    conn->channel_update_channel_map = 0;
    conn->channel_update_num_used_channels = 0;
  }
  update_data_channel(conn);

  if(tx_data || (conn->skipped_events >= conn->latency)
     || (conn->counter < CONN_EVENT_LATENCY_THRESHOLD)) {
    /* participating in the connection event */
    conn->skipped_events = 0;
    rf_ble_cmd_create_slave_params(
      conn->param_buf, &conn->rx_queue, &conn->tx_queue,
      conn->access_address, conn->crc_init_0, conn->crc_init_1,
      conn->crc_init_2,
      ticks_to_unit(ticks_from_unit(conn->win_size, TIME_UNIT_1_25_MS),
                    TIME_UNIT_RF_CORE),
      ticks_to_unit(CONN_WINDOW_WIDENING_TICKS, TIME_UNIT_RF_CORE),
      first_packet);

    rf_ble_cmd_create_slave_cmd(
      conn->cmd_buf, conn->mapped_channel, conn->param_buf,
      conn->output_buf, ticks_to_unit(conn->start_rt, TIME_UNIT_RF_CORE));

    if(on() != BLE_RESULT_OK) {
      LOG_ERR("connection_event: could not enable radio core\n");
      return;
    }

    /* append TX buffers */
    for(i = 0; i < CONN_TX_BUFFERS_NUM; i++) {
      if(TX_ENTRY_STATUS(conn->tx_buffers[i]) == DATA_ENTRY_QUEUED) {
        TX_ENTRY_STATUS(conn->tx_buffers[i]) = DATA_ENTRY_PENDING;
        rf_ble_cmd_add_data_queue_entry(&conn->tx_queue,
                                        conn->tx_buffers[i]);
      }
    }
    rf_ble_cmd_send(conn->cmd_buf);
    rf_ble_cmd_wait(conn->cmd_buf);
    off();

    if(CMD_GET_STATUS(conn->cmd_buf) != RF_CORE_RADIO_OP_STATUS_BLE_DONE_OK) {
      LOG_DBG(
        "command status: 0x%04X; connection event counter: %d, channel: %d\n",
        CMD_GET_STATUS(conn->cmd_buf), conn->counter,
        conn->mapped_channel);
    }

    /* free finished TX buffers */
    for(i = 0; i < CONN_TX_BUFFERS_NUM; i++) {
      if(TX_ENTRY_STATUS(conn->tx_buffers[i]) == DATA_ENTRY_FINISHED) {
        TX_ENTRY_STATUS(conn->tx_buffers[i]) = DATA_ENTRY_FREE;
        TX_ENTRY_LENGTH(conn->tx_buffers[i]) = 0;
        TX_ENTRY_NEXT_ENTRY(conn->tx_buffers[i]) = NULL;
      }
    }
  } else {
    /* skipping connection event */
    conn->skipped_events++;
    output->pktStatus.bTimeStampValid = 0;
  }
  wakeup = conn->start_rt + ticks_from_unit(conn->interval,
                                            TIME_UNIT_1_25_MS) - CONN_PREPROCESSING_TIME_TICKS;
  rtimer_set(&conn->timer, wakeup, 0, connection_event_slave, ptr);
  process_post(&ble_hal_rx_process, conn_rx_data_event, ptr);
}
/*---------------------------------------------------------------------------*/
#endif /* BLE_WITH_ADV_RX */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_hal_rx_process, ev, data)
{
  PROCESS_BEGIN()
  ;

  LOG_DBG("BLE-HAL: rx process start\n");

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(
      ev == conn_rx_data_event || ev == scan_rx_data_event);

    if(ev == conn_rx_data_event) {
      ble_conn_param_t *conn = (ble_conn_param_t *)data;
      rfc_bleMasterSlaveOutput_t *output =
        (rfc_bleMasterSlaveOutput_t *)conn->output_buf;
      uint8_t tx_buffers_sent;

      /* notify upper layers (L2CAP) when TX buffers were successfully transmitted */
      tx_buffers_sent = output->nTxEntryDone - conn->tx_buffers_sent;
      if(tx_buffers_sent != 0) {
        conn->tx_buffers_sent = output->nTxEntryDone;
        packetbuf_set_datalen(0);
        packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE,
                           FRAME_BLE_TX_EVENT);
        NETSTACK_MAC.input();
      }

      /* handle RX buffers */
      connection_rx(conn);

      /* generate an event if the connection parameter were updated */
      if(conn->counter == conn->conn_update_counter) {
        packetbuf_set_datalen(0);
        packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE,
                           FRAME_BLE_CONNECTION_UPDATED);
        NETSTACK_MAC.input();
      }

#if BLE_WITH_SCANNER
    } else if(ev == scan_rx_data_event) {
      /* process the advertisements */
      /*LOG_DBG("BLE-HAL: advertisements\n"); */

      ble_scanner_param_t *param = (ble_scanner_param_t *)data;
      scanner_rx(param);
#endif
    }
  }

  PROCESS_END();
}
#if BLE_WITH_SCANNER

PROCESS_THREAD(ble_hal_interrupt_handler, ev, data)
{

  PROCESS_BEGIN()
  ;
  LOG_DBG("BLE-HAL: interrupt process start\n");
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    /*if(ev == ble_hal_interrupt_event) { */
    volatile rfc_dataEntry_t *entry =
      (rfc_dataEntry_t *)scanner_param.rx_queue.pCurrEntry;
    volatile rfc_dataEntry_t *last_entry = entry;
    uint8_t num_ready = 0;
    uint32_t int_value = (uint32_t)data;
    if(int_value & (IRQ_LAST_FG_COMMAND_DONE | IRQ_LAST_COMMAND_DONE)) {
      LOG_DBG("Mission Complete################################\n");
      rfc_bleScannerOutput_t o;
      memcpy(&o, scanner_param.output_buf, sizeof(o));
      LOG_DBG("ADV packets nRxAdvOk: %d\n", o.nRxAdvOk);
      LOG_DBG("ADV packets nRxAdvIgnored: %d\n", o.nRxAdvIgnored);
      LOG_DBG("SCAN_RSP packets nRxScanRspOk: %d\n", o.nRxScanRspOk);
      LOG_DBG("SCAN_REQ packets nTxScanReq: %d\n", o.nTxScanReq);

      memset(scanner_param.output_buf, 0x0,
             sizeof(scanner_param.output_buf));
    }
    /*entry = (rfc_dataEntry_t *)entry->pNextEntry; */
    /* Go through all RX buffers and check their status */
    do {
      if(entry->status != DATA_ENTRY_PENDING) {
        last_entry = entry;
      }
      entry = (rfc_dataEntry_t *)entry->pNextEntry;
    }   while(entry != (rfc_dataEntry_t *)scanner_param.rx_queue.pCurrEntry);
    /*scanner_rx(&scanner_param); */
    entry = (rfc_dataEntry_t *)entry->pNextEntry;
    while(entry != last_entry) {
      if(entry->status == DATA_ENTRY_FINISHED) {
#if BLE_WITH_SCANNER_CALLBACK
        scanner_rx_packet_process((uint8_t *)entry);
#endif
#if BLE_WITH_ADV_CALLBACK

        uint16_t length;
        uint8_t *rx_queue_buffer = (uint8_t *)entry;
        uint8_t *rx_data = RX_ENTRY_DATA_PTR(rx_queue_buffer);
        length = RX_ENTRY_DATA_LENGTH(rx_queue_buffer);
        int8_t rssi = (int8_t)*(rx_data + length - 2);

        /* call the user's callback */
        extern void ble_adv_callback(const uint8_t *rx_data, uint16_t length, int rssi);
        /* add channel information */
        ble_adv_callback(rx_data, length, rssi);
#endif
        num_ready += 1;
      }
      entry->status = DATA_ENTRY_PENDING;
      entry = (rfc_dataEntry_t *)entry->pNextEntry;
    }
    /*if(num_ready > 2) { */
    /*scanner_rx(&scanner_param); */
    /*LOG_INFO("Intrerrupt Triggered num_ready:%d", num_ready); */
    /*} */
    /*
       const volatile rfc_bleScannerOutput_t *o =
       (rfc_bleScannerOutput_t *)scanner_param.output_buf;
       LOG_DBG(
       "Radio interrupt:%08lX rx_queue_current:%p pCurrEntry:%p entry:%p last_entry:%p status:%x nRxAdvOk:%d num_ready:%d\n",
       int_value, scanner_param.rx_queue_current,
       scanner_param.rx_queue.pCurrEntry, entry, last_entry, entry->status,
       o->nRxAdvOk, num_ready);
       }
     */
  }
  /*} */
  PROCESS_END();
}
#endif

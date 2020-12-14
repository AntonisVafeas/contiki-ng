/*****************************************************************************************************

   Copyright (C) - All Rights Reserved

   SPHERE (an EPSRC IRC), 2013-2018
   University of Bristol
   University of Reading
   University of Southampton

   Filename: fgbridge.c
   Description: SPI Bridge from BLE to 802.15.4
   Primary Contributor(s): Xenofon (Fontas) Fafoutis (xenofon.fafoutis@bristol.ac.uk)
                         Atis Elsts (atis.elsts@bristol.ac.uk)

 *******************************************************************************************************/

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "ti-lib.h"
#include "fgbridge.h"
#include "ble-adv-def.h"
#include "board.h"
#include "lpm.h"
#include "hw-watchdog.h"
#include "dev/gpio-hal.h"
#include "dev/leds.h"
#include "sys/int-master.h"

#include "rf_ble_cmd.h"

#if SPHERE
#include "sphere.h"
#include "sphere-timestamps.h"
#endif /* SPHERE */

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/

/* 10 minutes */
#define REINITIALIZATION_TIME CLOCK_SECOND * 600
#define SPI_BRIDGE_TIMER CLOCK_SECOND * 5

PROCESS(fgbridge_process, "fgbridge process");

/*---------------------------------------------------------------------------*/

static bool is_operational;

static uint8_t fgbridge_dl_count = 0;

static uint8_t fgbridge_cmd_buf[FGBRIDGE_SPI_FRAME_LENGTH];
static uint8_t fgbridge_rcv_buf[FGBRIDGE_SPI_FRAME_LENGTH];
static uint8_t fgbridge_ble_addr[FGBRIDGE_BLE_ADDR_LENGTH];  /* BLE Address of G (big-endian) */

static wearable_adv_t fgbridge_wearable_adv;
static bool is_fgbridge_wearable_adv_present;

static fgbridge_callback_function *callback;
static void *callback_argument;

static uint8_t scanner_output[FGBRIDGE_MON_FRAME_LENGTH];

static volatile uint32_t num_adv;
static volatile uint32_t num_accepted_adv;

/* Keep the PERIPH domain always on */
LPM_MODULE(fgbridge_spi_module, NULL, NULL, NULL, LPM_DOMAIN_PERIPH);

/*---------------------------------------------------------------------------*/
static gpio_hal_event_handler_t interrupt_handler_object;
/*---------------------------------------------------------------------------*/
static void
fgbridge_irq_handler(gpio_hal_pin_mask_t mask)
{
  if(mask & FGBRIDGE_G2_INT) {
    process_poll(&fgbridge_process);
  }
}
/*---------------------------------------------------------------------------*/
static void
fgbridge_init_irq(void)
{
  /* set priority for RTC to be higher than for edge detect;
   * note that the same level of priority is also ok, as
   * "if an exception occurs with the same priority as the exception being handled,
   * the handler is not preempted, irrespective of the exception number." (TRM p. 231)
   */
  ti_lib_int_priority_set(INT_AON_RTC_COMB, INT_PRI_LEVEL0);
  ti_lib_int_priority_set(INT_AON_GPIO_EDGE, INT_PRI_LEVEL1);
  ti_lib_ioc_io_port_pull_set(FGBRIDGE_IOID_G2_INT, IOC_IOPULL_UP);

  ti_lib_gpio_clear_event_dio(FGBRIDGE_IOID_G2_INT);
  ti_lib_ioc_port_configure_set(FGBRIDGE_IOID_G2_INT, IOC_PORT_GPIO,
                                (IOC_INT_ENABLE | IOC_INPUT_ENABLE | IOC_FALLING_EDGE));
  ti_lib_gpio_set_output_enable_dio(FGBRIDGE_IOID_G2_INT, GPIO_OUTPUT_DISABLE);
  interrupt_handler_object.pin_mask = FGBRIDGE_G2_INT;
  interrupt_handler_object.handler = fgbridge_irq_handler;
  gpio_hal_register_handler(&interrupt_handler_object);
}
/*---------------------------------------------------------------------------*/
static bool
accessible(void)
{
  /* First, check the PD */
  if(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON) {
    return false;
  }

  /* Then check the 'run mode' clock gate */
  if(!(HWREG(PRCM_BASE + PRCM_O_SSICLKGR) & PRCM_SSICLKGR_CLK_EN_SSI1)) {
    return false;
  }
  return true;
}
/*---------------------------------------------------------------------------*/
static void
fgbridge_spi_flush()
{
  if(accessible() == false) {
    /* */
    PRINTF("SPI_flash not accessible\n");
    return;
  }

  uint32_t ul;
  while(ti_lib_ssi_data_get_non_blocking(SSI1_BASE, &ul)) {
  }
}
/*---------------------------------------------------------------------------*/
static bool
fgbridge_spi_transfer(const uint8_t *buf, uint8_t *outbuf, size_t len)
{

  if(accessible() == false) {
    return false;
  }
  size_t out_len = len;
  bool frame_start = false;
  bool good_frame = false;
  while(len > 0) {
    uint32_t ul;
    ti_lib_ssi_data_put(SSI1_BASE, *buf);

    ti_lib_ssi_data_get(SSI1_BASE, &ul);

    *outbuf = (uint8_t)ul;

    if((len == 1) && good_frame && (out_len > 0)) {
    } else {
      len--;
    }
    buf++;
    if(good_frame) {
      outbuf++;
      out_len--;
    } else {
      if((uint8_t)ul == 0x55) {
        frame_start = true;
        outbuf++;
        out_len--;
      }
      if(frame_start && ((uint8_t)ul == 0xAA)) {
        good_frame = true;
        outbuf++;
        out_len--;
      }
    }
  }
  return true;
}
/*---------------------------------------------------------------------------*/
static void
fgbridge_spi_open()
{

  /* Initialise and de-select the CS pin */
  ti_lib_ioc_pin_type_gpio_output(FGBRIDGE_IOID_SPI_CS);
  ti_lib_gpio_write_dio(FGBRIDGE_IOID_SPI_CS, 1);

  /* First, make sure the PERIPHERAL PD is on */
  ti_lib_prcm_power_domain_on(PRCM_DOMAIN_PERIPH);
  while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH)
         != PRCM_DOMAIN_POWER_ON));

  /* Enable clock in active mode */
  ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_SSI1);
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());

  /* SPI configuration */
  ti_lib_ssi_int_disable(SSI1_BASE, SSI_RXOR | SSI_RXFF | SSI_RXTO | SSI_TXFF);
  ti_lib_ssi_int_clear(SSI1_BASE, SSI_RXOR | SSI_RXTO);
  ti_lib_ssi_config_set_exp_clk(SSI1_BASE, ti_lib_sys_ctrl_clock_get(), FGBRIDGE_SPI_FRF, SSI_MODE_MASTER, FGBRIDGE_SPI_RATE, 8);
  ti_lib_ioc_pin_type_ssi_master(SSI1_BASE, FGBRIDGE_IOID_SPI_MISO, FGBRIDGE_IOID_SPI_MOSI, IOID_UNUSED, FGBRIDGE_IOID_SPI_CLK);
  ti_lib_ssi_enable(SSI1_BASE);

  fgbridge_spi_flush();
}
/*---------------------------------------------------------------------------*/

#if 0 /* this code is broken? */
static void
fgbridge_spi_close()
{

  /* Power down SSI1 */
  ti_lib_prcm_peripheral_run_disable(PRCM_PERIPH_SSI1);
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());

  /* Restore pins to a low-consumption state */
  ti_lib_ioc_pin_type_gpio_input(FGBRIDGE_IOID_SPI_MISO);
  ti_lib_ioc_io_port_pull_set(FGBRIDGE_IOID_SPI_MISO, IOC_IOPULL_DOWN);

  ti_lib_ioc_pin_type_gpio_input(FGBRIDGE_IOID_SPI_MOSI);
  ti_lib_ioc_io_port_pull_set(FGBRIDGE_IOID_SPI_MOSI, IOC_IOPULL_DOWN);

  ti_lib_ioc_pin_type_gpio_input(FGBRIDGE_IOID_SPI_CLK);
  ti_lib_ioc_io_port_pull_set(FGBRIDGE_IOID_SPI_CLK, IOC_IOPULL_DOWN);
}
#endif

/*---------------------------------------------------------------------------*/
static bool
fgbridge_transfer(const uint8_t *buf, uint8_t *outbuf, size_t len)
{
  unsigned int end;
  bool ret;
  int_master_status_t status;
  /* disable interrupts to avoid messing with flash */
  if(1) {
    status = int_master_read_and_disable();
  }

  ti_lib_gpio_write_dio(FGBRIDGE_IOID_SPI_CS, 0);

  end = RTIMER_NOW() + FGBRIDGE_CS_DELAY;
  while(RTIMER_CLOCK_LT(RTIMER_NOW(), end));


  ret = fgbridge_spi_transfer(buf, outbuf, len);

  ti_lib_gpio_write_dio(FGBRIDGE_IOID_SPI_CS, 1);

  if(1) {
    /* restore interrupts */
    int_master_status_set(status);
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
static bool
fgbridge_monitor_g(void)
{
  memset(fgbridge_cmd_buf, 0xCD, FGBRIDGE_SPI_FRAME_LENGTH);
  fgbridge_rcv_frame_t *frame;

  fgbridge_cmd_buf[0] = 0x55;
  fgbridge_cmd_buf[1] = 0xAA;
  fgbridge_cmd_buf[2] = fgbridge_dl_count++;
  fgbridge_cmd_buf[3] = 'R';
  fgbridge_cmd_buf[FGBRIDGE_SPI_FRAME_LENGTH - 1] = 0xFF;
  memset(fgbridge_rcv_buf, 0x00, FGBRIDGE_SPI_FRAME_LENGTH);
  if(fgbridge_transfer(fgbridge_cmd_buf, fgbridge_rcv_buf,
                       FGBRIDGE_SPI_FRAME_LENGTH) == false) {
    PRINTF("SPI reset transfer failed\n");
    return false;
  }
  for(size_t i = 0; i < FGBRIDGE_SPI_FRAME_LENGTH; ++i) {
    PRINTF("%.2x", fgbridge_rcv_buf[i]);
  }
  PRINTF("\n");
  frame = (fgbridge_rcv_frame_t *)fgbridge_rcv_buf;
  if((frame->byte_AA != 0xAA) && (frame->command != 'U')) {
    PRINTF("SPI command not present\n");
    return false;
  }
  /*Check the scanner status */
  if(frame->type != FGBRIDGE_UL_NOTIFICATION_MESSAGE) {
    PRINTF("Not notification message\n");
    return false;
  }
  rfc_bleScannerOutput_t *o = (rfc_bleScannerOutput_t *)frame->payload;
  if(o->nRxAdvIgnored == 0) {
    /*no advertisments received */
    PRINTF("no advertisments received\n");
    return false;
  }
  PRINTF("nRxAdvIgnored:%d spi:", o->nRxAdvIgnored);
  memcpy(scanner_output, frame->payload, FGBRIDGE_MON_FRAME_LENGTH);

  return true;
}
/*---------------------------------------------------------------------------*/
static bool
fgbridge_command_reset(void)
{
  memset(fgbridge_cmd_buf, 0xCD, FGBRIDGE_SPI_FRAME_LENGTH);
  fgbridge_rcv_frame_t *frame;

  fgbridge_cmd_buf[0] = 0x55;
  fgbridge_cmd_buf[1] = 0xAA;
  fgbridge_cmd_buf[2] = fgbridge_dl_count++;
  fgbridge_cmd_buf[3] = 'R';
  fgbridge_cmd_buf[FGBRIDGE_SPI_FRAME_LENGTH - 1] = 0xFF;

  /*fgbridge_rcv_buf[3] = 0; / * ?? * / */
  /*PRINTF("SPI reset started:"); */
  if(fgbridge_transfer(fgbridge_cmd_buf, fgbridge_rcv_buf, FGBRIDGE_SPI_FRAME_LENGTH) == false) {
    return false;
    PRINTF("SPI reset transfer failed\n");
  }

  for(size_t i = 0; i < FGBRIDGE_SPI_FRAME_LENGTH; i++) {
    PRINTF("%02x", (uint8_t)fgbridge_rcv_buf[i]);
  }

  /*
     if(fgbridge_transfer(fgbridge_cmd_buf, fgbridge_rcv_buf, FGBRIDGE_SPI_FRAME_LENGTH) == false) {
     return false;
     }
   */
  frame = (fgbridge_rcv_frame_t *)fgbridge_rcv_buf;
  if((frame->byte_AA != 0xAA) && (frame->command != 'C')) {
    /*if(fgbridge_rcv_buf[1] != 0xAA || fgbridge_rcv_buf[3] != 'C') { */
    PRINTF("\n");
    return false;
  }
  PRINTF("Success\n");

  fgbridge_ble_addr[0] = fgbridge_rcv_buf[9];
  fgbridge_ble_addr[1] = fgbridge_rcv_buf[8];
  fgbridge_ble_addr[2] = fgbridge_rcv_buf[7];
  fgbridge_ble_addr[3] = fgbridge_rcv_buf[6];
  fgbridge_ble_addr[4] = fgbridge_rcv_buf[5];
  fgbridge_ble_addr[5] = fgbridge_rcv_buf[4];

  return true;
}
/*---------------------------------------------------------------------------*/
bool
fgbridge_init(fgbridge_callback_function *cb, void *arg)
{

  callback = cb;
  callback_argument = arg;

  fgbridge_spi_open();

  /* XXX: this sleep code is broken in Contiki-NG.
   * Add this line to project-conf.h instead:
   * #define LPM_MODE_CONF_MAX_SUPPORTED LPM_MODE_AWAKE
   */
  lpm_register_module(&fgbridge_spi_module);
  ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_SSI1);
  ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_SSI1);

  process_start(&fgbridge_process, NULL);

  fgbridge_init_irq();

  if(!fgbridge_monitor_g()) {
    /* g side not ready */
    PRINTF("FGBRIDGE: not ready yet\n");
    return false;
  }

  PRINTF("FGBRIDGE: init OK\n");

  is_operational = true;
  /*is_operational = false; */
  return true;
}
/*---------------------------------------------------------------------------*/

static bool
reinitialize(void)
{
  rtimer_clock_t end_time;

  PRINTF("FGBRIDGE: reinitialize, num adv=%lu accepted=%lu\n",
         num_adv, num_accepted_adv);
  num_adv = 0;
  num_accepted_adv = 0;

  end_time = RTIMER_NOW() + (RTIMER_SECOND / 2);

  while(fgbridge_command_reset() == false) {
    if(!RTIMER_CLOCK_LT(RTIMER_NOW(), end_time)) {
      PRINTF("FGBRIDGE: reinitialization failed\n");
      is_operational = false;
      /* force reset of the system */
      hw_watchdog_reboot();
      return false;
    }
  }

  if(!is_fgbridge_wearable_adv_present) {
    /*
     * Generate a heartbeat for the gateway script.
     * A heartbeat is just ADV messages with the node ID set to 0,
     * a meesage to sent but be ignored by the GW script except
     * resetting the keep-alive timer.
     * Note that a heartbeat is not generated if there's already
     * a real message in the buffer.
     */
    fgbridge_wearable_adv.wid = 0;
    is_fgbridge_wearable_adv_present = true;

    /* Notify the CoAP engine about new data */
    if(callback) {
      callback(callback_argument);
    }
  }

  return true;
}
/*---------------------------------------------------------------------------*/
void *
fgbridge_adv_read(uint16_t *length /* out */)
{
  if(is_fgbridge_wearable_adv_present) {
    *length = sizeof(fgbridge_wearable_adv);
    /* Assume that once it's read, the buffer can be reused.
     * This is a fair assumption as processes in Contiki are not preemptive. */
    is_fgbridge_wearable_adv_present = false;
  } else {
    /* return zero length */
    *length = 0;
  }
  return &fgbridge_wearable_adv;
}
/*---------------------------------------------------------------------------*/
static bool
is_new_advertisment_acceptable(fgbridge_rcv_frame_t *frame)
{
  if(is_fgbridge_wearable_adv_present) {
    PRINTF("FGBRIDGE: buffer full\n");
    return false;
  }

#if SPHERE
  uint8_t wearable_project_id = frame->W_ble_addr[2];
  uint8_t wearable_network_id = frame->W_ble_addr[1];

  if(SPHERE_GET_PROJECT_ID() != wearable_project_id) {
    PRINTF("FGBRIDGE: wrong project 0x%x (0x%x)\n",
           wearable_project_id, SPHERE_GET_PROJECT_ID());
    return false;
  }

  if(SPHERE_GET_NETWORK_ID() != wearable_network_id) {
    PRINTF("FGBRIDGE: wrong network 0x%x (0x%x)\n",
           wearable_network_id, SPHERE_GET_NETWORK_ID());
    return false;
  }
#endif /* SPHERE */

  return true;
}
/*---------------------------------------------------------------------------*/
static inline uint32_t
get_timestamp(void)
{
#if SPHERE
  return sphere_timestamp_get();
#else
  return clock_time();
#endif
}
bool
fgbridge_stats_get(uint8_t *buf, uint8_t len)
{
  bool ret;
  ret = fgbridge_monitor_g();
  if(ret) {
    memcpy(buf, scanner_output, FGBRIDGE_MON_FRAME_LENGTH);
  } else {
    rfc_bleScannerOutput_t *o = (rfc_bleScannerOutput_t *)scanner_output;
    if(o->nRxAdvIgnored == 0) {
      /*no advertisments received */
      PRINTF("no advertisments received\n");
    }
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(fgbridge_process, ev, data)
{
  static struct etimer et, spi_init;
  fgbridge_rcv_frame_t *frame;
  static uint16_t num_failed_transfers;

  PROCESS_BEGIN();

  etimer_set(&et, REINITIALIZATION_TIME);
  etimer_set(&spi_init, SPI_BRIDGE_TIMER);

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL
                        || (ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) || (ev == PROCESS_EVENT_TIMER && etimer_expired(&spi_init)));

    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
      etimer_reset(&et);
      etimer_restart(&spi_init);
      PRINTF("Sphere Timer");
      /*Check G side if reporting BLE data */
      if(!is_operational) {
        /*send not operational to coap */
        reinitialize();
        continue;
      }
      if(fgbridge_monitor_g()) {
        /*fill in an empty packet and send it to coap */
        continue;
      } else {
        /* Reinitialize the system, mostly to see that the G side is still alive */
        /*reset g side maybe */
        /* force reset of the system */
        hw_watchdog_reboot();
      }
    }
    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&spi_init)) {
      etimer_reset(&spi_init);
      PRINTF("Bridge Timer");
      if(fgbridge_monitor_g()) {
        is_operational = true;

        /*fill in an empty packet and send it to coap */
        continue;
      }
    }
    memset(fgbridge_rcv_buf, 0x00, FGBRIDGE_SPI_FRAME_LENGTH);

    fgbridge_cmd_buf[0] = 0x55;
    fgbridge_cmd_buf[1] = 0xAA;
    fgbridge_cmd_buf[2] = fgbridge_dl_count++;
    fgbridge_cmd_buf[3] = 0; /* set this to 'C' if the DL actually has a command for G2 to capture */

    fgbridge_transfer(fgbridge_cmd_buf, fgbridge_rcv_buf, FGBRIDGE_SPI_FRAME_LENGTH);
    etimer_restart(&spi_init);
#if 0
      for(size_t i = 0; i < FGBRIDGE_SPI_FRAME_LENGTH; ++i) {
        PRINTF("%.2x", fgbridge_rcv_buf[i]);
      }
      PRINTF("\n");
#endif
      num_failed_transfers = 0;

      frame = (fgbridge_rcv_frame_t *)fgbridge_rcv_buf;
      if(frame->type == FGBRIDGE_UL_ADV_MESSAGE) {
        ble_frame_adv_t *adv;
        uint32_t timestamp;

        adv = (ble_frame_adv_t *)frame->payload;
        timestamp = get_timestamp();
#if 0

        PRINTF("%.2x:%.2x:%.2x", frame->W_ble_addr[2], frame->W_ble_addr[1], frame->W_ble_addr[0]);
        PRINTF("[#%d] RSSI: %d", adv->mc_msb * 256 + adv->mc, frame->rssi);

        PRINTF(" TS: %u", (unsigned int)timestamp);
        PRINTF("MONITOR: ");
        for(size_t i = 0; i < 4; ++i) {
          PRINTF("%.2x", adv->monitor[i]);
        }
        PRINTF(" DATA: ");
        for(size_t i = 0; i < FGBRIDGE_SPI_ADV_DATA_LEN; ++i) {
          PRINTF("%.2x", adv->data[i]);
        }
        PRINTF("\n");

#endif
        num_adv++;
        is_operational = true;
        if(is_new_advertisment_acceptable(frame)) {
          num_accepted_adv++;
          is_fgbridge_wearable_adv_present = true;

          leds_toggle(LEDS_GREEN);

          /* use only the last byte of the MAC */
          fgbridge_wearable_adv.wid = frame->W_ble_addr[0];
          fgbridge_wearable_adv.mc[0] = adv->mc;
          fgbridge_wearable_adv.mc[1] = adv->mc_msb;
          fgbridge_wearable_adv.rssi = frame->rssi;
          memcpy(fgbridge_wearable_adv.data, adv->data, FGBRIDGE_SPI_ADV_DATA_LEN);
          memcpy(fgbridge_wearable_adv.timestamp, &timestamp, sizeof(timestamp));
          memcpy(fgbridge_wearable_adv.monitor, adv->monitor, sizeof(adv->monitor));
        }

        /* call the callback to notify about new data */
        if(is_fgbridge_wearable_adv_present) {
          callback(callback_argument);
        }
      } else if(frame->type == FGBRIDGE_UL_NOTIFICATION_MESSAGE) {
        rfc_bleScannerOutput_t *o = (rfc_bleScannerOutput_t *)frame->payload;
        if(o->nRxAdvIgnored == 0) {
          /*no advertisments received */
          PRINTF("no advertisments received\n");
          continue;
        }
        PRINTF("nRxAdvIgnored:%d\n", o->nRxAdvIgnored);
        memcpy(scanner_output, frame->payload, FGBRIDGE_MON_FRAME_LENGTH);
      } else {
        PRINTF("FGBRIDGE: transfer failed\n");
        num_failed_transfers++;
        if(num_failed_transfers >= 5) {
          num_failed_transfers = 0;
          /* at least try to do something */
          /*reinitialize(); */
        }
      }

  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*****************************************************************************************************

   Copyright (C) - All Rights Reserved

   SPHERE (an EPSRC IRC), 2013-2018
   University of Bristol
   University of Reading
   University of Southampton

   Filename: gslave.c
   Description: SPI Bridge from BLE to 802.15.4
   Primary Contributor(s): Xenofon (Fontas) Fafoutis (xenofon.fafoutis@bristol.ac.uk)
                         Atis Elsts (atis.elsts@bristol.ac.uk)

 *******************************************************************************************************/

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "ti-lib.h"
#include "gslave.h"
#include "board.h"
#include "lpm.h"
#include "dev/gpio-hal.h"
#include "dev/leds.h"
#include "sys/int-master.h"
#include "dev/spi.h"

#include "fgbridge.h"
#include "ble-adv-def.h"

#include "net/netstack.h"
#include "dev/ble-hal.h"

#include "rf_ble_cmd.h"

#include "sys/mutex.h"

#if SPHERE
#include "sphere.h"
#include "sphere-timestamps.h"
#endif /* SPHERE */

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "cc26xx-aes.h"

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
#include "sys/log.h"
#define LOG_MODULE "G-SLAVE"
/*#define LOG_LEVEL LOG_LEVEL_INFO */
#ifdef LOG_CONF_LEVEL_MAC
#define LOG_LEVEL LOG_CONF_LEVEL_MAC
#else
#define LOG_LEVEL LOG_LEVEL_DBG
#endif
/*---------------------------------------------------------------------------*/
PROCESS(gslave_process, "gslave process");
/*---------------------------------------------------------------------------*/
#define GSLAVE_FRAME_LENGTH FGBRIDGE_SPI_FRAME_LENGTH + 2
/*---------------------------------------------------------------------------*/
/*SPI in and out buffers to send to SPI transfer function. */
uint8_t SPI_out_buffer[GSLAVE_FRAME_LENGTH] = { 0 };
uint8_t SPI_in_buffer[GSLAVE_FRAME_LENGTH] = { 0 };
/*---------------------------------------------------------------------------*/
static const spi_device_t gslave_spi_configuration = {
  .spi_controller = SPI_CONTROLLER_SPI1,/* ID of SPI controller to use */
  .pin_spi_sck = FGBRIDGE_IOID_SPI_CLK,/* SPI SCK pin */
  .pin_spi_miso = FGBRIDGE_IOID_SPI_MISO,/* SPI MISO  pin */
  .pin_spi_mosi = FGBRIDGE_IOID_SPI_MOSI,/* SPI MOSI pin */
  .pin_spi_cs = FGBRIDGE_IOID_SPI_CS,/* SPI Chip Select pin */
  .spi_bit_rate = FGBRIDGE_SPI_RATE,
  .spi_pha = 1,/* SPI mode phase */
  .spi_pol = 1,/* SPI mode polarity */
  .is_spi_slave = 1             /* Zero for master, nonzero for slave */
};
static const spi_device_t *gslave_spi_conf;
/*---------------------------------------------------------------------------*/
/* Arch-specific properties of each SPI controller */
typedef struct board_spi_controller_s {
  uint32_t ssi_base;
  uint32_t power_domain;
  uint32_t prcm_periph;
  uint32_t ssi_clkgr_clk_en;
} board_spi_controller_t;
static const board_spi_controller_t spi_controller_config = {
  .ssi_base = SSI1_BASE,
  .power_domain = PRCM_DOMAIN_PERIPH,
  .prcm_periph = PRCM_PERIPH_SSI1,
  .ssi_clkgr_clk_en = PRCM_SSICLKGR_CLK_EN_SSI1
};
/*---------------------------------------------------------------------------*/
typedef struct {
  uint8_t *pNextEntry;                 /*!<        Pointer to next entry in the queue, NULL if this is the last entry */
  uint8_t status;
  uint8_t W_ble_addr[FGBRIDGE_BLE_ADDR_LENGTH];     /* BLE Address of Wearable (little-endian) */
  uint8_t length;                                   /* Frame Length */
  FGBRIDGE_UL_FRAME_TYPE type;                        /* Frame Type */
  uint8_t payload[FGBRIDGE_BLE_MAX_LENGTH];         /* Frame Payload */
  int8_t rssi;                                      /* RSSI */
} rcv_frame_t;
/*---------------------------------------------------------------------------*/
#define FREE_ENTRY 0
#define FULL_ENTRY 1
#define FRAME_PAYLOAD FGBRIDGE_BLE_ADDR_LENGTH + FGBRIDGE_BLE_MAX_LENGTH + 3
#define CIRCULAR_BUFFERS_NUM 6
#define CIRCULAR_BUFFERS_LEN 5 + FRAME_PAYLOAD
uint8_t circular_buf[CIRCULAR_BUFFERS_NUM][CIRCULAR_BUFFERS_LEN];
uint8_t *current_buf;
/*---------------------------------------------------------------------------*/
static void
init_circular_buffer(void)
{
  rcv_frame_t *frame;
  fgbridge_rcv_frame_t *rcv_frame = (fgbridge_rcv_frame_t *)(SPI_out_buffer + 1);
  memset(SPI_out_buffer, 0x00, GSLAVE_FRAME_LENGTH);
  SPI_out_buffer[0] = 0xAA;
  rcv_frame->byte_55 = 0x55;
  rcv_frame->byte_AA = 0xAA;
  rcv_frame->command = 'C';
  memcpy(rcv_frame->G_ble_addr, ble_address, FGBRIDGE_BLE_ADDR_LENGTH);

  for(size_t i = 0; i < CIRCULAR_BUFFERS_NUM; i++) {
    memset(&circular_buf[i], 0x00, CIRCULAR_BUFFERS_LEN);
    frame = (rcv_frame_t *)circular_buf[i];
    frame->pNextEntry = circular_buf[(i + 1) % CIRCULAR_BUFFERS_NUM];
    frame->status = FREE_ENTRY;
    frame->type = FGBRIDGE_UL_ADV_MESSAGE;
    current_buf = frame->pNextEntry;
  }
}
/*---------------------------------------------------------------------------*/
#if SPHERE
#else
struct sphere_key_addr_info_s {
  uint8_t security_key[16];
  uint8_t ieee_address[8];
  uint8_t ble_address[6];
} __attribute__((packed));
/*---------------------------------------------------------------------------*/
extern struct sphere_key_addr_info_s sphere_key_addr_info;
#endif
/*---------------------------------------------------------------------------*/
/* By default, load the key of HID 1234 (Playground NUC) */
#define DEFAULT_BLE_ENCRYPTION_KEY { 0x0c, 0xd5, 0xe0, 0xc8, 0x93, 0x20, 0xd2, 0x83, 0xd1, 0xa1, 0xb4, 0xfb, 0x16, 0xa3, 0xf0, 0xb3 }
static uint8_t ble_encryption_key[16] = DEFAULT_BLE_ENCRYPTION_KEY;
static void
gslave_initialize_address_and_key(void)
{
  int i;
  bool use_sphere_key_addr_info = false;
  /* Look for a byte != 0xFF in the supposed address */
  for(i = 0; i < 6; i++) {
    if(sphere_key_addr_info.ble_address[i] != 0xFF) {
      /* A byte in the address location is not 0xFF. Use this address */
      use_sphere_key_addr_info = true;
      break;
    }
  }
  if(use_sphere_key_addr_info) {
    /* Copy from the flash */
    memcpy(ble_encryption_key, sphere_key_addr_info.security_key, 16);
  }
}
#if BLE_CONF_WITH_SCANNER_CALLBACK_G_SLAVE
/*---------------------------------------------------------------------------*/
static g_slave_result_t
check_adv_data(unsigned short data_len, const uint8_t *data, int rssi)
{
  g_slave_result_t err_code;
  ble_frame_adv_t *adv;
  /* At the start rxdata is 00 24 for advertisements */
  if((data[0] != 0x00) && (data[0] != 0x02)) {
    err_code = G_SLAVE_ERROR;
  }

/*data += FGBRIDGE_BLE_ADDR_LENGTH; */
  adv = (ble_frame_adv_t *)(data + 2 + FGBRIDGE_BLE_ADDR_LENGTH);

  if(adv->uuid_lo != BLE_ADV_UUID_LO || adv->uuid_hi != BLE_ADV_UUID_HI) {
    err_code = G_SLAVE_ERROR;
  }
  if(data_len < sizeof(ble_frame_adv_t) + FGBRIDGE_BLE_ADDR_LENGTH) {
    err_code = G_SLAVE_ERROR;
  }
  err_code = G_SLAVE_RESULT_OK;
  /*remove error codes for this version */
  return err_code;
}
/*---------------------------------------------------------------------------*/
void
ble_scanner_callback(const uint8_t *rx_data, uint16_t length, int rssi,
                     uint8_t channel,uint32_t timestamp)
{
  if(check_adv_data(length, rx_data, rssi) != G_SLAVE_RESULT_OK) {
    PRINTF("Check returned error length:%d\n", length);
    return;
  }
#if 1
  PRINTF("length:%d rssi:%d channel:%d data:", length - 2, rssi, channel);
#endif
  add_to_circular_buffer(length, rx_data, rssi);
}
#endif
/*---------------------------------------------------------------------------*/

g_slave_result_t
add_to_circular_buffer(unsigned short data_len, const uint8_t *data, int rssi)
{
  rcv_frame_t *frame;
  uint8_t *entry;

  frame = (rcv_frame_t *)current_buf;
  entry = current_buf;
  do {
    if(frame->status == FREE_ENTRY) {
      frame->status = FULL_ENTRY;
      memcpy(frame->W_ble_addr, data + 2, FGBRIDGE_BLE_ADDR_LENGTH);
      ble_frame_adv_t *adv;
      adv = (ble_frame_adv_t *)(data + 2 + FGBRIDGE_BLE_ADDR_LENGTH);
      cc26xx_aes_decrypt(adv->data + 2);
      cc26xx_aes_decrypt(adv->data);
#if 0
      for(size_t i = 0; i < 18; i++) {
        PRINTF("%02x", adv->data[i]);
      }
#endif
      PRINTF(" mc_msb:%d mc:%d", adv->mc_msb, adv->mc);
      PRINTF("\n");
      frame->rssi = rssi;
      memcpy(frame->payload, adv, sizeof(ble_frame_adv_t));
      toggleg_slave_int();
      return G_SLAVE_RESULT_OK;
    }
    frame = (rcv_frame_t *)frame->pNextEntry;
  } while(frame->pNextEntry != entry);
  PRINTF("GSLAVE: Buffer full\n");
  /*try to empty the buffer */
  toggleg_slave_int();
  return G_SLAVE_ERROR;
}
g_slave_result_t
gslave_adv_add(unsigned short data_len, const uint8_t *data, int rssi)
{
  uint8_t header_length = 2 + FGBRIDGE_BLE_ADDR_LENGTH;
  fgbridge_rcv_frame_t *frame = (fgbridge_rcv_frame_t *)(SPI_out_buffer + 1);
  if(data_len - header_length > FGBRIDGE_BLE_ADDR_LENGTH) {
  }
  for(size_t i = 0; i < FGBRIDGE_BLE_ADDR_LENGTH; i++) {
    frame->W_ble_addr[i] = data[2 + i];
  }
  /*memcpy(frame->W_ble_addr, &data[2], FGBRIDGE_BLE_ADDR_LENGTH); */
  frame->length = data_len;
  frame->type = FGBRIDGE_UL_ADV_MESSAGE;

  uint8_t length = data_len;
  /* add decrypt algorithm */
  /*do_CRYPTO(pEvent->deviceInfo.pEvtData+14,pEvent->deviceInfo.pEvtData+14, adv_sequence_number, device_BLE_MAC_address); //do AES (decrypt) */
/*do_CRYPTO(pEvent->deviceInfo.pEvtData+12,pEvent->deviceInfo.pEvtData+12, adv_sequence_number, device_BLE_MAC_address); //do AES (decrypt) */

  for(size_t i = 0; i < FGBRIDGE_BLE_MAX_LENGTH; i++) {
    if(length > 0) {
      frame->payload[i] = data[i + header_length];
    } else {
      frame->payload[i] = 0x00;
    }
  }

  frame->rssi = rssi;

  toggleg_slave_int();
  return G_SLAVE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static bool enable_interrupt = true;
/*---------------------------------------------------------------------------*/
static gpio_hal_event_handler_t interrupt_handler_object;
unsigned int timeit;
uint8_t scanner_output[sizeof(rfc_bleScannerOutput_t)] =
{ 0 };

/*---------------------------------------------------------------------------*/
static void
gslave_irq_handler(gpio_hal_pin_mask_t mask)
{
  if(mask & FGBRIDGE_SPI_CS) {
    timeit = RTIMER_NOW();
    /*if(enable_interrupt) { */
    process_poll(&gslave_process);
    /*} */
  }
}
/*---------------------------------------------------------------------------*/
#define cc26xx_spi_isr SSI1IntHandler
/*---------------------------------------------------------------------------*/
void
cc26xx_spi_isr(void)
{

  uint32_t intStatus;
  intStatus = ti_lib_ssi_int_status(SSI1_BASE, true);
  uint32_t c;
  while(ti_lib_ssi_data_get_non_blocking(SSI1_BASE, &c));
  ti_lib_ssi_data_put_non_blocking(SSI1_BASE, c);
  ti_lib_ssi_int_clear(SSI1_BASE, intStatus);
}
/*---------------------------------------------------------------------------*/
static void
gslave_init_irq(void)
{
  ti_lib_ioc_pin_type_gpio_input(FGBRIDGE_IOID_SPI_CS);
  ti_lib_ioc_int_disable(FGBRIDGE_IOID_SPI_CS);
  ti_lib_gpio_clear_event_dio(FGBRIDGE_IOID_SPI_CS);
  ti_lib_ioc_port_configure_set(FGBRIDGE_IOID_SPI_CS, IOC_PORT_GPIO,
                                (IOC_INT_ENABLE | IOC_INPUT_ENABLE | IOC_FALLING_EDGE));
  ti_lib_ioc_io_port_pull_set(FGBRIDGE_IOID_SPI_CS, IOC_IOPULL_UP);
  ti_lib_gpio_set_output_enable_dio(FGBRIDGE_IOID_SPI_CS, GPIO_OUTPUT_DISABLE);
  interrupt_handler_object.pin_mask = FGBRIDGE_SPI_CS;
  interrupt_handler_object.handler = gslave_irq_handler;
  gpio_hal_register_handler(&interrupt_handler_object);
  ti_lib_ioc_int_enable(FGBRIDGE_IOID_SPI_CS);
}
/*---------------------------------------------------------------------------*/
g_slave_result_t
spi_init(const spi_device_t *dev)
{

#if 1
  uint32_t c;
  static const board_spi_controller_t *spi_controller;

  spi_controller = &spi_controller_config;

  ti_lib_ioc_pin_type_gpio_input(dev->pin_spi_cs);
  ti_lib_ioc_io_port_pull_set(dev->pin_spi_cs, IOC_IOPULL_UP);
  /* First, make sure the SERIAL PD is on */
  ti_lib_prcm_power_domain_on(spi_controller->power_domain);
  while((ti_lib_prcm_power_domain_status(spi_controller->power_domain)
         != PRCM_DOMAIN_POWER_ON));

  /* Enable clock in active mode */
  ti_lib_prcm_peripheral_run_enable(spi_controller->prcm_periph);
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());
  /* SPI slave mode */
  ti_lib_ssi_int_disable(spi_controller->ssi_base, SSI_RXOR | SSI_RXFF | SSI_RXTO | SSI_TXFF);
  ti_lib_ssi_int_clear(spi_controller->ssi_base, SSI_RXOR | SSI_RXTO);
  ti_lib_ioc_pin_type_ssi_slave(spi_controller->ssi_base,
                                    dev->pin_spi_mosi,
                                    dev->pin_spi_miso,
                                    IOID_UNUSED,
                                    dev->pin_spi_sck);

  ti_lib_ssi_config_set_exp_clk(spi_controller->ssi_base,
                                    ti_lib_sys_ctrl_clock_get(),
                                    SSI_FRF_MOTO_MODE_3, SSI_MODE_SLAVE,
                                    dev->spi_bit_rate, 8);
  ti_lib_ssi_int_register(spi_controller->ssi_base, cc26xx_spi_isr);

  ti_lib_ssi_enable(spi_controller->ssi_base);
  /* Get rid of residual data from SSI port */
  while(ti_lib_ssi_data_get_non_blocking(spi_controller->ssi_base, &c));
  /* Enable calling of the right SSI ISR handler */
  /*ti_lib_int_enable(INT_SSI1_COMB); */
#endif

  return G_SLAVE_RESULT_OK;
}
g_slave_result_t
gslave_init(void)
{
  /* This uses the default SPI configuration*/
  /* Set the chip select pin */
  init_circular_buffer();
  gslave_initialize_address_and_key();
  /* Initialisation of Crypto driver */
  cc26xx_aes_128_driver.set_key(ble_encryption_key);
  process_start(&gslave_process, NULL);

  ti_lib_ioc_pin_type_gpio_output(FGBRIDGE_IOID_G2_INT);
  /*Initiate the flag to IRQ on F side to inactive (high) */
  ti_lib_gpio_set_dio(FGBRIDGE_IOID_G2_INT);
  /* Keep the PERIPH domain always on */

  gslave_spi_conf = &gslave_spi_configuration;
  /*spi_init(gslave_spi_conf); */
  g_slave_result_t ret = spi_init(gslave_spi_conf);
  gslave_init_irq();
  if(ret == G_SLAVE_RESULT_OK) {
    return G_SLAVE_RESULT_OK;
  }

  LOG_DBG("SLAVE: gslave_init error\n");
  return G_SLAVE_ERROR;
  /*Set periodic clock to read fifo */
}
/*---------------------------------------------------------------------------*/
void
toggleg_slave_int(void)
{
  /*pulse G2's IRQ (neg edge, so make low) for 10us to inform G1 that data is ready to be read on SPI */

  enable_interrupt = false;

  spi_init(gslave_spi_conf);
  gslave_init_irq();
  ti_lib_gpio_clear_dio(FGBRIDGE_IOID_G2_INT);
  ti_lib_gpio_clear_dio(FGBRIDGE_IOID_G2_INT);
  ti_lib_gpio_set_dio(FGBRIDGE_IOID_G2_INT);

  /*process_poll(&gslave_process); */
}
/*---------------------------------------------------------------------------*/
static bool
gslave_spi_transfer(const uint8_t *buf, uint8_t *outbuf, size_t len)
{
  size_t out_len = len;

  unsigned int end;
  end = RTIMER_NOW() + 210;

  while(((len > 2) || (out_len > 0)) && (RTIMER_CLOCK_LT(RTIMER_NOW(), end))) {
    if(out_len > 0) {
      if(ti_lib_ssi_data_put_non_blocking(SSI1_BASE, *buf)) {
        out_len--;
        buf++;
      }
    }
    if(len > 0) {
      uint32_t ul;
      if(ti_lib_ssi_data_get_non_blocking(SSI1_BASE, &ul)) {

        *outbuf = (uint8_t)ul;
        outbuf++;
        len--;
      }
    }
  }

  if(!RTIMER_CLOCK_LT(RTIMER_NOW(), end)) {
    return false;
  }
  return true;
}
/*---------------------------------------------------------------------------*/
g_slave_result_t
gslave_scanner_output_update(const uint8_t *data)
{
  radio_result_t res;
  res = NETSTACK_RADIO.get_object(RADIO_PARAM_BLE_SCAN_OUTPUT_STATUS,
                                  (void *)data,
                                  sizeof(rfc_bleScannerOutput_t));
  if(res != RADIO_RESULT_OK) {
    return G_SLAVE_ERROR;
  }
  const volatile rfc_bleScannerOutput_t *o =
    (rfc_bleScannerOutput_t *)data;
  if(o->nRxAdvIgnored == 0 && o->nRxAdvOk == 0) {
    /*PRINTF("nRxAdvOk%d nRxAdvIgnored%d \n", o->nRxAdvOk, o->nRxAdvIgnored); */
    return G_SLAVE_ERROR;
  }
  return G_SLAVE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(gslave_process, ev, data)
{

  PROCESS_BEGIN();

  fgbridge_rcv_frame_t *tx_frame = (fgbridge_rcv_frame_t *)(SPI_out_buffer + 1);
  bool f_request;
  rcv_frame_t *frame;

  PRINTF("Process start\n");
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    if(enable_interrupt) {
      enable_interrupt = false;
      f_request = true;
    }

    frame = (rcv_frame_t *)current_buf;
    if(frame->status == FULL_ENTRY) {
      tx_frame->type = FGBRIDGE_UL_ADV_MESSAGE;
      memcpy(tx_frame->W_ble_addr, frame->W_ble_addr,
             FRAME_PAYLOAD);
      f_request = false;
      /*record pointer to the last transmitted full entry for retransmission */
    } else if(gslave_scanner_output_update(scanner_output)
              == G_SLAVE_RESULT_OK) {
      f_request = true;
      tx_frame->length = sizeof(rfc_bleScannerOutput_t) - 1;
      memcpy(tx_frame->payload, scanner_output,
             tx_frame->length);
      tx_frame->command = 'U';
      tx_frame->type = FGBRIDGE_UL_NOTIFICATION_MESSAGE;
    } else {
      /*PRINTF("Scanner not operating\n"); */
      tx_frame->length = 0;
      memcpy(tx_frame->W_ble_addr, 0x00, FRAME_PAYLOAD);
    }

    tx_frame->ul_count++;
    bool ret = gslave_spi_transfer(SPI_out_buffer, SPI_in_buffer, GSLAVE_FRAME_LENGTH);
    PRINTF(".");
#if 0
    for(size_t i = 0; i < GSLAVE_FRAME_LENGTH; ++i) {
      PRINTF("%.2x", SPI_in_buffer[i]);
    }
    PRINTF("\n");
#endif

    if(!ret) {
      enable_interrupt = true;
      /*interrupt updates fail for some reason. */
      PRINTF("transfer failed elapsed:%0ld : \n", RTIMER_NOW() - timeit);
      toggleg_slave_int();
      continue;
    }
    if(f_request) {
      spi_init(gslave_spi_conf);
      gslave_init_irq();
    } else {
      frame = (rcv_frame_t *)current_buf;
      frame->status = FREE_ENTRY;
      current_buf = frame->pNextEntry;
      uint8_t *entry = (uint8_t *)frame->pNextEntry;
      uint8_t num_entries = 0;
      do {
        if(frame->status == FULL_ENTRY) {
          num_entries++;
        }
        frame = (rcv_frame_t *)frame->pNextEntry;
      } while(entry != frame->pNextEntry);

      if(num_entries) {
        PRINTF("More Entries in buffer\n");
        /*More enrtries in buffer */
        toggleg_slave_int();
      }
    }
    enable_interrupt = true;
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

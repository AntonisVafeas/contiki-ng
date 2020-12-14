/*****************************************************************************************************

 Copyright (C) - All Rights Reserved

 SPHERE (an EPSRC IRC), 2013-2018
 University of Bristol
 University of Reading
 University of Southampton

 Filename: fgbridge.c
 Description: Header file for fgbridge.h
 Primary Contributor(s): Xenofon (Fontas) Fafoutis (xenofon.fafoutis@bristol.ac.uk)

*******************************************************************************************************/

#ifndef FGBRIDGE_H_
#define FGBRIDGE_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define FGBRIDGE_IOID_SPI_MISO      IOID_9
#define FGBRIDGE_IOID_SPI_MOSI      IOID_8
#define FGBRIDGE_IOID_SPI_CLK       IOID_10
#define FGBRIDGE_IOID_G2_INT        IOID_12
#define FGBRIDGE_IOID_SPI_CS        IOID_11

#define FGBRIDGE_SPI_CS             (1 << FGBRIDGE_IOID_SPI_CS)
#define FGBRIDGE_G2_INT             (1 << FGBRIDGE_IOID_G2_INT)

#define FGBRIDGE_SPI_FRF            SSI_FRF_MOTO_MODE_3 // polarity 1, phase 1
#define FGBRIDGE_SPI_RATE           4000000

#define FGBRIDGE_CS_DELAY          55

#define FGBRIDGE_SPI_HEADER_LENGTH  10
#define FGBRIDGE_SPI_PDU_LENGTH     40
#define FGBRIDGE_SPI_FRAME_LENGTH   (FGBRIDGE_SPI_HEADER_LENGTH + FGBRIDGE_SPI_PDU_LENGTH)
#define FGBRIDGE_SPI_ADV_DATA_LEN   18

#define FGBRIDGE_BLE_MAX_LENGTH     31
#define FGBRIDGE_BLE_ADDR_LENGTH    6
#define FGBRIDGE_MON_FRAME_LENGTH    16

typedef enum
{
  FGBRIDGE_UL_ADV_MESSAGE = 1,
  FGBRIDGE_UL_SCAN_RESPONSE_MESSAGE,
  FGBRIDGE_UL_CONNECTION_REQUEST_MESSAGE,
  FGBRIDGE_UL_CONNECTION_SUCCESSFUL_MESSAGE,
  FGBRIDGE_UL_CONNECTION_ATTEMPT_FAILED_MESSAGE,
  FGBRIDGE_UL_NOTIFICATION_MESSAGE,
  FGBRIDGE_UL_TAG_DISCONNECTED_MESSAGE,
  FGBRIDGE_UL_UNABLE_TO_DO_WRITE_MESSAGE,
  FGBRIDGE_UL_GIVEN_TAG_NOT_CONNECTED_MESSAGE,
  FGBRIDGE_UL_SUCCESSFUL_WRITE_ACK_MESSAGE,
  FGBRIDGE_UL_UNSUCCESSFUL_WRITE_ACK_MESSAGE,
  FGBRIDGE_UL_RETURN_LIST_OF_CONNECTED_MACS_MESSAGE,
  FGBRIDGE_UL_RETURN_RSSI_OF_CONNECTED_TAG_MESSAGE,
  FGBRIDGE_UL_RSSI_OF_CONNECTED_TAG_STOPPED_MESSAGE,
  FGBRIDGE_UL_RSSI_UNABLE_TO_START_MESSAGE,
  FGBRIDGE_UL_FORCED_DISCONNECT_DUE_TO_TIMEOUT_MESSAGE
} FGBRIDGE_UL_FRAME_TYPE; /* SPI uplink commands (G2 to G1) */

typedef enum
{
  FGBRIDGE_DL_WRITE_TO_TAG_MESSAGE = 80,
  FGBRIDGE_DL_ENQUIRE_LIST_OF_CONNECTED_MACS_MESSAGE,
  FGBRIDGE_DL_START_RSSI_OF_CONNECTED_TAG_MESSAGE,
  FGBRIDGE_DL_STOP_RSSI_OF_CONNECTED_TAG_MESSAGE,
  FGBRIDGE_DL_CONNECTION_AUTHORISED_MESSAGE,
  FGBRIDGE_DL_ENABLE_DATA_CCCD_MESSAGE,
  FGBRIDGE_DL_DISSABLE_DATA_CCCD_MESSAGE,
  FGBRIDGE_DL_ENABLE_BATTERY_CCCD_MESSAGE,
  FGBRIDGE_DL_DISSABLE_BATTERY_CCCD_MESSAGE
} FGBRIDGE_DL_FRAME_TYPE; /* SPI downlink commands (G1 to G2) */


typedef struct {
  uint8_t byte_55;                                    // Should always be 0x55
  uint8_t byte_AA;                                    // Should always be 0xAA
  uint8_t ul_count;                                   // Frame counter
  uint8_t command;                                    // Command
  uint8_t G_ble_addr[FGBRIDGE_BLE_ADDR_LENGTH];       // BLE Address of G (little-endian)
  uint8_t W_ble_addr[FGBRIDGE_BLE_ADDR_LENGTH];       // BLE Address of Wearable (little-endian)
  uint8_t length;                                     // Frame Length
  FGBRIDGE_UL_FRAME_TYPE type;                        // Frame Type
  uint8_t payload[FGBRIDGE_BLE_MAX_LENGTH];           // Frame Payload
  int8_t rssi;                                        // RSSI
  // add a pointer to the fifo entry that has been transmitted for retransmissions
  //uint8_t *entry;
} fgbridge_rcv_frame_t;

typedef struct {
  uint16_t nTxScanReq;                 //!<        Number of transmitted SCAN_REQ packets
  uint16_t nBackedOffScanReq;          //!<        Number of SCAN_REQ packets not sent due to backoff procedure
  uint16_t nRxAdvOk;                   //!<        Number of ADV*_IND packets received with CRC OK and not ignored
  uint16_t nRxAdvIgnored;              //!<        Number of ADV*_IND packets received with CRC OK, but ignored
  uint16_t nRxAdvNok;                  //!<        Number of ADV*_IND packets received with CRC error
  uint16_t nRxScanRspOk;               //!<        Number of SCAN_RSP packets received with CRC OK and not ignored
  uint16_t nRxScanRspIgnored;          //!<        Number of SCAN_RSP packets received with CRC OK, but ignored
  uint16_t nRxScanRspNok;              //!<        Number of SCAN_RSP packets received with CRC error
} fgbridge_mon_t;
typedef struct {
  uint8_t wid;                                        /* Wearable ID (the last byte of BLE addr) */
  uint8_t mc[2];                                      /* ADV Counter (LSB) */
  int8_t rssi;                                        /* RSSI */
  uint8_t data[FGBRIDGE_SPI_ADV_DATA_LEN];            /* Acceleration Data */
  uint8_t timestamp[4];                               /* Absolute Sequence Number of TSCH */
  uint8_t monitor[4];                                 /* Monitoring Data */
} wearable_adv_t;

/*---------------------------------------------------------------------------*/

typedef void fgbridge_callback_function(void *resource);

bool fgbridge_init(fgbridge_callback_function *, void *);

void *fgbridge_adv_read(uint16_t *length /* out */);

bool fgbridge_stats_get(uint8_t *buf, uint8_t len);
/*---------------------------------------------------------------------------*/
#endif

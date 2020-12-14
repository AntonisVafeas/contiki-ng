/*****************************************************************************************************

   Copyright (C) - All Rights Reserved

   SPHERE (an EPSRC IRC), 2013-2018
   University of Bristol
   University of Reading
   University of Southampton

   Filename: fgbridge.c
   Description: SPI Bridge from BLE to 802.15.4 definitions
   Primary Contributor(s): Xenofon (Fontas) Fafoutis (xenofon.fafoutis@bristol.ac.uk)
   Atis Elsts (atis.elsts@bristol.ac.uk)
   Antonis Vafeas (antonis.vafeas@bristol.ac.uk)
 *******************************************************************************************************/
#ifndef BLE_ADV_DEF_H_
#define BLE_ADV_DEF_H_

#include <stdint.h>

#define ADV_DATA_LEN   18
/* For SPHERE use 0xFFF0 */
#define BLE_ADV_UUID_HI             0xFF
#define BLE_ADV_UUID_LO             0xF0
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

/*add scan response definitions */

#endif /* BLE_ADV_DEF_H_ */
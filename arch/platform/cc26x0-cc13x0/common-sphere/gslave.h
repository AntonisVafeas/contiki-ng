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

#ifndef GSLAVE_H_
#define GSLAVE_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>

/*---------------------------------------------------------------------------*/
/* Return values for functions of g_spi_slave implementations      */
typedef enum {
  G_SLAVE_RESULT_OK,
  G_SLAVE_NOT_SUPPORTED,
  G_SLAVE_INVALID_PARAM,
  G_SLAVE_ERROR
} g_slave_result_t;

/*---------------------------------------------------------------------------*/

/*initlialise spi and interrupt for slave operation */
g_slave_result_t gslave_init(void);

void *gslave_adv_read(uint16_t *length /* out */);
/*typedef void gslave_callback_function(void *resource); */

/**
 * Adds advertisment to SPI buffer
 *
 * \param data_len the length of the packet
 * \param data the data of the packet
 * \param data the rssi of the packet
 */
g_slave_result_t gslave_adv_add(unsigned short data_len, const uint8_t *data, int rssi);
/**
 * Adds advertisment to queue
 *
 * \param data_len the length of the packet
 * \param data the data of the packet
 * \param data the rssi of the packet
 */
g_slave_result_t add_to_circular_buffer(unsigned short data_len, const uint8_t *data, int rssi);
void toggleg_slave_int(void);

/*---------------------------------------------------------------------------*/
#endif

/*
 * Copyright (c) 2016, University of Bristol - http://www.bristol.ac.uk
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
 * \addtogroup sphere
 * @{
 *
 * \file
 * Driver for permanent statistics, stored in the flash
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "flash-stats.h"
#include "ext-flash.h"
#include "ti-lib.h"
#include "sys/int-master.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
#define REBOOT_STATS_OFFSET  0
/*---------------------------------------------------------------------------*/
typedef struct reboot_stats {
  /* overall number of reboots in node's lifetime */
  uint16_t num_reboots;
  /* true if the next expected reboot is deliberate */
  uint16_t next_reboot_deliberate;
  /* true if the last reboot was deliberate */
  uint16_t last_reboot_deliberate;
} reboot_stats_t;
/*---------------------------------------------------------------------------*/
static bool
read_reboot_stats(reboot_stats_t *stats)
{
  int rv;
  bool ok = false;
  int_master_status_t status;

#if CONTIKI_BOARD_SPG_2_CC2650
  /* XXX: disable this at the moment: the flash stats destabilize fgbrigde in the Contiki-NG */
  return false;

  if(is_g_side) {
    /* no flash on the G side */
    return false;
  }
#endif /* CONTIKI_BOARD_SPG_2_CC2650 */

  /* disable interrupts to avoid messing with fgbridge */
  status = int_master_read_and_disable();

  rv = ext_flash_open(NULL);

  if(rv) {
    rv = ext_flash_read(NULL, REBOOT_STATS_OFFSET, sizeof(*stats), (uint8_t *)stats);
    if(rv) {
      ok = true;
    } else {
      printf("Could not read flash\n");
    }
  } else {
    printf("Could not open flash\n");
  }

  ext_flash_close(NULL);

  /* restore interrupts */
  int_master_status_set(status);

  return ok;
}
/*---------------------------------------------------------------------------*/
static bool
write_reboot_stats(reboot_stats_t *stats)
{
  int rv;
  bool ok = false;
  reboot_stats_t copy;
  int_master_status_t status;

#if CONTIKI_BOARD_SPG_2_CC2650
  /* XXX: disable this at the moment: the flash stats destabilize fgbrigde in the Contiki-NG */
  return false;

  if(is_g_side) {
    /* no flash on the G side */
    return false;
  }
#endif

  /* disable interrupts to avoid messing with fgbridge */
  status = int_master_read_and_disable();

  rv = ext_flash_open(NULL);

  if(rv) {
    rv = ext_flash_erase(NULL, REBOOT_STATS_OFFSET, sizeof(*stats));
    if(rv) {
      rv = ext_flash_write(NULL, REBOOT_STATS_OFFSET, sizeof(*stats), (uint8_t *)stats);
      if(rv) {
        /* read the just-written value and check */
        rv = ext_flash_read(NULL, REBOOT_STATS_OFFSET, sizeof(copy), (uint8_t *)&copy);
        if(rv) {
          if(!memcmp(&copy, stats, sizeof(copy))) {
            ok = true;
          } else {
            printf("Flash data doesn't match with expected\n");
          }
        }
      } else {
        printf("Could not write flash\n");
      }
    } else {
      printf("Could not erase flash\n");
    }
  } else {
    printf("Could not open flash\n");
  }

  ext_flash_close(NULL);

  /* restore interrupts */
  int_master_status_set(status);

  return ok;
}
/*---------------------------------------------------------------------------*/
uint8_t
flash_stats_reboot_get(uint8_t *is_flash_ok, uint16_t *num_reboots, uint8_t *is_deliberate)
{
  uint8_t reason;
  reboot_stats_t stats;

  /* first, read the reason from TI lib */
  reason = ti_lib_sys_ctrl_reset_source_get();

  /* then read flash to determins the number reboots */
  if(read_reboot_stats(&stats)) {
    *is_flash_ok = 1;
    *num_reboots = stats.num_reboots;
    *is_deliberate = stats.last_reboot_deliberate;
  } else {
    *is_flash_ok = 0;
  }

  return reason;
}
/*---------------------------------------------------------------------------*/
bool
flash_stats_reboot_set(bool is_deliberate)
{
  reboot_stats_t stats;

  if(!read_reboot_stats(&stats)) {
    return false;
  }

  /* set it to "yes" or "no" */
  stats.next_reboot_deliberate = is_deliberate;

  return write_reboot_stats(&stats);
}
/*---------------------------------------------------------------------------*/
bool
flash_stats_reboot_init(void)
{
  reboot_stats_t stats;

  if(!read_reboot_stats(&stats)) {
    return false;
  }

  stats.last_reboot_deliberate = stats.next_reboot_deliberate;
  stats.next_reboot_deliberate = false;
  stats.num_reboots++;

  return write_reboot_stats(&stats);
}
/*---------------------------------------------------------------------------*/
/** @} */

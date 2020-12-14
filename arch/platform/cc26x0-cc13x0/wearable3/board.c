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
/** \addtogroup sphere
 * @{
 *
 * \file
 *  Board-initialisation for the SPHERE Wearable, SPW-2.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "ti-lib.h"
#include "lpm.h"
#include "prcm.h"
#include "hw_sysctl.h"
#include "board-peripherals.h"

#include <stdint.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
static void
wakeup_handler(void)
{
  /* Turn on the PERIPH PD */
  ti_lib_prcm_power_domain_on(PRCM_DOMAIN_PERIPH);
  while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH)
         != PRCM_DOMAIN_POWER_ON));
}
/*---------------------------------------------------------------------------*/
/*
 * Declare a data structure to register with LPM.
 * We don't care about what power mode we'll drop to, we don't care about
 * getting notified before deep sleep. All we need is to be notified when we
 * wake up so we can turn power domains back on
 */
LPM_MODULE(srf_module, NULL, NULL, wakeup_handler, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*/
void
board_init()
{
  uint8_t int_disabled = ti_lib_int_master_disable();

  /* Turn on relevant PDs */
  wakeup_handler();

  /* Enable GPIO peripheral */
  ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);

  /* Apply settings and wait for them to take effect */
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());
  /* Connected to buzzer gpio P-MOS active low */
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_BUZZER);
  ti_lib_gpio_set_dio(BOARD_IOID_BUZZER);
/* Connected to LP5907 active high powers v2.8 */
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_CHARGE_STATUS, IOC_IOPULL_UP);
  /* Connected to LP5907 active high powers v2.8 */
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_HR_POWER);
  ti_lib_gpio_clear_dio(BOARD_IOID_HR_POWER);
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_HR_TX);
  ti_lib_gpio_clear_dio(BOARD_IOID_HR_TX);
  /*UART Connected to CSD8750 with external pull up*/
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_UART_RX_USB);
  ti_lib_gpio_set_dio(BOARD_IOID_UART_RX_USB);
/*UART Connected to CSD8750 with external pull up*/
  ti_lib_ioc_pin_type_gpio_output(BOARD_UART_TX);
  ti_lib_gpio_set_dio(BOARD_UART_TX);

  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_OLED_POWER);
  ti_lib_gpio_clear_dio(BOARD_IOID_OLED_POWER);
  /* Make sure the external flash is in the lower power mode */
  ext_flash_init(NULL);
  mc3672_power_down();
  icm_power_down();
  tps62746_init();
  lpm_register_module(&srf_module);

  /* Re-enable interrupt if initially enabled. */
  if(!int_disabled) {
    ti_lib_int_master_enable();
  }
}
/*---------------------------------------------------------------------------*/
/** @} */

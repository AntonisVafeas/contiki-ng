/*
 * Copyright (c) 2016, University of Bristol - http://www.bris.ac.uk/
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
 * Driver for the SPHERE SPES-2 Hardware Watchdog
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "hw-watchdog.h"
#include "flash-stats.h"
#include "dev/gpio-hal.h"
#include "ti-lib.h"
/*---------------------------------------------------------------------------*/
#define WAKE_GPIO_CFG            (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO | \
                                 IOC_NO_IOPULL    | IOC_SLEW_DISABLE  | \
                                 IOC_HYST_DISABLE | IOC_RISING_EDGE   | \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL | \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)
/*---------------------------------------------------------------------------*/
static volatile bool is_disabled;
/*---------------------------------------------------------------------------*/
static gpio_hal_event_handler_t interrupt_handler_object;
/*---------------------------------------------------------------------------*/
static void
interrupt_handler(gpio_hal_pin_mask_t mask)
{
  if(mask & BOARD_HW_WATCHDOG_WAKE) {
    hw_watchdog_periodic();
  }
}
/*---------------------------------------------------------------------------*/
void
hw_watchdog_init(void)
{
  /* input pin */
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_HW_WATCHDOG_WAKE);
  ti_lib_ioc_int_disable(BOARD_IOID_HW_WATCHDOG_WAKE);
  ti_lib_gpio_clear_event_dio(BOARD_IOID_HW_WATCHDOG_WAKE);
  ti_lib_ioc_port_configure_set(BOARD_IOID_HW_WATCHDOG_WAKE, IOC_PORT_GPIO, WAKE_GPIO_CFG);
  ti_lib_gpio_set_output_enable_dio(BOARD_IOID_HW_WATCHDOG_WAKE, GPIO_OUTPUT_DISABLE);
  interrupt_handler_object.pin_mask = BOARD_HW_WATCHDOG_WAKE;
  interrupt_handler_object.handler = interrupt_handler;
  gpio_hal_register_handler(&interrupt_handler_object);

  ti_lib_ioc_int_enable(BOARD_IOID_HW_WATCHDOG_WAKE);

  /* output pin */
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_HW_WATCHDOG_DONE);
  ti_lib_gpio_write_dio(BOARD_IOID_HW_WATCHDOG_DONE, 0);
  ti_lib_cpu_delay(100);
  ti_lib_gpio_write_dio(BOARD_IOID_HW_WATCHDOG_DONE, 1);
  ti_lib_cpu_delay(100);
  ti_lib_gpio_write_dio(BOARD_IOID_HW_WATCHDOG_DONE, 0);
}
/*---------------------------------------------------------------------------*/
void
hw_watchdog_periodic(void)
{
  if(!is_disabled) {
    ti_lib_gpio_write_dio(BOARD_IOID_HW_WATCHDOG_DONE, 1);
    ti_lib_cpu_delay(100);
    ti_lib_gpio_write_dio(BOARD_IOID_HW_WATCHDOG_DONE, 0);
  }
}
/*---------------------------------------------------------------------------*/
void
hw_watchdog_reboot(void)
{
  /* mark this reboot as deliberate */
  flash_stats_reboot_set(true);

  is_disabled = true;
}
/*---------------------------------------------------------------------------*/
/** @} */

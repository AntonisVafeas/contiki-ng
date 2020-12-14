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
 *  Driver for the TPS62746 regulator for reading the battery voltage
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "tps62746.h"
#include "ti-lib.h"
#include "board.h"
#include "sys/clock.h"
#include "dev/adc-sensor.h"
#include "dev/aux-ctrl.h"
/*---------------------------------------------------------------------------*/
static aux_consumer_module_t aux = {
  .clocks = AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK
};
/*---------------------------------------------------------------------------*/
void
tps62746_init(void)
{
  /* Pin initialisation */

  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_TPS62746_CTRL);
  ti_lib_gpio_clear_dio(BOARD_IOID_TPS62746_CTRL);

  ti_lib_ioc_port_configure_set(BOARD_IOID_TPS62746_VINSW, IOC_PORT_GPIO, IOC_STD_OUTPUT);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_TPS62746_VINSW);
}
/*---------------------------------------------------------------------------*/
uint16_t
tps62746_value(void)
{
  uint32_t val;
  float vbat;

  /* Open the channel via the CTRL pin */
  ti_lib_gpio_set_dio(BOARD_IOID_TPS62746_CTRL);

  /* Configure ADC */
  aux_ctrl_register_consumer(&aux);
  ti_lib_aux_adc_select_input(BOARD_TPS62746_ADC_CHANNEL);

  /* Some delay, this can be implemented with a timer more efficiently */
  clock_delay_usec(3000);

  /* Configure ADC */
  ti_lib_aux_adc_enable_sync(AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_2P7_US,
                             AUXADC_TRIGGER_MANUAL);

  ti_lib_aux_adc_gen_manual_trigger();

  /* Dead value from ADC */
  val = ti_lib_aux_adc_read_fifo();

  /* Disable the ADC */
  ti_lib_aux_adc_disable();

  aux_ctrl_unregister_consumer(&aux);

  /* Close the channel via the CTRL pin */
  ti_lib_gpio_clear_dio(BOARD_IOID_TPS62746_CTRL);

  /* Convert the ADC value into voltage */
  vbat = (float) val;
  vbat = vbat * (4.3 / 4095.0);		//scale for 4.3v reference and 12-bit, this is volt at pin
  vbat = vbat * 1470.0 / 470.0;		//scale for going through potential divider

  return (uint16_t) (vbat * 1000);	//return integer type as voltage in mV
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */

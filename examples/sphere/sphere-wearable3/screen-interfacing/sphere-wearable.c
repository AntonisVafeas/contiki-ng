/*
 * Copyright (c) 2020, University of Bristol - http://www.bristol.ac.uk/
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
 *
 * \brief
 *        This example replicates the functionality of the SPHERE Wearable 3
 *        deployments for Wearable3 with some minor additions:
 *        bidirectional communication, screen, wall-clock time, etc.
 * \author
 *        Xenofon (Fontas) Fafoutis <xenofon.fafoutis@bristol.ac.uk>
 *        Alex Hamilton <ah14128@my.bristol.ac.uk>
 *        Atis Elsts <atis.elsts@bristol.ac.uk>
 *        Antonis Vafeas <antonis.vafeas@bristol.ac.uk>
 */
#include "contiki.h"

#include <stdio.h> /* For printf() */
#include "board-peripherals.h" /*oled and others */
#include "dev/button-hal.h"
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
typedef struct sphere_time_of_day_s {
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint8_t cs;
} sphere_time_of_day_t;

static inline sphere_time_of_day_t
sphere_convert_cs_to_timeofday(uint64_t cs)
{
  sphere_time_of_day_t result;
  uint64_t t;

  t = cs % (86400 * 100);

  result.cs = t % 100;
  t /= 100;
  result.seconds = t % 60;
  t /= 60;
  result.minutes = t % 60;
  t /= 60;
  result.hours = t % 24;

  return result;
}
static inline const char *
sphere_print_time_of_day(sphere_time_of_day_t t)
{
  static char buffer[100];
  sprintf(buffer, "%02d:%02d:%02d", t.hours, t.minutes, t.seconds);
  return buffer;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data) {
  static struct etimer timer;

  PROCESS_BEGIN();
  tps62746_init();
  oled_init("W3: Wait...", 11);
  /* Setup a periodic timer that expires after 10 seconds. */
  etimer_set(&timer, CLOCK_SECOND * 5);
  /*uint8_t button_toggle=0; */
  mcp73832t_init();
  uint16_t vbat;
  vbat = tps62746_value();
  /*wall_clock_time_offset_cs */
  uint64_t wall_clock_time_offset_cs = 0;
  while(1) {
    PROCESS_YIELD()
    ;
    if(ev == PROCESS_EVENT_TIMER) {
      if(etimer_expired(&timer)) {
        /*oled_print_message("Timer", 6); */
        etimer_reset(&timer);
      }
      if(mcp73832t_is_charging()) {
        vbat = tps62746_value();
        printf("[W3] vbat:%d.\n", vbat);
        oled_init("W3: Wait...", 11);
        /*4.2-3.2 */
        oled_print_battery((vbat - 3.6) / 5);
      } else {
        oled_shutdown();
      }
    } else if(ev == button_hal_press_event) {
      /* This runs if the button is pressed */
      oled_brightness(0xFF);
      oled_init("Button on", 9);
      uint64_t time_cs = wall_clock_time_offset_cs + clock_time() * 100 / CLOCK_SECOND;
      const char *time_msg = sphere_print_time_of_day(
        sphere_convert_cs_to_timeofday(time_cs));
      oled_print_time(time_msg);
      /*show time */
      printf("[W3] Button pressed.\n");
    } else if(ev == button_hal_release_event) {
      /* This runs if the button is released */
      oled_brightness(0xFF);
      printf("[W3] Button released.\n");
    } else if(ev == button_hal_periodic_event) {
      /* This runs if the button is long pressed */
      oled_brightness(0xFF);
      oled_init("Long Press!", 8);
      printf("[W3] Button still pressed.\n");
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

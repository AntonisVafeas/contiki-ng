/*
 * mcp73832t.c
 *
 *  Created on: May 2, 2019
 *      Author: antonisvafeas
 */
#include "mcp73832t.h"
#include <stdio.h>

static gpio_hal_event_handler_t interrupt_handler_object;

#define INT_GPIO_CFG           (IOC_CURRENT_2MA | IOC_STRENGTH_AUTO | \
                                IOC_IOPULL_UP | IOC_SLEW_DISABLE | \
                                IOC_HYST_DISABLE | IOC_BOTH_EDGES | \
                                IOC_INT_ENABLE | IOC_IOMODE_NORMAL | \
                                IOC_NO_WAKE_UP | IOC_INPUT_ENABLE)
void
mcp73832t_init(void){
  ti_lib_gpio_clear_event_dio(BOARD_IOID_MCP73832T_STATUS);
  clock_wait(10);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_MCP73832T_STATUS);
  clock_wait(10);
  ti_lib_ioc_port_configure_set(BOARD_IOID_MCP73832T_STATUS, IOC_PORT_GPIO, INT_GPIO_CFG);
  clock_wait(10);
}
void
mcp73832t_configure_interrupt(gpio_hal_callback_t callback_ptr)
{
  interrupt_handler_object.pin_mask= BOARD_MCP73832T_STATUS;
  interrupt_handler_object.handler=callback_ptr;

  gpio_hal_register_handler(&interrupt_handler_object);
  clock_wait(10);
  ti_lib_ioc_int_enable(BOARD_IOID_MCP73832T_STATUS);
  clock_wait(10);
}
uint8_t
mcp73832t_is_charging(void){
	return !gpio_hal_arch_no_port_read_pin(BOARD_IOID_MCP73832T_STATUS);
}
/*---------------------------------------------------------------------------*/

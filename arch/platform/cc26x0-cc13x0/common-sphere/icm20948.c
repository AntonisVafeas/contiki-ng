/*
 * icm.c
 *
 *  Created on: 15 Oct 2018
 *      Author: xf14883 , Pawel Zalewski.
 */

#include "contiki.h"
#include "icm20948.h"
#include "board-i2c.h"
#include "ti-lib.h"
#include "sys/clock.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

static bool
icm_write_register(uint8_t reg, uint8_t val)
{
  uint8_t buf[3];

  buf[0] = reg;
  buf[1] = val;

  if(board_i2c_write(buf, sizeof(buf)) == false) {
    return false;
  }
  return true;
}
static uint8_t
icm_read_register(uint8_t reg)
{
  uint8_t val;

  if(board_i2c_write_read(&reg, 1, &val, 1) == false) {
    printf("I2C read error\n");
    return 0;
  }
  return val;
}
/*Generic function that read-writes registers to change specific bits while preserving others*/
static bool
icm_read_write_register(uint8_t reg, uint8_t reg_mask, uint8_t value, uint8_t value_mask)
{
  uint8_t v;
  v = icm_read_register(reg);
  v = (v & reg_mask) | ((value & value_mask));
  if(icm_write_register(reg, v) == false) {
    return false;
  }
  return true;
}
/*	#####################################
 #			Magnetometer			#
 ##################################### */

bool
icm_reset()
{
  return icm_read_write_register(REG_PWR_MGMT_1, 0b00001000, BIT_H_RESET, 0xFF);
}
void
icm_power_down(void)
{
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ICM20948_POWER);
  ti_lib_gpio_set_dio(BOARD_IOID_ICM20948_POWER);
  printf("[icm] Power down\n");
}
bool
icm_init(void)
{

  printf("[icm] Power Up\n");
  board_i2c_select(BOARD_I2C_INTERFACE_0, ICM_I2C_ADDR);
  board_i2c_wakeup();

  /*printf("whoami: %02x\n", icm_read_register(MCMAG_REG_WHO_I_AM)); */
  uint8_t whoami = icm_read_register(REG_WHO_AM_I);

  if(icm_reset() == false) {
    printf("[icm] Reset Error\n");
    return false;
  }
  board_i2c_deselect();

  return whoami == 0xEA;
}

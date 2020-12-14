/*
 * MC3672.c
 *
 *  Created on: 9 Jun 2017
 *      Author: User
 */

#include "contiki.h"
#include "mc3672.h"
#include "dev/spi.h"
/*#include <math.h> */
#include <stdio.h>

#if 0
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define INT_GPIO_CFG (IOC_CURRENT_2MA | IOC_STRENGTH_AUTO | \
                      IOC_IOPULL_UP | IOC_SLEW_DISABLE | \
                      IOC_HYST_DISABLE | IOC_FALLING_EDGE | \
                      IOC_INT_ENABLE | IOC_IOMODE_NORMAL | \
                      IOC_NO_WAKE_UP | IOC_INPUT_ENABLE)

/*---------------------------------------------------------------------------*/
/* This is the default SPI configuration to be used, unless another
 * is provided in initialisation
 */
static const spi_device_t mc3672_spi_configuration_default = {
  .spi_controller = BOARD_MC3672_SPI,
  .pin_spi_sck = BOARD_IOID_MC3672_SCK,
  .pin_spi_miso = BOARD_IOID_MC3672_MISO,
  .pin_spi_mosi = BOARD_IOID_MC3672_MOSI,
  .pin_spi_cs = BOARD_IOID_MC3672_CS,
  .spi_bit_rate = 2000000,
  .spi_pha = 1,
  .spi_pol = 1
};

static const spi_device_t *mc3672_spi_configuration;

static gpio_hal_event_handler_t interrupt_handler_object;

/*---------------------------------------------------------------------------*/

/*This function performs a read write action to a register to modify specific bit places*/
static bool
configure_register(uint8_t reg, uint8_t reg_mask, uint8_t value, uint8_t value_mask, uint8_t shift)
{
  uint8_t x;
  if(mc3672_read_reg(reg, &x) == false) {
    PRINTF("Configure: Can't Read Register \n");
    return false;
  }

  /* Apply the register and value mask */
  value = (x & reg_mask) | ((value & value_mask) << shift);

  /*Set to standby before writing back to the register*/
  mc3672_set_mode_standby();
  if(mc3672_write_reg(reg, value) == false) {
    PRINTF("Configure: Can't Write to Register \n");
    return false;
  }

  return true;
}
/*---------------------------------------------------------------------------*/

bool
mc3672_spi_open(void)
{
  /* Open the SPI controller */
  return spi_acquire(mc3672_spi_configuration) == SPI_DEV_STATUS_OK;
}
bool
mc3672_spi_close(void)
{
  return spi_release(mc3672_spi_configuration) == SPI_DEV_STATUS_OK;
}
/*---------------------------------------------------------------------------*/

bool
mc3672_write_reg(uint8_t address, uint8_t value)
{
  uint8_t wbuf[2];
  bool ret;

  wbuf[0] = (address | 0x40);
  wbuf[1] = value;

  if(mc3672_spi_open() == false) {
    PRINTF("[mc3672_write_reg ]SPI failed to open\n");
    return false;
  }

  /* Select the accelerometer */
  if(spi_select(mc3672_spi_configuration) != SPI_DEV_STATUS_OK) {
    mc3672_spi_close();
    return false;
  }

  /* Write the command, the register address, and the value */
  ret = (spi_write(mc3672_spi_configuration, wbuf, sizeof(wbuf)) == SPI_DEV_STATUS_OK);

  spi_deselect(mc3672_spi_configuration);
  mc3672_spi_close();
  return ret;
}
/*---------------------------------------------------------------------------*/

bool
mc3672_read_reg(uint8_t address, uint8_t *value)
{
  /* Reading a register is equivalent to reading a block of 1 byte */
  return mc3672_read_block(address, value, 1) == MC3672_RESULT_OK;
}
/*---------------------------------------------------------------------------*/

mc3672_result_t
mc3672_read_block(uint8_t address, uint8_t *buf, uint8_t len)
{
  uint8_t wbuf[1];
  mc3672_result_t ret = MC3672_RESULT_ERROR;

  wbuf[0] = address | 0x80 | 0x40;

  if(mc3672_spi_open() == false) {
    PRINTF("[mc3672_read_block]SPI failed to open\n");
    return MC3672_RESULT_ERROR;
  }

  /* Select the accelerometer */
  if(spi_select(mc3672_spi_configuration) != SPI_DEV_STATUS_OK) {
    mc3672_spi_close();
    return MC3672_RESULT_ERROR;
  }

  /* Write the command and the register address */
  if(spi_write(mc3672_spi_configuration, wbuf, sizeof(wbuf)) == SPI_DEV_STATUS_OK) {
    /* Read the specified amount of bytes */
    if(spi_read(mc3672_spi_configuration, buf, len) == SPI_DEV_STATUS_OK) {
      ret = MC3672_RESULT_OK;
    }
  }

  /* Deselect the accelerometer */
  spi_deselect(mc3672_spi_configuration);
  mc3672_spi_close();
  return ret;
}
/*---------------------------------------------------------------------------*/

bool
mc3672_init(mc3672_configuration_t *conf)
{
  /* This uses the default SPI configuration*/
  mc3672_spi_configuration = &mc3672_spi_configuration_default;

  /* Set the chip select pin */
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_MC3672_CS);
  ti_lib_gpio_set_dio(BOARD_IOID_MC3672_CS);

  /* Turn off the accelerometer */
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_MC3672_POWER);
  ti_lib_gpio_clear_dio(BOARD_IOID_MC3672_POWER);

  clock_wait(50);

  ti_lib_gpio_set_dio(BOARD_IOID_MC3672_POWER);

  clock_wait(10);

  if(mc3672_reset() == false) {
    PRINTF("[MC3672] Reset Error\n");
    mc3672_spi_close();
    return false;
  }

  /*Start Configurations*/
  if(mc3672_configuration(conf) == false) {
    PRINTF("[MC3672] Configuration Error\n");
    mc3672_spi_close();
    return false;
  }

  mc3672_spi_close();
  return true;
}
/*---------------------------------------------------------------------------*/

void
mc3672_power_down()
{
  /* Turn off the accelerometer V_1.8V from TPS78318 (EN low-OFF)*/
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_MC3672_POWER);
  ti_lib_gpio_clear_dio(BOARD_IOID_MC3672_POWER);
  ti_lib_ioc_int_disable(BOARD_IOID_MC3672_INT);
}
/*---------------------------------------------------------------------------*/
bool
mc3672_reset()
{
  if(mc3672_write_reg(MC3672_REG_MODE_C, 0x01) == false) {
    PRINTF("fail to write MC3672_REG_MODE_C\n");
    return false;
  }

  clock_wait(12);

  if(mc3672_write_reg(MC3672_REG_RESET, 0x40) == false) {
    PRINTF("fail to write MC3672_REG_RESET\n");
  }

  clock_wait(60);
  if(mc3672_write_reg(0x09, 0x00) == false) {
    PRINTF("fail to write to reg 0x09\n");
  }

  clock_wait(12);
  if(mc3672_write_reg(MC3672_REG_FEATURE_1, 0x90) == false) {
    PRINTF("fail to write MC3672_REG_FEATURE_1\n");
  }

  clock_wait(12);
  if(mc3672_write_reg(MC3672_REG_INIT_1, 0x42) == false) {
    PRINTF("fail to write MC3672_REG_INIT_1\n");
  }

  clock_wait(12);
  if(mc3672_write_reg(MC3672_REG_DMX, 0x01) == false) {
    PRINTF("fail to write MC3672_REG_DMX\n");
  }

  clock_wait(12);
  if(mc3672_write_reg(MC3672_REG_DMY, 0x80) == false) {
    PRINTF("fail to write MC3672_REG_DMY\n");
  }

  clock_wait(12);
  if(mc3672_write_reg(MC3672_REG_INIT_2, 0x00) == false) {
    PRINTF("fail to write MC3672_REG_INIT_2\n");
  }

  clock_wait(12);
  if(mc3672_write_reg(MC3672_REG_INIT_3, 0x00) == false) {
    PRINTF("fail to write MC3672_REG_INIT_3\n");
  }

  clock_wait(12);
  if(mc3672_write_reg(0x10, 0x01) == false) {
    PRINTF("fail to write reg 0x10\n");
  }

  clock_wait(12);
  return true;
}
/*---------------------------------------------------------------------------*/

MC3672_acc_data_t
mc3672_read_fifo(uint8_t data_resolution)
{
  uint8_t data[6];
  if(mc3672_read_block(MC3672_REG_XOUT_LSB, data, 6) != MC3672_RESULT_OK) {
    PRINTF("[MC3672] Couldn't Read FIFO\n");
  }

  /* Ignore the most-significant bit of the lowest byte */
  AccRaw.XAxisRaw = (((uint16_t)(data[1]) << 7) | (data[0] & 0x7f));
  AccRaw.YAxisRaw = (((uint16_t)(data[3]) << 7) | (data[2] & 0x7f));
  AccRaw.ZAxisRaw = (((uint16_t)(data[5]) << 7) | (data[4] & 0x7f));
  AccRaw.XAxisLSB = (uint8_t)AccRaw.XAxisRaw;
  AccRaw.YAxisLSB = (uint8_t)AccRaw.YAxisRaw;
  AccRaw.ZAxisLSB = (uint8_t)AccRaw.ZAxisRaw;

  return AccRaw;
}
mc3672_result_t
mc3672_fifo_empty()
{
  uint8_t v;
  if(mc3672_read_reg(MC3672_REG_STATUS_1, &v) == false) {
    return MC3672_RESULT_ERROR;
  }
  if(v & 0x10) {
    return MC3672_RESULT_OK;
  }
  return MC3672_RESULT_ERROR;
}
/*---------------------------------------------------------------------------*/
mc3672_result_t
mc3672_fill_payload(uint8_t *payload)
{
  uint8_t rbuf[6];
  for(int i = 0; i < 6; ++i) {
    if(mc3672_read_block(MC3672_REG_XOUT_LSB, rbuf, 6) != MC3672_RESULT_OK) {
      PRINTF("mc3672_read_block");
      return MC3672_RESULT_ERROR;
    }
    uint8_t *ptr = rbuf;
    /*Data is in 2's complement, for full +/- range cast to int */
    /*
       int8_t x, y, z;
       x = rbuf[0];
       y = rbuf[2];
       z = rbuf[4];
       PRINTF("x:%d y:%d z:%d g:%d\n", x, y, z, (uint16_t)(sqrt(x * x + y * y + z * z)) << 5);
     */
    for(int j = 0; j < 3; ++j) {
      /*FIFO Readings start from register 0x02 and end at 0x07
       |Address|Description|Bit 7   |Bit 6   |Bit 5   |Bit 4   |Bit 3   |Bit 2   |Bit 1  |Bit 0  |
       |0x02   |XOUT_LSB   |XOUT[7] |XOUT[6] |XOUT[5] |XOUT[4] |XOUT[3] |XOUT[2] |XOUT[1]|XOUT[0]|
       |0x03   |XOUT_MSB   |XOUT[15]|XOUT[14]|XOUT[13]|XOUT[12]|XOUT[11]|XOUT[10]|XOUT[9]|XOUT[8]|
       |0x04   |YOUT_LSB   |YOUT[7] |YOUT[6] |YOUT[5] |YOUT[4] |YOUT[3] |YOUT[2] |YOUT[1]|YOUT[0]|
       |0x05   |YOUT_MSB   |YOUT[15]|YOUT[14]|YOUT[13]|YOUT[12]|YOUT[11]|YOUT[10]|YOUT[9]|YOUT[8]|
       |0x06   |ZOUT_LSB   |ZOUT[7] |ZOUT[6] |ZOUT[5] |ZOUT[4] |ZOUT[3] |ZOUT[2] |ZOUT[1]|ZOUT[0]|
       |0x07   |ZOUT_MSB   |ZOUT[15]|ZOUT[14]|ZOUT[13]|ZOUT[12]|ZOUT[11]|ZOUT[10]|ZOUT[9]|ZOUT[8]|
       */
      /* For 8 bits OUT_LSB is only required */
      *payload = *ptr++;
      payload++;
      ptr++;
    }
  }
  PRINTF("\n");
  return MC3672_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
mc3672_result_t
check_error_code(int8_t key)
{
  switch(key) {
  case MC3672_RETCODE_SUCCESS:
    return MC3672_RESULT_OK;
    break;
  case MC3672_RETCODE_ERROR_WRONG_ARGUMENT:
    return MC3672_RESULT_NOT_SUPPORTED;
    break;
  default:
    return MC3672_RESULT_ERROR;
    break;
  }
}
/*---------------------------------------------------------------------------*/

bool
mc3672_set_mode(uint8_t mode)
{
  return configure_register(MC3672_REG_MODE_C, 0b11111000, mode, 0b00000111, 0);
}
/*---------------------------------------------------------------------------*/

void
mc3672_set_mode_standby(void)
{
  uint8_t v;

  if(mc3672_read_reg(0x10, &v) == false) {
    PRINTF("Standby Error\n");
    return;
  }

  v = (v & 0b11111000) | 0x01;

  if(mc3672_write_reg(0x10, v) == false) {
    PRINTF("Standby Error\n");
    return;
  }
}
/*---------------------------------------------------------------------------*/

bool
mc3672_clear_interrupt(void)
{
  uint8_t v;

  if(mc3672_read_reg(MC3672_REG_STATUS_2, &v) == false) {
    return false;
  }

  v &= 0x03;
  if(mc3672_write_reg(MC3672_REG_STATUS_2, v) == false) {
    return false;
  }

  return true;
}
/*---------------------------------------------------------------------------*/

uint8_t
mc3672_get_register(uint8_t reg)
{
  uint8_t v;

  if(mc3672_read_reg(reg, &v) == false) {
    PRINTF("Get register error\n");
  }

  return v;
}
/*---------------------------------------------------------------------------*/
/* Configures both the range and resolution */

bool
mc3672_configure_range_resolution(mc3672_configuration_t *conf)
{
  uint8_t value;
  value = ((conf->g_range << 4) | conf->data_resolution);
  return configure_register(MC3672_REG_RANGE_C, 0b10001000, value, 0b01110111, 0);
}
/*---------------------------------------------------------------------------*/

bool
mc3672_configure_FIFO(mc3672_configuration_t *conf)
{
  uint8_t value;
  uint8_t fifo;

  mc3672_set_mode_standby();

  if(conf->fifo_thres > 31) {
    fifo = 31;
  } else {
    fifo = conf->fifo_thres;
  }

  value = (conf->fifo_enable << 6) | (conf->fifo_control << 5) | fifo;
  return mc3672_write_reg(MC3672_REG_FIFO_C, value);
}
bool
mc3672_configure_fifo_burst(uint8_t enable)
{
  return configure_register(MC3672_REG_FREG_2, 0b00000010, enable, 0b00000000, 1);
}
/*---------------------------------------------------------------------------*/

bool
mc3672_configure_sniff(mc3672_configuration_t *conf)
{
  uint8_t value = 0b00000000;

  if(configure_register(MC3672_REG_SNIFF_CONF_C, 0b11110111, 1, 0b00000001, 3) == false) {
    return false;
  }

  value |= (conf->sniff_mode << 1) | conf->sniff_and_or;
  if(configure_register(MC3672_REG_SNIFFTH_C, 0b00111111, value, 0b00000011, 6) == false) {
    return false;
  }

  return true;
}
bool
mc3672_configure_sniff_threshold(mc3672_configuration_t *conf, uint8_t axis)
{
  uint8_t threshold, count;

  switch(axis) {
  case MC3672_SNIFF_THRES_X:
    threshold = conf->sniff_thres_x;
    count = conf->sniff_count_x;
    break;
  case MC3672_SNIFF_THRES_Y:
    threshold = conf->sniff_thres_y;
    count = conf->sniff_count_y;
    break;
  case MC3672_SNIFF_THRES_Z:
    threshold = conf->sniff_thres_z;
    count = conf->sniff_count_z;
    break;
  default:
    return false;
  }

  if(configure_register(MC3672_REG_SNIFF_CONF_C, 0b11111000, axis, 0b00000111, 0) == false) {
    return false;
  }
  if(configure_register(MC3672_REG_SNIFFTH_C, 0b11000000, threshold, 0b0011111, 0) == false) {
    return false;
  }
  if(configure_register(MC3672_REG_SNIFF_CONF_C, 0b11111000, axis + 4, 0b00011111, 0) == false) {
    return false;
  }
  if(configure_register(MC3672_REG_SNIFFTH_C, 0b11000000, count, 0b00111111, 0) == false) {
    return false;
  }

  return true;
}
bool
mc3672_configure_sniff_SR(uint8_t sample_rate)
{
  return configure_register(MC3672_REG_SNIFF_C, 0b11100000, sample_rate, 0b00011111, 0);
}
/*---------------------------------------------------------------------------*/

bool
mc3672_configure_power(mc3672_configuration_t *conf)
{
  uint8_t value;
  value = (conf->sniff_power << 4) | conf->wake_power;
  return configure_register(MC3672_REG_POWER_MODE, 0b10001000, value, 0b01110111, 0);
}
/*---------------------------------------------------------------------------*/

bool
mc3672_configure_wake_SR(uint8_t sample_rate)
{
  uint8_t value;
  value = sample_rate;
  mc3672_set_mode_standby();
  return mc3672_write_reg(MC3672_REG_WAKE_C, value);
}
/*---------------------------------------------------------------------------*/

bool
mc3672_configure_mode_set(uint8_t mode)
{
  uint8_t x;

  if(mc3672_read_reg(MC3672_REG_MODE_C, &x) == false) {
    PRINTF("Configure: Can't Read Register \n");
    return false;
  }

  mode = (x & 0b11111000) | ((mode & 0b00000111));

  if(mc3672_write_reg(MC3672_REG_MODE_C, mode) == false) {
    PRINTF("Configure: Can't Write to Register \n");
    return false;
  }

  /* Wait before checking the status register for an updated mode */
  clock_wait(15);

  if(mc3672_read_reg(MC3672_REG_STATUS_1, &x) == false) {
    PRINTF("Configure: Can't Read Register \n");
    return false;
  }

  x &= 0x07;
  if(mode != x) {
    PRINTF("mode=0x%02x vs. x=0x%02x\n", mode, x);
    return false;
  }
  return true;
}
/*---------------------------------------------------------------------------*/

bool
mc3672_configure_interrupt(mc3672_configuration_t *conf)
{
  uint8_t value;

  /* Check which of the interrupts has been enabled */
  value = conf->int_swake << 7;
  value |= conf->int_fifo_thres << 6;
  value |= conf->int_fifo_full << 5;
  value |= conf->int_fifo_empty << 4;
  value |= conf->int_ACQ << 3;
  value |= conf->int_wake << 2;

  if(mc3672_write_reg(MC3672_REG_INTR_C, value) == false) {
    PRINTF("Configure: Can't Write to Register \n");
    return false;
  }

  /* Configure the interrupt and the interrupt handler */
  ti_lib_gpio_clear_event_dio(BOARD_IOID_MC3672_INT);
  clock_wait(10);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_MC3672_INT);
  clock_wait(10);
  ti_lib_ioc_port_configure_set(BOARD_IOID_MC3672_INT, IOC_PORT_GPIO, INT_GPIO_CFG);
  clock_wait(10);
  interrupt_handler_object.pin_mask = BOARD_MC3672_INT;
  interrupt_handler_object.handler = conf->interrupt_handler;
  gpio_hal_register_handler(&interrupt_handler_object);
  clock_wait(10);
  ti_lib_ioc_int_enable(BOARD_IOID_MC3672_INT);
  clock_wait(10);
  return true;
}
/*---------------------------------------------------------------------------*/

/*  Changes the offset for any of the axes*/
bool
mc3672_configure_offset(int value, uint8_t axis)
{
  uint8_t v;
  uint8_t reg = MC3672_REG_XOFFL + (2 * axis);

  if(mc3672_read_reg(reg, &v) == false) {
    return false;
  }

  v += value;

  if(mc3672_write_reg(reg, v) == false) {
    return false;
  }

  return true;
}
/*---------------------------------------------------------------------------*/

/* Use a conf struct to set the variables to be used in the Accelerometer */
bool
mc3672_configuration(mc3672_configuration_t *conf)
{
  if(mc3672_configure_range_resolution(conf) == false) {
    PRINTF("Range or Resolution Error\n");
    return false;
  }

  if(mc3672_configure_FIFO(conf) == false) {
    PRINTF("FIFO Error\n");
    return false;
  }

  if(mc3672_configure_power(conf) == false) {
    PRINTF("Power mode error\n");
    return false;
  }

  if(mc3672_configure_wake_SR(conf->wake_sr) == false) {
    PRINTF("WAKE SR error\n");
    return false;
  }

  if(conf->modes == MC3672_MODE_SNIFF || conf->modes == MC3672_MODE_SWAKE) {
    if(mc3672_configure_sniff_SR(conf->sniff_sr) == false) {
      PRINTF("SNIFF SR error\n");
      return false;
    }

    if(mc3672_configure_sniff_threshold(conf, MC3672_SNIFF_THRES_X) == false) {
      PRINTF("SNIFF THRESHOLD error\n");
      return false;
    }

    if(mc3672_configure_sniff_threshold(conf, MC3672_SNIFF_THRES_Y) == false) {
      PRINTF("SNIFF THRESHOLD error\n");
      return false;
    }

    if(mc3672_configure_sniff_threshold(conf, MC3672_SNIFF_THRES_Z) == false) {
      PRINTF("SNIFF THRESHOLD error\n");
      return false;
    }

    if(mc3672_configure_sniff(conf) == false) {
      PRINTF("SNIFF error\n");
      return false;
    }
  }
  /* if(mc3672_configure_offset(conf->offset_y, YAxis)==false)
     {
     return false;
     }  */

  if(mc3672_configure_fifo_burst(1) == false) {
    return false;
  }

  if(mc3672_configure_interrupt(conf) == false) {
    PRINTF("Interrupt error\n");
    return false;
  }
  mc3672_clear_interrupt();

  if(mc3672_configure_mode_set(conf->modes) == false) {
    PRINTF("Mode set error\n");
    return false;
  }

  return true;
}
uint8_t
mc3672_interrupt_pin_active(void)
{
  return !gpio_hal_arch_no_port_read_pin(BOARD_IOID_MC3672_INT);
}

/*
 * MC3672.h
 *
 *  Created on: 14 Jun 2017
 *      Author: User
 */

#ifndef MC3672_H_
#define MC3672_H_

#include "dev/gpio-hal.h"
#include <stdbool.h>

#define MSGSIZE                               8
#define SELECT_ON                             0
#define SELECT_OFF                            1
#define MAX_SAMPLES                           12

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define MC3672_RETCODE_SUCCESS                 (0)
#define MC3672_RETCODE_ERROR_BUS               (-1)
#define MC3672_RETCODE_ERROR_NULL_POINTER      (-2)
#define MC3672_RETCODE_ERROR_STATUS            (-3)
#define MC3672_RETCODE_ERROR_SETUP             (-4)
#define MC3672_RETCODE_ERROR_GET_DATA          (-5)
#define MC3672_RETCODE_ERROR_IDENTIFICATION    (-6)
#define MC3672_RETCODE_ERROR_NO_DATA           (-7)
#define MC3672_RETCODE_ERROR_WRONG_ARGUMENT    (-8)
#define MC3672_FIFO_DEPTH             32
#define MC3672_REG_MAP_SIZE             64

/* Radio return values when setting or getting radio parameters. */
typedef enum {
  MC3672_RESULT_OK,
  MC3672_RESULT_NOT_SUPPORTED,
  MC3672_RESULT_ERROR,
} mc3672_result_t;

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/

/*============================================= */
#define MC3672_INTR_C_IPP_MODE_OPEN_DRAIN    (0x00)
#define MC3672_INTR_C_IPP_MODE_PUSH_PULL     (0x01)

#define MC3672_INTR_C_IAH_ACTIVE_LOW      (0x00)
#define MC3672_INTR_C_IAH_ACTIVE_HIGH     (0x02)

/*******************************************************************************
 *** Register Map
 *******************************************************************************/
/*============================================= */
#define MC3672_REG_EXT_STAT_1       (0x00)
#define MC3672_REG_EXT_STAT_2       (0x01)
#define MC3672_REG_XOUT_LSB         (0x02)
#define MC3672_REG_XOUT_MSB         (0x03)
#define MC3672_REG_YOUT_LSB         (0x04)
#define MC3672_REG_YOUT_MSB         (0x05)
#define MC3672_REG_ZOUT_LSB         (0x06)
#define MC3672_REG_ZOUT_MSB         (0x07)
#define MC3672_REG_STATUS_1         (0x08)
#define MC3672_REG_STATUS_2         (0x09)
#define MC3672_REG_FEATURE_1    (0x0D)
#define MC3672_REG_FREG_2           (0x0E)
#define MC3672_REG_INIT_1     (0x0F)
#define MC3672_REG_MODE_C           (0x10)
#define MC3672_REG_WAKE_C           (0x11)
#define MC3672_REG_SNIFF_C          (0x12)
#define MC3672_REG_SNIFFTH_C        (0x13)
#define MC3672_REG_SNIFF_CONF_C     (0x14)
#define MC3672_REG_RANGE_C          (0x15)
#define MC3672_REG_FIFO_C           (0x16)
#define MC3672_REG_INTR_C           (0x17)
#define MC3672_REG_PROD             (0x18)
#define MC3672_REG_INIT_3     (0x1A)
#define MC3672_REG_POWER_MODE       (0x1C)
#define MC3672_REG_DMX              (0x20)
#define MC3672_REG_DMY              (0x21)
/*#define MC3672_REG_GAIN             (0x21) */
#define MC3672_REG_DMZ              (0x22)
#define MC3672_REG_RESET            (0x24)
#define MC3672_REG_INIT_2     (0X28)
#define MC3672_REG_XOFFL            (0x2A)
#define MC3672_REG_XOFFH            (0x2B)
#define MC3672_REG_YOFFL            (0x2C)
#define MC3672_REG_YOFFH            (0x2D)
#define MC3672_REG_ZOFFL            (0x2E)
#define MC3672_REG_ZOFFH            (0x2F)
#define MC3672_REG_XGAIN            (0x30)
#define MC3672_REG_YGAIN            (0x31)
#define MC3672_REG_ZGAIN            (0x32)
#define MC3672_REG_OPT              (0x3B)
#define MC3672_REG_LOC_X            (0x3C)
#define MC3672_REG_LOC_Y            (0x3D)
#define MC3672_REG_LOT_dAOFSZ       (0x3E)
#define MC3672_REG_WAF_LOT          (0x3F)

#define MC3672_NULL_ADDR        (0)

/*Interrupt values*/

#define MC3672_INT_WAKE       (0x02)
#define MC3672_INT_ACQ        (0x03)
#define MC3672_INT_FIFO_EMPTY   (0x04)
#define MC3672_INT_FIFO_FULL    (0x05)
#define MC3672_INT_FIFO_THRES   (0x06)
#define MC3672_INT_SWAKE      (0x07)

/*Configuration values*/

#define ENABLE            0x01
#define DISABLE           0x00

typedef struct MC3672_acc_data_s {
  uint16_t XAxisRaw;
  uint16_t YAxisRaw;
  uint16_t ZAxisRaw;
  uint8_t XAxisLSB;
  uint8_t YAxisLSB;
  uint8_t ZAxisLSB;
} MC3672_acc_data_t;

typedef enum {
  MC3672_GAIN_DEFAULT = 0b00,      /* 0 */
  MC3672_GAIN_4X = 0b01,           /* 1 */
  MC3672_GAIN_1X = 0b10,           /* 2 */
  MC3672_GAIN_NOT_USED = 0b11,     /* 3 */
}   MC3672_gain_t;

typedef enum {
  MC3672_MODE_SLEEP = 0b000,
  MC3672_MODE_STANDBY = 0b001,
  MC3672_MODE_SNIFF = 0b010,
  MC3672_MODE_CWAKE = 0b101,
  MC3672_MODE_SWAKE = 0b110,
  MC3672_MODE_TRIG = 0b111,
}   MC3672_mode_t;

/* Note: these values are as the datasheet says.
   Not sure if the 16g and 12g are mixed up or not: not tesed! */
typedef enum {
  MC3672_RANGE_2G = 0b000,
  MC3672_RANGE_4G = 0b001,
  MC3672_RANGE_8G = 0b010,
  MC3672_RANGE_16G = 0b011,
  MC3672_RANGE_12G = 0b100,
  MC3672_RANGE_END,
}   MC3672_range_t;

typedef enum {
  MC3672_RESOLUTION_6BIT = 0b000,
  MC3672_RESOLUTION_7BIT = 0b001,
  MC3672_RESOLUTION_8BIT = 0b010,
  MC3672_RESOLUTION_10BIT = 0b011,
  MC3672_RESOLUTION_12BIT = 0b100,
  MC3672_RESOLUTION_14BIT = 0b101,      /*(Do not select if FIFO enabled) */
  MC3672_RESOLUTION_END,
}   MC3672_resolution_t;
/*
   CWAKE ODR
   For Ultra-Low Power Register 0x1C[2:0]  Value 0x03
   For Low Power Register      0x1C[2:0]   Value 0x00
   Notice: There is difference in ODR Sampling rate depending on power mode
 */
typedef enum {
  MC3672_CWAKE_SR_DEFAULT_54Hz = 0b0000,
  MC3672_CWAKE_SR_14Hz = 0b0101,
  MC3672_CWAKE_SR_28Hz = 0b0110,
  MC3672_CWAKE_SR_54Hz = 0b0111,
  MC3672_CWAKE_SR_105Hz = 0b1000,
  MC3672_CWAKE_SR_210Hz = 0b1001,
  MC3672_CWAKE_SR_400Hz = 0b1010,
  MC3672_CWAKE_SR_600Hz = 0b1011,
  MC3672_CWAKE_SR_END,
}   MC3672_cwake_sr_t;
/*
   //Ultra Low Power
   typedef enum {
   MC3672_CWAKE_SR_DEFAULT_54Hz = 0b0000,
   MC3672_CWAKE_SR_25Hz = 0b0110,
   MC3672_CWAKE_SR_50Hz = 0b0111,
   MC3672_CWAKE_SR_100Hz = 0b1000,
   MC3672_CWAKE_SR_190Hz = 0b1001,
   MC3672_CWAKE_SR_380Hz = 0b1010,
   MC3672_CWAKE_SR_750Hz = 0b1011,
   }   MC3672_cwake_sr_t;
 */
typedef enum {
  MC3672_SNIFF_SR_DEFAULT_7Hz = 0b0000,
  MC3672_SNIFF_SR_0p4Hz = 0b0001,
  MC3672_SNIFF_SR_0p8Hz = 0b0010,
  MC3672_SNIFF_SR_1p5Hz = 0b0011,
  MC3672_SNIFF_SR_7Hz = 0b0100,
  MC3672_SNIFF_SR_14Hz = 0b0101,
  MC3672_SNIFF_SR_28Hz = 0b0110,
  MC3672_SNIFF_SR_54Hz = 0b0111,
  MC3672_SNIFF_SR_105Hz = 0b1000,
  MC3672_SNIFF_SR_210Hz = 0b1001,
  MC3672_SNIFF_SR_400Hz = 0b1010,
  MC3672_SNIFF_SR_600Hz = 0b1011,
  MC3672_SNIFF_SR_END,
}   MC3672_sniff_sr_t;

typedef enum {
  MC3672_FIFO_CONTROL_DISABLE = 0,
  MC3672_FIFO_CONTROL_ENABLE = 1,
  MC3672_FIFO_CONTROL_END,
}   MC3672_fifo_control_t;

typedef enum {
  MC3672_FIFO_MODE_NORMAL = 0,
  MC3672_FIFO_MODE_WATERMARK,
  MC3672_FIFO_MODE_END,
}   MC3672_fifo_mode_t;

typedef enum {
  MC3672_LOWPOWER = 0b000,
  MC3672_ULTRALOWPOWER = 0b011,
  MC3672_PRECISION = 0b100,
}   MC3672_power_mode_t;

typedef enum {
  MC3672_SNIFF_C2P = 0b0,
  MC3672_SNIFF_C2B = 0b1,
}MC3672_sniff_mode_t;

typedef enum {
  MC3672_SNIFF_OR = 0b0,
  MC3672_SNIFF_AND = 0b1,
}MC3672_sniff_and_or_t;

typedef enum {
  MC3672_SNIFF_THRES_X = 0b001,
  MC3672_SNIFF_THRES_Y = 0b010,
  MC3672_SNIFF_THRES_Z = 0b011,
  MC3672_SNIFF_COUNT_X = 0b101,
  MC3672_SNIFF_COUNT_Y = 0b110,
  MC3672_SNIFF_COUNT_Z = 0b111,
}MC3672_thres_reg_t;

typedef struct {
  unsigned char bWAKE;                   /* Sensor wakes from sniff mode. */
  unsigned char bACQ;                    /* New sample is ready and acquired. */
  unsigned char bFIFO_EMPTY;             /* FIFO is empty. */
  unsigned char bFIFO_FULL;              /* FIFO is full. */
  unsigned char bFIFO_THRESHOLD;         /* FIFO sample count is equal to or greater than the threshold count. */
  unsigned char bRESV;
  unsigned char baPadding[2];
}   MC3672_InterruptEvent;

typedef enum {
  MC3672_WAKE_INT = 0,
  MC3672_ACQ = 0,
  MC3672_SWAKE_MODE = 0,
  MC3672_TEST_MODE = 0b01000000,
} MC3672_Interrupt_t;

typedef enum {
  XAxis = 0b00,
  YAxis = 0b01,
  ZAxis = 0b10,
}MC3672_axisOffset_t;

/*The different configuration options available */
typedef struct mc3672_configuration_s {
  uint8_t g_range;
  uint8_t data_resolution;
  uint8_t modes;
  /*fifo */
  uint8_t fifo_control;
  uint8_t fifo_enable;
  uint8_t fifo_thres;
  /*sniff */
  uint8_t sniff_sr;
  uint8_t sniff_thres;
  uint8_t sniff_thres_x;
  uint8_t sniff_thres_y;
  uint8_t sniff_thres_z;
  uint8_t sniff_count;
  uint8_t sniff_count_x;
  uint8_t sniff_count_y;
  uint8_t sniff_count_z;
  uint8_t sniff_and_or;
  uint8_t sniff_conf;
  uint8_t sniff_mode;
  uint8_t sniff_power;
  /*Wake */
  uint8_t wake_sr;
  uint8_t wake_power;
  uint8_t intsc_en;
  uint8_t freeze;
  /*Interrupts */
  uint8_t int_conf;
  uint8_t int_wake;
  uint8_t int_ACQ;
  uint8_t int_fifo_empty;
  uint8_t int_fifo_full;
  uint8_t int_fifo_thres;
  uint8_t int_swake;
  gpio_hal_callback_t interrupt_handler;
  /*offset */
  int8_t offset_x;
  int8_t offset_y;
  int8_t offset_z;
} mc3672_configuration_t;

MC3672_acc_data_t AccRaw;

/**
 *  \brief Opens the SPI bus for the MC3672.
 *  \return True when successfully opened.
 *
 *  This function must be called before any read and write functions on the SPI bus.
 *
 */

bool mc3672_spi_open(void);

/**
 *  \brief Closes the SPI bus for the MC3672.
 *  \return True when successfully closed .
 *
 *  The SPI must be closed after the each SPI operation to allow the SPI bus to be
 *  used by other peripherals.
 */

bool mc3672_spi_close(void);

/**
 *
 * \brief These functions perform the read and write actions of the MC3672.
 * \param address  : Register address
 * \param value  : Pointer being either read or written to the register
 * \param buf    : Point used to write the data from the accelerometer
 * \param len    : Length of the data to read
 *  These functions return true when they are successful.
 */

bool mc3672_write_reg(uint8_t address, uint8_t value);
bool mc3672_read_reg(uint8_t address, uint8_t *value);
mc3672_result_t mc3672_read_block(uint8_t address, uint8_t *buf, uint8_t len);
mc3672_result_t mc3672_fill_payload(uint8_t *payload);
/**
 *\brief Initialises the accelerometer and calls the configuration function.
 *\return True when successfully initialised.
 *
 *  This function also performs a hard reset to reset the registers to their POR state.
 *
 */

bool mc3672_init();

/**
 *\brief Initialise the accelerometer to a power down mode
 *\return True when successfully initialised.
 *
 *	Set the accelerometer pin to shut down device and minimise power consumption
 *
 */
void mc3672_power_down();

/**
 *\brief performs a soft reset
 *\retrun True when successful.
 */
bool mc3672_reset();

/**
 *\brief Calls all configurations.
 *\param conf : The configurations provided by the main file.
 *\return True when successful.
 */

bool mc3672_configuration(mc3672_configuration_t *conf);

/**
 * \brief Configures register 0x15 to change the G range and data resolution
 * \param *conf   : configuration provided by main file
 * \return True when successful.
 *
 * This functions allows for the changing of two options since they are both on
 * the same register. The function uses .g_range and .data_resolution from conf.
 *
 */

bool mc3672_configure_range_resolution(mc3672_configuration_t *conf);

/**
 * \brief Configures register 0x16 to change FIFO options
 * \param *conf   : configuration provided by main file
 * \return True when successful.
 *
 */

bool mc3672_configure_FIFO(mc3672_configuration_t *conf);

/**
 * \brief Configures the FIFO burst feature
 *   \param enable : Set to 1 to enable
 * \return True when successful
 *
 * Currently not a variable in the conf structure and is enabled by default in the configuration function
 */

bool mc3672_configure_fifo_burst(uint8_t enable);

/**
 * \brief Configures register 0x1C to change power options.
 * \param *conf   : configuration provided by main file.
 * \return True when successful.
 *
 * This function changes both the WAKE and SNIFF power at the same time.
 *
 */

bool mc3672_configure_power(mc3672_configuration_t *conf);

/**
 *  \brief Configures register 0x11 to modify the WAKE mode sampling rate.
 *  \param  conf  : The configuration provided by the main file.
 *  \return True when successful.
 *
 *  The sampling rate also depends on the power mode the accelerometer is set at
 *  see the data sheet (register 0x11) for more information.
 *  These rates are not the same as those for the SNIFF mode.
 *
 */

bool mc3672_configure_wake_SR(uint8_t sample_rate);

/**
 *  \brief Configures register 0x12 to modify the SNIFF mode sampling rate.
 *  \param  conf  : The configuration provided by the main file.
 *  \return True when successful.
 *
 *  The sampling rate also depends on the power mode the accelerometer is set at
 *  see the data sheet (register 0x12) for more information.
 *  These rates are not the same as those for the WAKE mode.
 *
 */

bool mc3672_configure_sniff_SR(uint8_t sample_rate);

/**
 * \brief Configures options for SNIFF including, enabling SNIFF, change delta count mode, and the
 *  logical operation between separate axis counts.
 * \param *conf   : configuration provided by main file
 * \return True when successful.
 *
 */

bool mc3672_configure_sniff(mc3672_configuration_t *conf);

/**
 * \brief Configures registers 0x13 and 0x14 changing the SNIFF activity.
 * threshold as well as the counts for each time the threshold is crossed.
 * \param *conf   : configuration provided by main file.
 * \param axis    : specifies the axis for the threshold level to be set at
 * \return True when successful.
 *
 * There are several shadow registers in register 0x13. Which register is being written to depends on the value of 0x14.
 * The value in register 0x14 specifies which axis the threshold is being set at.
 * The value in register 0x13 specifies the threshold level or count.
 *
 */
bool mc3672_configure_sniff_threshold(mc3672_configuration_t *conf, uint8_t axis);

/**
 *  \brief  Configures register 0x10 to change the mode of operation.
 *  \param  mode  : The starting mode of operation.
 *  \return True when successful.
 *
 *  This function differs from the mode set function bellow by checking register 0x08
 *  that the correct mode has been set. This function usually returns an error if two
 *  conflicting configuration options are given as the mode is set to sleep or standby
 *
 */

bool mc3672_configure_mode_set(uint8_t mode);

/**
 * \brief Configures registers 0x17 to allow for different interrupts .
 * \param *conf   : configuration provided by main file.
 * \return True when successful.
 *
 *  Writing to this register enables and disables interrupts such as the
 *  the FIFO threshold interrupt or the an interrupt when a WAKE event occurs.
 *
 */

bool mc3672_configure_interrupt(mc3672_configuration_t *conf);

/**
 * \brief Writes to register 0x09 to clear pending interrupts.
 * \return True when successful.
 */

bool mc3672_clear_interrupt();

/**
 * \brief Changes the mode that the accelerometer is in.
 * \param mode    : The mode of the accelerometer to be changed to.
 * \return True when successful.
 */

bool mc3672_set_mode(uint8_t mode);

/**
 * \brief Reads data from the FIFO.
 * \param data_resolution : The resolution of bits to be sent.
 * \return This function returns the struct MC3672_acc_data_t which contains
 * both the raw data from the accelerometer and data formatted in the resolution
 * specified by the configuration.
 */

/**
 * \brief Check the error Code
 * \param key : The return code from spi
 * \return This function returns the struct mc3672_result_t which contains
 * three error conditions
 */
mc3672_result_t check_error_code(int8_t key);

MC3672_acc_data_t mc3672_read_fifo(uint8_t data_resolution);

/**
 * \brief Checks the status register (0x08) to see if the FIFO is empty.
 * \return False when FIFO is empty or if the register could not be read.
 * True if FIFO is not empty.
 */

mc3672_result_t mc3672_fifo_empty(void);

/**
 * \brief Sets the accelerometer to standby.
 *
 * The accelerometer must be in standby when writing to registers so it
 * is stable.
 *
 */

void mc3672_set_mode_standby(void);

/**
 *\brief Allows for other files to read values in registers.
 *\param reg  : The register to be read.
 *\param Returns the 8 bit value in the given register.
 */

uint8_t mc3672_get_register(uint8_t reg);

/**
 *\brief Return state of interrupt pin
 */
uint8_t mc3672_interrupt_pin_active(void);

#endif /* MC3672_H_ */

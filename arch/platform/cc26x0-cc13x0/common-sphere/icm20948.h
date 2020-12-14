#ifndef icm_H_
#define icm_H_

#include "contiki.h"
#include <stdbool.h>

/*******************************************************************************
 *** H/W CONFIGURATION
 *******************************************************************************/
/**************************
 *** SENSOR I2C ADDR
 **************************/
#define ICM_I2C_ADDR    (0x69)

#define FIFO_DIVIDER    19

#define REG_BANK_0      0x00
#define REG_BANK_1      0x01

#define BANK_0                  (0 << 7)
#define BANK_1                  (0 << 7)
#define BANK_2                  (0 << 7)
#define BANK_3                  (0 << 7)

/* user bank register maps */

/* bank 0 register map */
#define REG_WHO_AM_I                    0x00
#define REG_LPF                         0x01
#define REG_USER_CTRL                   0x03

#define REG_LP_CONFIG                   0x05

#define BIT_I2C_MST_CYCLE               0x40
#define BIT_ACCEL_CYCLE                 0x20
#define BIT_GYRO_CYCLE                  0x10

#define REG_PWR_MGMT_1                  0x06
#define BIT_H_RESET                     0x80
#define BIT_SLEEP                       0x40
#define BIT_TMP_DIS                     0b00001000
#define BIT_LP_EN                       0x20
#define BIT_CLK_PLL                     0x01
void icm_power_down(void);
bool icm_init(void);
bool icm_reset(void);

#endif /* ICM_20948_H_ */

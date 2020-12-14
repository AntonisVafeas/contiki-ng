/*
 * Copyright (c) 2018, University of Bristol - http://www.bristol.ac.uk/
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

#ifndef ARCH_PLATFORM_SRF06_CC26XX_COMMON_OLED_H_
#define ARCH_PLATFORM_SRF06_CC26XX_COMMON_OLED_H_

#define OEL_DISPLAY_I2C_ADDR          0x3C

#define OEL_DISPLAY_COMMAND           0x00
#define OEL_DISPLAY_COMMAND_SINGLE    0x80
#define OEL_DISPLAY_DATA              0x40
#define OEL_DISPLAY_X                 0xF0
#define OEL_DISPLAY_Y                 0xB0

#define SSD1306_COLUMNADDR            0x21
#define SSD1306_PAGEADDR              0x22
#define SSD1306_SETSTARTPAGE          0xb0

#define USINT2DECASCII_MAX_DIGITS    5

void oled_init(const char *msg, int n);

void oled_shutdown(void);
/**
 *	\brief Prints out a string to the OLED display
 *	\param char String to print out
 *	\param int length of the string to print
 *
 */

void oled_print_message(const char *msg, int length);

/**
 *	\brief Prints out the given time in hours, minutes, and seconds.
 *	\param msg[] the formatted time (hh:mm:ss) given in the form of a string.
 *
 *	If this function is changed to to only hours and minutes, the set coordinates must be changed to centre the time.
 *
 */
void oled_print_time(const char *msg);

/**
 *	\brief 	Prints the battery charge percentage as well as a visual indicator
 *			of the battery charge.
 *	\param 	battery	: The percentage of the battery charge between 0 - 100
 */ 
void oled_print_battery(int battery);
/**
 *	\brief 	Changes the brightness of the screen
 *	\param 	value:  0-FF
 */
void oled_brightness(uint8_t value);
#endif /* ARCH_PLATFORM_SRF06_CC26XX_COMMON_OLED_H_ */

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

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "oled.h"
#include "ti-lib.h"
#include "board.h"
#include "sys/clock.h"
#include "board-i2c.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Standard ASCII 8x16 font */
static const uint8_t oled_font8x16[96 * 16] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0 */
  0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x30, 0x00, 0x00, 0x00, /* ! 1 */
  0x00, 0x10, 0x0C, 0x06, 0x10, 0x0C, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* " 2 */
  0x40, 0xC0, 0x78, 0x40, 0xC0, 0x78, 0x40, 0x00, 0x04, 0x3F, 0x04, 0x04, 0x3F, 0x04, 0x04, 0x00, /* # 3 */
  0x00, 0x70, 0x88, 0xFC, 0x08, 0x30, 0x00, 0x00, 0x00, 0x18, 0x20, 0xFF, 0x21, 0x1E, 0x00, 0x00, /* $ 4 */
  0xF0, 0x08, 0xF0, 0x00, 0xE0, 0x18, 0x00, 0x00, 0x00, 0x21, 0x1C, 0x03, 0x1E, 0x21, 0x1E, 0x00, /* % 5 */
  0x00, 0xF0, 0x08, 0x88, 0x70, 0x00, 0x00, 0x00, 0x1E, 0x21, 0x23, 0x24, 0x19, 0x27, 0x21, 0x10, /* & 6 */
  0x10, 0x16, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ' 7 */
  0x00, 0x00, 0x00, 0xE0, 0x18, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x07, 0x18, 0x20, 0x40, 0x00, /* ( 8 */
  0x00, 0x02, 0x04, 0x18, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0x18, 0x07, 0x00, 0x00, 0x00, /* ) 9 */
  0x40, 0x40, 0x80, 0xF0, 0x80, 0x40, 0x40, 0x00, 0x02, 0x02, 0x01, 0x0F, 0x01, 0x02, 0x02, 0x00, /* * 10 */
  0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x1F, 0x01, 0x01, 0x01, 0x00, /* + 11 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xB0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, /* , 12 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, /* - 13 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, /* . 14 */
  0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x18, 0x04, 0x00, 0x60, 0x18, 0x06, 0x01, 0x00, 0x00, 0x00, /* / 15 */
  0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x0F, 0x10, 0x20, 0x20, 0x10, 0x0F, 0x00, /* 0 16 */
  0x00, 0x10, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, /* 1 17 */
  0x00, 0x70, 0x08, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x30, 0x28, 0x24, 0x22, 0x21, 0x30, 0x00, /* 2 18 */
  0x00, 0x30, 0x08, 0x88, 0x88, 0x48, 0x30, 0x00, 0x00, 0x18, 0x20, 0x20, 0x20, 0x11, 0x0E, 0x00, /* 3 19 */
  0x00, 0x00, 0xC0, 0x20, 0x10, 0xF8, 0x00, 0x00, 0x00, 0x07, 0x04, 0x24, 0x24, 0x3F, 0x24, 0x00, /* 4 20 */
  0x00, 0xF8, 0x08, 0x88, 0x88, 0x08, 0x08, 0x00, 0x00, 0x19, 0x21, 0x20, 0x20, 0x11, 0x0E, 0x00, /* 5 21 */
  0x00, 0xE0, 0x10, 0x88, 0x88, 0x18, 0x00, 0x00, 0x00, 0x0F, 0x11, 0x20, 0x20, 0x11, 0x0E, 0x00, /* 6 22 */
  0x00, 0x38, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, /* 7 23 */
  0x00, 0x70, 0x88, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00, 0x1C, 0x22, 0x21, 0x21, 0x22, 0x1C, 0x00, /* 8 24 */
  0x00, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x00, 0x00, 0x31, 0x22, 0x22, 0x11, 0x0F, 0x00, /* 9 25 */
  0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, /* : 26 */
  0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00, /* ; 27 */
  0x00, 0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, /* < 28 */
  0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, /* = 29 */
  0x00, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, /* > 30 */
  0x00, 0x70, 0x48, 0x08, 0x08, 0x08, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x30, 0x36, 0x01, 0x00, 0x00, /* ? 31 */
  0xC0, 0x30, 0xC8, 0x28, 0xE8, 0x10, 0xE0, 0x00, 0x07, 0x18, 0x27, 0x24, 0x23, 0x14, 0x0B, 0x00, /* @ 32 */
  0x00, 0x00, 0xC0, 0x38, 0xE0, 0x00, 0x00, 0x00, 0x20, 0x3C, 0x23, 0x02, 0x02, 0x27, 0x38, 0x20, /* A 33 */
  0x08, 0xF8, 0x88, 0x88, 0x88, 0x70, 0x00, 0x00, 0x20, 0x3F, 0x20, 0x20, 0x20, 0x11, 0x0E, 0x00, /* B 34 */
  0xC0, 0x30, 0x08, 0x08, 0x08, 0x08, 0x38, 0x00, 0x07, 0x18, 0x20, 0x20, 0x20, 0x10, 0x08, 0x00, /* C 35 */
  0x08, 0xF8, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x20, 0x3F, 0x20, 0x20, 0x20, 0x10, 0x0F, 0x00, /* D 36 */
  0x08, 0xF8, 0x88, 0x88, 0xE8, 0x08, 0x10, 0x00, 0x20, 0x3F, 0x20, 0x20, 0x23, 0x20, 0x18, 0x00, /* E 37 */
  0x08, 0xF8, 0x88, 0x88, 0xE8, 0x08, 0x10, 0x00, 0x20, 0x3F, 0x20, 0x00, 0x03, 0x00, 0x00, 0x00, /* F 38 */
  0xC0, 0x30, 0x08, 0x08, 0x08, 0x38, 0x00, 0x00, 0x07, 0x18, 0x20, 0x20, 0x22, 0x1E, 0x02, 0x00, /* G 39 */
  0x08, 0xF8, 0x08, 0x00, 0x00, 0x08, 0xF8, 0x08, 0x20, 0x3F, 0x21, 0x01, 0x01, 0x21, 0x3F, 0x20, /* H 40 */
  0x00, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, /* I 41 */
  0x00, 0x00, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x00, 0xC0, 0x80, 0x80, 0x80, 0x7F, 0x00, 0x00, 0x00, /* J 42 */
  0x08, 0xF8, 0x88, 0xC0, 0x28, 0x18, 0x08, 0x00, 0x20, 0x3F, 0x20, 0x01, 0x26, 0x38, 0x20, 0x00, /* K 43 */
  0x08, 0xF8, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x3F, 0x20, 0x20, 0x20, 0x20, 0x30, 0x00, /* L 44 */
  0x08, 0xF8, 0xF8, 0x00, 0xF8, 0xF8, 0x08, 0x00, 0x20, 0x3F, 0x00, 0x3F, 0x00, 0x3F, 0x20, 0x00, /* M 45 */
  0x08, 0xF8, 0x30, 0xC0, 0x00, 0x08, 0xF8, 0x08, 0x20, 0x3F, 0x20, 0x00, 0x07, 0x18, 0x3F, 0x00, /* N 46 */
  0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x0F, 0x10, 0x20, 0x20, 0x20, 0x10, 0x0F, 0x00, /* O 47 */
  0x08, 0xF8, 0x08, 0x08, 0x08, 0x08, 0xF0, 0x00, 0x20, 0x3F, 0x21, 0x01, 0x01, 0x01, 0x00, 0x00, /* P 48 */
  0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0, 0x00, 0x0F, 0x18, 0x24, 0x24, 0x38, 0x50, 0x4F, 0x00, /* Q 49 */
  0x08, 0xF8, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00, 0x20, 0x3F, 0x20, 0x00, 0x03, 0x0C, 0x30, 0x20, /* R 50 */
  0x00, 0x70, 0x88, 0x08, 0x08, 0x08, 0x38, 0x00, 0x00, 0x38, 0x20, 0x21, 0x21, 0x22, 0x1C, 0x00, /* S 51 */
  0x18, 0x08, 0x08, 0xF8, 0x08, 0x08, 0x18, 0x00, 0x00, 0x00, 0x20, 0x3F, 0x20, 0x00, 0x00, 0x00, /* T 52 */
  0x08, 0xF8, 0x08, 0x00, 0x00, 0x08, 0xF8, 0x08, 0x00, 0x1F, 0x20, 0x20, 0x20, 0x20, 0x1F, 0x00, /* U 53 */
  0x08, 0x78, 0x88, 0x00, 0x00, 0xC8, 0x38, 0x08, 0x00, 0x00, 0x07, 0x38, 0x0E, 0x01, 0x00, 0x00, /* V 54 */
  0xF8, 0x08, 0x00, 0xF8, 0x00, 0x08, 0xF8, 0x00, 0x03, 0x3C, 0x07, 0x00, 0x07, 0x3C, 0x03, 0x00, /* W 55 */
  0x08, 0x18, 0x68, 0x80, 0x80, 0x68, 0x18, 0x08, 0x20, 0x30, 0x2C, 0x03, 0x03, 0x2C, 0x30, 0x20, /* X 56 */
  0x08, 0x38, 0xC8, 0x00, 0xC8, 0x38, 0x08, 0x00, 0x00, 0x00, 0x20, 0x3F, 0x20, 0x00, 0x00, 0x00, /* Y 57 */
  0x10, 0x08, 0x08, 0x08, 0xC8, 0x38, 0x08, 0x00, 0x20, 0x38, 0x26, 0x21, 0x20, 0x20, 0x18, 0x00, /* Z 58 */
  0x00, 0x00, 0x00, 0xFE, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x40, 0x40, 0x40, 0x00, /* [ 59 */
  0x00, 0x0C, 0x30, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x06, 0x38, 0xC0, 0x00, /* \ 60 */
  0x00, 0x02, 0x02, 0x02, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x7F, 0x00, 0x00, 0x00, /* ] 61 */
  0x00, 0x00, 0x04, 0x02, 0x02, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ^ 62 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, /* _ 63 */
  0x00, 0x02, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ` 64 */
  0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x19, 0x24, 0x22, 0x22, 0x22, 0x3F, 0x20, /* a 65 */
  0x08, 0xF8, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x11, 0x20, 0x20, 0x11, 0x0E, 0x00, /* b 66 */
  0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x0E, 0x11, 0x20, 0x20, 0x20, 0x11, 0x00, /* c 67 */
  0x00, 0x00, 0x00, 0x80, 0x80, 0x88, 0xF8, 0x00, 0x00, 0x0E, 0x11, 0x20, 0x20, 0x10, 0x3F, 0x20, /* d 68 */
  0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x22, 0x22, 0x22, 0x22, 0x13, 0x00, /* e 69 */
  0x00, 0x80, 0x80, 0xF0, 0x88, 0x88, 0x88, 0x18, 0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, /* f 70 */
  0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x6B, 0x94, 0x94, 0x94, 0x93, 0x60, 0x00, /* g 71 */
  0x08, 0xF8, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x20, 0x3F, 0x21, 0x00, 0x00, 0x20, 0x3F, 0x20, /* h 72 */
  0x00, 0x80, 0x98, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, /* i 73 */
  0x00, 0x00, 0x00, 0x80, 0x98, 0x98, 0x00, 0x00, 0x00, 0xC0, 0x80, 0x80, 0x80, 0x7F, 0x00, 0x00, /* j 74 */
  0x08, 0xF8, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x20, 0x3F, 0x24, 0x02, 0x2D, 0x30, 0x20, 0x00, /* k 75 */
  0x00, 0x08, 0x08, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x3F, 0x20, 0x20, 0x00, 0x00, /* l 76 */
  0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x20, 0x3F, 0x20, 0x00, 0x3F, 0x20, 0x00, 0x3F, /* m 77 */
  0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x20, 0x3F, 0x21, 0x00, 0x00, 0x20, 0x3F, 0x20, /* n 78 */
  0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x20, 0x20, 0x20, 0x20, 0x1F, 0x00, /* o 79 */
  0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xA1, 0x20, 0x20, 0x11, 0x0E, 0x00, /* p 80 */
  0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x0E, 0x11, 0x20, 0x20, 0xA0, 0xFF, 0x80, /* q 81 */
  0x80, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x20, 0x20, 0x3F, 0x21, 0x20, 0x00, 0x01, 0x00, /* r 82 */
  0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x33, 0x24, 0x24, 0x24, 0x24, 0x19, 0x00, /* s 83 */
  0x00, 0x80, 0x80, 0xE0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x20, 0x20, 0x00, 0x00, /* t 84 */
  0x80, 0x80, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x1F, 0x20, 0x20, 0x20, 0x10, 0x3F, 0x20, /* u 85 */
  0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x01, 0x0E, 0x30, 0x08, 0x06, 0x01, 0x00, /* v 86 */
  0x80, 0x80, 0x00, 0x80, 0x00, 0x80, 0x80, 0x80, 0x0F, 0x30, 0x0C, 0x03, 0x0C, 0x30, 0x0F, 0x00, /* w 87 */
  0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x20, 0x31, 0x2E, 0x0E, 0x31, 0x20, 0x00, /* x 88 */
  0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x81, 0x8E, 0x70, 0x18, 0x06, 0x01, 0x00, /* y 89 */
  0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x21, 0x30, 0x2C, 0x22, 0x21, 0x30, 0x00, /* z 90 */
  0x00, 0x00, 0x00, 0x00, 0x80, 0x7C, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x40, 0x40, /* { 91 */
  0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, /* | 92 */
  0x00, 0x02, 0x02, 0x7C, 0x80, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x3F, 0x00, 0x00, 0x00, 0x00, /* } 93 */
  0x00, 0x06, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ~ 94 */
};

/* Character bitmaps for Palatino Linotype 20pt */
static const uint8_t numbers16[11 * 16] =
{
  /* @0 '0' (8 pixels wide) */
  /*   #### */
  /*  ##  ## */
  /* ##    ## */
  /* ##    ## */
  /* ##    ## */
  /* ##    ## */
  /* ##    ## */
  /* ##    ## */
  /* ##    ## */
  /* ##    ## */
  /* ##    # */
  /*  ##  ## */
  /*   #### */
  0xF8, 0xFE, 0x03, 0x01, 0x01, 0x03, 0xFE, 0xFC,
  0x07, 0x0F, 0x18, 0x10, 0x10, 0x18, 0x0F, 0x03,

  /* @16 '1' (8 pixels wide) */
  /*    ## */
  /*  #### */
  /* ## ## */
  /*    ## */
  /*    ## */
  /*    ## */
  /*    ## */
  /*    ## */
  /*    ## */
  /*    ## */
  /*    ## */
  /*    ## */
  /* ######## */
  0x04, 0x06, 0x02, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x10, 0x10, 0x10, 0x1F, 0x1F, 0x10, 0x10, 0x10,

  /* @32 '2' (8 pixels wide) */
  /*  #### */
  /* #   ### */
  /*      ## */
  /*      ## */
  /*      ## */
  /*      ## */
  /*     ## */
  /*     ## */
  /*    ## */
  /*   ## */
  /*  ## */
  /* ## */
  /* ######## */
  0x02, 0x01, 0x01, 0x01, 0xC3, 0xFE, 0x3E, 0x00,
  0x18, 0x1C, 0x16, 0x13, 0x11, 0x10, 0x10, 0x10,

  /* @48 '3' (8 pixels wide) */
  /*  ##### */
  /* #   ### */
  /*      ## */
  /*      ## */
  /*      ## */
  /*     ## */
  /*  #### */
  /*      ## */
  /*       ## */
  /*       ## */
  /*       ## */
  /* #    ## */
  /*  ##### */
  0x02, 0x41, 0x41, 0x41, 0x63, 0xBF, 0x9E, 0x00,
  0x08, 0x10, 0x10, 0x10, 0x10, 0x18, 0x0F, 0x07,

  /* @64 '4' (9 pixels wide) */
  /*     ### */
  /*     ### */
  /*    #### */
  /*    # ## */
  /*   ## ## */
  /*   #  ## */
  /*  ##  ## */
  /*  #   ## */
  /* ##   ## */
  /* ######### */
  /*      ## */
  /*      ## */
  /*      ## */
  0x00, 0xC0, 0x70, 0x1C, 0x07, 0xFF, 0xFF, 0x00,
  0x03, 0x03, 0x02, 0x02, 0x02, 0x1F, 0x1F, 0x02,

  /* @82 '5' (8 pixels wide) */
  /*  ###### */
  /*  ## */
  /*  ## */
  /*  ## */
  /*  ## */
  /*  ##### */
  /*      ## */
  /*       ## */
  /*       ## */
  /*       ## */
  /*       ## */
  /* #    ## */
  /*  ##### */
  0x00, 0x3F, 0x3F, 0x21, 0x21, 0x61, 0xC1, 0x80,
  0x08, 0x10, 0x10, 0x10, 0x10, 0x18, 0x0F, 0x07,

  /* @98 '6' (8 pixels wide) */
  /*    #### */
  /*  ## */
  /*  ## */
  /* ## */
  /* ## */
  /* ## #### */
  /* ###  ### */
  /* ##    ## */
  /* ##    ## */
  /* ##    ## */
  /* ##    ## */
  /*  ##  ## */
  /*   #### */
  0xF8, 0xFE, 0x46, 0x21, 0x21, 0x61, 0xE1, 0xC0,
  0x07, 0x0F, 0x18, 0x10, 0x10, 0x18, 0x0F, 0x07,

  /* @114 '7' (8 pixels wide) */
  /* ######## */
  /*       ## */
  /*       # */
  /*      ## */
  /*      ## */
  /*     ## */
  /*     ## */
  /*     # */
  /*    ## */
  /*    ## */
  /*   ## */
  /*   ## */
  /*   # */
  0x01, 0x01, 0x01, 0x01, 0xE1, 0x79, 0x1F, 0x03,
  0x00, 0x00, 0x1C, 0x0F, 0x03, 0x00, 0x00, 0x00,

  /* @130 '8' (9 pixels wide) */
  /*   ##### */
  /*  ##   ### */
  /* ##     ## */
  /* ##     ## */
  /* ###   ## */
  /*  ### ## */
  /*    #### */
  /*  ### ### */
  /* ##    ### */
  /* ##     ## */
  /* ##     ## */
  /* ###   ## */
  /*   ##### */
  0x1C, 0xBE, 0xB3, 0xE1, 0x41, 0xE1, 0xF3, 0x9E,
  0x0F, 0x0F, 0x18, 0x10, 0x10, 0x10, 0x19, 0x0F,

  /* @148 '9' (8 pixels wide) */
  /*   #### */
  /*  ##  ## */
  /* ##    ## */
  /* ##    ## */
  /* ##    ## */
  /* ##    ## */
  /* ###   ## */
  /*  ####### */
  /*       ## */
  /*       # */
  /*      ## */
  /*     ## */
  /* ##### */
  0x7C, 0xFE, 0xC3, 0x81, 0x81, 0x83, 0xFE, 0xFC,
  0x10, 0x10, 0x10, 0x10, 0x18, 0x0C, 0x07, 0x01,

  /* @164 ':' (2 pixels wide) */
  /* */
  /* */
  /* */
  /* */
  /* ## */
  /* ## */
  /* */
  /* */
  /* */
  /* */
  /* */
  /* ## */
  /* ## */
  0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00
};

/* THESE ICONS ARE OBSOLETE NOW */

static const uint8_t custom_56[44 * 3] =
{
  /* Empty Battery - 22 Pixels wide, 16 Pixels in height
           # # # # # # # # # # # # # # # # # # # # # # #
           #                                             #
   # # # # #                                             #
   #                                                     #
   #                                                     #
   #                                                     #
   #                                                     #
   #                                                     #
   #                                                     #
   # # # # #                                             #
           #                                             #
           # # # # # # # # # # # # # # # # # # # # # # #
   */

  0xF0, 0x10, 0x10, 0x10, 0x1C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0xF8,

  0x0F, 0x08, 0x08, 0x08, 0x38, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x1F,

  /*Full Battery - 28 Pixels wide, 12 Pixels in height

            # # # # # # # # # # # # # # # # # # # # # # #
           # # # # # # # # # # # # # # # # # # # # # # # #
   # # # # # # # # # # # # # # # # # # # # # # # # # # # #
   # # # # # # # # # # # # # # # # # # # # # # # # # # # #
   # # # # # # # # # # # # # # # # # # # # # # # # # # # #
   # # # # # # # # # # # # # # # # # # # # # # # # # # # #
   # # # # # # # # # # # # # # # # # # # # # # # # # # # #
   # # # # # # # # # # # # # # # # # # # # # # # # # # # #
   # # # # # # # # # # # # # # # # # # # # # # # # # # # #
   # # # # # # # # # # # # # # # # # # # # # # # # # # # #
           # # # # # # # # # # # # # # # # # # # # # # # #
            # # # # # # # # # # # # # # # # # # # # # # #

   */
  0xF0, 0xF0, 0xF0, 0xF0, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
  0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8,

  0x0F, 0x0F, 0x0F, 0x0F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F,
  0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F,

  /*Half Battery - 28 Pixels wide, 12 Pixels in height

           # # # # # # # # # # # # # # # # # # # # # # #
           #                       # # # # # # # # # # # #
   # # # # #                       # # # # # # # # # # # #
   #                               # # # # # # # # # # # #
   #                               # # # # # # # # # # # #
   #                               # # # # # # # # # # # #
   #                               # # # # # # # # # # # #
   #                               # # # # # # # # # # # #
   #                               # # # # # # # # # # # #
   # # # # #                       # # # # # # # # # # # #
           #                       # # # # # # # # # # # #
            # # # # # # # # # # # # # # # # # # # # # # #

   */
  0xF0, 0x10, 0x10, 0x10, 0x1C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
  0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8,

  0x0F, 0x08, 0x08, 0x08, 0x38, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
  0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F
};

/* ------------------------------------------------------- */
static uint8_t display_enabled=0;
static uint8_t brightness;
void
oled_write_register(uint8_t reg, uint8_t val)
{
  uint8_t buf[2];

  buf[0] = reg;
  buf[1] = val;

  board_i2c_write(buf, sizeof(buf));
}

static void
init_sequence(void)
{
  /*set display off 0xAE */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xAE);
  /*Task_sleep(10); */
  /*set clock divide ratio, oscillator frequency 0xD5, 0x80 */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xD5);
  oled_write_register(OEL_DISPLAY_COMMAND, 0x80);
  /*set multiplex ratio, 0xA8, 0x1F */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xA8);
  oled_write_register(OEL_DISPLAY_COMMAND, 0x1F);
  /*set display offset 0xD3, 0x00 */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xD3);
  oled_write_register(OEL_DISPLAY_COMMAND, 0x00);
  /*set display srat line 0x40 */
  oled_write_register(OEL_DISPLAY_COMMAND, 0x40);
  /*set segment re-map 0xA1 */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xA1);
  /*set COM output scan direction 0xC8 */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xC8);
  /*set COM pins hardware 0xDa, 0x12 */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xDA);
  oled_write_register(OEL_DISPLAY_COMMAND, 0x12);
  /*set contrast control 0x81,0x60 */
  oled_write_register(OEL_DISPLAY_COMMAND, 0x81);
  oled_write_register(OEL_DISPLAY_COMMAND, 0x60);
  /*set pre-charge period 0xD9, 0x1F */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xD9);
  oled_write_register(OEL_DISPLAY_COMMAND, 0x1F);
  /*set VCOMH 0xDB, 0x40 */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xDB);
  oled_write_register(OEL_DISPLAY_COMMAND, 0x40);
  /*set entire display on off 0xA4 */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xA4);
  /*set normal inverse display 0xA6 */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xA6);
  /*clear screen */
  /*set charge pump 0x8D, 0x14 */
  /*Task_sleep(100); */
  oled_write_register(OEL_DISPLAY_COMMAND, 0x8D);
  oled_write_register(OEL_DISPLAY_COMMAND, 0x14);
  /*set display on 0xAF */
  oled_write_register(OEL_DISPLAY_COMMAND, 0xAF);
}

static void
display_on(void)
{
  oled_write_register(OEL_DISPLAY_COMMAND, 0x8D);
  oled_write_register(OEL_DISPLAY_COMMAND, 0x14);
  oled_write_register(OEL_DISPLAY_COMMAND, 0xAF);
  //change the brightness
  oled_write_register(OEL_DISPLAY_COMMAND, 0x81);
  oled_write_register(OEL_DISPLAY_COMMAND, brightness);

}
void
oled_brightness(uint8_t value){
	brightness=value;
	  }
/*set the postion in the LED matrix - this needs to be called before any data to display */
void
oled_setXY(uint8_t x, uint8_t y)
{
  /*set xy, pos */
  uint8_t val = y;
  val |= OEL_DISPLAY_Y;
  oled_write_register(OEL_DISPLAY_COMMAND, val);
  val = x;
  val &= OEL_DISPLAY_X;
  val = val >> 4;
  val |= 0x10;
  oled_write_register(OEL_DISPLAY_COMMAND, val);
  oled_write_register(OEL_DISPLAY_COMMAND, (x & 0x0F));
}

/*clears the screen */
void
oled_clear(void)
{
  int j, i;
  for(i = 0; i < 8; i++) {
    oled_write_register(OEL_DISPLAY_COMMAND, 0xB0 + i);
    oled_write_register(OEL_DISPLAY_COMMAND, 0x00);
    oled_write_register(OEL_DISPLAY_COMMAND, 0x11);
    for(j = 0; j < 96; j++) {
      oled_write_register(OEL_DISPLAY_DATA, 0x00);
    }
  }
}

/*draw a line */
void
oled_draw(uint8_t x, uint8_t y)
{
  int i;
  oled_clear();
  oled_setXY(x, y);
  for(i = 0; i < 128; i++) {
    oled_write_register(OEL_DISPLAY_DATA, 0x99);
  }
}


void
oled_custom_56(int n)
{
  /* Print the top row */
  oled_setXY(72, 1);
  for(int i = 44 * n; i < (44 * n + 22); i++) {
    oled_write_register(OEL_DISPLAY_DATA, custom_56[i]);
  }
  /* Print the bottom row */
  oled_setXY(72, 2);
  for(int i = (44 * n) + 22; i < (44 * n + 44); i++) {
    oled_write_register(OEL_DISPLAY_DATA, custom_56[i]);
  }
}

/*This function prints out the battery icon dependent on a given percentage*/
static void
oled_battery(int bat)
{
  int i;
  /*The icon is 22 pixels wide and so each 5% is one pixel wide*/
  bat = (bat * 20) / 100;

  /* Print the top row */
  oled_setXY(72, 1);
  oled_write_register(OEL_DISPLAY_DATA, 0xF0);

  for(i = 0; i < 20; i++) {
    /*'Empty' battery*/
    if((20 - i) > bat) {
      if(i > 2) {
        oled_write_register(OEL_DISPLAY_DATA, 0x04);
      } else if(i < 2) {
        oled_write_register(OEL_DISPLAY_DATA, 0x10);
      } else {
        oled_write_register(OEL_DISPLAY_DATA, 0x1C);
      }
    }
    /*'Full' battery*/
    else {
      if(i >= 2) {
        oled_write_register(OEL_DISPLAY_DATA, 0xFC);
      } else if(i < 2) {
        oled_write_register(OEL_DISPLAY_DATA, 0xF0);
      }
    }
  }
  oled_write_register(OEL_DISPLAY_DATA, 0xF8);
  /* Print the bottom row */
  oled_setXY(72, 2);
  oled_write_register(OEL_DISPLAY_DATA, 0x0F);

  for(i = 0; i < 20; i++) {
    if((20 - i) > bat) {
      if(i > 2) {
        oled_write_register(OEL_DISPLAY_DATA, 0x20);
      } else if(i < 2) {
        oled_write_register(OEL_DISPLAY_DATA, 0x08);
      } else if(i == 2) {
        oled_write_register(OEL_DISPLAY_DATA, 0x38);
      }
    } else {
      if(i >= 2) {
        oled_write_register(OEL_DISPLAY_DATA, 0x3F);
      } else if(i < 2) {
        oled_write_register(OEL_DISPLAY_DATA, 0x0F);
      }
    }
  }
  oled_write_register(OEL_DISPLAY_DATA, 0x1F);
}

/*
static void
oled_test(void)
{
  int j;
  for(j = 0; j < 1; j++) {
    oled_draw(0, j);
  }
}
*/

/* takes an integer, position and prints large font size 16 */
static void
oled_print_number_large(uint8_t value, uint8_t x, uint8_t y)
{
  /*NB larger font is spread over 2 rows in the dot matrix */
  uint8_t i;
  /*print the top row */
  oled_setXY(x, y);
  for(i = 0; i < 8; i++) {
    oled_write_register(OEL_DISPLAY_DATA, numbers16[value * 16 + i]);
  }
  /*print the bottom row */
  oled_setXY(x, y + 1);
  for(i = 0; i < 8; i++) {
    oled_write_register(OEL_DISPLAY_DATA, numbers16[value * 16 + i + 8]);
  }
}

/* takes an integer, position and prints large font size 16 */
void
oled_font_16(uint8_t value, uint8_t x, uint8_t y)
{
  /*NB larger font is spread over 2 rows in the dot matrix */
  uint8_t i;
  /*print the top row */
  for(i = 0; i < 8; i++) {
    oled_write_register(OEL_DISPLAY_DATA, oled_font8x16[value * 16 + i]);
  }
  /*print the bottom row */
  oled_setXY(x, y + 1);
  for(i = 0; i < 8; i++) {
    oled_write_register(OEL_DISPLAY_DATA, oled_font8x16[value * 16 + i + 8]);
  }
}

/* Display two rows of large fonts */
/*
static void
test_large_font(void)
{
  static uint8_t i, y = 0;
  static uint8_t x = 0;
  static uint8_t n = 0;

  oled_clear();
  for(y = 0; y < 4; y += 2) {
    for(i = 0; i < 8; i++) {
      x = x + 9;
      oled_setXY(x, y);
      oled_print_number_large(n++, x, y);
    }
    n = 0;
    x = 0;
  }
}
*/

/*
static void
contiki(void)
{
  oled_clear();

  uint8_t i, x = 9;
  uint8_t contiki_ng[11] = { 35, 79, 78, 84, 73, 75, 73, 13, 46, 39 };

  for(i = 0; i < 11; i++) {
    x = x + 9;
    oled_setXY(x, 0);
    oled_font_16(contiki_ng[i], x, 0);
  }
}
*/

void
oled_print_message(const char *msg, int n)
{
  uint8_t i;
  uint8_t x = 9;

  board_i2c_select(BOARD_I2C_INTERFACE_0, OEL_DISPLAY_I2C_ADDR);
  /*board_i2c_wakeup(); */
  oled_clear();

  /*uint8_t contiki_ng[11] = {35, 79, 78, 84, 73, 75, 73, 13, 46, 39}; */

  if(n > 11) {
    n = 11;
  }

  for(i = 0; i < n && msg[i] >= 32; i++) {
    x = x + 9;
    oled_setXY(x, 0);
    oled_font_16(msg[i] - 32, x, 0);
  }
  board_i2c_deselect();
}
void
oled_shutdown(void)
{
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_OLED_POWER);
  ti_lib_gpio_clear_dio(BOARD_IOID_OLED_POWER);
  display_enabled=0;
}

void
oled_init(const char *msg, int n)
{
	if(display_enabled){
		return;
	}
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_OLED_POWER);
  ti_lib_gpio_set_dio(BOARD_IOID_OLED_POWER);

  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_OLED_RESET);
  ti_lib_gpio_clear_dio(BOARD_IOID_OLED_RESET);
  clock_delay(100000);
  ti_lib_gpio_set_dio(BOARD_IOID_OLED_RESET);

  board_i2c_select(BOARD_I2C_INTERFACE_0, OEL_DISPLAY_I2C_ADDR);
  board_i2c_wakeup();

  init_sequence();
  display_on();

  oled_print_message(msg, n);

  board_i2c_deselect();
  display_enabled=1;
}

/* Prints the battery percentage with ASCII characters over two lines */
static void
print_bat(const char *msg)
{
  oled_clear();
  uint8_t i, x = 24;
  for(i = 0; i < 4; i++) {
    x = x + 9;
    oled_setXY(x, 1);
    oled_font_16(msg[i] - 32, x, 1);
  }
}

void
oled_print_battery(int battery)
{
  char msg[4];

  board_i2c_select(BOARD_I2C_INTERFACE_0, OEL_DISPLAY_I2C_ADDR);
  /* board_i2c_wakeup(); */
  display_on();

  if(battery > 100) {
    battery = 100;
  } else if(battery <= 0) {
    battery = 0;
  }

  sprintf(msg, "%d", battery);

  /* Formatting for when the percentage is less than 10% */
  if(battery < 10) {
    msg[2] = msg[0];
    msg[1] = ' ';
    msg[0] = ' ';
  } else if(battery < 100) {
    msg[2] = msg[1];
    msg[1] = msg[0];
    msg[0] = ' ';
  }

  msg[3] = '%';
  /* Print 00% */
  print_bat(msg);
  /* Print battery icon */
  oled_battery(battery);
  board_i2c_deselect();
}

void
oled_print_time(const char *msg)
{
  board_i2c_select(BOARD_I2C_INTERFACE_0, OEL_DISPLAY_I2C_ADDR);
  /*board_i2c_wakeup(); */
  oled_clear();
  for(int i = 0; i < 8; i++) {
    oled_print_number_large(msg[i] - 48, (i * 8) + 36, 1);
  }
  board_i2c_deselect();
}
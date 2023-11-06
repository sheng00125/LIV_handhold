/* FastCRC library code is placed under the MIT license
 * Copyright (c) 2014,2015 Frank Bosing
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


// Teensy 3.0, Teensy 3.1:
// See K20P64M72SF1RM.pdf (Kinetis), Pages 638 - 641 for documentation of CRC Device
// See KINETIS_4N30D.pdf for Errata (Errata ID 2776)
//
// So, ALL HW-calculations are done as 32 bit.
//
//
//
// Thanks to:
// - Catalogue of parametrised CRC algorithms, CRC RevEng
// http://reveng.sourceforge.net/crc-catalogue/
//
// - Danjel McGougan (CRC-Table-Generator)
//

//
// modify from FastCRC library @ 2018/11/20
//

#ifndef FASTCRC_FASTCRC_H_
#define FASTCRC_FASTCRC_H_

#include <stdint.h>


// ================= 16-BIT CRC ===================

class FastCRC16 {
public:
  FastCRC16(uint16_t seed);

  // change function name from mcrf4xx_upd to mcrf4xx
  uint16_t mcrf4xx_calc(const uint8_t *data,const uint16_t datalen); // Equivalent to _crc_ccitt_update() in crc16.h from avr_libc

private:
  uint16_t seed_;

};

// ================= 32-BIT CRC ===================

class FastCRC32 {
public:
  FastCRC32(uint32_t seed);

  // change function name from crc32_upd to crc32
  uint32_t crc32_calc(const uint8_t *data, uint16_t len);			// Call for subsequent calculations with previous seed

private:
  uint32_t seed_;
};

#endif // FASTCRC_FASTCRC_H_


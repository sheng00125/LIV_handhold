/* FastCRC library code is placed under the MIT license
 * Copyright (c) 2014,2015,2016 Frank Bosing
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

#include "../include/third_party/FastCRC/FastCRC.h"
#include "FastCRC_tables.hpp"


// ================= 16-BIT CRC ===================
/** Constructor
 */
FastCRC16::FastCRC16(uint16_t seed) {
  seed_ = seed;
}

#define crc_n4(crc, data, table) crc ^= data; \
	crc = (table[(crc & 0xff) + 0x300]) ^		\
	      (table[((crc >> 8) & 0xff) + 0x200]) ^	\
	      (table[((data >> 16) & 0xff) + 0x100]) ^	\
	      (table[data >> 24]);

/** MCRF4XX
 * equivalent to _crc_ccitt_update() in crc16.h from avr_libc
 * @param data Pointer to Data
 * @param datalen Length of Data
 * @return CRC value
 */

uint16_t FastCRC16::mcrf4xx_calc(const uint8_t *data, uint16_t len) {

	uint16_t crc = seed_;

	while (((uintptr_t)data & 3) && len) {
		crc = (crc >> 8) ^ crc_table_mcrf4xx[(crc & 0xff) ^ *data++];
		len--;
	}

	while (len >= 16) {
		len -= 16;
		crc_n4(crc, ((uint32_t *)data)[0], crc_table_mcrf4xx);
		crc_n4(crc, ((uint32_t *)data)[1], crc_table_mcrf4xx);
		crc_n4(crc, ((uint32_t *)data)[2], crc_table_mcrf4xx);
		crc_n4(crc, ((uint32_t *)data)[3], crc_table_mcrf4xx);
		data += 16;
	}

	while (len--) {
		crc = (crc >> 8) ^ crc_table_mcrf4xx[(crc & 0xff) ^ *data++];
	}

	//seed = crc;
	return crc;
}


// ================= 32-BIT CRC ===================
/** Constructor
 */
FastCRC32::FastCRC32(uint32_t seed) {
  seed_ = seed;
}

#define crc_n4d(crc, data, table) crc ^= data; \
	crc = (table[(crc & 0xff) + 0x300]) ^	\
	      (table[((crc >> 8) & 0xff) + 0x200]) ^	\
	      (table[((crc >> 16) & 0xff) + 0x100]) ^	\
	      (table[(crc >> 24) & 0xff]);

#define crcsm_n4d(crc, data, table) crc ^= data; \
	crc = (crc >> 8) ^ (table[crc & 0xff]); \
	crc = (crc >> 8) ^ (table[crc & 0xff]); \
	crc = (crc >> 8) ^ (table[crc & 0xff]); \
	crc = (crc >> 8) ^ (table[crc & 0xff]);

/** CRC32
 * Alias CRC-32/ADCCP, PKZIP, Ethernet, 802.3
 * @param data Pointer to Data
 * @param datalen Length of Data
 * @return CRC value
 */
#if CRC_BIGTABLES
#define CRC_TABLE_CRC32 crc_table_crc32_big
#else
#define CRC_TABLE_CRC32 crc_table_crc32
#endif

uint32_t FastCRC32::crc32_calc(const uint8_t *data, uint16_t len) {

	uint32_t crc = seed_^0xffffffff;

	while (((uintptr_t)data & 3) && len) {
		crc = (crc >> 8) ^ CRC_TABLE_CRC32[(crc & 0xff) ^ *data++];
		len--;
	}

	while (len >= 16) {
		len -= 16;
		#if CRC_BIGTABLES
		crc_n4d(crc, ((uint32_t *)data)[0], CRC_TABLE_CRC32);
		crc_n4d(crc, ((uint32_t *)data)[1], CRC_TABLE_CRC32);
		crc_n4d(crc, ((uint32_t *)data)[2], CRC_TABLE_CRC32);
		crc_n4d(crc, ((uint32_t *)data)[3], CRC_TABLE_CRC32);
		#else
		crcsm_n4d(crc, ((uint32_t *)data)[0], CRC_TABLE_CRC32);
		crcsm_n4d(crc, ((uint32_t *)data)[1], CRC_TABLE_CRC32);
		crcsm_n4d(crc, ((uint32_t *)data)[2], CRC_TABLE_CRC32);
		crcsm_n4d(crc, ((uint32_t *)data)[3], CRC_TABLE_CRC32);
		#endif
		data += 16;
	}

	while (len--) {
		crc = (crc >> 8) ^ CRC_TABLE_CRC32[(crc & 0xff) ^ *data++];
	}

	//seed = crc;
	crc ^= 0xffffffff;

	return crc;
}



// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2004, 2005 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "helper/replacements.h"
#include "log.h"
#include "binarybuffer.h"

static const unsigned char bit_reverse_table256[] = {
	0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
	0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
	0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
	0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
	0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
	0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
	0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
	0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
	0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
	0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
	0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
	0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
	0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
	0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
	0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
	0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

static const char hex_digits[] = {
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
	'a', 'b', 'c', 'd', 'e', 'f'
};

void *buf_cpy(const void *from, void *_to, unsigned int size)
{
	if (!from || !_to)
		return NULL;

	/* copy entire buffer */
	memcpy(_to, from, DIV_ROUND_UP(size, 8));

	/* mask out bits that don't belong to the buffer */
	unsigned int trailing_bits = size % 8;
	if (trailing_bits) {
		uint8_t *to = _to;
		to[size / 8] &= (1 << trailing_bits) - 1;
	}
	return _to;
}

static bool buf_eq_masked(uint8_t a, uint8_t b, uint8_t m)
{
	return (a & m) == (b & m);
}
static bool buf_eq_trailing(uint8_t a, uint8_t b, uint8_t m, unsigned int trailing)
{
	uint8_t mask = (1 << trailing) - 1;
	return buf_eq_masked(a, b, mask & m);
}

bool buf_eq(const void *_buf1, const void *_buf2, unsigned int size)
{
	if (!_buf1 || !_buf2)
		return _buf1 == _buf2;

	unsigned int last = size / 8;
	if (memcmp(_buf1, _buf2, last) != 0)
		return false;

	unsigned int trailing = size % 8;
	if (!trailing)
		return true;

	const uint8_t *buf1 = _buf1, *buf2 = _buf2;
	return buf_eq_trailing(buf1[last], buf2[last], 0xff, trailing);
}

bool buf_eq_mask(const void *_buf1, const void *_buf2,
	const void *_mask, unsigned int size)
{
	if (!_buf1 || !_buf2)
		return _buf1 == _buf2 && _buf1 == _mask;

	const uint8_t *buf1 = _buf1, *buf2 = _buf2, *mask = _mask;
	unsigned int last = size / 8;
	for (unsigned int i = 0; i < last; i++) {
		if (!buf_eq_masked(buf1[i], buf2[i], mask[i]))
			return false;
	}
	unsigned int trailing = size % 8;
	if (!trailing)
		return true;
	return buf_eq_trailing(buf1[last], buf2[last], mask[last], trailing);
}

void *buf_set_ones(void *_buf, unsigned int size)
{
	uint8_t *buf = _buf;
	if (!buf)
		return NULL;

	memset(buf, 0xff, size / 8);

	unsigned int trailing_bits = size % 8;
	if (trailing_bits)
		buf[size / 8] = (1 << trailing_bits) - 1;

	return buf;
}

void *buf_set_buf(const void *_src, unsigned int src_start,
	void *_dst, unsigned int dst_start, unsigned int len)
{
	const uint8_t *src = _src;
	uint8_t *dst = _dst;
	unsigned int i, sb, db, sq, dq, lb, lq;

	sb = src_start / 8;
	db = dst_start / 8;
	sq = src_start % 8;
	dq = dst_start % 8;
	lb = len / 8;
	lq = len % 8;

	src += sb;
	dst += db;

	/* check if both buffers are on byte boundary and
	 * len is a multiple of 8bit so we can simple copy
	 * the buffer */
	if ((sq == 0) && (dq == 0) &&  (lq == 0)) {
		for (i = 0; i < lb; i++)
			*dst++ = *src++;
		return _dst;
	}

	/* fallback to slow bit copy */
	for (i = 0; i < len; i++) {
		if (((*src >> (sq&7)) & 1) == 1)
			*dst |= 1 << (dq&7);
		else
			*dst &= ~(1 << (dq&7));
		if (sq++ == 7) {
			sq = 0;
			src++;
		}
		if (dq++ == 7) {
			dq = 0;
			dst++;
		}
	}

	return _dst;
}

uint32_t flip_u32(uint32_t value, unsigned int num)
{
	uint32_t c = (bit_reverse_table256[value & 0xff] << 24) |
		(bit_reverse_table256[(value >> 8) & 0xff] << 16) |
		(bit_reverse_table256[(value >> 16) & 0xff] << 8) |
		(bit_reverse_table256[(value >> 24) & 0xff]);

	if (num < 32)
		c = c >> (32 - num);

	return c;
}

char *buf_to_hex_str(const void *_buf, unsigned int buf_len)
{
	unsigned int len_bytes = DIV_ROUND_UP(buf_len, 8);
	char *str = calloc(len_bytes * 2 + 1, 1);

	const uint8_t *buf = _buf;
	for (unsigned int i = 0; i < len_bytes; i++) {
		uint8_t tmp = buf[len_bytes - i - 1];
		if ((i == 0) && (buf_len % 8))
			tmp &= (0xff >> (8 - (buf_len % 8)));
		str[2 * i] = hex_digits[tmp >> 4];
		str[2 * i + 1] = hex_digits[tmp & 0xf];
	}

	return str;
}

/*
 * TCL standard prefix is '0b', '0o', '0d' or '0x' respectively for binary,
 * octal, decimal or hexadecimal.
 * The prefix '0' is interpreted by TCL <= 8.6 as octal, but is ignored and
 * interpreted as part of a decimal number by JimTCL and by TCL >= 9.
 */
int str_to_buf(const char *str, void *_buf, unsigned int buf_bitsize)
{
	assert(str);
	assert(_buf);
	assert(buf_bitsize > 0);

	uint8_t *buf = _buf;
	unsigned int radix = 10; /* default when no prefix */

	if (str[0] == '0') {
		switch (str[1]) {
		case 'b':
		case 'B':
			radix = 2;
			str += 2;
			break;
		case 'o':
		case 'O':
			radix = 8;
			str += 2;
			break;
		case 'd':
		case 'D':
			radix = 10;
			str += 2;
			break;
		case 'x':
		case 'X':
			radix = 16;
			str += 2;
			break;
		default:
			break;
		}
	}

	const size_t str_len = strlen(str);
	if (str_len == 0)
		return ERROR_INVALID_NUMBER;

	const size_t buf_len = DIV_ROUND_UP(buf_bitsize, 8);
	memset(buf, 0, buf_len);

	/* Go through the zero-terminated buffer
	 * of input digits (ASCII) */
	for (; *str; str++) {
		unsigned int tmp;
		const char c = *str;

		if ((c >= '0') && (c <= '9')) {
			tmp = c - '0';
		} else if ((c >= 'a') && (c <= 'f')) {
			tmp = c - 'a' + 10;
		} else if ((c >= 'A') && (c <= 'F')) {
			tmp = c - 'A' + 10;
		} else {
			/* Characters other than [0-9,a-f,A-F] are invalid */
			return ERROR_INVALID_NUMBER;
		}

		/* Error on invalid digit for current radix */
		if (tmp >= radix)
			return ERROR_INVALID_NUMBER;

		/* Add the current digit (tmp) to the intermediate result in buf */
		for (unsigned int j = 0; j < buf_len; j++) {
			tmp += buf[j] * radix;
			buf[j] = tmp & 0xFFu;
			tmp >>= 8;
		}

		/* buf should be large enough to contain the whole result. */
		if (tmp != 0)
			return ERROR_NUMBER_EXCEEDS_BUFFER;
	}

	/* Check the partial most significant byte */
	if (buf_bitsize % 8) {
		const uint8_t mask = 0xFFu << (buf_bitsize % 8);
		if ((buf[buf_len - 1] & mask) != 0x0)
			return ERROR_NUMBER_EXCEEDS_BUFFER;
	}

	return ERROR_OK;
}

void bit_copy_queue_init(struct bit_copy_queue *q)
{
	INIT_LIST_HEAD(&q->list);
}

int bit_copy_queued(struct bit_copy_queue *q, uint8_t *dst, unsigned int dst_offset, const uint8_t *src,
	unsigned int src_offset, unsigned int bit_count)
{
	struct bit_copy_queue_entry *qe = malloc(sizeof(*qe));
	if (!qe)
		return ERROR_FAIL;

	qe->dst = dst;
	qe->dst_offset = dst_offset;
	qe->src = src;
	qe->src_offset = src_offset;
	qe->bit_count = bit_count;
	list_add_tail(&qe->list, &q->list);

	return ERROR_OK;
}

void bit_copy_execute(struct bit_copy_queue *q)
{
	struct bit_copy_queue_entry *qe;
	struct bit_copy_queue_entry *tmp;
	list_for_each_entry_safe(qe, tmp, &q->list, list) {
		bit_copy(qe->dst, qe->dst_offset, qe->src, qe->src_offset, qe->bit_count);
		list_del(&qe->list);
		free(qe);
	}
}

void bit_copy_discard(struct bit_copy_queue *q)
{
	struct bit_copy_queue_entry *qe;
	struct bit_copy_queue_entry *tmp;
	list_for_each_entry_safe(qe, tmp, &q->list, list) {
		list_del(&qe->list);
		free(qe);
	}
}

/**
 * Convert a string of hexadecimal pairs into its binary
 * representation.
 *
 * @param[out] bin Buffer to store binary representation. The buffer size must
 *                 be at least @p count.
 * @param[in] hex String with hexadecimal pairs to convert into its binary
 *                representation.
 * @param[in] count Number of hexadecimal pairs to convert.
 *
 * @return The number of converted hexadecimal pairs.
 */
size_t unhexify(uint8_t *bin, const char *hex, size_t count)
{
	size_t i;
	char tmp;

	if (!bin || !hex)
		return 0;

	memset(bin, 0, count);

	for (i = 0; i < 2 * count; i++) {
		if (hex[i] >= 'a' && hex[i] <= 'f')
			tmp = hex[i] - 'a' + 10;
		else if (hex[i] >= 'A' && hex[i] <= 'F')
			tmp = hex[i] - 'A' + 10;
		else if (hex[i] >= '0' && hex[i] <= '9')
			tmp = hex[i] - '0';
		else
			return i / 2;

		bin[i / 2] |= tmp << (4 * ((i + 1) % 2));
	}

	return i / 2;
}

/**
 * Convert binary data into a string of hexadecimal pairs.
 *
 * @param[out] hex Buffer to store string of hexadecimal pairs. The buffer size
 *                 must be at least @p length.
 * @param[in] bin Buffer with binary data to convert into hexadecimal pairs.
 * @param[in] count Number of bytes to convert.
 * @param[in] length Maximum number of characters, including null-terminator,
 *                   to store into @p hex.
 *
 * @returns The length of the converted string excluding null-terminator.
 */
size_t hexify(char *hex, const uint8_t *bin, size_t count, size_t length)
{
	size_t i;
	uint8_t tmp;

	if (!length)
		return 0;

	for (i = 0; i < length - 1 && i < 2 * count; i++) {
		tmp = (bin[i / 2] >> (4 * ((i + 1) % 2))) & 0x0f;
		hex[i] = hex_digits[tmp];
	}

	hex[i] = 0;

	return i;
}

void buffer_shr(void *_buf, unsigned int buf_len, unsigned int count)
{
	unsigned int i;
	unsigned char *buf = _buf;
	unsigned int bytes_to_remove;
	unsigned int shift;

	bytes_to_remove = count / 8;
	shift = count - (bytes_to_remove * 8);

	for (i = 0; i < (buf_len - 1); i++)
		buf[i] = (buf[i] >> shift) | ((buf[i+1] << (8 - shift)) & 0xff);

	buf[(buf_len - 1)] = buf[(buf_len - 1)] >> shift;

	if (bytes_to_remove) {
		memmove(buf, &buf[bytes_to_remove], buf_len - bytes_to_remove);
		memset(&buf[buf_len - bytes_to_remove], 0, bytes_to_remove);
	}
}

/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Base64 encoding/decoding (RFC1341)
 * Copyright (c) 2005, Jouni Malinen <j@w1.fi>
 *
 * Original file from FreeBSD code
 * https://cgit.freebsd.org/src/tree/contrib/wpa/src/utils/base64.h?id=f05cddf940db
 */

#ifndef BASE64_H
#define BASE64_H

unsigned char *base64_encode(const unsigned char *src, size_t len,
			      size_t *out_len);
unsigned char *base64_decode(const unsigned char *src, size_t len,
			      size_t *out_len);

#endif /* BASE64_H */

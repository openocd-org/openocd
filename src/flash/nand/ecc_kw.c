/*
 * Reed-Solomon ECC handling for the Marvell Kirkwood SOC
 * Copyright (C) 2009 Marvell Semiconductor, Inc.
 *
 * Authors: Lennert Buytenhek <buytenh@wantstofly.org>
 *          Nicolas Pitre <nico@fluxnic.net>
 *
 * This file is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 or (at your option) any
 * later version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "core.h"

/*****************************************************************************
 * Arithmetic in GF(2^10) ("F") modulo x^10 + x^3 + 1.
 *
 * For multiplication, a discrete log/exponent table is used, with
 * primitive element x (F is a primitive field, so x is primitive).
 */
#define MODPOLY 0x409		/* x^10 + x^3 + 1 in binary */

/*
 * Maps an integer a [0..1022] to a polynomial b = gf_exp[a] in
 * GF(2^10) mod x^10 + x^3 + 1 such that b = x ^ a.  There's two
 * identical copies of this array back-to-back so that we can save
 * the mod 1023 operation when doing a GF multiplication.
 */
static uint16_t gf_exp[1023 + 1023];

/*
 * Maps a polynomial b in GF(2^10) mod x^10 + x^3 + 1 to an index
 * a = gf_log[b] in [0..1022] such that b = x ^ a.
 */
static uint16_t gf_log[1024];

static void gf_build_log_exp_table(void)
{
	int i;
	int p_i;

	/*
	 * p_i = x ^ i
	 *
	 * Initialise to 1 for i = 0.
	 */
	p_i = 1;

	for (i = 0; i < 1023; i++) {
		gf_exp[i] = p_i;
		gf_exp[i + 1023] = p_i;
		gf_log[p_i] = i;

		/*
		 * p_i = p_i * x
		 */
		p_i <<= 1;
		if (p_i & (1 << 10))
			p_i ^= MODPOLY;
	}
}


/*****************************************************************************
 * Reed-Solomon code
 *
 * This implements a (1023,1015) Reed-Solomon ECC code over GF(2^10)
 * mod x^10 + x^3 + 1, shortened to (520,512).  The ECC data consists
 * of 8 10-bit symbols, or 10 8-bit bytes.
 *
 * Given 512 bytes of data, computes 10 bytes of ECC.
 *
 * This is done by converting the 512 bytes to 512 10-bit symbols
 * (elements of F), interpreting those symbols as a polynomial in F[X]
 * by taking symbol 0 as the coefficient of X^8 and symbol 511 as the
 * coefficient of X^519, and calculating the residue of that polynomial
 * divided by the generator polynomial, which gives us the 8 ECC symbols
 * as the remainder.  Finally, we convert the 8 10-bit ECC symbols to 10
 * 8-bit bytes.
 *
 * The generator polynomial is hardcoded, as that is faster, but it
 * can be computed by taking the primitive element a = x (in F), and
 * constructing a polynomial in F[X] with roots a, a^2, a^3, ..., a^8
 * by multiplying the minimal polynomials for those roots (which are
 * just 'x - a^i' for each i).
 *
 * Note: due to unfortunate circumstances, the bootrom in the Kirkwood SOC
 * expects the ECC to be computed backward, i.e. from the last byte down
 * to the first one.
 */
int nand_calculate_ecc_kw(struct nand_device *nand, const uint8_t *data, uint8_t *ecc)
{
	unsigned int r7, r6, r5, r4, r3, r2, r1, r0;
	int i;
	static int tables_initialized;

	if (!tables_initialized) {
		gf_build_log_exp_table();
		tables_initialized = 1;
	}

	/*
	 * Load bytes 504..511 of the data into r.
	 */
	r0 = data[504];
	r1 = data[505];
	r2 = data[506];
	r3 = data[507];
	r4 = data[508];
	r5 = data[509];
	r6 = data[510];
	r7 = data[511];

	/*
	 * Shift bytes 503..0 (in that order) into r0, followed
	 * by eight zero bytes, while reducing the polynomial by the
	 * generator polynomial in every step.
	 */
	for (i = 503; i >= -8; i--) {
		unsigned int d;

		d = 0;
		if (i >= 0)
			d = data[i];

		if (r7) {
			uint16_t *t = gf_exp + gf_log[r7];

			r7 = r6 ^ t[0x21c];
			r6 = r5 ^ t[0x181];
			r5 = r4 ^ t[0x18e];
			r4 = r3 ^ t[0x25f];
			r3 = r2 ^ t[0x197];
			r2 = r1 ^ t[0x193];
			r1 = r0 ^ t[0x237];
			r0 = d  ^ t[0x024];
		} else {
			r7 = r6;
			r6 = r5;
			r5 = r4;
			r4 = r3;
			r3 = r2;
			r2 = r1;
			r1 = r0;
			r0 = d;
		}
	}

	ecc[0] = r0;
	ecc[1] = (r0 >> 8) | (r1 << 2);
	ecc[2] = (r1 >> 6) | (r2 << 4);
	ecc[3] = (r2 >> 4) | (r3 << 6);
	ecc[4] = (r3 >> 2);
	ecc[5] = r4;
	ecc[6] = (r4 >> 8) | (r5 << 2);
	ecc[7] = (r5 >> 6) | (r6 << 4);
	ecc[8] = (r6 >> 4) | (r7 << 6);
	ecc[9] = (r7 >> 2);

	return 0;
}

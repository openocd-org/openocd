/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * The content of this file is mainly copied/inspired from Linux kernel
 * code in include/linux/align.h and include/uapi/linux/const.h
 *
 * Macro name 'ALIGN' conflicts with macOS/BSD file param.h
 */

#ifndef OPENOCD_HELPER_ALIGN_H
#define OPENOCD_HELPER_ALIGN_H

#define ALIGN_MASK(x, mask)             \
({                                      \
	typeof(mask) _mask = (mask);        \
	((x) + _mask) & ~_mask;             \
})

/* @a is a power of 2 value */
#define ALIGN_UP(x, a)          ALIGN_MASK(x, (typeof(x))(a) - 1)
#define ALIGN_DOWN(x, a)        ((x) & ~((typeof(x))(a) - 1))
#define IS_ALIGNED(x, a)        (((x) & ((typeof(x))(a) - 1)) == 0)

#define IS_PWR_OF_2(x)                  \
({                                      \
	typeof(x) _x = (x);                 \
	_x == 0 || (_x & (_x - 1)) == 0;    \
})

#endif /* OPENOCD_HELPER_ALIGN_H */

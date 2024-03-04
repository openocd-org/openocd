/* SPDX-License-Identifier: BSD-2-Clause */

/*
* Copyright (c) 2020-2024 The FreeBSD Foundation
*
* This software was developed by Björn Zeeb under sponsorship from
* the FreeBSD Foundation.
*/

#ifndef OPENOCD_HELPER_BITFIELD_H
#define OPENOCD_HELPER_BITFIELD_H

/**
 * These macros come from FreeBSD, check source on
 * https://cgit.freebsd.org/src/tree/sys/compat/linuxkpi/common/include/linux/bitfield.h
 * Which does not include example usages of them.
 */

#define __bf_shf(x) (__builtin_ffsll(x) - 1)

/**
 * FIELD_FIT(_mask, _value) - Check if a value fits in the specified bitfield mask
 * @_mask: Bitfield mask
 * @_value: Value to check
 *
 * This macro checks whether a given value fits within the range defined by the
 * specified bitfield mask. It ensures that no bits outside the mask are set.
 *
 * Return: true if the value fits, false otherwise.
 */
#define FIELD_FIT(_mask, _value) \
	(!(((typeof(_mask))(_value) << __bf_shf(_mask)) & ~(_mask)))

/**
 * FIELD_PREP(_mask, _value) - Prepare a value for insertion into a bitfield
 * @_mask: Bitfield mask
 * @_value: Value to insert
 *
 * This macro prepares a value for insertion into a bitfield by shifting the
 * value into the position defined by the mask and applying the mask.
 *
 * Return: The prepared bitfield value.
 */
#define FIELD_PREP(_mask, _value) \
	(((typeof(_mask))(_value) << __bf_shf(_mask)) & (_mask))

/**
 * FIELD_GET(_mask, _value) - Extract a value from a bitfield
 * @_mask: Bitfield mask
 * @_value: Bitfield value to extract from
 *
 * This macro extracts a value from a bitfield by masking and shifting the
 * relevant bits down to the least significant position.
 *
 * Return: The extracted value.
 */
#define FIELD_GET(_mask, _value) \
	((typeof(_mask))(((_value) & (_mask)) >> __bf_shf(_mask)))

#endif /* OPENOCD_HELPER_BITFIELD_H */

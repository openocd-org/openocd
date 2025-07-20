/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_HELPER_STRING_CHOICES_H
#define OPENOCD_HELPER_STRING_CHOICES_H

#include <helper/types.h>

/*
 * This file contains helper functions that return one of two strings depending
 * on a boolean value. The format of these functions is 'str_$true_$false' where
 * $true and $false are the two corresponding strings.
 *
 * These helper functions are beneficial because they improve code consistency
 * and reduce the number of hardcoded strings.
 */

static inline const char *str_enabled_disabled(bool value)
{
	return value ? "enabled" : "disabled";
}

#endif /* OPENOCD_HELPER_STRING_CHOICES_H */

/*
 * Spansion FM4 flash macros
 *
 * Copyright (c) 2015 Andreas Färber
 *
 * Based on S6E2CC_MN709-00007 for S6E2CC/C5/C4/C3/C2/C1 series
 */

	.text
	.syntax unified
	.cpu cortex-m4
	.thumb
	.thumb_func


#define FLASH_DPOL	(1 << 7)
#define FLASH_TOGG	(1 << 6)
#define FLASH_TLOV	(1 << 5)
#define FLASH_TOGG2	(1 << 2)

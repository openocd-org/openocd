// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2022 by Tarek BOCHKATI for STMicroelectronics
 * <tarek.bouchkati@st.com>
 */

#define OPENOCD_CONTRIB_LOADERS_FLASH_STM32_STM32H5X

#include <stdint.h>
#include "../../../../src/flash/nor/stm32h5x.h"

static inline __attribute__((always_inline))
void copy_buffer_u32(uint32_t *dst, uint32_t *src, int len)
{
	for (int i = 0; i < len; i++)
		dst[i] = src[i];
}

/* this function is assumes that fifo_size is multiple of flash_word_size
 * this condition is ensured by target_run_flash_async_algorithm
 */

void write(volatile struct stm32h5x_work_area *work_area,
		   uint8_t *fifo_end,
		   uint8_t *target_address,
		   uint32_t count)
{
	volatile uint32_t *flash_sr = (uint32_t *)work_area->params.flash_sr_addr;
	volatile uint32_t *flash_cr = (uint32_t *)work_area->params.flash_cr_addr;

	/* optimization to avoid reading from memory each time */
	uint8_t *rp_cache  = work_area->fifo.rp;

	/* fifo_start is used to wrap when we reach fifo_end */
	uint8_t *fifo_start = rp_cache;

	/* enable flash programming */
	*flash_cr = FLASH_PG;

	while (count) {
		/* optimization to avoid reading from memory each time */
		uint8_t *wp_cache  = work_area->fifo.wp;
		if (wp_cache == 0)
			break; /* aborted by target_run_flash_async_algorithm */

		int32_t fifo_size = wp_cache - rp_cache;
		if (fifo_size < 0) {
			/* consider the linear fifo, we will wrap later */
			fifo_size = fifo_end - rp_cache;
		}

		/* wait for at least a flash word */
		while (fifo_size >= work_area->params.flash_word_size) {
			copy_buffer_u32((uint32_t *)target_address,
					(uint32_t *)rp_cache,
					work_area->params.flash_word_size / 4);

			/* update target_address and rp_cache */
			target_address += work_area->params.flash_word_size;
			rp_cache += work_area->params.flash_word_size;

			/* wait for the busy flag */
			while (*flash_sr & FLASH_BSY)
				;

			if (*flash_sr & FLASH_ERROR) {
				work_area->fifo.rp = 0; /* set rp to zero 0 on error */
				goto write_end;
			}

			/* wrap if reach the fifo_end, and update rp in memory */
			if (rp_cache >= fifo_end)
				rp_cache = fifo_start;

			/* flush the rp cache value,
			 * so target_run_flash_async_algorithm can fill the circular fifo */
			work_area->fifo.rp = rp_cache;

			/* update fifo_size and count */
			fifo_size -= work_area->params.flash_word_size;
			count--;
		}
	}

write_end:
	/* disable flash programming */
	*flash_cr = 0;

	/* soft break the loader */
	__asm("bkpt 0");
}

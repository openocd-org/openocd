// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2022 by Tarek BOCHKATI for STMicroelectronics
 * <tarek.bouchkati@st.com>
 */

#ifndef OPENOCD_FLASH_NOR_STM32H5X
#define OPENOCD_FLASH_NOR_STM32H5X

/* IMPORTANT: this file is included by stm32h5x driver and flashloader,
 * so please when changing this file, do not forget to check the flashloader */

/* #include "helper/bits.h" causes build errors when compiling
 * the flashloader, for now just redefine the needed 'BIT 'macro */

#ifndef BIT
#define BIT(nr)                 (1UL << (nr))
#endif

/* Enough timeouts for flash operations */
#define FLASH_ERASE_TIMEOUT 250
#define FLASH_WRITE_TIMEOUT 50
#define FLASH_TIMEOUT 250

/* Relevant STM32H5 flags ****************************************************/
#define F_NONE              0
/* This flag indicates if the device flash is with dual bank architecture */
#define F_HAS_DUAL_BANK     BIT(0)
/* This flag indicates if the device embeds a TrustZone security feature */
#define F_HAS_TZ            BIT(2)
/* End of STM32H5 flags ******************************************************/

/* FLASH secure block based bank 1/2 register offsets */
#define FLASH_SECBB1(X) (0xA0 + 4 * ((X) - 1))
#define FLASH_SECBB2(X) (0x1A0 + 4 * ((X) - 1))

#define FLASH_SECBB_SECURE      0xFFFFFFFF
#define FLASH_SECBB_NON_SECURE  0

/* IDCODE register address */
#define DBGMCU_IDCODE			0x44024000

/* FLASH_OPTCR register bits */
#define FLASH_OPTLOCK		BIT(0)
#define FLASH_OPTSTRT		BIT(1)

/* FLASH_SR register bits */
#define FLASH_BSY			BIT(0)
#define FLASH_WBNE			BIT(1)
#define FLASH_DBNE			BIT(3)
#define FLASH_EOP			BIT(16)
#define FLASH_WRPERR		BIT(17)
#define FLASH_PGSERR		BIT(18)
#define FLASH_STRBERR		BIT(19)
#define FLASH_INCERR		BIT(20)
#define FLASH_OBKERR		BIT(21)
#define FLASH_OBKWERR		BIT(22)
#define FLASH_OPTCHANGERRR	BIT(23)

#define FLASH_ERROR (FLASH_WRPERR | FLASH_PGSERR | FLASH_STRBERR | FLASH_INCERR | \
		FLASH_OBKERR | FLASH_OBKWERR | FLASH_OPTCHANGERRR)

/* FLASH_CR register bits */
#define FLASH_LOCK			BIT(0)
#define FLASH_PG			BIT(1)
#define FLASH_SER			BIT(2)
#define FLASH_BER			BIT(3)
#define FLASH_STRT			BIT(5)
#define FLASH_SNB_POS		6
#define FLASH_MER			BIT(15)
#define FLASH_BKSEL			BIT(31)

/* FLASH_OPTSR register bits */
#define FLASH_PSTATE_POS	8
#define FLASH_PSTATE_MASK	0xFF
#define FLASH_SWAP_BANK		BIT(31)

/* FLASH_OPTSR2 register bits */
#define FLASH_TZEN_POS		24
#define FLASH_TZEN_MASK		0xFF

/* Register unlock keys */
#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* Option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F

/* Supported device IDs */
#define DEVID_STM32H50XX		0x474    /* RM0492 */
#define DEVID_STM32H56_H57XX	0x484    /* RM0481 */

/* Known Flash base addresses */
#define STM32_FLASH_BANK_BASE	0x08000000
#define STM32_FLASH_S_BANK_BASE	0x0C000000

/* Offset between non-secure and secure flash registers */
#define STM32_REGS_SEC_OFFSET	0x10000000

/* Flash data width (128 bit) */
#define FLASH_DATA_WIDTH		16

/* 100 bytes as loader stack should be large enough for the loader to operate */
#define LDR_STACK_SIZE			100

struct stm32h5x_work_area {
	struct stm32h5x_loader_params {
		uint32_t flash_sr_addr;
		uint32_t flash_cr_addr;
		uint32_t flash_word_size;
	} params;
	uint8_t stack[LDR_STACK_SIZE];
	struct flash_async_algorithm_circbuf {
		/* note: stm32h5x_work_area struct is shared between the loader
		 * and stm32h5x flash driver.
		 *
		 * '*wp' and '*rp' pointers' size is 4 bytes each since stm32h5x
		 * devices have 32-bit processors.
		 * however when used in openocd code, their size depends on the host
		 *   if the host is 32-bit, then the size is 4 bytes each.
		 *   if the host is 64-bit, then the size is 8 bytes each.
		 * to avoid this size difference, change their types depending on the
		 * usage (pointers for the loader, and 32-bit integers in openocd code).
		 */
#ifdef OPENOCD_CONTRIB_LOADERS_FLASH_STM32_STM32H5X
		uint8_t *wp;
		uint8_t *rp;
#else
		uint32_t wp;
		uint32_t rp;
#endif /* OPENOCD_CONTRIB_LOADERS_FLASH_STM32_STM32H5X */
	} fifo;
};

#endif

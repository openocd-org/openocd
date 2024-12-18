/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2015 by Uwe Bonnes                                      *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_STM32L4X
#define OPENOCD_FLASH_NOR_STM32L4X

/* IMPORTANT: this file is included by stm32l4x driver and flashloader,
 * so please when changing this file, do not forget to check the flashloader */

/* FIXME: #include "helper/bits.h" cause build errors when compiling
 * the flashloader, for now just redefine the needed 'BIT 'macro */

#ifndef BIT
#define BIT(nr)                 (1UL << (nr))
#endif

/* FLASH_CR register bits */
#define FLASH_PG				BIT(0)
#define FLASH_PER				BIT(1)
#define FLASH_MER1				BIT(2)
#define FLASH_PAGE_SHIFT		3
#define FLASH_BKER				BIT(11)
#define FLASH_BKER_G0			BIT(13)
#define FLASH_MER2				BIT(15)
#define FLASH_STRT				BIT(16)
#define FLASH_OPTSTRT			BIT(17)
#define FLASH_EOPIE				BIT(24)
#define FLASH_ERRIE				BIT(25)
#define FLASH_OBL_LAUNCH		BIT(27)
#define FLASH_OPTLOCK			BIT(30)
#define FLASH_LOCK				BIT(31)

/* FLASH_SR register bits */
#define FLASH_BSY				BIT(16)
#define FLASH_BSY2				BIT(17)

/* Fast programming not used => related errors not used*/
#define FLASH_PGSERR			BIT(7) /* Programming sequence error */
#define FLASH_SIZERR			BIT(6) /* Size error */
#define FLASH_PGAERR			BIT(5) /* Programming alignment error */
#define FLASH_WRPERR			BIT(4) /* Write protection error */
#define FLASH_PROGERR			BIT(3) /* Programming error */
#define FLASH_OPERR				BIT(1) /* Operation error */
#define FLASH_EOP				BIT(0) /* End of operation */
#define FLASH_ERROR				(FLASH_PGSERR | FLASH_SIZERR | FLASH_PGAERR | \
								FLASH_WRPERR | FLASH_PROGERR | FLASH_OPERR)

/* register unlock keys */
#define KEY1					0x45670123
#define KEY2					0xCDEF89AB

/* option register unlock key */
#define OPTKEY1					0x08192A3B
#define OPTKEY2					0x4C5D6E7F

/* FLASH_OPTR register bits */
#define FLASH_RDP_MASK			0xFF
#define FLASH_G0_DUAL_BANK		BIT(21)
#define FLASH_G4_DUAL_BANK		BIT(22)
#define FLASH_L4_DUAL_BANK		BIT(21)
#define FLASH_L4R_DBANK			BIT(22)
#define FLASH_LRR_DB1M			BIT(21)
#define FLASH_L5_DBANK			BIT(22)
#define FLASH_L5_DB256			BIT(21)
#define FLASH_U5_DUALBANK		BIT(21)
#define FLASH_TZEN				BIT(31)

/* FLASH secure block based bank 1/2 register offsets */
#define FLASH_SECBB1(X) (0x80 + 4 * (X - 1))
#define FLASH_SECBB2(X) (0xA0 + 4 * (X - 1))

#define FLASH_SECBB_SECURE      0xFFFFFFFF
#define FLASH_SECBB_NON_SECURE  0

/* IDCODE register possible addresses */
#define DBGMCU_IDCODE_G0		0x40015800
#define DBGMCU_IDCODE_L4_G4		0xE0042000
#define DBGMCU_IDCODE_L5		0xE0044000
#define UID64_DEVNUM			0x1FFF7580
#define UID64_IDS				0x1FFF7584
#define UID64_IDS_STM32WL		0x0080E115

/* Supported device IDs */
#define DEVID_STM32L47_L48XX	0x415
#define DEVID_STM32L43_L44XX	0x435
#define DEVID_STM32C01XX		0x443
#define DEVID_STM32C03XX		0x453
#define DEVID_STM32U53_U54XX	0x455
#define DEVID_STM32G05_G06XX	0x456
#define DEVID_STM32U031XX		0x459
#define DEVID_STM32G07_G08XX	0x460
#define DEVID_STM32L49_L4AXX	0x461
#define DEVID_STM32L45_L46XX	0x462
#define DEVID_STM32L41_L42XX	0x464
#define DEVID_STM32G03_G04XX	0x466
#define DEVID_STM32G0B_G0CXX	0x467
#define DEVID_STM32G43_G44XX	0x468
#define DEVID_STM32G47_G48XX	0x469
#define DEVID_STM32L4R_L4SXX	0x470
#define DEVID_STM32L4P_L4QXX	0x471
#define DEVID_STM32L55_L56XX	0x472
#define DEVID_STM32G49_G4AXX	0x479
#define DEVID_STM32U59_U5AXX	0x481
#define DEVID_STM32U57_U58XX	0x482
#define DEVID_STM32U073_U083XX	0x489
#define DEVID_STM32WBA5X		0x492
#define DEVID_STM32C071XX		0x493
#define DEVID_STM32WB1XX		0x494
#define DEVID_STM32WB5XX		0x495
#define DEVID_STM32WB3XX		0x496
#define DEVID_STM32WLE_WL5XX	0x497

/* known Flash base addresses */
#define STM32_FLASH_BANK_BASE	0x08000000
#define STM32_FLASH_S_BANK_BASE	0x0C000000

/* offset between non-secure and secure flash registers */
#define STM32L5_REGS_SEC_OFFSET 0x10000000

/* 100 bytes as loader stack should be large enough for the loader to operate */
#define LDR_STACK_SIZE			100

struct stm32l4_work_area {
	struct stm32l4_loader_params {
		uint32_t flash_sr_addr;
		uint32_t flash_cr_addr;
		uint32_t flash_word_size;
		uint32_t flash_sr_bsy_mask;
	} params;
	uint8_t stack[LDR_STACK_SIZE];
	struct flash_async_algorithm_circbuf {
		/* note: stm32l4_work_area struct is shared between the loader
		 * and stm32l4x flash driver.
		 *
		 * '*wp' and '*rp' pointers' size is 4 bytes each since stm32l4x
		 * devices have 32-bit processors.
		 * however when used in openocd code, their size depends on the host
		 *   if the host is 32-bit, then the size is 4 bytes each.
		 *   if the host is 64-bit, then the size is 8 bytes each.
		 * to avoid this size difference, change their types depending on the
		 * usage (pointers for the loader, and 32-bit integers in openocd code).
		 */
#ifdef OPENOCD_CONTRIB_LOADERS_FLASH_STM32_STM32L4X
		uint8_t *wp;
		uint8_t *rp;
#else
		uint32_t wp;
		uint32_t rp;
#endif /* OPENOCD_CONTRIB_LOADERS_FLASH_STM32_STM32L4X */
	} fifo;
};

#endif

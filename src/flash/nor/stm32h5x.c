// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2022 by Tarek BOCHKATI for STMicroelectronics
 * <tarek.bouchkati@st.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>
#include <target/arm_adi_v5.h>
#include "stm32h5x.h"


enum stm32h5x_bank_id {
	STM32_BANK1 = 1,
	STM32_BANK2 = 2,
	STM32_ALL_BANKS
};

enum stm32h5x_pstate {
	PSTATE_OPEN				= 0xED,
	PSTATE_PROVISIONING		= 0x17,
	PSTATE_IROT_PROVISIONED	= 0x2E,
	PSTATE_TZ_CLOSED		= 0xC6,
	PSTATE_CLOSED			= 0x72,
	PSTATE_LOCKED			= 0x5C,
	PSTATE_REGRESSION		= 0x9A,
	PSTATE_NS_REGRESSION	= 0xA3
};

static const char *stm32h5x_product_state_str(enum stm32h5x_pstate pstate)
{
	switch (pstate) {
	case PSTATE_OPEN:
		return "Open";
	case PSTATE_PROVISIONING:
		return "Provisioning";
	case PSTATE_IROT_PROVISIONED:
		return "iROT-Provisioned";
	case PSTATE_TZ_CLOSED:
		return "TZ-Closed";
	case PSTATE_CLOSED:
		return "Closed";
	case PSTATE_LOCKED:
		return "Locked";
	case PSTATE_REGRESSION:
		return "Regression";
	case PSTATE_NS_REGRESSION:
		return "NS-Regression";
	}

	return "Unknown";
}

enum stm32h5x_tzen {
	TZEN_DISABLED	= 0xC3,
	TZEN_ENABLED	= 0xB4
};

struct stm32h5x_rev {
	const uint16_t rev;
	const char *name;
};

struct stm32h5x_dev_info {
	uint16_t id;
	const char *name;
	const struct stm32h5x_rev *revs;
	const uint32_t flags; /* one bit per feature, see STM32H5 flags */
	const uint16_t max_flash_size_kb;
	const uint32_t flash_regs_base;
	const uint32_t flash_size_addr;
	const uint32_t wps_group_size; /* write protection group sectors' count */
	const uint32_t wps_mask;
};

struct stm32h5x_flash_bank {
	bool probed;
	uint32_t idcode;
	const struct stm32h5x_dev_info *dev_info;
	unsigned int bank1_sectors, bank1_prot_blocks;
	bool dual_bank;
	uint32_t user_bank_size;
	uint32_t flash_regs_base;
	const uint32_t *flash_regs;
	enum stm32h5x_pstate pstate;
	enum stm32h5x_tzen tzen;
	uint32_t optsr_cur, optsr2_cur; /* options cache to trigger re-probing */
};

/* Human readable list of families this drivers supports (sorted alphabetically) */
static const char *supported_devices_desc = "STM32H5x";

/* Known revisions */
#define NO_MORE_REVS { 0x0, NULL }

static const struct stm32h5x_rev stm32h50xx_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x1003, "Y" }, NO_MORE_REVS
};

static const struct stm32h5x_rev stm32h56_h57xx_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x1007, "X" }, NO_MORE_REVS
};

/* Devices identities */
static const struct stm32h5x_dev_info stm32h5x_dev_info_db[] = {
	{
		.id					= DEVID_STM32H50XX,
		.name				= "STM32H50xx",
		.revs				= stm32h50xx_revs,
		.flags				= F_HAS_DUAL_BANK,
		.max_flash_size_kb	= 128,
		.flash_regs_base	= 0x40022000,
		.flash_size_addr	= 0x08FFF80C,
		.wps_group_size		= 1,
		.wps_mask			= 0xFF,
	},
	{
		.id					= DEVID_STM32H56_H57XX,
		.name				= "STM32H56/H57xx",
		.revs				= stm32h56_h57xx_revs,
		.flags				= F_HAS_DUAL_BANK | F_HAS_TZ,
		.max_flash_size_kb	= 2048,
		.flash_regs_base	= 0x40022000,
		.flash_size_addr	= 0x08FFF80C,
		.wps_group_size		= 4,
		.wps_mask			= 0xFFFFFFFF,
	},
};

/* Register maps*/
enum stm32h5x_flash_reg_index {
	STM32_FLASH_ACR_INDEX,
	STM32_FLASH_KEYR_INDEX,
	STM32_FLASH_OPTKEYR_INDEX,
	STM32_FLASH_OPTCR_INDEX,
	STM32_FLASH_SR_INDEX,
	STM32_FLASH_CR_INDEX,
	STM32_FLASH_CCR_INDEX,
	STM32_FLASH_OPTSR_CUR_INDEX,
	STM32_FLASH_OPTSR_PRG_INDEX,
	STM32_FLASH_OPTSR2_CUR_INDEX,
	STM32_FLASH_OPTSR2_PRG_INDEX,
	STM32_FLASH_WRP1_CUR_INDEX,
	STM32_FLASH_WRP1_PRG_INDEX,
	STM32_FLASH_WRP2_CUR_INDEX,
	STM32_FLASH_WRP2_PRG_INDEX,
	STM32_FLASH_REG_INDEX_NUM,
};

static const uint32_t stm32h5x_ns_flash_regs[STM32_FLASH_REG_INDEX_NUM] = {
	[STM32_FLASH_ACR_INDEX]			= 0x000,
	[STM32_FLASH_KEYR_INDEX]		= 0x004,
	[STM32_FLASH_OPTKEYR_INDEX]		= 0x00C,
	[STM32_FLASH_OPTCR_INDEX]		= 0x01C,
	[STM32_FLASH_SR_INDEX]			= 0x020,
	[STM32_FLASH_CR_INDEX]			= 0x028,
	[STM32_FLASH_CCR_INDEX]			= 0x030,
	[STM32_FLASH_OPTSR_CUR_INDEX]	= 0x050,
	[STM32_FLASH_OPTSR_PRG_INDEX]	= 0x054,
	[STM32_FLASH_OPTSR2_CUR_INDEX]	= 0x070,
	[STM32_FLASH_OPTSR2_PRG_INDEX]	= 0x074,
	[STM32_FLASH_WRP1_CUR_INDEX]	= 0x0E8,
	[STM32_FLASH_WRP1_PRG_INDEX]	= 0x0EC,
	[STM32_FLASH_WRP2_CUR_INDEX]	= 0x1E8,
	[STM32_FLASH_WRP2_PRG_INDEX]	= 0x1EC,
};

static const uint32_t stm32h5x_s_flash_regs[STM32_FLASH_REG_INDEX_NUM] = {
	[STM32_FLASH_ACR_INDEX]			= 0x000,
	[STM32_FLASH_KEYR_INDEX]		= 0x008,
	[STM32_FLASH_OPTKEYR_INDEX]		= 0x00C,
	[STM32_FLASH_OPTCR_INDEX]		= 0x01C,
	[STM32_FLASH_SR_INDEX]			= 0x024,
	[STM32_FLASH_CR_INDEX]			= 0x02C,
	[STM32_FLASH_CCR_INDEX]			= 0x034,
	[STM32_FLASH_OPTSR_CUR_INDEX]	= 0x050,
	[STM32_FLASH_OPTSR_PRG_INDEX]	= 0x054,
	[STM32_FLASH_OPTSR2_CUR_INDEX]	= 0x070,
	[STM32_FLASH_OPTSR2_PRG_INDEX]	= 0x074,
	[STM32_FLASH_WRP1_CUR_INDEX]	= 0x0E8,
	[STM32_FLASH_WRP1_PRG_INDEX]	= 0x0EC,
	[STM32_FLASH_WRP2_CUR_INDEX]	= 0x1E8,
	[STM32_FLASH_WRP2_PRG_INDEX]	= 0x1EC,
};

/* Usage: flash bank stm32h5x <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(stm32h5x_flash_bank_command)
{
	struct stm32h5x_flash_bank *stm32h5x_bank;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* fix-up bank base address: 0 is used for normal flash memory */
	if (bank->base == 0)
		bank->base = STM32_FLASH_BANK_BASE;

	stm32h5x_bank = calloc(1, sizeof(struct stm32h5x_flash_bank));
	if (!stm32h5x_bank)
		return ERROR_FAIL;

	bank->driver_priv = stm32h5x_bank;

	stm32h5x_bank->probed = false;
	stm32h5x_bank->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline uint32_t stm32h5x_get_flash_reg(struct flash_bank *bank, uint32_t reg_offset)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	return stm32h5x_bank->flash_regs_base + reg_offset;
}

static inline uint32_t stm32h5x_get_flash_reg_by_index(struct flash_bank *bank,
	enum stm32h5x_flash_reg_index reg_index)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	return stm32h5x_get_flash_reg(bank, stm32h5x_bank->flash_regs[reg_index]);
}

static inline int stm32h5x_read_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t *value)
{
	return target_read_u32(bank->target, stm32h5x_get_flash_reg(bank, reg_offset), value);
}

static inline int stm32h5x_read_flash_reg_by_index(struct flash_bank *bank,
	enum stm32h5x_flash_reg_index reg_index, uint32_t *value)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	return stm32h5x_read_flash_reg(bank, stm32h5x_bank->flash_regs[reg_index], value);
}

static inline int stm32h5x_write_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t value)
{
	return target_write_u32(bank->target, stm32h5x_get_flash_reg(bank, reg_offset), value);
}

static inline int stm32h5x_write_flash_reg_by_index(struct flash_bank *bank,
	enum stm32h5x_flash_reg_index reg_index, uint32_t value)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	return stm32h5x_write_flash_reg(bank, stm32h5x_bank->flash_regs[reg_index], value);
}

static int stm32h5x_wait_status_flags(struct flash_bank *bank, int timeout, uint32_t flags)
{
	uint32_t status;
	int retval = ERROR_OK;

	/* Wait for busy flags to clear */
	for (;;) {
		retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_SR_INDEX, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);

		if ((status & flags) == 0)
			break;

		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}

		alive_sleep(1);
	}

	if (status & FLASH_WRPERR) {
		LOG_ERROR("stm32x device protected");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & FLASH_ERROR) {
		if (retval == ERROR_OK)
			retval = ERROR_FAIL;

		/* If this operation fails, we ignore it and report the original retval */
		stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_CCR_INDEX, status & FLASH_ERROR);
	}

	return retval;
}

static inline int stm32h5x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	return stm32h5x_wait_status_flags(bank, timeout, FLASH_BSY);
}

static inline int stm32h5x_wait_status_prev_op(struct flash_bank *bank, int timeout)
{
	return stm32h5x_wait_status_flags(bank, timeout, FLASH_BSY | FLASH_WBNE | FLASH_DBNE);
}

static int stm32h5x_unlock_reg(struct flash_bank *bank)
{
	uint32_t cr;

	/* First check if not already unlocked,
	 * otherwise writing on STM32_FLASH_KEYR will fail
	 */
	int retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, &cr);
	if (retval != ERROR_OK)
		return retval;

	if ((cr & FLASH_LOCK) == 0)
		return ERROR_OK;

	/* Unlock flash registers */
	retval = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_KEYR_INDEX, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_KEYR_INDEX, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, &cr);
	if (retval != ERROR_OK)
		return retval;

	if (cr & FLASH_LOCK) {
		LOG_ERROR("flash not unlocked STM32_FLASH_CR: %" PRIx32, cr);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32h5x_unlock_option_reg(struct flash_bank *bank)
{
	uint32_t optcr;

	int retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_OPTCR_INDEX, &optcr);
	if (retval != ERROR_OK)
		return retval;

	if ((optcr & FLASH_OPTLOCK) == 0)
		return ERROR_OK;

	/* Unlock option registers */
	retval = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_OPTKEYR_INDEX, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_OPTKEYR_INDEX, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_OPTCR_INDEX, &optcr);
	if (retval != ERROR_OK)
		return retval;

	if (optcr & FLASH_OPTLOCK) {
		LOG_ERROR("options not unlocked STM32_FLASH_OPTCR: %" PRIx32, optcr);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

/** set all FLASH_SECBB registers to the same value */
static int stm32h5x_set_secbb(struct flash_bank *bank, uint32_t value)
{
	/* This function should be used only with device with TrustZone, do just a security check */
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	assert(stm32h5x_bank->dev_info->flags & F_HAS_TZ);

	int retval = stm32h5x_wait_status_prev_op(bank, FLASH_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* write SECBBxRy registers */
	LOG_DEBUG("setting secure block-based areas registers (SECBBxRy) to 0x%08x", value);

	const uint16_t secbb_regs[] = {
			FLASH_SECBB1(1), FLASH_SECBB1(2), FLASH_SECBB1(3), FLASH_SECBB1(4), /* bank 1 SECBB register offsets */
			FLASH_SECBB2(1), FLASH_SECBB2(2), FLASH_SECBB2(3), FLASH_SECBB2(4)  /* bank 2 SECBB register offsets */
	};

	unsigned int num_secbb_regs = ARRAY_SIZE(secbb_regs);

	/* in single bank mode, it's useless to modify FLASH_SECBB2Rx registers
	 * then consider only the first half of secbb_regs
	 */
	if (!stm32h5x_bank->dual_bank)
		num_secbb_regs /= 2;

	for (unsigned int i = 0; i < num_secbb_regs; i++) {
		retval = stm32h5x_write_flash_reg(bank, secbb_regs[i], value);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int stm32h5x_write_option(struct flash_bank *bank, uint32_t reg_offset,
	uint32_t value, uint32_t mask)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	uint32_t optiondata;
	int retval, retval2;

	retval = stm32h5x_read_flash_reg(bank, reg_offset, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	/* for STM32H5 and similar devices, use always non-secure
	 * registers for option bytes programming */
	const uint32_t *saved_flash_regs = stm32h5x_bank->flash_regs;
	if (stm32h5x_bank->dev_info->flags & F_HAS_TZ)
		stm32h5x_bank->flash_regs = stm32h5x_ns_flash_regs;

	retval = stm32h5x_wait_status_prev_op(bank, FLASH_TIMEOUT);

	retval = stm32h5x_unlock_option_reg(bank);
	if (retval != ERROR_OK)
		goto err_lock;

	optiondata = (optiondata & ~mask) | (value & mask);

	retval = stm32h5x_write_flash_reg(bank, reg_offset, optiondata);
	if (retval != ERROR_OK)
		goto err_lock;

	retval = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_OPTCR_INDEX, FLASH_OPTSTRT);
	if (retval != ERROR_OK)
		goto err_lock;

	retval = stm32h5x_wait_status_busy(bank, FLASH_TIMEOUT);

err_lock:
	retval2 = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_OPTCR_INDEX, FLASH_OPTLOCK);
	stm32h5x_bank->flash_regs = saved_flash_regs;

	if (retval != ERROR_OK)
		return retval;

	return retval2;
}

static int stm32h5x_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	int retval, retval2;

	assert((first <= last) && (last < bank->num_sectors));

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32h5x_bank->tzen == TZEN_ENABLED && stm32h5x_bank->pstate == PSTATE_OPEN) {
		/* set all FLASH pages as secure */
		retval = stm32h5x_set_secbb(bank, FLASH_SECBB_SECURE);
		if (retval != ERROR_OK) {
			/* restore all FLASH pages as non-secure */
			stm32h5x_set_secbb(bank, FLASH_SECBB_NON_SECURE); /* ignore the return value */
			return retval;
		}
	}

	/* Standard Flash sector erase sequence
	 * 1. Make sure protection mechanism does not prevent sector erase
	 *    (WRP, secure flag, HDP) [this step is ignored]
	 * 2. Check there is no ongoing FLASH operation by checking BSY and DBNE in FLASH_NSSR,
	 *    and that the write buffer is empty by checking the WBNE in the FLASH_NSSR
	 * 3. Check and clear all previous non-secure error flags
	 * 4. Unlock the FLASH_NSCR register
	 * 5. Set BKSEL, SER and SNB in FLASH_NSCR
	 * 6. Set STRT in FLASH_NSCR
	 * 7. Wait for BSY to be cleared in the FLASH_NSSR
	 * 8. STRT is automatically cleared
	 * 9. Clear SER in FLASH_NSCR after all sector erase request are issued
	 */

	/* Step 2 & 3 */
	retval = stm32h5x_wait_status_prev_op(bank, FLASH_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Step 4 */
	retval = stm32h5x_unlock_reg(bank);
	if (retval != ERROR_OK)
		goto err_lock;

	/* Step 5, 6 and 7 in a loop */
	for (unsigned int i = first; i <= last; i++) {
		uint32_t erase_flags = FLASH_SER | FLASH_STRT;

		if (i >= stm32h5x_bank->bank1_sectors) {
			uint8_t snb;
			snb = i - stm32h5x_bank->bank1_sectors;
			erase_flags |= snb << FLASH_SNB_POS | FLASH_BKSEL;
		} else {
			erase_flags |= i << FLASH_SNB_POS;
		}

		retval = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, erase_flags);
		if (retval != ERROR_OK)
			break;

		retval = stm32h5x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			break;
	}

err_lock:
	/* this implicitly clears SER (Step 9) */
	retval2 = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, FLASH_LOCK);

	if (stm32h5x_bank->tzen == TZEN_ENABLED && stm32h5x_bank->pstate == PSTATE_OPEN) {
		/* restore all FLASH pages as non-secure */
		int retval3 = stm32h5x_set_secbb(bank, FLASH_SECBB_NON_SECURE);
		if (retval3 != ERROR_OK)
			return retval3;
	}

	if (retval != ERROR_OK)
		return retval;

	return retval2;
}

static int stm32h5x_mass_erase(struct flash_bank *bank, enum stm32h5x_bank_id bank_id)
{
	/* FIXME mass erase not working */
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	int retval, retval2;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32h5x_bank->tzen == TZEN_ENABLED && stm32h5x_bank->pstate == PSTATE_OPEN) {
		/* set all FLASH pages as secure */
		retval = stm32h5x_set_secbb(bank, FLASH_SECBB_SECURE);
		if (retval != ERROR_OK) {
			/* restore all FLASH pages as non-secure */
			stm32h5x_set_secbb(bank, FLASH_SECBB_NON_SECURE); /* ignore the return value */
			return retval;
		}
	}

	/* Flash mass erase sequence
	 * 1. Make sure protection mechanism does not prevent mass erase
	 *    (WRP, secure flag, HDP) [this step is ignored]
	 * 2. Check there is no ongoing FLASH operation by checking BSY and DBNE in FLASH_NSSR,
	 *    and that the write buffer is empty by checking the WBNE in the FLASH_NSSR
	 * 3. Check and clear all previous non-secure error flags
	 * 4. Unlock the FLASH_NSCR
	 * 5. Set MER in FLASH_NSCR
	 * 6. Set STRT in FLASH_NSCR
	 * 7. Wait for the BSY to be cleared in the FLASH_NSSR
	 * 8. STRT is automatically cleared
	 * 9. Clear MER in FLASH_NSCR
	 */

	/* Bank erase sequence, same as above with this change
	 * 5. Set BER and select the bank using BKSEL in FLASH_NSCR
	 * 9. Clear BER and BKSEL
	 */

	uint32_t flash_cr_op;

	switch (bank_id) {
	case STM32_BANK1:
		flash_cr_op = FLASH_BER;
		break;
	case STM32_BANK2:
		flash_cr_op = FLASH_BER | FLASH_BKSEL;
		break;
	case STM32_ALL_BANKS:
		flash_cr_op = FLASH_MER;
		break;
	default:
		LOG_ERROR("incorrect bank_id");
		return ERROR_OK;
	}

	/* Step 2 & 3 */
	retval = stm32h5x_wait_status_prev_op(bank, FLASH_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Step 4 */
	retval = stm32h5x_unlock_reg(bank);
	if (retval != ERROR_OK)
		goto err_lock;

	/* Step 5, 6 and 7 in a loop */

	retval = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, flash_cr_op);
	if (retval != ERROR_OK)
		goto err_lock;

	retval = stm32h5x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);

err_lock:
	/* this implicitly clears MER/BER/BKSEL (Step 9) */
	retval2 = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, FLASH_LOCK);

	if (stm32h5x_bank->tzen == TZEN_ENABLED && stm32h5x_bank->pstate == PSTATE_OPEN) {
		/* restore all FLASH pages as non-secure */
		int retval3 = stm32h5x_set_secbb(bank, FLASH_SECBB_NON_SECURE);
		if (retval3 != ERROR_OK)
			return retval3;
	}

	if (retval != ERROR_OK)
		return retval;

	return retval2;
}

static int stm32h5x_protect_inner(struct flash_bank *bank, int set,
		enum stm32h5x_bank_id bank_id, unsigned int first, unsigned int last)
{
	assert(bank_id != STM32_ALL_BANKS);

	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;

	if (bank_id == STM32_BANK2 && !stm32h5x_bank->dual_bank) {
		LOG_ERROR("this device have a single bank");
		return ERROR_FAIL;
	}

	enum stm32h5x_flash_reg_index wrp_cur_idx = STM32_FLASH_WRP1_CUR_INDEX;
	enum stm32h5x_flash_reg_index wrp_prg_idx = STM32_FLASH_WRP1_PRG_INDEX;

	if (bank_id == STM32_BANK2) {
		wrp_cur_idx = STM32_FLASH_WRP2_CUR_INDEX;
		wrp_prg_idx = STM32_FLASH_WRP2_PRG_INDEX;
		first -= stm32h5x_bank->bank1_prot_blocks;
		last -= stm32h5x_bank->bank1_prot_blocks;
	}

	uint32_t protection;

	/* Read 'write protection' settings */
	int retval = stm32h5x_read_flash_reg_by_index(bank, wrp_cur_idx, &protection);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read BANK %d WRP_CUR register", bank_id);
		return retval;
	}

	for (unsigned int i = first; i <= last; i++) {
		if (set)
			protection &= ~(1 << i);
		else
			protection |= (1 << i);
	}

	/* Apply new option value */
	return stm32h5x_write_option(bank, stm32h5x_bank->flash_regs[wrp_prg_idx],
			protection, stm32h5x_bank->dev_info->wps_mask);
}

static int stm32h5x_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The requested block for protection could be spread between the 2 banks */
	if (last < stm32h5x_bank->bank1_prot_blocks)
		return stm32h5x_protect_inner(bank, set, STM32_BANK1, first, last);
	else if (first >= stm32h5x_bank->bank1_prot_blocks)
		return stm32h5x_protect_inner(bank, set, STM32_BANK2, first, last);

	int retval = stm32h5x_protect_inner(bank, set, STM32_BANK1,
			first, stm32h5x_bank->bank1_prot_blocks - 1);
	if (retval != ERROR_OK)
		return retval;

	return stm32h5x_protect_inner(bank, set, STM32_BANK1,
			stm32h5x_bank->bank1_prot_blocks, last);
}

static int stm32h5x_protect_check(struct flash_bank *bank)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	uint32_t wrp1;

	/* Read bank 1 'write protection' settings */
	int retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_WRP1_CUR_INDEX, &wrp1);
	if (retval != ERROR_OK) {
		LOG_ERROR("unable to read WRP1_CUR register");
		return retval;
	}

	for (unsigned int i = 0; i < stm32h5x_bank->bank1_prot_blocks; i++)
		bank->prot_blocks[i].is_protected = wrp1 & (1 << i) ? 0 : 1;

	if (!stm32h5x_bank->dual_bank)
		return ERROR_OK;

	/* Read bank 2 'write protection' settings */
	uint32_t wrp2;
	retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_WRP2_CUR_INDEX, &wrp2);
	if (retval != ERROR_OK) {
		LOG_ERROR("unable to read WRP2_CUR register");
		return retval;
	}

	const unsigned int bank2_prot_blocks = bank->num_prot_blocks - stm32h5x_bank->bank1_prot_blocks;

	for (unsigned int i = 0; i < bank2_prot_blocks; i++)
		bank->prot_blocks[i + stm32h5x_bank->bank1_prot_blocks].is_protected = wrp2 & (1 << i) ? 0 : 1;

	return ERROR_OK;
}

/* count is the size divided by FLASH_DATA_WIDTH */
static int stm32h5x_write_block_fast(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	static const uint8_t stm32h5x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/stm32/stm32h5x.inc"
	};

	if (target_alloc_working_area(target, sizeof(stm32h5x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(stm32h5x_flash_write_code),
			stm32h5x_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* data_width should be multiple of double-word */
	assert(FLASH_DATA_WIDTH % 8 == 0);
	const size_t extra_size = sizeof(struct stm32h5x_work_area);
	uint32_t buffer_size = target_get_working_area_avail(target) - extra_size;
	/* buffer_size should be multiple of stm32h5_info->data_width */
	buffer_size &= ~(FLASH_DATA_WIDTH - 1);

	if (buffer_size < 256) {
		LOG_WARNING("large enough working area not available, can't do block memory writes");
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	} else if (buffer_size > 16384) {
		/* probably won't benefit from more than 16k ... */
		buffer_size = 16384;
	}

	if (target_alloc_working_area_try(target, buffer_size + extra_size, &source) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* contrib/loaders/flash/stm32/stm32h5x.c:write() arguments */
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* stm32h5_work_area ptr , status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* target address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* count (of stm32h5_info->data_width) */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count);

	/* write algo stack pointer */
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[4].value, 0, 32, source->address +
			offsetof(struct stm32h5x_work_area, stack) + LDR_STACK_SIZE);

	struct stm32h5x_loader_params loader_extra_params;

	target_buffer_set_u32(target, (uint8_t *)&loader_extra_params.flash_sr_addr,
			stm32h5x_get_flash_reg_by_index(bank, STM32_FLASH_SR_INDEX));
	target_buffer_set_u32(target, (uint8_t *)&loader_extra_params.flash_cr_addr,
			stm32h5x_get_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX));
	target_buffer_set_u32(target, (uint8_t *)&loader_extra_params.flash_word_size,
			FLASH_DATA_WIDTH);

	retval = target_write_buffer(target, source->address, sizeof(loader_extra_params),
			(uint8_t *)&loader_extra_params);
	if (retval != ERROR_OK)
		return retval;

	retval = target_run_flash_async_algorithm(target, buffer, count, FLASH_DATA_WIDTH,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address + offsetof(struct stm32h5x_work_area, fifo),
			source->size - offsetof(struct stm32h5x_work_area, fifo),
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("error executing stm32l4 flash write algorithm");

		uint32_t error;
		stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_SR_INDEX, &error);
		error &= FLASH_ERROR;

		if (error & FLASH_WRPERR)
			LOG_ERROR("flash memory write protected");

		if (error != 0) {
			LOG_ERROR("flash write failed = %08" PRIx32, error);
			/* Clear but report errors */
			stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_SR_INDEX, error);
			retval = ERROR_FAIL;
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

/* count is the size divided by FLASH_DATA_WIDTH */
static int stm32h5x_write_block_slow(struct flash_bank *bank, const uint8_t *buffer,
				uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t address = bank->base + offset;
	int retval = ERROR_OK;

	/* The recommended single-write sequence
	 * 1. Make sure protection mechanism does not prevent sector erase
	 *    (WRP, secure flag, HDP) [this step is ignored]
	 * 2. Check there is no ongoing FLASH operation by checking BSY and DBNE in FLASH_NSSR,
	 *    and that the write buffer is empty by checking the WBNE in the FLASH_NSSR
	 * 3. Check and clear all previous non-secure error flags
	 * 4. Unlock the FLASH_NSCR register
	 * 5. Enable write operations by setting PG FLASH_NSCR
	 * 6. Write one Flash-word at aligned address
	 * 7. Wait for BSY to be cleared in the FLASH_NSSR
	 * 8. Clear PG
	 */

	/* Step 2 & 3 */
	retval = stm32h5x_wait_status_prev_op(bank, FLASH_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Step 4, done in the caller function 'stm32h5x_write()' */

	/* set PG in FLASH_CR */
	retval = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, FLASH_PG);
	if (retval != ERROR_OK)
		return retval;

	/* write directly to flash memory */
	const uint8_t *src = buffer;
	const uint32_t data_width_in_words = FLASH_DATA_WIDTH / 4;
	while (count--) {
		retval = target_write_memory(target, address, 4, data_width_in_words, src);
		if (retval != ERROR_OK)
			return retval;

		/* wait for BSY bit */
		retval = stm32h5x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		keep_alive();
		src += FLASH_DATA_WIDTH;
		address += FLASH_DATA_WIDTH;
	}

	/* reset PG in FLASH_CR */
	retval = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, 0);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int stm32h5x_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	int retval = ERROR_OK, retval2;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32h5x_bank->tzen == TZEN_ENABLED && stm32h5x_bank->pstate == PSTATE_OPEN) {
		/* set all FLASH pages as secure */
		retval = stm32h5x_set_secbb(bank, FLASH_SECBB_SECURE);
		if (retval != ERROR_OK) {
			/* restore all FLASH pages as non-secure */
			stm32h5x_set_secbb(bank, FLASH_SECBB_NON_SECURE); /* ignore the return value */
			return retval;
		}
	}

	/* The flash write must be aligned to the 'FLASH_DATA_WIDTH' boundary.
	 * The flash infrastructure ensures it, do just a security check */
	assert(offset % FLASH_DATA_WIDTH == 0);
	assert(count % FLASH_DATA_WIDTH == 0);

	if (retval != ERROR_OK)
		return retval;

	retval = stm32h5x_unlock_reg(bank);
	if (retval != ERROR_OK)
		goto err_lock;

	if (stm32h5x_bank->tzen == TZEN_ENABLED && stm32h5x_bank->pstate != PSTATE_OPEN) {
		LOG_INFO("TZ On and PState not OPEN: programming without flash loader (slower)");
		retval = stm32h5x_write_block_slow(bank, buffer, offset, count / FLASH_DATA_WIDTH);
	} else {
		/* write using the loader, for better performance */
		retval = stm32h5x_write_block_fast(bank, buffer, offset, count / FLASH_DATA_WIDTH);

		/* if resources are not available write without a loader */
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			LOG_WARNING("falling back to programming without a flash loader (slower)");
			retval = stm32h5x_write_block_slow(bank, buffer, offset,
					count / FLASH_DATA_WIDTH);
		}
	}

err_lock:
	retval2 = stm32h5x_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, FLASH_LOCK);

	if (stm32h5x_bank->tzen == TZEN_ENABLED && stm32h5x_bank->pstate == PSTATE_OPEN) {
		/* restore all FLASH pages as non-secure */
		int retval3 = stm32h5x_set_secbb(bank, FLASH_SECBB_NON_SECURE);
		if (retval3 != ERROR_OK)
			return retval3;
	}

	if (retval != ERROR_OK) {
		LOG_ERROR("block write failed");
		return retval;
	}
	return retval2;
}

int stm32h5x_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32h5x_bank->tzen == TZEN_ENABLED && stm32h5x_bank->pstate == PSTATE_OPEN) {
		/* set all FLASH pages as secure */
		retval = stm32h5x_set_secbb(bank, FLASH_SECBB_SECURE);
		if (retval != ERROR_OK) {
			/* restore all FLASH pages as non-secure */
			stm32h5x_set_secbb(bank, FLASH_SECBB_NON_SECURE); /* ignore the return value */
			return retval;
		}
	}

	int retval2 = target_read_buffer(bank->target, offset + bank->base, count, buffer);

	if (stm32h5x_bank->tzen == TZEN_ENABLED && stm32h5x_bank->pstate == PSTATE_OPEN) {
		/* restore all FLASH pages as non-secure */
		retval = stm32h5x_set_secbb(bank, FLASH_SECBB_NON_SECURE);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval2;
}

static int stm32h5x_read_idcode(struct flash_bank *bank, uint32_t *id)
{
	int retval = ERROR_OK;
	struct target *target = bank->target;

	/* read stm32 device id register */
	retval = target_read_u32(target, DBGMCU_IDCODE, id);

	if (retval != ERROR_OK) {
		LOG_ERROR("can't get the device id");
	}

	return retval;
}

static const char *get_stm32h5x_device_str(struct flash_bank *bank)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	return stm32h5x_bank->dev_info->name;
}

static const char *get_stm32h5x_rev_str(struct flash_bank *bank)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	const struct stm32h5x_rev *revs = stm32h5x_bank->dev_info->revs;

	const uint16_t rev_id = stm32h5x_bank->idcode >> 16;

	for (unsigned int i = 0; revs[i].rev != 0; i++) {
		if (rev_id == revs[i].rev)
			return revs[i].name;
	}

	return "'unknown'";
}

static const char *get_stm32h5x_bank_type_str(struct flash_bank *bank)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	return stm32h5x_bank->dual_bank ? "dual" : "single";
}

static void stm32h5x_sync_product_state(struct flash_bank *bank)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;

	stm32h5x_bank->tzen = TZEN_DISABLED;
	if (stm32h5x_bank->dev_info->flags & F_HAS_TZ)
		stm32h5x_bank->tzen = (stm32h5x_bank->optsr2_cur >> FLASH_TZEN_POS) & FLASH_TZEN_MASK;

	stm32h5x_bank->pstate = (stm32h5x_bank->optsr_cur >> FLASH_PSTATE_POS) & FLASH_PSTATE_MASK;
}

static int stm32h5x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;

	stm32h5x_bank->probed = false;

	/* Read stm32 device id registers */
	int retval = stm32h5x_read_idcode(bank, &stm32h5x_bank->idcode);
	if (retval != ERROR_OK)
		return retval;

	const uint32_t device_id = stm32h5x_bank->idcode & 0xFFF;

	for (unsigned int n = 0; n < ARRAY_SIZE(stm32h5x_dev_info_db); n++) {
		if (device_id == stm32h5x_dev_info_db[n].id) {
			stm32h5x_bank->dev_info = &stm32h5x_dev_info_db[n];
			break;
		}
	}

	if (!stm32h5x_bank->dev_info) {
		LOG_WARNING("Cannot identify target as an %s family device.",
				supported_devices_desc);

		return ERROR_FAIL;
	}

	const struct stm32h5x_dev_info *dev_info = stm32h5x_bank->dev_info;
	const char *device_str = get_stm32h5x_device_str(bank);
	const char *rev_str = get_stm32h5x_rev_str(bank);
	const uint16_t rev_id = stm32h5x_bank->idcode >> 16;

	LOG_INFO("device idcode = 0x%08" PRIx32 " (%s - Rev %s : 0x%04x)",
			stm32h5x_bank->idcode, device_str, rev_str, rev_id);

	stm32h5x_bank->flash_regs_base = dev_info->flash_regs_base;
	/* Set flash write alignment boundaries.
	 * Ask the flash infrastructure to ensure required alignment */
	bank->write_start_alignment = FLASH_DATA_WIDTH;
	bank->write_end_alignment = FLASH_DATA_WIDTH;

	/* Initialize the flash register map */
	stm32h5x_bank->flash_regs = stm32h5x_ns_flash_regs;

	/* Read flash option register */
	retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_OPTSR_CUR_INDEX,
			&stm32h5x_bank->optsr_cur);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_OPTSR2_CUR_INDEX,
			&stm32h5x_bank->optsr2_cur);
	if (retval != ERROR_OK)
		return retval;

	stm32h5x_sync_product_state(bank);

	/* For devices with TrustZone enabled and Product State 'Open', use flash secure registers */
	if (stm32h5x_bank->tzen == TZEN_ENABLED && stm32h5x_bank->pstate == PSTATE_OPEN) {
		stm32h5x_bank->flash_regs_base |= STM32_REGS_SEC_OFFSET;
		stm32h5x_bank->flash_regs = stm32h5x_s_flash_regs;
	}

	if (dev_info->flags & F_HAS_TZ)
		LOG_INFO("TZEN = 0x%02X : TrustZone %s by option bytes",
				stm32h5x_bank->tzen,
				stm32h5x_bank->tzen == TZEN_ENABLED ? "enabled" : "disabled");

	LOG_INFO("Product State = 0x%02X : '%s'",
			stm32h5x_bank->pstate, stm32h5x_product_state_str(stm32h5x_bank->pstate));

	if (bank->base != STM32_FLASH_BANK_BASE && bank->base != STM32_FLASH_S_BANK_BASE) {
		LOG_ERROR("invalid bank base address");
		return ERROR_FAIL;
	}

	/* Get flash size from target. */
	uint16_t flash_size_kb = 0xffff;
	retval = target_read_u16(target, dev_info->flash_size_addr, &flash_size_kb);

	/* If failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_kb == 0xffff || flash_size_kb == 0
			|| flash_size_kb > dev_info->max_flash_size_kb) {
		LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming %dk flash",
				dev_info->max_flash_size_kb);
		flash_size_kb = dev_info->max_flash_size_kb;
	}

	/* If the user sets the size manually then ignore the probed value,
	 * this allows us to work around devices that have a invalid flash size value */
	if (stm32h5x_bank->user_bank_size) {
		LOG_WARNING("overriding size register by configured bank size - MAY CAUSE TROUBLE");
		flash_size_kb = stm32h5x_bank->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %dkbytes", flash_size_kb);

	/* Did we assign a flash size? */
	assert((flash_size_kb != 0xffff) && flash_size_kb);


	stm32h5x_bank->dual_bank = false;
	stm32h5x_bank->bank1_sectors = 0;
	int num_sectors = 0;
	int sector_size_kb = 0;

	switch (device_id) {
	case DEVID_STM32H50XX:
	case DEVID_STM32H56_H57XX:
		/* Always dual-bank */
		stm32h5x_bank->dual_bank = true;
		sector_size_kb = 8;
		num_sectors = flash_size_kb / sector_size_kb;
		stm32h5x_bank->bank1_sectors = num_sectors / 2;
		break;
	default:
		LOG_ERROR("unsupported device");
		return ERROR_FAIL;
	}

	LOG_INFO("flash mode : %s-bank", get_stm32h5x_bank_type_str(bank));

	/* Initialize flash infrastructure */
	free(bank->sectors);
	bank->size = flash_size_kb * 1024;
	bank->num_sectors = num_sectors;
	bank->sectors = alloc_block_array(0, sector_size_kb * 1024, bank->num_sectors);

	if (!bank->sectors) {
		LOG_ERROR("failed to allocate bank sectors");
		return ERROR_FAIL;
	}

	/* setup protection blocks */
	stm32h5x_bank->bank1_prot_blocks = stm32h5x_bank->bank1_sectors;
	const uint32_t wpsn = dev_info->wps_group_size;
	assert(bank->num_sectors % wpsn == 0);

	bank->num_prot_blocks = bank->num_sectors / wpsn;
	assert(bank->num_prot_blocks > 0);

	stm32h5x_bank->bank1_prot_blocks = bank->num_prot_blocks / 2;

	free(bank->prot_blocks);

	bank->prot_blocks = alloc_block_array(0, sector_size_kb * wpsn * 1024,
			bank->num_prot_blocks);

	if (!bank->prot_blocks) {
		LOG_ERROR("failed to allocate bank prot_blocks");
		return ERROR_FAIL;
	}

	stm32h5x_bank->probed = true;
	return ERROR_OK;
}

static int stm32h5x_auto_probe(struct flash_bank *bank)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	if (stm32h5x_bank->probed) {
		uint32_t optsr_cur, optsr2_cur;

		/* Read flash option register and re-probe if optsr value is changed */
		int retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_OPTSR_CUR_INDEX, &optsr_cur);
		if (retval != ERROR_OK)
			return retval;

		if (stm32h5x_bank->optsr_cur == optsr_cur)
			return ERROR_OK;

		/* Read flash option register and re-probe if optsr2 value is changed */
		retval = stm32h5x_read_flash_reg_by_index(bank, STM32_FLASH_OPTSR2_CUR_INDEX, &optsr2_cur);
		if (retval != ERROR_OK)
			return retval;

		if (stm32h5x_bank->optsr_cur == optsr_cur && stm32h5x_bank->optsr2_cur == optsr2_cur)
			return ERROR_OK;
	}

	return stm32h5x_probe(bank);
}

static int get_stm32h5x_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct stm32h5x_flash_bank *stm32h5x_bank = bank->driver_priv;
	const struct stm32h5x_dev_info *dev_info = stm32h5x_bank->dev_info;

	if (dev_info) {
		const uint16_t rev_id = stm32h5x_bank->idcode >> 16;
		command_print_sameline(cmd, "%s - Rev %s : 0x%04x", get_stm32h5x_device_str(bank),
				get_stm32h5x_rev_str(bank), rev_id);
		if (stm32h5x_bank->probed)
			command_print_sameline(cmd, " - Flash %s-bank", get_stm32h5x_bank_type_str(bank));
	} else {
		command_print_sameline(cmd, "Cannot identify target as an %s device", supported_devices_desc);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(stm32h5x_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32h5x_mass_erase(bank, STM32_ALL_BANKS);
	if (retval == ERROR_OK)
		command_print(CMD, "mass erase complete");
	else
		command_print(CMD, "mass erase failed");

	return retval;
}

COMMAND_HANDLER(stm32h5x_handle_option_read_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	uint32_t reg_offset, reg_addr;
	uint32_t value = 0;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], reg_offset);
	reg_addr = stm32h5x_get_flash_reg(bank, reg_offset);

	retval = stm32h5x_read_flash_reg(bank, reg_offset, &value);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "Option Register: <0x%" PRIx32 "> = 0x%" PRIx32 "", reg_addr, value);

	return retval;
}

COMMAND_HANDLER(stm32h5x_handle_option_write_command)
{
	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	uint32_t reg_offset;
	uint32_t value = 0;
	uint32_t mask = 0xFFFFFFFF;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], reg_offset);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], value);

	if (CMD_ARGC > 3)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], mask);

	command_print(CMD, "%s Option written.\n"
				"INFO: a reset or power cycle is required "
				"for the new settings to take effect.", bank->driver->name);

	retval = stm32h5x_write_option(bank, reg_offset, value, mask);
	return retval;
}

COMMAND_HANDLER(stm32h5x_handle_trustzone_command)
{
	/* TODO */
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

COMMAND_HANDLER(stm32h5x_handle_option_load_command)
{
	/* TODO */
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

static const struct command_registration stm32h5x_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = stm32h5x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "option_read",
		.handler = stm32h5x_handle_option_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id reg_offset",
		.help = "Read & Display device option bytes.",
	},
	{
		.name = "option_write",
		.handler = stm32h5x_handle_option_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id reg_offset value mask",
		.help = "Write device option bit fields with provided value.",
	},
	{
		.name = "trustzone",
		.handler = stm32h5x_handle_trustzone_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank_id> [enable|disable]",
		.help = "Configure TrustZone security",
	},
	{
		.name = "option_load",
		.handler = stm32h5x_handle_option_load_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Force re-load of device options (will cause device reset).",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32h5x_command_handlers[] = {
	{
		.name = "stm32h5x",
		.mode = COMMAND_ANY,
		.help = "stm32h5x flash command group",
		.usage = "",
		.chain = stm32h5x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver stm32h5x_flash = {
	.name = "stm32h5x",
	.commands = stm32h5x_command_handlers,
	.flash_bank_command = stm32h5x_flash_bank_command,
	.erase = stm32h5x_erase,
	.protect = stm32h5x_protect,
	.write = stm32h5x_write,
	.read = stm32h5x_read,
	.probe = stm32h5x_probe,
	.auto_probe = stm32h5x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stm32h5x_protect_check,
	.info = get_stm32h5x_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

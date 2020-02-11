/***************************************************************************
 *   Flash drivers for Analog Devices ADuCM302x and ADuCM4x50              *
 *   Modified from stellaris.c                                             *
 *   Copyright (C) 2014 - 2018 Analog Devices, Inc.                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <helper/time_support.h>


/* ADuCM302x ID registers */
#define SYS_CHIPID    0x40002024

#define ADUCM4X50_CHIPID 0x2a0

#define FLASH_CTRL_BASE 0x40018000
/* ADuCM302x cache flash control registers */
#define STAT      0x40018000
#define IEN       0x40018004
#define CMND       0x40018008
#define KH_ADDR     0x4001800c
#define KH_DATA0    0x40018010
#define KH_DATA1    0x40018014
#define PAGE_ADDR0    0x40018018
#define PAGE_ADDR1    0x4001801c
#define KEY       0x40018020
#define WR_ABORT_ADDR 0x40018024
#define WRPROT      0x40018028
#define SIGNATURE   0x4001802c
#define UCFG      0x40018030
#define TIME_PARAM0   0x40018034
#define TIME_PARAM1   0x40018038
#define ABORT_EN_LO   0x4001803c
#define ABORT_EN_HI   0x40018040
#define ECC_CFG     0x40018044
#define ECC_ADDR    0x40018048
#define VOL_CFG     0x40018054

#define USER_KEY    0x676c7565

#define STAT_CMDBUSY        (1 << 0)
#define STAT_WRCLOSE        (1 << 1)
#define STAT_CMDCOMP        (1 << 2)
#define STAT_WRALCOMP       (1 << 3)
#define STAT_CMDFAIL_MASK     (3 << 4)
#define STAT_CMDFAIL_SUCCESS    (0 << 4)
#define STAT_CMDFAIL_IGNORED    (1 << 4)
#define STAT_CMDFAIL_VERIFY_ERR   (2 << 4)
#define STAT_CMDFAIL_ABORT      (3 << 4)
#define STAT_SLEEPING       (1 << 6)
#define STAT_ECCERRCMD_MASK     (3 << 7)
#define STAT_ECCERRCMD_SUCCESS    (0 << 7)
#define STAT_ECCERRCMD_ERR_2BIT   (1 << 7)
#define STAT_ECCERRCMD_ERR_1BIT   (2 << 7)
#define STAT_ECCERRCMD_ERR_1OR2   (3 << 7)
#define STAT_ECCRDERR_MASK      (3 << 9)
#define STAT_ECCRDERR_SUCCESS   (0 << 9)
#define STAT_ECCRDERR_ERR_2BIT    (1 << 9)
#define STAT_ECCRDERR_ERR_1BIT    (2 << 9)
#define STAT_ECCRDERR_ERR_1OR2    (3 << 9)
#define STAT_OVERLAP        (1 << 11)
#define STAT_SIGNERR        (1 << 13)
#define STAT_INIT         (1 << 14)
#define STAT_ECCINFOSIGN_MASK   (2 << 15)
#define STAT_ECCINFOSIGN_SUCCESS  (0 << 15)
#define STAT_ECCINFOSIGN_ERR_2BIT (1 << 15)
#define STAT_ECCINFOSIGN_ERR_1BIT (2 << 15)
#define STAT_ECCINFOSIGN_ERR_1OR2 (3 << 15)
#define STAT_ECCERRCNT_MASK     (7 << 17)
#define STAT_ECCICODE_MASK      (3 << 25)
#define STAT_ECCICODE_SUCCESS   (0 << 25)
#define STAT_ECCICODE_ERR_2BIT    (1 << 25)
#define STAT_ECCICODE_ERR_1BIT    (2 << 25)
#define STAT_ECCDCODE_MASK      (3 << 27)
#define STAT_ECCDCODE_SUCCESS   (0 << 27)
#define STAT_ECCDCODE_ERR_2BIT    (1 << 27)
#define STAT_ECCDCODE_ERR_1BIT    (2 << 27)
#define STAT_CACHESRAMPERR      (1 << 29)

#define ECC_CFG_EN          (1 << 0)
#define ECC_CFG_INFOEN        (1 << 1)

#define TIME_PARAM0_TERASE_POS    24
#define TIME_PARAM0_TERASE_MASK   (0xf << TIME_PARAM0_TERASE_POS)

#define VOL_CFG_INFO_REMAP      (1 << 0)

#define CMD_IDLE    0
#define CMD_ABORT   1
#define CMD_SLEEP   2
#define CMD_SIGN    3
#define CMD_WRITE   4
#define CMD_CHECK   5
#define CMD_ERASEPAGE 6
#define CMD_MASSERASE 7

/* Flash timeout in ms */
#define FLASH_TIMEOUT 200

struct aducm302x_flash_bank {
  /* ADuCM302x or ADuCM4x50 */
  bool is_aducm302x;
  bool probed;

  /* flash geometry */
  uint32_t pagesize;
  /* how many pages in one protect block */
  int pages_per_block;
};

static int aducm302x_probe(struct flash_bank *bank)
{
  struct target *target = bank->target;
  struct aducm302x_flash_bank *aducm302x_info = bank->driver_priv;
  uint16_t chipid;

  LOG_DEBUG("bank=%p", bank);

  if (aducm302x_info->probed)
    return ERROR_OK;

  aducm302x_info->pagesize = 2048;
  bank->num_sectors = bank->size / aducm302x_info->pagesize;
  bank->sectors = alloc_block_array(0, aducm302x_info->pagesize, bank->num_sectors);
  if (bank->sectors == NULL) {
    LOG_ERROR("failed to allocate block array");
    return ERROR_FAIL;
  }

  bank->num_prot_blocks = bank->num_sectors / aducm302x_info->pages_per_block;
  uint32_t block_size = aducm302x_info->pagesize * aducm302x_info->pages_per_block;
  bank->prot_blocks = alloc_block_array(0, block_size, bank->num_prot_blocks);
  if (bank->prot_blocks == NULL) {
    LOG_ERROR("failed to allocate block array");
    return ERROR_FAIL;
  }

  target_read_u16(target, SYS_CHIPID, &chipid);

  /* The recommended TERASE value in TIME_PARAM0 is now 0x9 for ADuCM4x50 */
  if ((chipid & 0xfff0) == ADUCM4X50_CHIPID) {
    uint32_t time_param0;

    target_read_u32(target, TIME_PARAM0, &time_param0);

    time_param0 &= ~TIME_PARAM0_TERASE_MASK;
    time_param0 |= 0x9 << TIME_PARAM0_TERASE_POS;

    target_write_u32(target, KEY, USER_KEY);
    target_write_u32(target, TIME_PARAM0, time_param0);
    /* Invalidate User Key */
    target_write_u32(target, KEY, 0);
  }

  /* Clear existing errors */
  target_write_u32(target, STAT, STAT_CMDFAIL_MASK);

  bank->write_start_alignment = 8;
  bank->write_end_alignment = 8;
  bank->default_padded_value = 0xff;

  aducm302x_info->probed = true;

  return ERROR_OK;
}

static int aducm302x_check_cmdfail(struct target *target)
{
  uint32_t flash_stat;
  int64_t start_time = timeval_ms();

  do {
    target_read_u32(target, STAT, &flash_stat);
  } while ((flash_stat & STAT_CMDCOMP) == 0 && timeval_ms() - start_time < FLASH_TIMEOUT);

  if ((flash_stat & STAT_CMDCOMP) == 0) {
    LOG_ERROR("command time out");
    return ERROR_FLASH_OPERATION_FAILED;
  }

  if ((flash_stat & STAT_CMDFAIL_MASK) == STAT_CMDFAIL_SUCCESS)
    return ERROR_OK;

  if ((flash_stat & STAT_CMDFAIL_MASK) == STAT_CMDFAIL_IGNORED)
    LOG_ERROR("command ignored for attempted access of a protected or out of memory location)");
  else if ((flash_stat & STAT_CMDFAIL_MASK) == STAT_CMDFAIL_VERIFY_ERR)
    LOG_ERROR("verify error occurred for failed erase or failed signature check");
  else if ((flash_stat & STAT_CMDFAIL_MASK) == STAT_CMDFAIL_ABORT)
    LOG_ERROR("command aborted by either user code or a system interrupt");

  /* Clear error after reporting it */
  target_write_u32(target, STAT, flash_stat & STAT_CMDFAIL_MASK);

  return ERROR_FLASH_OPERATION_FAILED;
}

static int aducm302x_mass_erase(struct flash_bank *bank)
{
  struct target *target = bank->target;
  int retval;

  /* Write user key */
  target_write_u32(target, KEY, USER_KEY);
  /* Write massive erase command */
  target_write_u32(target, CMND, CMD_MASSERASE);

  retval = aducm302x_check_cmdfail(target);
  if (retval != ERROR_OK)
    return retval;

  return ERROR_OK;
}

static int aducm302x_erase(struct flash_bank *bank, int first, int last)
{
  struct aducm302x_flash_bank *aducm302x_info = bank->driver_priv;
  struct target *target = bank->target;
  int retval, i;

  LOG_DEBUG("bank=%p first=%d last = %d", bank, first, last);

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  if (!aducm302x_info->probed)
    return ERROR_FLASH_BANK_NOT_PROBED;

  if (first < 0 || last < first || last >= bank->num_sectors)
    return ERROR_FLASH_SECTOR_INVALID;

  if (first == 0 && last == bank->num_sectors - 1) {
    retval = aducm302x_mass_erase(bank);
    return retval;
  }

  for (i = first; i <= last; i++) {
    /* Address is first word in page */
    target_write_u32(target, PAGE_ADDR0, i * aducm302x_info->pagesize);
    /* Write user key */
    target_write_u32(target, KEY, USER_KEY);
    /* Write page erase command */
    target_write_u32(target, CMND, CMD_ERASEPAGE);

    retval = aducm302x_check_cmdfail(target);
    if (retval != ERROR_OK)
      return retval;
  }

  return ERROR_OK;
}

static int aducm302x_protect(struct flash_bank *bank, int set, int first, int last)
{
  struct aducm302x_flash_bank *aducm302x_info = bank->driver_priv;
  struct target *target = bank->target;
  uint32_t wrprot;
  int i;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  if (!set) {
    LOG_ERROR("Hardware doesn't support page-level unprotect");
    return ERROR_COMMAND_SYNTAX_ERROR;
  }

  if (!aducm302x_info->probed)
    return ERROR_FLASH_BANK_NOT_PROBED;

  if (first < 0 || last >= bank->num_prot_blocks) {
    LOG_ERROR("Can't protect out-of-range pages.");
    return ERROR_FLASH_SECTOR_INVALID;
  }

  target_read_u32(target, WRPROT, &wrprot);

  for (i = first; i <= last; i++)
    wrprot &= ~(1 << i);

  LOG_DEBUG("WRPROT 0x%"PRIx32, wrprot);

  target_write_u32(target, KEY, USER_KEY);
  target_write_u32(target, WRPROT, wrprot);
  target_write_u32(target, KEY, 0);

  return ERROR_OK;
}

static int aducm302x_protect_check(struct flash_bank *bank)
{
  struct aducm302x_flash_bank *aducm302x_info = bank->driver_priv;
  struct target *target = bank->target;
  uint32_t wrprot;
  int i;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  if (!aducm302x_info->probed)
    return ERROR_FLASH_BANK_NOT_PROBED;

  target_read_u32(target, WRPROT, &wrprot);

  for (i = 0; i < bank->num_prot_blocks; i++)
    bank->prot_blocks[i].is_protected = !(wrprot & (1 << i));

  return ERROR_OK;
}

static const uint8_t aducm302x_write_code[] = {
#include "../../../contrib/loaders/flash/aducm302x/aducm302x.inc"
};

static int aducm302x_write_block(struct flash_bank *bank, const uint8_t *buffer,
                 uint32_t offset, uint32_t dwcount)
{
  struct target *target = bank->target;
  uint32_t buffer_size = 16384 + 8; /* 8 bytes for wp and rp */
  struct working_area *source;
  struct working_area *write_algorithm;
  uint32_t address = bank->base + offset;
  struct reg_param reg_params[6];
  struct armv7m_algorithm armv7m_info;
  int retval;

  /* power of two, and multiple of 8 bytes */
  static const unsigned buf_min = 128;

  LOG_DEBUG("bank=%p buffer=%p offset=%08"PRIx32" dwcount=%"PRIx32,
        bank, buffer, offset, dwcount);

  /* For small buffers it's faster not to download the algorithm */
  if (dwcount * 8 < buf_min)
    return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

  /* flash write code */
  if (target_alloc_working_area(target, sizeof(aducm302x_write_code),
                  &write_algorithm) != ERROR_OK) {
    LOG_DEBUG("no working area for block memory writes");
    return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
  }

  /* plus a buffer big enough for this data */
  if (dwcount * 8 < buffer_size)
    buffer_size = dwcount * 8 + 8;

  /* memory buffer */
  while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
    buffer_size = (buffer_size - 8) / 2;
    /* make sure it's multiple of 8 bytes */
    buffer_size = buffer_size / 8 * 8;
    if (buffer_size <= buf_min) {
      target_free_working_area(target, write_algorithm);
      return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }
    /* 8 bytes for wp and rp */
    buffer_size += 8;
    LOG_DEBUG("retry target_alloc_working_area(%s, size=%"PRIu32")",
          target_name(target), buffer_size);
  }

  target_write_buffer(target, write_algorithm->address,
            sizeof(aducm302x_write_code), aducm302x_write_code);

  armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
  armv7m_info.core_mode = ARM_MODE_THREAD;

  init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
  init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
  init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
  init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
  init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);
  init_reg_param(&reg_params[5], "r5", 32, PARAM_OUT);

  buf_set_u32(reg_params[0].value, 0, 32, source->address);
  buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
  buf_set_u32(reg_params[2].value, 0, 32, address);
  buf_set_u32(reg_params[3].value, 0, 32, dwcount);
  buf_set_u32(reg_params[4].value, 0, 32, FLASH_CTRL_BASE);
  buf_set_u32(reg_params[5].value, 0, 32, CMD_WRITE);

  retval = target_run_flash_async_algorithm(target,
                        buffer, dwcount, 8,
                        0, NULL,
                        6, reg_params,
                        source->address, source->size,
                        write_algorithm->address, 0,
                        &armv7m_info);

  if (retval == ERROR_FLASH_OPERATION_FAILED)
    LOG_ERROR("error %d executing ADuCM302x flash write algorithm", retval);

  target_free_working_area(target, write_algorithm);
  target_free_working_area(target, source);

  destroy_reg_param(&reg_params[0]);
  destroy_reg_param(&reg_params[1]);
  destroy_reg_param(&reg_params[2]);
  destroy_reg_param(&reg_params[3]);
  destroy_reg_param(&reg_params[4]);
  destroy_reg_param(&reg_params[5]);

  return retval;
}

static int aducm302x_write(struct flash_bank *bank, const uint8_t *buffer,
               uint32_t offset, uint32_t count)
{
  struct aducm302x_flash_bank *aducm302x_info = bank->driver_priv;
  struct target *target = bank->target;
  uint32_t address = offset;
  uint32_t dwords_remaining;
  uint32_t ecc_cfg;
  bool ecc_cfg_modified;
  int retval = ERROR_OK;

  LOG_DEBUG("bank=%p buffer=%p offset=%08"PRIx32" count=%"PRIx32,
        bank, buffer, offset, count);

  if (bank->target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  if (!aducm302x_info->probed)
    return ERROR_FLASH_BANK_NOT_PROBED;

  /* We need to enable it before programming the flash such that when ECC
     is enabled later, it will not cause an ECC error. ECC_CFG_PTR is also
     forced to be 0. */

  target_read_u32(target, ECC_CFG, &ecc_cfg);
  if (ecc_cfg == (ECC_CFG_INFOEN | ECC_CFG_EN)) {
    ecc_cfg_modified = false;
  } else {
    target_write_u32(target, ECC_CFG, ECC_CFG_INFOEN | ECC_CFG_EN);
    ecc_cfg_modified = true;
  }

  /* The infrastructure should have already handled alignment for us */
  assert((offset & 0x7) == 0);
  assert((count % 8) == 0);

  dwords_remaining = count / 8;

  if (dwords_remaining > 0) {
    retval = aducm302x_write_block(bank, buffer, offset, dwords_remaining);
    if (retval != ERROR_OK) {
      if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
        LOG_DEBUG("writing flash word-at-a-time");
      else if (retval == ERROR_FLASH_OPERATION_FAILED) {
        LOG_ERROR("flash writing failed");
        retval = ERROR_FLASH_OPERATION_FAILED;
        goto finish;
      }
    } else {
      buffer += dwords_remaining * 8;
      address += dwords_remaining * 8;
      dwords_remaining = 0;
    }
  }

  while (dwords_remaining > 0) {
    target_write_u32(target, KH_ADDR, address);
    target_write_buffer(target, KH_DATA0, 4, buffer);
    target_write_buffer(target, KH_DATA1, 4, buffer + 4);
    target_write_u32(target, CMND, CMD_WRITE);

    retval = aducm302x_check_cmdfail(target);
    if (retval != ERROR_OK)
      goto finish;

    buffer += 8;
    address += 8;
    dwords_remaining--;
  }

finish:

  /* restore ECC_CFG if it was modified */
  if (ecc_cfg_modified)
    target_write_u32(target, ECC_CFG, ecc_cfg);

  return retval;
}

FLASH_BANK_COMMAND_HANDLER(aducm302x_flash_bank_command)
{
  struct aducm302x_flash_bank *aducm302x_info;

  if (CMD_ARGC < 6)
    return ERROR_COMMAND_SYNTAX_ERROR;

  aducm302x_info = calloc(sizeof(struct aducm302x_flash_bank), 1);
  if (aducm302x_info == NULL) {
    LOG_ERROR("calloc failed");
    return ERROR_FAIL;
  }

  bank->base = 0x0;
  bank->driver_priv = aducm302x_info;

  aducm302x_info->is_aducm302x = true;

  /* part wasn't probed for info yet */
  aducm302x_info->probed = false;

  /* ADuCM302x flash has 4 pages in one protect block */
  aducm302x_info->pages_per_block = 4;

  return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(aducm4x50_flash_bank_command)
{
  struct aducm302x_flash_bank *aducm302x_info;

  if (CMD_ARGC < 6)
    return ERROR_COMMAND_SYNTAX_ERROR;

  aducm302x_info = calloc(sizeof(struct aducm302x_flash_bank), 1);
  if (aducm302x_info == NULL) {
    LOG_ERROR("calloc failed");
    return ERROR_FAIL;
  }

  bank->base = 0x0;
  bank->driver_priv = aducm302x_info;

  aducm302x_info->is_aducm302x = false;

  /* part wasn't probed for info yet */
  aducm302x_info->probed = false;

  /* ADuCM4x50 flash has 8 pages in one protect block */
  aducm302x_info->pages_per_block = 8;

  return ERROR_OK;
}

const struct flash_driver aducm302x_flash = {
  .name = "aducm302x",
  .usage = NULL,
  .commands = NULL,
  .flash_bank_command = aducm302x_flash_bank_command,
  .erase = aducm302x_erase,
  .protect = aducm302x_protect,
  .write = aducm302x_write,
  .read = default_flash_read,
  .probe = aducm302x_probe,
  .erase_check = default_flash_blank_check,
  .protect_check = aducm302x_protect_check,
  .info = NULL,
  .auto_probe = aducm302x_probe,
  .free_driver_priv = default_flash_free_driver_priv,
};

const struct flash_driver aducm4x50_flash = {
  .name = "aducm4x50",
  .usage = NULL,
  .commands = NULL,
  .flash_bank_command = aducm4x50_flash_bank_command,
  .erase = aducm302x_erase,
  .protect = aducm302x_protect,
  .write = aducm302x_write,
  .read = default_flash_read,
  .probe = aducm302x_probe,
  .erase_check = default_flash_blank_check,
  .protect_check = aducm302x_protect_check,
  .info = NULL,
  .auto_probe = aducm302x_probe,
  .free_driver_priv = default_flash_free_driver_priv,
};
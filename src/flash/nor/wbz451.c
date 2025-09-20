// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2025 by Microchip Technologies Inc                      *
 *   Author: Dinesh Arasu - dinesh.arasu@microchip.com                     *
 *                                                                         *
 *   Description: Flash driver for WBZ451 Microchip Curiosity Board        *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "helper/binarybuffer.h"
#include <helper/time_support.h>
#include <jtag/jtag.h>
#include <target/cortex_m.h>
#include <stdbool.h>
#include <string.h>
#include <target/target_type.h>

#define WBZ451_FLASH        0x01000000
#define WBZ451_NVMDATA      0x44000640
#define WBZ451_NVMADDR      0x44000630
#define WBZ451_NVMCON       0x44000600
#define WBZ451_NVMKEY       0x44000620
#define WBZ451_NVMCONSET    0x44000608
#define WBZ451_NVMCONCLR    0x44000604
#define WBZ451_NVMSRCADDR   0x440006C0
#define WBZ451_PAC          0x40000000
#define WBZ451_NVM_PARAM    0x44000610
#define WBZ451_NVMLBWP      0x440006F0
#define WBZ451_NVMLBWP_UNLOCK_KEY       0x80000000

#define NVMCON_NVMWREN          (1 << 14)
#define NVMCON_NVMWR            (1 << 15)
#define WBZ451_NVM_ERR_MASK     0x3F00

#define NVMCON_OP_WORD_PROG     0x4001
#define NVMCON_OP_ROW_PROG      0x4003
#define NVMCON_OP_PAGE_ERASE    0x4004
#define NVMCON_OP_PBC           0x4007
#define WBZ451_NVMCON_OP_MASK   0x7FFF

#define RAM_BUF_ADDR 0x20000000
#define ROW_SIZE 1024                                                   // Row Size - 1KB
#define WBZ451_FLASH_END (0x01000000U + 1024U * 1024U)                  // Flash Size - Flash End Address

struct wbz451_info {
    struct target *target;
    bool probed;
    uint32_t page_size;
    uint32_t num_pages;
};

static int wbz451_unlock_flash(struct target *t) {
    uint32_t status;
    int res = target_read_u32(t, WBZ451_PAC + 0x18, &status);
    if (res != ERROR_OK)
        return res;

    if (status & (1 << 1)) {
        LOG_INFO("PAC indicates NVMCTRL is locked. Attempting to unlock...");
        res = target_write_u32(t, WBZ451_PAC + 0x20, (1 << 1));
        if (res != ERROR_OK)
            return res;
    }

    res = target_read_u32(t, WBZ451_PAC + 0x18, &status);

    return res;
}

static int wbz451_issue_nvmcmd(struct target *t, uint32_t flash_addr, uint16_t cmd, uint32_t src_addr)
{
    int res;

    if(flash_addr < WBZ451_FLASH)  //Boot Flash, Device Config, Config Bits, OTP unlock sequence
    {
        res = target_write_u32(t, WBZ451_NVMLBWP, WBZ451_NVMLBWP_UNLOCK_KEY);
        if (res != ERROR_OK) return res;
    }
    else{
        // Step 1: Unlock NVMCTRL PAC (already does write to PAC/unlocks)
        res = wbz451_unlock_flash(t);
        if (res != ERROR_OK)
            return res;
    }

    // Step 2: Clear previous error flags (NVMERR, BORERR)
    res = target_write_u32(t, WBZ451_NVMCONCLR, WBZ451_NVM_ERR_MASK);
    if (res != ERROR_OK)
        return res;
    
    // Row alignment
    flash_addr &= ~(ROW_SIZE - 1);

    // Step 3: Set NVMSRCADDR if command is ROW_PROG
    if (cmd == NVMCON_OP_ROW_PROG) {
        res = target_write_u32(t, WBZ451_NVMSRCADDR, src_addr);
        if (res != ERROR_OK)
            return res;
    }

    // Step 4: Set NVMADDR with the destination Flash address
    res = target_write_u32(t, WBZ451_NVMADDR, flash_addr);
    if (res != ERROR_OK)
        return res;

    // Step 5: Set WREN and operation code in NVMCON
    res = target_write_u32(t, WBZ451_NVMCON, NVMCON_NVMWREN | (cmd & WBZ451_NVMCON_OP_MASK));
    if (res != ERROR_OK)
        return res;

    // Step 6: NVMKEY unlock sequence
    res = target_write_u32(t, WBZ451_NVMKEY, 0x00000000);       // Reset
    if (res != ERROR_OK) return res;
    res = target_write_u32(t, WBZ451_NVMKEY, 0xAA996655);       // Step 1
    if (res != ERROR_OK) return res;
    res = target_write_u32(t, WBZ451_NVMKEY, 0x556699AA);       // Step 2
    if (res != ERROR_OK) return res;

    // Step 7: Set NVMWR bit to begin operation
    res = target_write_u32(t, WBZ451_NVMCONSET, NVMCON_NVMWR);
    if (res != ERROR_OK)
        return res;

    // Step 8: Wait for NVMWR to clear (operation done)
    uint32_t val;
    int timeout = 10000;
    do {
        res = target_read_u32(t, WBZ451_NVMCON, &val);
        if (res != ERROR_OK)
            return res;

        if (!--timeout) {
            LOG_ERROR("Timeout waiting for NVMWR clear (addr: 0x%08" PRIx32 ", cmd: 0x%X)",
                    flash_addr, cmd);
            return ERROR_FAIL;
        }

        alive_sleep(1);
    } while (val & NVMCON_NVMWR);

    // Step 9: Check error bits
    if (val & WBZ451_NVM_ERR_MASK) {
        LOG_ERROR("NVM error detected (NVMCON=0x%08" PRIx32 ")", val);
        return ERROR_FAIL;
    }

    // Step 10: Clear NVMWREN
    return target_write_u32(t, WBZ451_NVMCONCLR, NVMCON_NVMWREN);
}

static int wbz451_probe(struct flash_bank *bank) {
    struct wbz451_info *info = bank->driver_priv;
    if (!info) return ERROR_FAIL;
    if (info->probed) return ERROR_OK;

    uint32_t base = bank->base;
    uint32_t param = 0;

    target_read_u32(bank->target, WBZ451_NVM_PARAM, &param);

    if (base < 0x00005000) {
        info->page_size = ROW_SIZE;
        info->num_pages = 20;                                           // 20 kB BootFlash
    } 
    else if (base >= 0x00005000 && base < 0x00006000) {
        info->page_size = ROW_SIZE;
        info->num_pages = 4;                                            // 4KB Device Config/Boot (Alias)
    } 
    else if (base >= 0x00006000 && base < 0x00007000) {
        info->page_size = ROW_SIZE;
        info->num_pages = 4;                                            // 4KB OTP Page Alias
    } 
    else if (base >= 0x00045000 && base < 0x00047000) {
        info->page_size = ROW_SIZE;
        info->num_pages = 8;                                            // 8KB ConfigBits
    } 
    else if (base >= 0x01000000 && base < 0x01100000) {
        info->page_size = ROW_SIZE;
        info->num_pages = 1024;                                         // 1MB flash memory
    }
    else if (bank->base == 0xE000ED10){
        info->page_size = ROW_SIZE;
        info->num_pages = 1;                                            
    }

    bank->size = info->page_size * info->num_pages;
    bank->num_sectors = info->num_pages;

    /* Free existing memory to prevent leaks */
    if (bank->sectors) {
        free(bank->sectors);
        bank->sectors = NULL;
    }

    bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));

    if (!bank->sectors)
        return ERROR_FAIL;

    // LOG_INFO("Bank Size 0x%32" PRIx32, bank->size);
    for (unsigned int i = 0; i < bank->num_sectors; ++i) {
        bank->sectors[i].offset = i * info->page_size;
        bank->sectors[i].size = info->page_size;
        bank->sectors[i].is_protected = 0;
    }
    info->probed = true;
    return ERROR_OK;
}

static int wbz451_erase(struct flash_bank *bank, unsigned int first, unsigned int last) {
    struct target *t = bank->target;
    if (t->state != TARGET_HALTED) return ERROR_TARGET_NOT_HALTED;

    for (unsigned int i = first; i <= last; i++) {
        uint32_t addr = bank->base + i * ROW_SIZE;
        int res = wbz451_issue_nvmcmd(t, addr, NVMCON_OP_PAGE_ERASE, 0);
        if (res != ERROR_OK) return res;
    }

    return ERROR_OK;
}

static int wbz451_write(struct flash_bank *bank, const uint8_t *buf, uint32_t offset, uint32_t count)
{
    struct target *target = bank->target;
    uint32_t addr = bank->base + offset;
    const uint32_t end = addr + count;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    for (uint32_t row_addr = addr & ~(ROW_SIZE - 1); row_addr < end; row_addr += ROW_SIZE) {
        uint8_t row_buf[ROW_SIZE];

        // LOG_INFO("Reading row at 0x%08" PRIx32 " for modify", row_addr);

        // Read current row content from flash
        int res = target_read_memory(target, row_addr, 4, ROW_SIZE / 4, row_buf);
        if (res != ERROR_OK) {
            LOG_ERROR("Failed to read flash row at 0x%08" PRIx32, row_addr);
            return res;
        }

        // Merge new data into row buffer
        for (uint32_t i = 0; i < ROW_SIZE; ++i) {
            uint32_t abs_addr = row_addr + i;
            if (abs_addr >= addr && abs_addr < end) {
                row_buf[i] = buf[abs_addr - addr];
            }
        }
        // Write modified row buffer to RAM
        // LOG_INFO("Writing row to RAM buffer at 0x%08" PRIx32, RAM_BUF_ADDR);
        res = target_write_buffer(target, RAM_BUF_ADDR, ROW_SIZE, row_buf);
        if (res != ERROR_OK) {
            LOG_ERROR("Failed to write row buffer to RAM at 0x%08" PRIx32, RAM_BUF_ADDR);
            return res;
        }

        // Program the flash row from RAM
        // LOG_INFO("Programming row at 0x%08" PRIx32, row_addr);
        res = wbz451_issue_nvmcmd(target, row_addr, NVMCON_OP_ROW_PROG, RAM_BUF_ADDR);
        if (res != ERROR_OK) {
            LOG_ERROR("Failed to program row at 0x%08" PRIx32, row_addr);
            return res;
        }

        alive_sleep(2);
    }

    LOG_INFO("WBZ451: Write completed");

    return ERROR_OK;
}

COMMAND_HANDLER(wbz451_handle_dsu_reset_deassert)
{
    struct flash_bank *bank;
    int result = get_flash_bank_by_name("wbz451_flash", &bank);

    if (result != ERROR_OK || !bank)
        return ERROR_FAIL;

    struct target *target = bank->target;

    target_write_u8(target,0x44000001, 0x02);

    target_write_u32(target,0x44000100, 0x8000);

    target_write_u32(target, 0xE000ED0C, 0x05FA0004);

    target_write_u32(target, 0xE000ED0C, 0x05fa0000);

    alive_sleep(100);
    
    return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(wbz451_flash_bank_command) {
    struct wbz451_info *chip = calloc(1, sizeof(*chip));
    if (!chip) return ERROR_FAIL;
    chip->target = bank->target;
    chip->probed = false;
    bank->driver_priv = chip;
    return ERROR_OK;
}

COMMAND_HANDLER(wbz451_handle_erase_page_command) {
    struct target *target = get_current_target(CMD_CTX);
    if (!target || CMD_ARGC != 1) return ERROR_COMMAND_SYNTAX_ERROR;
    uint32_t addr;
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
    return wbz451_issue_nvmcmd(target, addr, NVMCON_OP_PAGE_ERASE, 0);
}

COMMAND_HANDLER(wbz451_handle_write_word_command) {
    struct target *target = get_current_target(CMD_CTX);
    if (!target || CMD_ARGC != 2) return ERROR_COMMAND_SYNTAX_ERROR;
    uint32_t addr, value;
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
    target_write_u32(target, WBZ451_NVMDATA, value);
    return wbz451_issue_nvmcmd(target, addr, NVMCON_OP_WORD_PROG, 0);
}

static const struct command_registration wbz451_exec_command_handlers[] = {
    {
        .name = "erase_page",
        .handler = wbz451_handle_erase_page_command,
        .mode = COMMAND_EXEC,
        .usage = "<address>",
        .help = "Erase a flash page at the given address",
    },
    {
        .name = "write_word",
        .handler = wbz451_handle_write_word_command,
        .mode = COMMAND_EXEC,
        .usage = "<address> <32bit_hex_value>",
        .help = "Write a 32-bit word to flash at the given address",
    },
    {
        .name = "dsu_reset_deassert",
        .handler = wbz451_handle_dsu_reset_deassert,
        .mode = COMMAND_EXEC,
        .usage = "<address> <32bit_hex_value>",
        .help = "WBZ451 custom reset deassert sequence",
    },
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration wbz451_command_handlers[] = {
    {
        .name = "wbz451",
        .mode = COMMAND_ANY,
        .help = "WBZ451 flash command group",
        .usage = "",
        .chain = wbz451_exec_command_handlers,
    },
    COMMAND_REGISTRATION_DONE
};

const struct flash_driver wbz451_flash = {
    .name                = "wbz451",
    .commands            = wbz451_command_handlers,
    .flash_bank_command  = wbz451_flash_bank_command,
    .erase               = wbz451_erase,
    .protect             = NULL,
    .write               = wbz451_write,
    .read                = default_flash_read,
    .probe               = wbz451_probe,
    .auto_probe          = wbz451_probe,
    .erase_check         = default_flash_blank_check,
    .protect_check       = NULL,
    .free_driver_priv    = default_flash_free_driver_priv,
};
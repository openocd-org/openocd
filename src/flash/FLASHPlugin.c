/***************************************************************************
 *   Copyright (C) 2016 by Sysprogs                                        *
 *   sysprogs@sysprogs.com                                                 *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/breakpoints.h>
#include <target/image.h>
#include "advanced_elf_image.h"
#include "nor/imp.h"


struct WorkAreaInfo
{
    uint32_t Address;
    uint32_t Size;
};

struct plugin_flash_bank
{
    int probed;
    struct advanced_elf_image image;
    uint32_t FLASHPlugin_InitDone;
    uint32_t FLASHPlugin_Probe;
    uint32_t FLASHPlugin_FindWorkArea;
    uint32_t FLASHPlugin_EraseSectors;
    uint32_t FLASHPlugin_Unload;
    
    uint32_t FLASHPlugin_ProgramSync;
    uint32_t FLASHPlugin_ProgramAsync;
    uint32_t FLASHPlugin_NotImplemented;
    
    //Optional entries
    uint32_t FLASHPlugin_ProtectSectors;
    uint32_t FLASHPlugin_CheckSectorProtection;
    
    struct WorkAreaInfo WorkArea;
    unsigned int stack_size;
    unsigned int write_block_size;
    char *plugin_file;
};

static int call_plugin_func(struct target *target, int timeout, uint32_t function, uint32_t sp, int32_t *result, int argc, ...);

struct memory_backup
{
    long long base_address;
    int original_section;
    unsigned size;
    void *original_contents;
};

struct plugin_timeouts
{
    unsigned erase;
    unsigned write;
    unsigned init;
    unsigned load;
    unsigned protect;
};

struct image_plugin_timeouts
{
    unsigned size;
    struct plugin_timeouts timeouts;
};

struct loaded_plugin
{
    int region_count;
    uint32_t sp;
    long long entry;
    struct memory_backup *regions;
    struct target *target;
    uint32_t usable_stack_pointer;
    
    struct memory_backup work_area_backup;
    struct plugin_timeouts timeouts;
};

int save_region(struct target *target, struct memory_backup *region, Elf32_Addr sh_addr, Elf32_Size sh_size, int sectionNumber)
{
    region->base_address = sh_addr;
    region->size = sh_size;
    region->original_contents = malloc(sh_size);
    region->original_section = sectionNumber;
        
    return target_read_memory(target, region->base_address, 4, (region->size + 3) / 4, (uint8_t *)region->original_contents);
}

static uint32_t plugin_make_return_addr(uint32_t addr)
{
    return addr | 1;    //ARM-specific: Force THUMB mode
}

static int loaded_plugin_load(struct target *target, struct advanced_elf_image *image, struct loaded_plugin *plugin, uint32_t init_done_address, unsigned stackSize)
{
    int retval;
    memset(plugin, 0, sizeof(*plugin));
    plugin->region_count = 0;
    plugin->regions = (struct memory_backup *)calloc(image->num_sections + 1, sizeof(struct memory_backup));
    plugin->target = target;
    plugin->entry = image->header.e_entry;
    
    plugin->timeouts.erase = 60000;
    plugin->timeouts.write = 1000;
    plugin->timeouts.init = 1000;
    plugin->timeouts.load = 10000;
    plugin->timeouts.protect = 1000;
    
    unsigned lastSectionEnd = 0;
    
    uint32_t timeoutTable = advanced_elf_image_find_symbol(image, "FLASHPlugin_TimeoutTable");
    
    unsigned maxSize = 0;
    
    for (int i = 0; i < image->num_sections; i++)
    {
        if (!(image->sections[i].sh_flags & 2 /*SHF_ALLOC*/))
            continue;
        struct memory_backup *region = &plugin->regions[plugin->region_count];
        retval = save_region(target, region, image->sections[i].sh_addr, image->sections[i].sh_size, i);
        if (retval != ERROR_OK)
            break;
        
        if (region->size > maxSize)
            maxSize = region->size;
        plugin->region_count++;
        
        if ((region->base_address + region->size) > lastSectionEnd)
            lastSectionEnd = region->base_address + region->size;
    }
    
    lastSectionEnd =  ((lastSectionEnd + 15) & ~15);
    plugin->sp = lastSectionEnd + stackSize;
	LOG_DEBUG("FLASH plugin: placing the stack at 0x%08x-0x%08x", lastSectionEnd, lastSectionEnd + stackSize);
    save_region(target, &plugin->regions[plugin->region_count], lastSectionEnd, stackSize, -1);
    plugin->region_count++;
    
    if (retval == ERROR_OK)
    {
        void *pBuf = malloc(maxSize);
        for (int i = 0; i < plugin->region_count; i++)
        {
            if (plugin->regions[i].original_section < 0 || image->sections[plugin->regions[i].original_section].sh_type == 8 /* NOBITS */)
                continue;
            
            size_t done;
            retval = advanced_elf_image_read_section(image, plugin->regions[i].original_section, (uint8_t *)pBuf, plugin->regions[i].size, &done);
            if (retval != ERROR_OK)
                break;
            if (done != plugin->regions[i].size)
            {
                LOG_ERROR("Failed to read FLASH plugin contents\n");
                retval = ERROR_FILEIO_OPERATION_FAILED;
            }
            
            if (timeoutTable && timeoutTable >= plugin->regions[i].base_address && timeoutTable < (plugin->regions[i].base_address + plugin->regions[i].size))
            {
	            void *timeouts_from_image = ((char *)pBuf + (timeoutTable - plugin->regions[i].base_address));
	            uint32_t timeout_struct_size = 0;
	            memcpy(&timeout_struct_size, timeouts_from_image, sizeof(timeout_struct_size));
	            
	            if (timeout_struct_size > sizeof(struct plugin_timeouts))
		            LOG_WARNING("Invalid size of timeouts structure: %d\n", timeout_struct_size);
	            else
		            memcpy(&plugin->timeouts, &((struct image_plugin_timeouts *)timeouts_from_image)->timeouts, timeout_struct_size);
            }
            
            retval = target_write_memory(target, plugin->regions[i].base_address, 4, (plugin->regions[i].size + 3) / 4, (uint8_t *)pBuf);
            if (retval != ERROR_OK)
                break;
        }
        free(pBuf);
    }
    
    if (retval == ERROR_OK && init_done_address)
    {
        struct reg_param reg_params[1];
        init_reg_param(&reg_params[0], "sp", 32, PARAM_IN);
        buf_set_u32(reg_params[0].value, 0, 32, plugin->sp); 
        
        init_done_address &= ~1;
        
        struct armv7m_algorithm armv7m_info;
        armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
        armv7m_info.core_mode = ARM_MODE_THREAD;
        int bp = breakpoint_add(target, init_done_address, 2, BKPT_SOFT);
	    (void)bp;

        retval = target_run_algorithm(target, 0, NULL, sizeof(reg_params) / sizeof(reg_params[0]), reg_params, plugin->entry, init_done_address, plugin->timeouts.load, &armv7m_info);
        breakpoint_remove(target, init_done_address);
        if (retval != ERROR_OK)
            LOG_ERROR("FLASH plugin did not call FLASHPlugin_InitDone(). Ensure it is declared as non-inline and try increasing load timeout setting inside the plugin timeout table.");
        destroy_reg_param(&reg_params[0]);
    }
    
    return retval;
}

static int loaded_plugin_unload(struct loaded_plugin *plugin, uint32_t unload_func)
{
    int retval = ERROR_OK;
    
    if (!plugin->target || !plugin->regions)
        return ERROR_OK;
    
    if (unload_func)
    {
        int32_t result;
        retval = call_plugin_func(plugin->target, plugin->timeouts.init, unload_func, plugin->sp, &result, 0);
        if (retval == ERROR_OK && result != 0)
        {
            LOG_ERROR("Plugin's Unload() function returned error %d\n", result);
            retval = ERROR_FLASH_BANK_INVALID;
        }
    }
    
    for (int i = 0; i < plugin->region_count; i++)
    {
        if (!plugin->regions[i].original_contents)
            continue;
        
        int r = target_write_memory(plugin->target, plugin->regions[i].base_address, 4, (plugin->regions[i].size + 3) / 4, (uint8_t *)plugin->regions[i].original_contents);
        free(plugin->regions[i].original_contents);
        plugin->regions[i].original_contents = NULL;
        
        if (r != ERROR_OK)
            retval = r;
    }
    
    if (plugin->work_area_backup.size && plugin->work_area_backup.original_contents)
    {
        int r = target_write_memory(plugin->target, plugin->work_area_backup.base_address, 4, plugin->work_area_backup.size / 4, plugin->work_area_backup.original_contents);
        free(plugin->work_area_backup.original_contents);
        
        if (r != ERROR_OK)
            retval = r;
    }
    
    free(plugin->regions);
    plugin->regions = NULL;
    
    return retval;
}

static int loaded_plugin_backup_workarea(struct loaded_plugin *plugin, uint32_t start, uint32_t size)
{
    if (!plugin)
        return ERROR_COMMAND_ARGUMENT_INVALID;
    if (plugin->work_area_backup.size && plugin->work_area_backup.base_address != start)
    {
        LOG_ERROR("Inconsistent work area address: expected 0x%x, got 0x%x\n", (uint32_t)plugin->work_area_backup.base_address, start);
        return ERROR_COMMAND_ARGUMENT_INVALID;
    }
    
    size = (size + 3) & ~3;
    
    uint32_t baseoff = plugin->work_area_backup.size;
    if (size <= baseoff)
        return ERROR_OK;    //nothing to do
    
    plugin->work_area_backup.original_contents = realloc(plugin->work_area_backup.original_contents, size);
    int retval = target_read_memory(plugin->target, start + baseoff, 4, size / 4, ((uint8_t  *)plugin->work_area_backup.original_contents) + baseoff);
    if (retval != ERROR_OK)
        return retval;
    
    plugin->work_area_backup.base_address = start;
    plugin->work_area_backup.original_section = -1;
    plugin->work_area_backup.size = size;
    
    return ERROR_OK;
}

static int locate_symbol(struct advanced_elf_image *image, const char *URL, uint32_t *ptr, const char *symbol)
{
    *ptr = advanced_elf_image_find_symbol(image, symbol);
    if (!*ptr)
    {
        LOG_ERROR("%s: invalid FLASH plugin. No '%s' symbol found.", URL, symbol);
        return ERROR_IMAGE_FORMAT_ERROR;
    }
    return ERROR_OK;
}

/* flash bank <name> plugin <base> <size> 0 0 <target#> <plugin ELF file> [stack size = 512]
 */
FLASH_BANK_COMMAND_HANDLER(plugin_flash_bank_command)
{
    struct plugin_flash_bank *info;
    int retval;

    if (CMD_ARGC < 7)
        return ERROR_COMMAND_SYNTAX_ERROR;
    
    unsigned stackSize = 512;
    if (CMD_ARGC >= 8)
    {
        COMMAND_PARSE_NUMBER(uint, CMD_ARGV[7], stackSize);
    }
    
    info = malloc(sizeof(struct plugin_flash_bank));
    memset(info, 0, sizeof(struct plugin_flash_bank));
    
    const char *URL = CMD_ARGV[6];
    
    retval = advanced_elf_image_open(&info->image, URL);
    if (retval != ERROR_OK)
        return retval;
    
    retval = locate_symbol(&info->image, URL, &info->FLASHPlugin_InitDone, "FLASHPlugin_InitDone");
    if (retval != ERROR_OK)
        return retval;
    
    retval = locate_symbol(&info->image, URL, &info->FLASHPlugin_Probe, "FLASHPlugin_Probe");
    if (retval != ERROR_OK)
        return retval;
    
    retval = locate_symbol(&info->image, URL, &info->FLASHPlugin_FindWorkArea, "FLASHPlugin_FindWorkArea");
    if (retval != ERROR_OK)
        return retval;
    
    retval = locate_symbol(&info->image, URL, &info->FLASHPlugin_EraseSectors, "FLASHPlugin_EraseSectors");
    if (retval != ERROR_OK)
        return retval;
    
    retval = locate_symbol(&info->image, URL, &info->FLASHPlugin_Unload, "FLASHPlugin_Unload");
    if (retval != ERROR_OK)
        return retval;
    
    info->FLASHPlugin_ProgramSync = advanced_elf_image_find_symbol(&info->image, "FLASHPlugin_ProgramSync");
    info->FLASHPlugin_ProgramAsync = advanced_elf_image_find_symbol(&info->image, "FLASHPlugin_ProgramAsync");
    info->FLASHPlugin_NotImplemented = advanced_elf_image_find_symbol(&info->image, "FLASHPlugin_NotImplemented");
    
    if (!info->FLASHPlugin_ProgramAsync && !info->FLASHPlugin_ProgramSync)
    {
        LOG_ERROR("%s: invalid FLASH plugin. Neither 'FLASHPlugin_ProgramSync' nor 'FLASHPlugin_ProgramAsync' is defined.", URL);
        return ERROR_IMAGE_FORMAT_ERROR;
    }
    
    info->FLASHPlugin_CheckSectorProtection = advanced_elf_image_find_symbol(&info->image, "FLASHPlugin_CheckSectorProtection");
    info->FLASHPlugin_ProtectSectors = advanced_elf_image_find_symbol(&info->image, "FLASHPlugin_ProtectSectors");
    
  
    info->stack_size = stackSize;
    bank->driver_priv = info;
    info->probed = 0;
    info->plugin_file = strdup(URL);

    return ERROR_OK;
}

int plugin_write_sync(struct target *target,
    struct plugin_flash_bank *plugin_info,
    struct loaded_plugin *loaded_plugin,
    uint32_t offset,
    const uint8_t *buffer,
    uint32_t size)
{
    uint32_t todo = MIN(size, plugin_info->WorkArea.Size);
    int retval = loaded_plugin_backup_workarea(loaded_plugin, plugin_info->WorkArea.Address, todo);
    if (retval != ERROR_OK)
        return -1;
        
    retval = target_write_memory(target, plugin_info->WorkArea.Address, 4, todo / 4, buffer);
    if (retval != ERROR_OK)
        return -1;
    if (todo & 3)
    {
        retval = target_write_memory(target, plugin_info->WorkArea.Address + (todo & ~3), 1, todo & 3, buffer + (todo & ~3));
        if (retval != ERROR_OK)
            return -1;
    }
    
    int32_t result = 0;
    retval = call_plugin_func(target, loaded_plugin->timeouts.write, plugin_info->FLASHPlugin_ProgramSync, loaded_plugin->sp, &result, 3, offset, plugin_info->WorkArea.Address, todo);
    if (retval != ERROR_OK)
        return -1;
    return result;
}

int plugin_write_async(struct target *target,
    struct plugin_flash_bank *plugin_info,
    struct loaded_plugin *loaded_plugin,
    uint32_t offset,
    const uint8_t *buffer,
    uint32_t size)
{
    const int fifo_header_size = 8;
    unsigned workAreaSize = MIN(MAX(size, plugin_info->write_block_size) + plugin_info->write_block_size + fifo_header_size, plugin_info->WorkArea.Size);
    workAreaSize -= fifo_header_size;
    workAreaSize = (workAreaSize / plugin_info->write_block_size) * plugin_info->write_block_size;
    workAreaSize += fifo_header_size;
    if (workAreaSize > plugin_info->WorkArea.Size)
    {
        LOG_ERROR("Computed worka area size (0x%x) is smaller then the available size (0x%x)", workAreaSize, plugin_info->WorkArea.Size);
        return ERROR_FLASH_BANK_INVALID;
    }
    
    int retval = loaded_plugin_backup_workarea(loaded_plugin, plugin_info->WorkArea.Address, workAreaSize);
    if (retval != ERROR_OK)
        return retval;
    
    struct armv7m_algorithm armv7m_info;
    struct reg_param reg_params[6];
    unsigned sp = (loaded_plugin->sp - 4) & ~3;
    unsigned return_addr = sp;
    armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
    armv7m_info.core_mode = ARM_MODE_THREAD;

    init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
    init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
    init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
    init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
    init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);
    init_reg_param(&reg_params[5], "lr", 32, PARAM_OUT);

    buf_set_u32(reg_params[0].value, 0, 32, offset);
    buf_set_u32(reg_params[1].value, 0, 32, plugin_info->WorkArea.Address);
    buf_set_u32(reg_params[2].value, 0, 32, plugin_info->WorkArea.Address + workAreaSize);
    buf_set_u32(reg_params[3].value, 0, 32, size);
    buf_set_u32(reg_params[4].value, 0, 32, sp);
    buf_set_u32(reg_params[5].value, 0, 32, plugin_make_return_addr(return_addr));
    
    int bp = breakpoint_add(target, return_addr, 2, BKPT_SOFT);
	(void)bp;
    
    retval = target_run_flash_async_algorithm(target,
        buffer,
        (size + plugin_info->write_block_size - 1) / plugin_info->write_block_size,
        plugin_info->write_block_size,
        0,
        NULL,
        sizeof(reg_params)/sizeof(reg_params[0]),
        reg_params,
        plugin_info->WorkArea.Address,
        workAreaSize,
        plugin_info->FLASHPlugin_ProgramAsync,
        return_addr,
        &armv7m_info);
    
    breakpoint_remove(target, return_addr);
    unsigned result = 0;
    
    if (retval == ERROR_OK)
    {
        result = buf_get_u32(reg_params[0].value, 0, 32);

        if (result < size)
        {
            LOG_ERROR("FLASHPlugin_ProgramAsync returned %d\n", result);
            retval = ERROR_FLASH_BANK_INVALID;
        }
    }
    
    for (int i = 0; i < 6; i++)
        destroy_reg_param(&reg_params[i]);
    
    return retval;
}

static int plugin_write(struct flash_bank *bank,
    const uint8_t *buffer,
    uint32_t offset,
    uint32_t count)
{
    struct target *target = bank->target;
    struct plugin_flash_bank *plugin_info = bank->driver_priv;
    struct loaded_plugin loaded_plugin;
    
    int retval = loaded_plugin_load(target, &plugin_info->image, &loaded_plugin, plugin_info->FLASHPlugin_InitDone, plugin_info->stack_size);
    if (retval == ERROR_OK)
    {
        if (plugin_info->FLASHPlugin_ProgramAsync && plugin_info->FLASHPlugin_ProgramAsync != plugin_info->FLASHPlugin_NotImplemented)
            retval = plugin_write_async(target, plugin_info, &loaded_plugin, offset, buffer, count);
        else if (plugin_info->FLASHPlugin_ProgramSync && plugin_info->FLASHPlugin_ProgramSync != plugin_info->FLASHPlugin_NotImplemented)
        {
            uint32_t done = 0;
            while (done < count)
            {
                int doneNow = plugin_write_sync(target, plugin_info, &loaded_plugin, offset + done, buffer + done, count - done);
                if (doneNow < 0)
                {
                    retval = ERROR_FLASH_BANK_INVALID;   
                    break;
                }
                
                if (target->report_flash_progress)
                    report_flash_progress("flash_write_progress_sync", bank->base + offset + done, bank->base + offset + done + doneNow, bank->name);

                done += doneNow;
            }
        }
        else
        {
            LOG_ERROR("Neither FLASHPlugin_ProgramAsync() or FLASHPlugin_ProgramSync() are properly defined in the FLASH plugin. Cannot program FLASH memory.");
            retval = ERROR_FLASH_BANK_INVALID;
        }
    }
    
    loaded_plugin_unload(&loaded_plugin, plugin_info->FLASHPlugin_Unload);
    return retval;
}

static int call_plugin_func(struct target *target, int timeout, uint32_t function, uint32_t sp, int32_t *result, int argc, ...)
{
    sp = (sp - 4) & ~3;
    const int r0ParamIndex = 2;
    char *arg_reg_names[] = { "r0", "r1", "r2", "r3" };
    
    uint32_t return_addr = sp;
    struct reg_param reg_params[3 + 4];
    init_reg_param(&reg_params[0], "sp", 32, PARAM_IN_OUT);
    init_reg_param(&reg_params[1], "lr", 32, PARAM_IN_OUT); //ARM-specific!
    init_reg_param(&reg_params[r0ParamIndex], arg_reg_names[0], 32, PARAM_IN_OUT); //ARM-specific!
    buf_set_u32(reg_params[1].value, 0, 32, plugin_make_return_addr(sp));    //Forced thumb mode!
    int reg_param_count = r0ParamIndex;
    
    sp -= 4;
    
    va_list ap;
    va_start(ap, argc);
    for (int arg = 0; arg < argc; arg++)
    {
        uint32_t argVal = va_arg(ap, uint32_t);
        if ((unsigned)arg >= (sizeof(arg_reg_names) / sizeof(arg_reg_names[0])))
        {
            sp -= 4;
            target_write_memory(target, sp, 4, 1, (uint8_t *)&argVal);
        }
        else 
        {
            if (arg == 0)
                reg_params[r0ParamIndex].direction = PARAM_IN_OUT;
            else
            {
                init_reg_param(&reg_params[r0ParamIndex + arg], arg_reg_names[arg], 32, PARAM_IN_OUT);
                reg_param_count = r0ParamIndex + arg + 1;
            }
            
            buf_set_u32(reg_params[r0ParamIndex + arg].value, 0, 32, argVal);
        }
            
    }
    va_end(ap);
    
    if (argc == 0)
        reg_param_count = r0ParamIndex + 1;
    
    buf_set_u32(reg_params[0].value, 0, 32, sp);
    int bp = breakpoint_add(target, return_addr, 2, BKPT_SOFT);
	(void)bp;
        
    struct armv7m_algorithm armv7m_info;
    armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
    armv7m_info.core_mode = ARM_MODE_THREAD;

    int retval = target_run_algorithm(target, 0, NULL, reg_param_count, reg_params, function, return_addr, timeout, &armv7m_info);
    breakpoint_remove(target, return_addr);
    
    if (retval == ERROR_OK && result)
        *result = (int32_t)buf_get_u32(reg_params[r0ParamIndex].value, 0, 32);
    
    for (int i = 0; i < reg_param_count; i++)
        destroy_reg_param(&reg_params[i]);
    
    return retval;
}

struct FLASHBankInfo
{
    unsigned BaseAddress;
    unsigned BlockCount;
    unsigned BlockSize;
    unsigned WriteBlockSize;
};


static int plugin_probe(struct flash_bank *bank)
{
    struct target *target = bank->target;
    struct plugin_flash_bank *plugin_info = bank->driver_priv;
    struct FLASHBankInfo bankInfo;
    struct loaded_plugin loaded_plugin;
    
    plugin_info->probed = 1;
    
    int retval = loaded_plugin_load(target, &plugin_info->image, &loaded_plugin, plugin_info->FLASHPlugin_InitDone, plugin_info->stack_size);
    if (retval == ERROR_OK)
    {
        int32_t result;
        uint32_t sp = (loaded_plugin.sp - 4) & ~3;
        
        sp -= sizeof(struct FLASHBankInfo);
        
	    retval = call_plugin_func(target, loaded_plugin.timeouts.init, plugin_info->FLASHPlugin_Probe, sp, &result, 5, sp, (uint32_t)bank->base, (uint32_t)bank->size, (uint32_t)bank->chip_width, (uint32_t)bank->bus_width);
        if (retval == ERROR_OK)
        {
            retval = target_read_memory(target, sp, 4, sizeof(bankInfo) / 4, (uint8_t *)&bankInfo);
        }
    }
    
    if (plugin_info->FLASHPlugin_FindWorkArea && retval == ERROR_OK)
    {
        int32_t result;
        uint32_t sp = (loaded_plugin.sp - 4) & ~3;
        
        sp -= sizeof(struct WorkAreaInfo);
	    retval = call_plugin_func(target, loaded_plugin.timeouts.init, plugin_info->FLASHPlugin_FindWorkArea, sp, &result, (uint32_t)2, sp, loaded_plugin.sp);
        if (retval == ERROR_OK)
        {
            retval = target_read_memory(target, sp, 4, sizeof(plugin_info->WorkArea) / 4, (uint8_t *)&plugin_info->WorkArea);
        }
    }

    if (retval == ERROR_OK)
    {
        bank->base = bankInfo.BaseAddress;
        bank->size = bankInfo.BlockCount * bankInfo.BlockSize;
        bank->num_sectors = bankInfo.BlockCount;
        bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
        memset(bank->sectors, 0, sizeof(struct flash_sector) * bank->num_sectors);
        plugin_info->write_block_size = bankInfo.WriteBlockSize;
        if (!plugin_info->write_block_size)
        {
            LOG_ERROR("FLASH plugin returned invalid WriteBlockSize. Writing external FLASH will not be possible.");
            retval = ERROR_FLASH_BANK_INVALID;
        }
        
        for (unsigned i = 0; i < bank->num_sectors; i++)
        {
            bank->sectors[i].offset = i * bankInfo.BlockSize;
            bank->sectors[i].size = bankInfo.BlockSize;
        }
    }

    loaded_plugin_unload(&loaded_plugin, plugin_info->FLASHPlugin_Unload);
    

    return retval;
}

static int plugin_auto_probe(struct flash_bank *bank)
{
    struct plugin_flash_bank *plugin_info = bank->driver_priv;
    if (plugin_info->probed)
        return ERROR_OK;
    return plugin_probe(bank);
}

static int get_plugin_info(struct flash_bank *bank, struct command_invocation *cmd)
{
    struct plugin_flash_bank *plugin_info = bank->driver_priv;
    
    command_print(cmd, "Plugin-managed FLASH\r\nPlugin file: %s", plugin_info->plugin_file);
    return ERROR_OK;
}


static int plugin_erase(struct flash_bank *bank, unsigned first, unsigned last)
{
    struct target *target = bank->target;
    struct plugin_flash_bank *plugin_info = bank->driver_priv;
    struct loaded_plugin loaded_plugin;
    
    int retval = loaded_plugin_load(target, &plugin_info->image, &loaded_plugin, plugin_info->FLASHPlugin_InitDone, plugin_info->stack_size);
    if (retval == ERROR_OK)
    {
        while (first <= last)
        {
            int32_t result;
	        retval = call_plugin_func(target, loaded_plugin.timeouts.erase, plugin_info->FLASHPlugin_EraseSectors, loaded_plugin.sp, &result, 2, (uint32_t)first, (uint32_t)last - (uint32_t)first + 1);
            if (retval != ERROR_OK)
                break;
            if (result < 0)
            {
                LOG_ERROR("FLASHPlugin_EraseSectors() returned %d", result);
                retval = ERROR_FLASH_BANK_INVALID;
            }
            
            if (target->report_flash_progress)
                report_flash_progress("flash_erase_progress", bank->base + bank->sectors[first].offset, bank->base + bank->sectors[first + result - 1].offset + bank->sectors[first + result - 1].size, bank->name);
            
            first += result;
        }
    }
    
    loaded_plugin_unload(&loaded_plugin, plugin_info->FLASHPlugin_Unload);
    return retval;
}

static int plugin_protect(struct flash_bank *bank, int set, unsigned first, unsigned last)
{
    struct target *target = bank->target;
    struct plugin_flash_bank *plugin_info = bank->driver_priv;
    struct loaded_plugin loaded_plugin;
    
    if (!plugin_info->FLASHPlugin_ProtectSectors || plugin_info->FLASHPlugin_ProtectSectors == plugin_info->FLASHPlugin_NotImplemented)
        return ERROR_OK;    //Nothing to do
    
    int retval = loaded_plugin_load(target, &plugin_info->image, &loaded_plugin, plugin_info->FLASHPlugin_InitDone, plugin_info->stack_size);
    if (retval == ERROR_OK)
    {
        while (first <= last)
        {
            int32_t result;
	        retval = call_plugin_func(target, loaded_plugin.timeouts.protect, plugin_info->FLASHPlugin_ProtectSectors, loaded_plugin.sp, &result, 3, (uint32_t)set, (uint32_t)first, (uint32_t)last - (uint32_t)first + 1);
            if (retval != ERROR_OK)
                break;
            if (result < 0)
            {
                LOG_ERROR("FLASHPlugin_ProtectSectors() returned %d", result);
                retval = ERROR_FLASH_BANK_INVALID;
            }
            
            first += result;
        }
    }
    
    loaded_plugin_unload(&loaded_plugin, plugin_info->FLASHPlugin_Unload);
    return retval;
}

static int plugin_protect_check(struct flash_bank *bank)
{
    struct target *target = bank->target;
    struct plugin_flash_bank *plugin_info = bank->driver_priv;
    struct loaded_plugin loaded_plugin;
    
    if (!plugin_info->FLASHPlugin_CheckSectorProtection || plugin_info->FLASHPlugin_CheckSectorProtection == plugin_info->FLASHPlugin_NotImplemented)
        return ERROR_OK;    //Nothing to do
    
    unsigned workAreaSize = MIN((bank->num_sectors + 7) / 8, plugin_info->WorkArea.Size);
    uint8_t *pBuf = malloc(workAreaSize);
    
    int retval = loaded_plugin_load(target, &plugin_info->image, &loaded_plugin, plugin_info->FLASHPlugin_InitDone, plugin_info->stack_size);
    if (retval == ERROR_OK)
    {
        retval = loaded_plugin_backup_workarea(&loaded_plugin, plugin_info->WorkArea.Address, workAreaSize);
        if (retval == ERROR_OK)
        {
            for (unsigned sector =  0; sector < bank->num_sectors;)
            {
                int32_t sectors_to_check = MIN(workAreaSize * 8, bank->num_sectors - sector);
                int32_t result;
                retval = call_plugin_func(target, loaded_plugin.timeouts.protect, plugin_info->FLASHPlugin_CheckSectorProtection, loaded_plugin.sp, &result, 3, sector, sectors_to_check, plugin_info->WorkArea.Address);
                if (retval != ERROR_OK)
                    break;
                if (result < 0 || result > sectors_to_check)
                {
                    LOG_ERROR("FLASHPlugin_ProtectSectors() returned %d", result);
                    retval = ERROR_FLASH_BANK_INVALID;
                    break;
                }
                
                retval = target_read_buffer(target, plugin_info->WorkArea.Address,(sectors_to_check + 7) / 8, pBuf);
                if (retval != ERROR_OK)
                    break;
            
                for (int i = 0; i < result; i++)
                {
                    bank->sectors[sector + i].is_protected = pBuf[i / 8] & (1 << (i % 8));
                }
                
                sector += result;
            }
        }
    }
    
    free(pBuf);
    loaded_plugin_unload(&loaded_plugin, plugin_info->FLASHPlugin_Unload);
    return retval;
}
 
struct flash_driver plugin_flash = {
    .name = "plugin",
    .flash_bank_command = plugin_flash_bank_command,
    .erase = plugin_erase,
    .write = plugin_write,
    .read = default_flash_read,
    .probe = plugin_probe,
    .auto_probe = plugin_auto_probe,
    .erase_check = default_flash_blank_check,
    .info = get_plugin_info,
    .protect_check = plugin_protect_check,
    .protect = plugin_protect
};

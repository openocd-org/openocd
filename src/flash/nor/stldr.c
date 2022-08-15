// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022-2026, STMicroelectronics                           *
 *   fedi.bouzazi@st.com                                                   *
 *   ahmed.haoues@st.com                                                   *
 *   tarek.bouchkati@st.com                                                *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <helper/replacements.h>

#ifdef HAVE_ELF_H
#include <elf.h>
#endif

#include <helper/align.h>
#include <target/algorithm.h>
#include <helper/binarybuffer.h>
#include <target/breakpoints.h>
#include <target/cortex_m.h>
#include <sys/stat.h>

#include "imp.h"

// stldr functions return values
#define STLDR_FUNC_FAILURE                 0
#define STLDR_FUNC_SUCCESS                 1

#define STLDR_FUNC_ADDR_UNKNOWN            0xFFFFFFFF

// offsets to mimic STM32CubeProgrammer memory usage
#define STACK_OFFSET                       0x400

// stldr functions timeouts in milliseconds
#define STLDR_INIT_TIMEOUT_MS              100000
#define STLDR_SECTOR_ERASE_TIMEOUT_MS      1000000
#define STLDR_MASS_ERASE_INT_TIMEOUT_MS    10000    // for MCU internal flash
#define STLDR_MASS_ERASE_EXT_TIMEOUT_MS    600000   // for other (external) flash types
#define STLDR_READ_TIMEOUT_MS              1000
#define STLDR_WRITE_TIMEOUT_MS             1000

// size and offset of StorageInfo members
#define STORAGE_INFO_NAME_OFFSET           0
#define STORAGE_INFO_NAME_SIZE             100
#define STORAGE_INFO_TYPE_OFFSET          (STORAGE_INFO_NAME_OFFSET + STORAGE_INFO_NAME_SIZE)
#define STORAGE_INFO_BASE_ADDR_OFFSET     (STORAGE_INFO_TYPE_OFFSET + 4)
#define STORAGE_INFO_SIZE_OFFSET          (STORAGE_INFO_BASE_ADDR_OFFSET + 4)
#define STORAGE_INFO_PAGE_SIZE_OFFSET     (STORAGE_INFO_SIZE_OFFSET + 4)
#define STORAGE_INFO_ERASE_VALUE_OFFSET   (STORAGE_INFO_PAGE_SIZE_OFFSET + 4)
#define STORAGE_INFO_SECTORS_OFFSET       (STORAGE_INFO_ERASE_VALUE_OFFSET + 4)
#define STORAGE_INFO_SECTORS_MAX_ITEMS    10

// size and offset of DeviceSectors members
#define DEVICE_SECTORS_SIZE                8
#define DEVICE_SECTORS_COUNT_OFFSET        0
#define DEVICE_SECTORS_SIZE_OFFSET         4

enum stldr_func_id {
	FUNC_ID_INIT,
	FUNC_ID_READ,
	FUNC_ID_SECTOR_ERASE,
	FUNC_ID_MASS_ERASE,
	FUNC_ID_WRITE,
	FUNC_ID_CHECKSUM,
	FUNC_ID_VERIFY,
	FUNC_ID_COUNT
};

enum stldr_func_type {
	STLDR_FUNC_OPTIONAL,
	STLDR_FUNC_MANDATORY
};

static const struct {
	enum stldr_func_id id;
	const char *name;
	enum stldr_func_type type;
} stldr_functions[] = {
	{ FUNC_ID_INIT,         "Init",        STLDR_FUNC_MANDATORY },
	{ FUNC_ID_READ,         "Read",        STLDR_FUNC_OPTIONAL  },
	{ FUNC_ID_SECTOR_ERASE, "SectorErase", STLDR_FUNC_MANDATORY },
	{ FUNC_ID_MASS_ERASE,   "MassErase",   STLDR_FUNC_OPTIONAL },
	{ FUNC_ID_WRITE,        "Write",       STLDR_FUNC_MANDATORY },
	{ FUNC_ID_CHECKSUM,     "CheckSum",    STLDR_FUNC_OPTIONAL  },
	{ FUNC_ID_VERIFY,       "Verify",      STLDR_FUNC_OPTIONAL  }
};

struct stldr_section {
	uint32_t addr;
	uint32_t size;
	uint8_t *content;
	bool do_write;
	Elf32_Section idx;
	struct list_head lh;
};

struct stldr_loader {
	bool parsed;
	bool relocatable;
	bool user_flash_base_addr;
	bool user_flash_size;
	bool sections_initialized;
	uint32_t return_addr;
	uint32_t offset;
	uint32_t size;
	uint32_t func_addr[FUNC_ID_COUNT];
	struct list_head sections;
	struct working_area *work_area;
};

enum stldr_type {
	STLDR_TYPE_MCU_FLASH = 1,
	STLDR_TYPE_NAND_FLASH,
	STLDR_TYPE_NOR_FLASH,
	STLDR_TYPE_SRAM,
	STLDR_TYPE_PSRAM,
	STLDR_TYPE_PC_CARD,
	STLDR_TYPE_SPI_FLASH,
	STLDR_TYPE_I2C_FLASH,
	STLDR_TYPE_SDRAM,
	STLDR_TYPE_I2C_EEPROM
};

struct stldr_dev_sector {
	uint32_t count;
	uint32_t size;
};

struct stldr_dev_info {
	char name[STORAGE_INFO_NAME_SIZE + 1];
	enum stldr_type type;
	uint32_t base_addr;
	uint32_t size;
	uint32_t page_size;
	uint8_t erase_value;
	struct stldr_dev_sector sectors[STORAGE_INFO_SECTORS_MAX_ITEMS];
	unsigned int n_sectors;
};

struct stldr_flash_bank {
	bool probed;
	uint32_t data_width;
	struct stldr_loader loader;
	struct stldr_dev_info dev_info;
};

struct stldr_func_args {
	uint32_t arg_0;
	uint32_t arg_1;
	uint32_t arg_2;
	uint32_t arg_3;
	uint32_t ret;
};

static inline void stldr_func_args_set(struct stldr_func_args *args, uint32_t arg_0,
		uint32_t arg_1, uint32_t arg_2, uint32_t arg_3)
{
	args->ret = STLDR_FUNC_FAILURE;
	args->arg_0 = arg_0;
	args->arg_1 = arg_1;
	args->arg_2 = arg_2;
	args->arg_3 = arg_3;
}

static inline void stldr_func_args_init(struct stldr_func_args *args)
{
	stldr_func_args_set(args, 0, 0, 0, 0);
}

static int stldr_parse_dev_info(struct flash_bank *bank, uint8_t *data, int size)
{
	if (size < STORAGE_INFO_SECTORS_OFFSET + (STORAGE_INFO_SECTORS_MAX_ITEMS * DEVICE_SECTORS_SIZE)) {
		LOG_ERROR("stldr: invalid StorageInfo size (%d)", size);
		return ERROR_FAIL;
	}

	struct stldr_flash_bank *stldr_info = bank->driver_priv;
	struct stldr_dev_info *dev_info = &stldr_info->dev_info;

	memcpy(dev_info->name, data, STORAGE_INFO_NAME_SIZE);
	// guarantee we have a zero-terminated string
	dev_info->name[STORAGE_INFO_NAME_SIZE] = 0;
	LOG_DEBUG("name %s", dev_info->name);

	dev_info->type = le_to_h_u16(data + STORAGE_INFO_TYPE_OFFSET);
	LOG_DEBUG("flash type %x", (unsigned int)dev_info->type);

	dev_info->base_addr = le_to_h_u32(data + STORAGE_INFO_BASE_ADDR_OFFSET);
	LOG_DEBUG("base_addr %x", dev_info->base_addr);

	dev_info->size = le_to_h_u32(data + STORAGE_INFO_SIZE_OFFSET);
	LOG_DEBUG("size %x", dev_info->size);

	dev_info->erase_value = data[STORAGE_INFO_ERASE_VALUE_OFFSET];
	LOG_DEBUG("erase value %x", dev_info->erase_value);

	dev_info->page_size = le_to_h_u32(data + STORAGE_INFO_PAGE_SIZE_OFFSET);
	LOG_DEBUG("page size %x", dev_info->page_size);

	dev_info->n_sectors = STORAGE_INFO_SECTORS_MAX_ITEMS;
	LOG_DEBUG("n_sectors %x", dev_info->n_sectors);

	for (unsigned int i = 0; i < STORAGE_INFO_SECTORS_MAX_ITEMS; i++) {
		const unsigned int offset = i * DEVICE_SECTORS_SIZE + STORAGE_INFO_SECTORS_OFFSET;
		const uint32_t count = le_to_h_u32(data + offset + DEVICE_SECTORS_COUNT_OFFSET);
		if (!count) {
			dev_info->n_sectors = i;
			break;
		}

		dev_info->sectors[i].count = count;
		dev_info->sectors[i].size = le_to_h_u32(data + offset + DEVICE_SECTORS_SIZE_OFFSET);
	}

	return ERROR_OK;
}

static void stldr_free_sections(struct stldr_flash_bank *stldr_info)
{
	struct stldr_section *section, *tmp;

	if (!stldr_info->loader.sections_initialized) {
		INIT_LIST_HEAD(&stldr_info->loader.sections);
		stldr_info->loader.sections_initialized = true;
	} else {
		list_for_each_entry_safe(section, tmp, &stldr_info->loader.sections, lh) {
			list_del(&section->lh);
			free(section->content);
			free(section);
		}

		INIT_LIST_HEAD(&stldr_info->loader.sections);
	}

	stldr_info->loader.parsed = false;
	stldr_info->loader.offset = 0;
	stldr_info->loader.size = 0;
	stldr_info->loader.relocatable = false;
}

static int stldr_parse(struct flash_bank *bank, const char *stldr_path)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;
	FILE *fp = NULL;
	char *symbol_names = NULL;
	int retval = ERROR_FAIL;

	stldr_free_sections(stldr_info);

	// Check that path exists and is a regular file, not a directory
	struct stat st;
	if (stat(stldr_path, &st) != 0) {
		LOG_ERROR("stldr: cannot access '%s'", stldr_path);
		goto out;
	}

	if (!S_ISREG(st.st_mode)) {
		LOG_ERROR("stldr: '%s' is not a regular file", stldr_path);
		goto out;
	}

	// Open file in read mode
	fp = fopen(stldr_path, "rb");
	if (!fp) {
		LOG_ERROR("cannot open '%s'", stldr_path);
		goto out;
	}

	// Extract the ELF header
	Elf32_Ehdr elf_header;
	if (fread(&elf_header, sizeof(Elf32_Ehdr), 1, fp) != 1) {
		LOG_ERROR("stldr: failed to read ELF header");
		goto out;
	}

	// Basic ELF sanity checks (avoid OOB seeks/allocations on malformed files)
	if (memcmp(elf_header.e_ident, ELFMAG, SELFMAG) != 0 ||
			elf_header.e_ident[EI_CLASS] != ELFCLASS32 ||
			elf_header.e_shentsize != sizeof(Elf32_Shdr) ||
			elf_header.e_shnum == 0 ||
			elf_header.e_shstrndx >= elf_header.e_shnum ||
			(uint64_t)elf_header.e_shoff +
			(uint64_t)elf_header.e_shnum * elf_header.e_shentsize >
			(uint64_t)st.st_size) {
		LOG_ERROR("stldr: unsupported or malformed ELF file '%s'", stldr_path);
		goto out;
	}

	// Extract the section header string table offset
	Elf32_Off offset_shst = elf_header.e_shoff +
			elf_header.e_shstrndx * sizeof(Elf32_Shdr);

	// Read the section header string table
	if (fseek(fp, offset_shst, SEEK_SET) != 0) {
		LOG_ERROR("stldr: failed to seek to section header string table");
		goto out;
	}

	Elf32_Shdr section_header;
	if (fread(&section_header, 1, sizeof(section_header), fp) != sizeof(section_header)) {
		LOG_ERROR("stldr: failed to read section header string table header");
		goto out;
	}

	// Iterate all sections and get the loader binary and functions
	for (unsigned int idx = 0; idx < elf_header.e_shnum; idx++) {
		if (fseek(fp, elf_header.e_shoff + idx * sizeof(section_header), SEEK_SET) != 0) {
			LOG_ERROR("stldr: failed to seek to section header %u", idx);
			goto out;
		}
		if (fread(&section_header, 1, sizeof(section_header), fp) != sizeof(section_header)) {
			LOG_ERROR("stldr: failed to read section header %u", idx);
			goto out;
		}

		// Ignore non loadable sections
		if ((section_header.sh_flags & SHF_ALLOC) == 0)
			continue;

		uint8_t *content = (section_header.sh_type == 8) ?
			calloc(1, section_header.sh_size) : malloc(section_header.sh_size);
		if (!content) {
			LOG_ERROR("Out of memory");
			goto out;
		}

		if (section_header.sh_type != 8) {
			if (fseek(fp, section_header.sh_offset, SEEK_SET) != 0) {
				LOG_ERROR("stldr: failed to seek to section content");
				free(content);
				goto out;
			}
			if (fread(content, 1, section_header.sh_size, fp) != section_header.sh_size) {
				LOG_ERROR("stldr: failed to read section content");
				free(content);
				goto out;
			}
		}

		struct stldr_section *section = calloc(1, sizeof(struct stldr_section));
		if (!section) {
			free(content);
			LOG_ERROR("Out of memory");
			goto out;
		}

		section->addr = section_header.sh_addr;
		section->size = section_header.sh_size;
		section->content = content;
		section->idx = idx;
		// sh_flags already checked against SHF_ALLOC
		section->do_write = (section_header.sh_flags & SHF_EXECINSTR) == SHF_EXECINSTR;

		list_add_tail(&section->lh, &stldr_info->loader.sections);

		// check if the flash loader is relocatable
		if ((section->addr & 0xFF000000) == 0 && section->do_write)
			stldr_info->loader.relocatable = true;

		LOG_DEBUG("Loader section found { addr : 0x%08X , size : 0x%08X }",
				section->addr, section->size);
	}

	// get symtab
	bool symtab_set = false;
	Elf32_Shdr symtab_sh;
	for (unsigned int idx = 0; idx < elf_header.e_shnum; idx++) {
		if (fseek(fp, elf_header.e_shoff + idx * sizeof(section_header), SEEK_SET) != 0) {
			LOG_ERROR("stldr: failed to seek to section header %u", idx);
			goto out;
		}
		if (fread(&section_header, 1, sizeof(section_header), fp) != sizeof(section_header)) {
			LOG_ERROR("stldr: failed to read section header %u", idx);
			goto out;
		}

		if (section_header.sh_type == SHT_SYMTAB) {
			symtab_sh = section_header;
			symtab_set = true;
			break;
		}
	}

	if (!symtab_set) {
		LOG_ERROR("symtab not found");
		goto out;
	}

	// Get the symbol table string section header
	if (fseek(fp, elf_header.e_shoff + symtab_sh.sh_link * sizeof(symtab_sh), SEEK_SET) != 0) {
		LOG_ERROR("stldr: failed to seek to symbol string table header");
		goto out;
	}
	Elf32_Shdr str_sym;
	if (fread(&str_sym, 1, sizeof(str_sym), fp) != sizeof(str_sym)) {
		LOG_ERROR("stldr: failed to read symbol string table header");
		goto out;
	}

	// Create the table of symbol names
	symbol_names = malloc(str_sym.sh_size);
	if (!symbol_names) {
		LOG_ERROR("Out of memory");
		goto out;
	}
	if (fseek(fp, str_sym.sh_offset, SEEK_SET) != 0) {
		LOG_ERROR("stldr: failed to seek to symbol string table");
		goto out;
	}
	if (fread(symbol_names, 1, str_sym.sh_size, fp) != str_sym.sh_size) {
		LOG_ERROR("stldr: failed to read symbol string table");
		goto out;
	}

	// Iterate all symbols
	for (unsigned int f = 0; f < FUNC_ID_COUNT; f++)
		stldr_info->loader.func_addr[stldr_functions[f].id] = STLDR_FUNC_ADDR_UNKNOWN;

	if (symtab_sh.sh_entsize == 0) {
		LOG_ERROR("stldr: invalid symtab entry size (0)");
		goto out;
	}

	const unsigned int symbol_count = symtab_sh.sh_size / symtab_sh.sh_entsize;
	for (unsigned int j = 0; j < symbol_count; j++) {
		Elf32_Sym symbol;

		if (fseek(fp, symtab_sh.sh_offset + j * symtab_sh.sh_entsize, SEEK_SET) != 0) {
			LOG_ERROR("stldr: failed to seek to symbol %u", j);
			goto out;
		}
		if (fread(&symbol, sizeof(symbol), 1, fp) != 1) {
			LOG_ERROR("stldr: failed to read symbol %u", j);
			goto out;
		}

		if (symbol.st_name >= str_sym.sh_size)
			continue;

		const char *symbol_name = symbol_names + symbol.st_name;

		if (!memchr(symbol_name, '\0', str_sym.sh_size - symbol.st_name))
			continue;
		// check for loader descriptor
		if (strcmp("StorageInfo", symbol_name) == 0) {
			uint8_t *storage_info = malloc(symbol.st_size);
			if (!storage_info) {
				LOG_ERROR("Out of memory");
				goto out;
			}

			struct stldr_section *tmp_sec;
			bool found_section = false;

			list_for_each_entry(tmp_sec, &stldr_info->loader.sections, lh) {
				LOG_DEBUG("This section starts in %x with size %x", tmp_sec->addr, tmp_sec->size);

				if ((stldr_info->loader.size > tmp_sec->size + tmp_sec->addr) && tmp_sec->do_write)
					stldr_info->loader.size = tmp_sec->size + tmp_sec->addr;

				if (tmp_sec->idx == symbol.st_shndx) {
					found_section = true;

					if (symbol.st_value < tmp_sec->addr ||
							(symbol.st_value - tmp_sec->addr) + symbol.st_size > tmp_sec->size) {
						free(storage_info);
						LOG_ERROR("stldr: StorageInfo symbol out of section bounds");
						goto out;
					}

					const uint32_t sym_off = symbol.st_value - tmp_sec->addr;
					memcpy(storage_info, tmp_sec->content + sym_off, symbol.st_size);

					retval = stldr_parse_dev_info(bank, storage_info, symbol.st_size);
					free(storage_info);
					if (retval != ERROR_OK)
						goto out;

					break;
				}
			}

			if (!found_section) {
				free(storage_info);
				LOG_ERROR("stldr: StorageInfo symbol section not found");
				goto out;
			}
		}

		// get loader functions
		for (unsigned int f = 0; f < FUNC_ID_COUNT; f++) {
			if (strcmp(stldr_functions[f].name, symbol_name) == 0) {
				LOG_DEBUG("Loader function '%s' found at 0x%08X", symbol_name, symbol.st_value);
				stldr_info->loader.func_addr[stldr_functions[f].id] = symbol.st_value;
				break;
			}
		}
	}

	// Check that mandatory functions are found
	for (unsigned int f = 0; f < FUNC_ID_COUNT; f++) {
		if (stldr_functions[f].type == STLDR_FUNC_OPTIONAL)
			continue;
		if (stldr_info->loader.func_addr[stldr_functions[f].id] == STLDR_FUNC_ADDR_UNKNOWN) {
			LOG_ERROR("Loader function %s not found", stldr_functions[f].name);
			goto out;
		}
	}

	// Compute loader size
	{
		struct stldr_section *section;
		uint32_t last_section_addr = 0;
		uint32_t count_do_write = 0;

		list_for_each_entry(section, &stldr_info->loader.sections, lh) {
			if (!section->do_write)
				continue;

			count_do_write++;
			if (section->addr >= last_section_addr) {
				last_section_addr = section->addr;
				stldr_info->loader.size += section->size;
			}
		}

		if (count_do_write > 1) {
			LOG_ERROR("FlashLoader contains multiple execute sections");
			goto out;
		}
	}

	stldr_info->loader.parsed = true;
	retval = ERROR_OK;

out:
	free(symbol_names);

	if (fp)
		fclose(fp);

	if (retval != ERROR_OK)
		stldr_free_sections(stldr_info);

	return retval;
}

// flash bank stldr <base> <size> 0 0 <target#> [stldr_path]

FLASH_BANK_COMMAND_HANDLER(stldr_flash_bank_command)
{
	if (CMD_ARGC != 6 && CMD_ARGC != 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct stldr_flash_bank *stldr_info;
	stldr_info = calloc(1, sizeof(struct stldr_flash_bank));
	if (!stldr_info) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = stldr_info;

	unsigned int chip_width = bank->chip_width;
	if (chip_width == 0) {
		chip_width = 32;
		LOG_DEBUG("The chip_width argument is set to 32");
	} else if (chip_width & (chip_width - 1)) {
		LOG_ERROR("chip_width must be a power of two (bytes)");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	bank->write_start_alignment = chip_width;
	bank->write_end_alignment = chip_width;

	stldr_info->loader.user_flash_base_addr = bank->base != 0;
	stldr_info->loader.user_flash_size = bank->size != 0;
	stldr_info->loader.parsed = false;
	stldr_info->probed = false;

	if (CMD_ARGC == 7) // the stldr_path is specified
		return stldr_parse(bank, CMD_ARGV[6]);

	return ERROR_OK;
}

static void stldr_free_driver_priv(struct flash_bank *bank)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;

	if (stldr_info->loader.sections.prev || stldr_info->loader.sections.next) {
		struct stldr_section *tmp, *section;
		list_for_each_entry_safe(section, tmp, &stldr_info->loader.sections, lh) {
			free(section->content);
			free(section);
		}
	}

	free(bank->driver_priv);
	bank->driver_priv = NULL;
}

static int stldr_write_loader(struct flash_bank *bank)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;

	if (stldr_info->loader.relocatable) {
		if (target_alloc_working_area_try(bank->target,
				stldr_info->loader.size + 4 + STACK_OFFSET,
				&stldr_info->loader.work_area) != ERROR_OK) {
			LOG_ERROR("working area not large enough");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		// update loader functions offset
		stldr_info->loader.return_addr = stldr_info->loader.work_area->address;
		stldr_info->loader.offset = stldr_info->loader.work_area->address + 4;
		LOG_DEBUG("loader offset 0x%08X", stldr_info->loader.offset);
	}

	struct target *target = bank->target;
	struct stldr_section *section;
	uint32_t working_area = target->working_area_phys;
	uint32_t working_area_size = target->working_area_size;
	list_for_each_entry(section, &stldr_info->loader.sections, lh) {
		if (!section->do_write)
			continue;

		if (!stldr_info->loader.relocatable) {
			const uint32_t wa_start = working_area;
			const uint32_t wa_end = working_area + working_area_size;
			const uint32_t sec_start = section->addr;
			const uint32_t sec_end = section->addr + section->size;
			if (sec_start < wa_end && sec_end > wa_start) {
				LOG_ERROR("loader section overlaps target work area");
				return ERROR_FAIL;
			}

			int retval = target_write_buffer(target, section->addr, section->size, section->content);
			if (retval != ERROR_OK)
				return retval;

			// update loader functions offset
			stldr_info->loader.return_addr = section->addr - 4;
			stldr_info->loader.offset = section->addr;
			LOG_DEBUG("loader offset 0x%08X", stldr_info->loader.offset);
		} else {
			int retval = target_write_buffer(target, section->addr + stldr_info->loader.offset,
							section->size, section->content);
			if (retval != ERROR_OK) {
				target_free_working_area(target, stldr_info->loader.work_area);
				return retval;
			}
		}
	}

	return ERROR_OK;
}

static int stldr_exec_function(struct flash_bank *bank, enum stldr_func_id func_id,
		unsigned int timeout_ms, struct stldr_func_args *args)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;

	struct target *target = bank->target;
	struct armv7m_algorithm armv7m_info = {
		.common_magic = ARMV7M_COMMON_MAGIC,
		.core_mode = ARM_MODE_THREAD
	};

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stldr_info->loader.func_addr[func_id] == STLDR_FUNC_ADDR_UNKNOWN) {
		LOG_ERROR("Loader function %s not found", stldr_functions[func_id].name);
		return ERROR_FAIL;
	}

	const uint32_t func_addr = stldr_info->loader.func_addr[func_id] +
			(stldr_info->loader.relocatable ? stldr_info->loader.offset : 0);

	LOG_DEBUG("Running loader function %s(0x%" PRIx32 ", 0x%" PRIx32 ", 0x%" PRIx32 ", 0x%" PRIx32 ")",
		stldr_functions[func_id].name, args->arg_0, args->arg_1, args->arg_2, args->arg_3);

	struct reg_param reg_params[6];

	// write function parameters
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, args->arg_0);
	buf_set_u32(reg_params[1].value, 0, 32, args->arg_1);
	buf_set_u32(reg_params[2].value, 0, 32, args->arg_2);
	buf_set_u32(reg_params[3].value, 0, 32, args->arg_3);

	// write algo stack pointer
	uint32_t estack;
	if (stldr_info->loader.relocatable)
		estack = stldr_info->loader.work_area->size + stldr_info->loader.work_area->address;
	else
		estack = stldr_info->loader.size + stldr_info->loader.offset + STACK_OFFSET;
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[4].value, 0, 32, estack);
	init_reg_param(&reg_params[5], "lr", 32, PARAM_OUT);
	buf_set_u32(reg_params[5].value, 0, 32, stldr_info->loader.return_addr | 0x1);

	// execute function
	breakpoint_add(target, stldr_info->loader.return_addr, 2, BKPT_SOFT);
	int retval = target_run_algorithm(target,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			func_addr,
			stldr_info->loader.return_addr, timeout_ms, &armv7m_info);

	breakpoint_remove(target, stldr_info->loader.return_addr);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to run loader function %s(...) %s",
			stldr_functions[func_id].name,
			retval == ERROR_TARGET_TIMEOUT ? " : timeout reached" : ""
		);
		goto exit;
	}

	args->ret = buf_get_u32(reg_params[0].value, 0, 32);
	LOG_DEBUG("Loader function %s(...) returned %d", stldr_functions[func_id].name, args->ret);
exit:
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);
	return retval;
}

static int stldr_exec_function_deinit(struct flash_bank *bank)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;
	return target_free_working_area(bank->target, stldr_info->loader.work_area);
}

static int stldr_exec_function_init(struct flash_bank *bank)
{
	struct stldr_func_args args;
	stldr_func_args_init(&args);

	int retval = stldr_write_loader(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = stldr_exec_function(bank, FUNC_ID_INIT, STLDR_INIT_TIMEOUT_MS, &args);
	if (retval != ERROR_OK || args.ret != STLDR_FUNC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

static int stldr_exec_function_mass_erase(struct flash_bank *bank)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;

	unsigned int timeout_ms = (stldr_info->dev_info.type == STLDR_TYPE_MCU_FLASH) ?
			STLDR_MASS_ERASE_INT_TIMEOUT_MS : STLDR_MASS_ERASE_EXT_TIMEOUT_MS;

	struct stldr_func_args args;
	stldr_func_args_init(&args);

	int retval = stldr_exec_function_init(bank);
		if (retval != ERROR_OK) {
			stldr_exec_function_deinit(bank);
			return retval;
		}

	retval = stldr_exec_function(bank, FUNC_ID_MASS_ERASE, timeout_ms, &args);
	if (retval != ERROR_OK || args.ret != STLDR_FUNC_SUCCESS) {
		stldr_exec_function_deinit(bank);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return stldr_exec_function_deinit(bank);
}

static int stldr_exec_function_sector_erase(struct flash_bank *bank,
		uint32_t start_addr, uint32_t end_addr)
{
	struct stldr_func_args args;
	stldr_func_args_set(&args, start_addr, end_addr, 0, 0);
	int retval = stldr_exec_function(bank, FUNC_ID_SECTOR_ERASE, STLDR_SECTOR_ERASE_TIMEOUT_MS, &args);
	if (retval != ERROR_OK || args.ret != STLDR_FUNC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

static int stldr_exec_function_read(struct flash_bank *bank,
		uint32_t addr, uint32_t size, uint32_t buffer_addr)
{
	struct stldr_func_args args;
	stldr_func_args_set(&args, addr, size, buffer_addr, 0);
	int retval = stldr_exec_function(bank, FUNC_ID_READ, STLDR_READ_TIMEOUT_MS, &args);
	if (retval != ERROR_OK || args.ret != STLDR_FUNC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

static int stldr_exec_function_write(struct flash_bank *bank,
		uint32_t addr, uint32_t size, uint32_t buffer_addr)
{
	struct stldr_func_args args;
	stldr_func_args_set(&args, addr, size, buffer_addr, 0);
	int retval = stldr_exec_function(bank, FUNC_ID_WRITE, STLDR_WRITE_TIMEOUT_MS, &args);
	if (retval != ERROR_OK || args.ret != STLDR_FUNC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

static int stldr_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int retval = stldr_exec_function_init(bank);
	if (retval != ERROR_OK) {
		stldr_exec_function_deinit(bank);
		return retval;
	}

	retval = stldr_exec_function_sector_erase(bank,
			bank->base + bank->sectors[first].offset,
			bank->base + bank->sectors[last].offset);
	stldr_exec_function_deinit(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stldr_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;

	int retval = stldr_exec_function_init(bank);
	if (retval != ERROR_OK) {
		stldr_exec_function_deinit(bank);
		return retval;
	}

	uint32_t buffer_size = target_get_working_area_avail(bank->target);
	// buffer_size should be multiple of stldr_info->data_width
	buffer_size = ALIGN_DOWN(buffer_size, stldr_info->data_width);

	if (buffer_size < 256) {
		LOG_ERROR("Block memory writes cannot be performed because working area is not large enough");
		stldr_exec_function_deinit(bank);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	LOG_DEBUG("workarea size: %x", buffer_size);
	LOG_DEBUG("workarea address: " TARGET_ADDR_FMT, bank->target->working_area_phys);

	/* The flash write must be aligned to the 'stldr_info->data_width' boundary.
	 * The flash infrastructure ensures it, do just a security check */
	assert(offset % stldr_info->data_width == 0);
	assert(count % stldr_info->data_width == 0);

	struct working_area *source;
	if (target_alloc_working_area_try(bank->target, buffer_size, &source) != ERROR_OK) {
		LOG_ERROR("working area not large enough");
		stldr_exec_function_deinit(bank);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	uint32_t address = bank->base + offset;

	while (count > 0) {
		if (count < buffer_size)
			buffer_size = count;

		retval = target_write_buffer(bank->target, source->address, buffer_size, buffer);
		if (retval != ERROR_OK)
			goto exit_error;

		retval = stldr_exec_function_write(bank, address, buffer_size, source->address);
		if (retval != ERROR_OK)
			goto exit_error;

		buffer += buffer_size;
		address += buffer_size;
		count -= buffer_size;
	}
	stldr_exec_function_deinit(bank);
	target_free_working_area(bank->target, source);
	return ERROR_OK;

exit_error:
	stldr_exec_function_deinit(bank);
	target_free_working_area(bank->target, source);
	return ERROR_FLASH_OPERATION_FAILED;
}

static int stldr_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;
	struct stldr_dev_info *dev_info = &stldr_info->dev_info;

	// init is needed for default_flash_read as well
	int retval = stldr_exec_function_init(bank);
	if (retval != ERROR_OK) {
		stldr_exec_function_deinit(bank);
		return retval;
	}

	if (stldr_info->loader.func_addr[FUNC_ID_READ] == STLDR_FUNC_ADDR_UNKNOWN) {
		int ret = default_flash_read(bank, buffer, offset, count);
		stldr_exec_function_deinit(bank);
		return ret;
	}

	const uint32_t block_size = dev_info->page_size;

	struct working_area *buffer_area;
	if (target_alloc_working_area_try(bank->target, block_size, &buffer_area) != ERROR_OK) {
		LOG_ERROR("working area not large enough");
		stldr_exec_function_deinit(bank);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	uint32_t address = bank->base + offset;
	while (count > 0) {
		const uint32_t read_block_size = MIN(count, block_size);

		retval = stldr_exec_function_read(bank, address, read_block_size, buffer_area->address);
		if (retval != ERROR_OK)
			goto exit_error;

		retval = target_read_buffer(bank->target, buffer_area->address, read_block_size, buffer);
		if (retval != ERROR_OK)
			goto exit_error;

		buffer += read_block_size;
		address += read_block_size;
		count -= read_block_size;
	}

	stldr_exec_function_deinit(bank);
	target_free_working_area(bank->target, buffer_area);
	return ERROR_OK;

exit_error:
	stldr_exec_function_deinit(bank);
	target_free_working_area(bank->target, buffer_area);
	return ERROR_FLASH_OPERATION_FAILED;
}

static int stldr_probe(struct flash_bank *bank)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;
	struct stldr_dev_info *dev_info = &stldr_info->dev_info;

	if (!stldr_info->loader.parsed) {
		LOG_ERROR("Please check if you have set correctly the loader file");
		return ERROR_FAIL;
	}

	free(bank->sectors);

	if (!stldr_info->loader.user_flash_base_addr)
		bank->base = dev_info->base_addr;
	if (!stldr_info->loader.user_flash_size)
		bank->size = dev_info->size;

	stldr_info->data_width = bank->write_start_alignment;

	bank->num_sectors = 0;
	for (unsigned int i = 0; i < dev_info->n_sectors; i++)
		bank->num_sectors += dev_info->sectors[i].count;

	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!bank->sectors) {
		LOG_ERROR("failed to allocate bank sectors");
		return ERROR_FAIL;
	}

	uint32_t s_i = 0, s_offset = 0;
	for (unsigned int i = 0; i < dev_info->n_sectors; i++) {
		for (unsigned int j = 0; j < dev_info->sectors[i].count; j++) {
			bank->sectors[s_i].offset = s_offset;
			bank->sectors[s_i].size = dev_info->sectors[i].size;
			bank->sectors[s_i].is_erased = -1;
			bank->sectors[s_i].is_protected = -1;

			s_i++;
			s_offset += dev_info->sectors[i].size;
		}
	}

	LOG_DEBUG("Bank (%u) size is %" PRIu32 " kb, base address is " TARGET_ADDR_FMT,
			bank->bank_number, bank->size >> 10, bank->base);

	stldr_info->probed = true;
	return ERROR_OK;
}

static int stldr_auto_probe(struct flash_bank *bank)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;

	if (stldr_info->probed)
		return ERROR_OK;

	return stldr_probe(bank);
}

static int stldr_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct stldr_flash_bank *stldr_info = bank->driver_priv;
	struct stldr_dev_info *dev_info = &stldr_info->dev_info;

	if (stldr_info->loader.parsed)
		command_print_sameline(cmd, "Loader file for %s ", dev_info->name);
	else
		command_print_sameline(cmd, "Please check if you have set correctly the loader file");

	return ERROR_OK;
}

COMMAND_HANDLER(stldr_handle_set_loader_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank_probe_optional, 0, &bank, false);
	if (retval != ERROR_OK)
		return retval;

	struct stldr_flash_bank *stldr_info = bank->driver_priv;
	stldr_info->probed = false;

	retval = stldr_parse(bank, CMD_ARGV[1]);
	if (retval != ERROR_OK)
		command_print(CMD, "stldr file parsing failed");

	return retval;
}

COMMAND_HANDLER(stldr_handle_set_size_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank_probe_optional, 0, &bank, false);
	if (retval != ERROR_OK)
		return retval;

	struct stldr_flash_bank *stldr_info = bank->driver_priv;
	stldr_info->probed = false;

	uint32_t bank_size;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], bank_size);
	if (bank_size == 0) {
		command_print(CMD, "The bank size could not be zero");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	bank->size = bank_size;
	stldr_info->loader.user_flash_size = true;

	return retval;
}

COMMAND_HANDLER(stldr_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = stldr_exec_function_mass_erase(bank);
	if (retval != ERROR_OK)
		command_print(CMD, "stldr mass erase failed");

	return retval;
}

COMMAND_HANDLER(stldr_handle_init_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = stldr_exec_function_init(bank);
	if (retval != ERROR_OK)
		command_print(CMD, "stldr init failed");

	return retval;
}

static const struct command_registration stldr_subcommand_handlers[] = {
	{
		.name = "set_loader",
		.handler = stldr_handle_set_loader_command,
		.mode = COMMAND_ANY,
		.usage = "bank_id path/to/stldr",
		.help = "set loader path"
	},
	{
		.name = "set_size",
		.handler = stldr_handle_set_size_command,
		.mode = COMMAND_ANY,
		.usage = "bank_id size",
		.help = "set bank size"
	},
	{
		.name = "mass_erase",
		.handler = stldr_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash bank"
	},
	{
		.name = "init",
		.handler = stldr_handle_init_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Initialize memory"
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stldr_command_handlers[] = {
	{
		.name = "stldr",
		.mode = COMMAND_ANY,
		.help = "stldr flash command group",
		.usage = "",
		.chain = stldr_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver stldr_flash = {
	.name = "stldr",
	.commands = stldr_command_handlers,
	.flash_bank_command = stldr_flash_bank_command,
	.erase = stldr_erase,
	.write = stldr_write,
	.read = stldr_read,
	.probe = stldr_probe,
	.auto_probe = stldr_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = stldr_get_info,
	.free_driver_priv = stldr_free_driver_priv
};

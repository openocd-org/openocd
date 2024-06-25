/***************************************************************************
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

/* Only tested on AT32F415 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

#define MCU_DEVICE_ID_ADDR      0xE0042000
#define FLASH_SIZE_ADDR         0x1FFFF7E0
#define DEVICE_UID_ADDR         0x1FFFF7E8
#define MASK_VERSION_ADDR       0x1FFFF7F1
#define FLASH_BASE_ADDR         0x08000000

#define EFC_CTRL_REG            0x40022010
#define EFC_PRGM_BIT            (1<<0)
#define EFC_PGERS_BIT           (1<<1)
#define EFC_RSTR_BIT            (1<<6)
#define EFC_LOCK_BIT            (1<<7)
#define EFC_FCKEY_REG           0x40022004
#define EFC_KEY1                0x45670123
#define EFC_KEY2                0xCDEF89AB
#define EFC_RDPRTEN             0x00A5
#define EFC_STS_REG             0x4002200C
#define EFC_BSY_BIT             (1<<0)
#define EFC_PRGMFLR_BIT         (1<<2)
#define EFC_WRPRTFLR_BIT        (1<<4)
#define EFC_PRCDN_BIT           (1<<5)
#define EFC_ADDR_REG            0x40022014

#define FLASH_ERASE_TIMEOUT 100     /* 10ms actually required */
#define FLASH_WRITE_TIMEOUT 10      /* 42us actually required */

struct artery_flash_bank {
	bool probed;
};

struct artery_chip_info {
	uint32_t chip_id;
	uint32_t flash_size_kB;
	uint32_t sector_size;
	char *chip_name;
};

static const struct artery_chip_info known_artery_chips[] = {
{ 0xF0050340	, 1024	, 2048	, "AR8F403CGT6-A"	},
{ 0xF0050340	, 1024	, 2048	, "AR8F403CGT6"     },
{ 0x70050242	, 256	, 2048	, "AT32F403ACCT7"	},
{ 0x70050243	, 256	, 2048	, "AT32F403ACCU7"	},
{ 0x700502CF	, 512	, 2048	, "AT32F403ACET7"	},
{ 0x700502D0	, 512	, 2048	, "AT32F403ACEU7"	},
{ 0x70050346	, 1024	, 2048	, "AT32F403ACGT7"	},
{ 0x70050347	, 1024	, 2048	, "AT32F403ACGU7"	},
{ 0x70050241	, 256	, 2048	, "AT32F403ARCT7"	},
{ 0x700502CE	, 512	, 2048	, "AT32F403ARET7"	},
{ 0x70050345	, 1024	, 2048	, "AT32F403ARGT7"	},
{ 0x70050240	, 256	, 2048	, "AT32F403AVCT7"	},
{ 0x700502CD	, 512	, 2048	, "AT32F403AVET7"	},
{ 0x70050344	, 1024	, 2048	, "AT32F403AVGT7"	},
{ 0xF0050355	, 1024	, 2048	, "AT32F403AVGW"	},
{ 0x700301CF	, 128	, 1024	, "AT32F403CBT6"	},
{ 0x70050243	, 256	, 2048	, "AT32F403CCT6"	},
{ 0x7005024E	, 256	, 2048	, "AT32F403CCU6"	},
{ 0x700502CB	, 512	, 2048	, "AT32F403CET6"	},
{ 0x700502CD	, 512	, 2048	, "AT32F403CEU6"	},
{ 0x70050347	, 1024	, 2048	, "AT32F403CGT6"	},
{ 0x7005034C	, 1024	, 2048	, "AT32F403CGU6"	},
{ 0x70050242	, 256	, 2048	, "AT32F403RCT6"	},
{ 0x700502CA	, 512	, 2048	, "AT32F403RET6"	},
{ 0x70050346	, 1024	, 2048	, "AT32F403RGT6"	},
{ 0x70050241	, 256	, 2048	, "AT32F403VCT6"	},
{ 0x700502C9	, 512	, 2048	, "AT32F403VET6"	},
{ 0x70050345	, 1024	, 2048	, "AT32F403VGT6"	},
{ 0x70050240	, 256	, 2048	, "AT32F403ZCT6"	},
{ 0x700502C8	, 512	, 2048	, "AT32F403ZET6"	},
{ 0x70050344	, 1024	, 2048	, "AT32F403ZGT6"	},
{ 0x70050254	, 256	, 2048	, "AT32F407AVCT7"	},
{ 0x70050353	, 1024	, 2048	, "AT32F407AVGT7"	},
{ 0x7005024A	, 256	, 2048	, "AT32F407RCT7"	},
{ 0x700502D2	, 512	, 2048	, "AT32F407RET7"	},
{ 0x7005034C	, 1024	, 2048	, "AT32F407RGT7"	},
{ 0x70050249	, 256	, 2048	, "AT32F407VCT7"	},
{ 0x700502D1	, 512	, 2048	, "AT32F407VET7"	},
{ 0x7005034B	, 1024	, 2048	, "AT32F407VGT7"	},
{ 0x70030106	, 64	, 1024	, "AT32F413C8T7"	},
{ 0x700301C3	, 128	, 1024	, "AT32F413CBT7"	},
{ 0x700301CA	, 128	, 1024	, "AT32F413CBU7"	},
{ 0x70030242	, 256	, 2048	, "AT32F413CCT7"	},
{ 0x70030247	, 256	, 2048	, "AT32F413CCU7"	},
{ 0x700301C5	, 128	, 1024	, "AT32F413KBU7-4"	},
{ 0x70030244	, 256	, 2048	, "AT32F413KCU7-4"	},
{ 0x700301C1	, 128	, 1024	, "AT32F413RBT7"	},
{ 0x70030240	, 256	, 2048	, "AT32F413RCT7"	},
{ 0x700301CB	, 128	, 1024	, "AT32F413TBU7"	},
{ 0x70030109	, 64	, 1024	, "AT32F415C8T7"	},
{ 0x700301C5	, 128	, 1024	, "AT32F415CBT7"	},
{ 0x700301CD	, 128	, 1024	, "AT32F415CBU7"	},
{ 0x70030241	, 256	, 2048	, "AT32F415CCT7"	},
{ 0x7003024C	, 256	, 2048	, "AT32F415CCU7"	},
{ 0x7003010A	, 64	, 1024	, "AT32F415K8U7-4"	},
{ 0x700301C6	, 128	, 1024	, "AT32F415KBU7-4"	},
{ 0x70030242	, 256	, 2048	, "AT32F415KCU7-4"	},
{ 0x7003010B	, 64	, 1024	, "AT32F415R8T7-7"	},
{ 0x70030108	, 64	, 1024	, "AT32F415R8T7"	},
{ 0x700301C7	, 128	, 1024	, "AT32F415RBT7-7"	},
{ 0x700301C4	, 128	, 1024	, "AT32F415RBT7"	},
{ 0x700301CF	, 128	, 1024	, "AT32F415RBW"	},
{ 0x70030243	, 256	, 2048	, "AT32F415RCT7-7"	},
{ 0x70030240	, 256	, 2048	, "AT32F415RCT7"	},
{ 0x7003024E	, 256	, 2048	, "AT32F415RCW"	},
{ 0x5001000C	, 16	, 1024	, "AT32F421C4T7"	},
{ 0x50020086	, 32	, 1024	, "AT32F421C6T7"	},
{ 0x50020100	, 64	, 1024	, "AT32F421C8T7"	},
{ 0xD0020100	, 64	, 1024	, "AT32F421C8W-YY"	},
{ 0x50020117	, 64	, 1024	, "AT32F421C8W"	},
{ 0x50010011	, 16	, 1024	, "AT32F421F4P7"	},
{ 0x50010010	, 16	, 1024	, "AT32F421F4U7"	},
{ 0x5002008B	, 32	, 1024	, "AT32F421F6P7"	},
{ 0x5002008A	, 32	, 1024	, "AT32F421F6U7"	},
{ 0x50020105	, 64	, 1024	, "AT32F421F8P7"	},
{ 0x50020104	, 64	, 1024	, "AT32F421F8U7"	},
{ 0x50010014	, 16	, 1024	, "AT32F421G4U7"	},
{ 0x50020093	, 32	, 1024	, "AT32F421G6U7"	},
{ 0x50020112	, 64	, 1024	, "AT32F421G8U7"	},
{ 0x5001000D	, 16	, 1024	, "AT32F421K4T7"	},
{ 0x5001000F	, 16	, 1024	, "AT32F421K4U7-4"	},
{ 0x5001000E	, 16	, 1024	, "AT32F421K4U7"	},
{ 0x50020087	, 32	, 1024	, "AT32F421K6T7"	},
{ 0x50020089	, 32	, 1024	, "AT32F421K6U7-4"	},
{ 0x50020088	, 32	, 1024	, "AT32F421K6U7"	},
{ 0x50020101	, 64	, 1024	, "AT32F421K8T7"	},
{ 0x50020103	, 64	, 1024	, "AT32F421K8U7-4"	},
{ 0x50020102	, 64	, 1024	, "AT32F421K8U7"	},
{ 0x50010016	, 16	, 1024	, "AT32F421PF4P7"	},
{ 0x50020115	, 64	, 1024	, "AT32F421PF8P7"	},
{ 0x7003210B	, 64	, 1024	, "AT32F423C8T7"	},
{ 0x7003210E	, 64	, 1024	, "AT32F423C8U7"	},
{ 0x700A21CA	, 128	, 1024	, "AT32F423CBT7"	},
{ 0x700A21CD	, 128	, 1024	, "AT32F423CBU7"	},
{ 0x700A3249	, 256	, 2048	, "AT32F423CCT7"	},
{ 0x700A324C	, 256	, 2048	, "AT32F423CCU7"	},
{ 0x70032115	, 64	, 1024	, "AT32F423K8U7-4"	},
{ 0x700A21D4	, 128	, 1024	, "AT32F423KBU7-4"	},
{ 0x700A3253	, 256	, 2048	, "AT32F423KCU7-4"	},
{ 0x70032108	, 64	, 1024	, "AT32F423R8T7-7"	},
{ 0x70032105	, 64	, 1024	, "AT32F423R8T7"	},
{ 0x700A21C7	, 128	, 1024	, "AT32F423RBT7-7"	},
{ 0x700A21C4	, 128	, 1024	, "AT32F423RBT7"	},
{ 0x700A3246	, 256	, 2048	, "AT32F423RCT7-7"	},
{ 0x700A3243	, 256	, 2048	, "AT32F423RCT7"	},
{ 0x70032112	, 64	, 1024	, "AT32F423T8U7"	},
{ 0x700A21D1	, 128	, 1024	, "AT32F423TBU7"	},
{ 0x700A3250	, 256	, 2048	, "AT32F423TCU7"	},
{ 0x70032102	, 64	, 1024	, "AT32F423V8T7"	},
{ 0x700A21C1	, 128	, 1024	, "AT32F423VBT7"	},
{ 0x700A3240	, 256	, 2048	, "AT32F423VCT7"	},
{ 0x50092087	, 32	, 1024	, "AT32F425C6T7"	},
{ 0x5009208A	, 32	, 1024	, "AT32F425C6U7"	},
{ 0x50092106	, 64	, 1024	, "AT32F425C8T7"	},
{ 0x50092109	, 64	, 1024	, "AT32F425C8U7"	},
{ 0x50092093	, 32	, 1024	, "AT32F425F6P7"	},
{ 0x50092112	, 64	, 1024	, "AT32F425F8P7"	},
{ 0x50092096	, 32	, 1024	, "AT32F425G6U7"	},
{ 0x50092115	, 64	, 1024	, "AT32F425G8U7"	},
{ 0x5009208D	, 32	, 1024	, "AT32F425K6T7"	},
{ 0x50092090	, 32	, 1024	, "AT32F425K6U7-4"	},
{ 0x5009210C	, 64	, 1024	, "AT32F425K8T7"	},
{ 0x5009210F	, 64	, 1024	, "AT32F425K8U7-4"	},
{ 0x50092084	, 32	, 1024	, "AT32F425R6T7-7"	},
{ 0x50092081	, 32	, 1024	, "AT32F425R6T7"	},
{ 0x50092103	, 64	, 1024	, "AT32F425R8T7-7"	},
{ 0x50092100	, 64	, 1024	, "AT32F425R8T7"	},
{ 0x7008449A	, 192	, 4096	, "AT32F435CCT7-W"	},
{ 0x7008324B	, 256	, 2048	, "AT32F435CCT7"	},
{ 0x7008449D	, 192	, 4096	, "AT32F435CCU7-W"	},
{ 0x7008324E	, 256	, 2048	, "AT32F435CCU7"	},
{ 0x700844D9	, 960	, 4096	, "AT32F435CGT7-W"	},
{ 0x7008334A	, 1024	, 2048	, "AT32F435CGT7"	},
{ 0x700844DC	, 960	, 4096	, "AT32F435CGU7-W"	},
{ 0x7008334D	, 1024	, 2048	, "AT32F435CGU7"	},
{ 0x70084558	, 4032	, 4096	, "AT32F435CMT7-E"	},
{ 0x70084549	, 4032	, 4096	, "AT32F435CMT7"	},
{ 0x7008455B	, 4032	, 4096	, "AT32F435CMU7-E"	},
{ 0x7008454C	, 4032	, 4096	, "AT32F435CMU7"	},
{ 0x70083248	, 256	, 2048	, "AT32F435RCT7"	},
{ 0x70083347	, 1024	, 2048	, "AT32F435RGT7"	},
{ 0x70084546	, 4032	, 4096	, "AT32F435RMT7"	},
{ 0x70083245	, 256	, 2048	, "AT32F435VCT7"	},
{ 0x70083344	, 1024	, 2048	, "AT32F435VGT7"	},
{ 0x70084543	, 4032	, 4096	, "AT32F435VMT7"	},
{ 0x70083242	, 256	, 2048	, "AT32F435ZCT7"	},
{ 0x70083341	, 1024	, 2048	, "AT32F435ZGT7"	},
{ 0x70084540	, 4032	, 4096	, "AT32F435ZMT7"	},
{ 0x70083257	, 256	, 2048	, "AT32F437RCT7"	},
{ 0x70083356	, 1024	, 2048	, "AT32F437RGT7"	},
{ 0x70084555	, 4032	, 4096	, "AT32F437RMT7"	},
{ 0x70083254	, 256	, 2048	, "AT32F437VCT7"	},
{ 0x70083353	, 1024	, 2048	, "AT32F437VGT7"	},
{ 0x70084552	, 4032	, 4096	, "AT32F437VMT7"	},
{ 0x70083251	, 256	, 2048	, "AT32F437ZCT7"	},
{ 0x70083350	, 1024	, 2048	, "AT32F437ZGT7"	},
{ 0x7008454F	, 4032	, 4096	, "AT32F437ZMT7"	},
{ 0x70030109	, 64	, 1024	, "AT32FEBKC8T7"	},
{ 0x10012006	, 16	, 1024	, "AT32L021C4T7"	},
{ 0x1001208D	, 32	, 1024	, "AT32L021C6T7"	},
{ 0x10012114	, 64	, 1024	, "AT32L021C8T7"	},
{ 0x10012001	, 16	, 1024	, "AT32L021F4P7"	},
{ 0x10012002	, 16	, 1024	, "AT32L021F4U7"	},
{ 0x10012088	, 32	, 1024	, "AT32L021F6P7"	},
{ 0x10012089	, 32	, 1024	, "AT32L021F6U7"	},
{ 0x1001210F	, 64	, 1024	, "AT32L021F8P7"	},
{ 0x10012110	, 64	, 1024	, "AT32L021F8U7"	},
{ 0x10012000	, 16	, 1024	, "AT32L021G4U7"	},
{ 0x10012087	, 32	, 1024	, "AT32L021G6U7"	},
{ 0x1001210E	, 64	, 1024	, "AT32L021G8U7"	},
{ 0x10012005	, 16	, 1024	, "AT32L021K4T7"	},
{ 0x10012003	, 16	, 1024	, "AT32L021K4U7-4"	},
{ 0x10012004	, 16	, 1024	, "AT32L021K4U7"	},
{ 0x1001208C	, 32	, 1024	, "AT32L021K6T7"	},
{ 0x1001208A	, 32	, 1024	, "AT32L021K6U7-4"	},
{ 0x1001208B	, 32	, 1024	, "AT32L021K6U7"	},
{ 0x10012113	, 64	, 1024	, "AT32L021K8T7"	},
{ 0x10012111	, 64	, 1024	, "AT32L021K8U7-4"	},
{ 0x10012112	, 64	, 1024	, "AT32L021K8U7"	},
{ 0x70030250	, 256	, 2048	, "AT32WB415CCU7-7"	},
{ 0xF00301C2	, 128	, 1024	, "KC9060"          },
{ 0             , 0     , 0     , NULL              }
};

static int artery_find_chip_from_id(uint32_t id, const struct artery_chip_info **chip_info)
{
	const struct artery_chip_info *curr_chip = known_artery_chips;
	while (curr_chip->chip_id != 0 || curr_chip->chip_name != NULL) {
		if (curr_chip->chip_id == id) {
			if (chip_info)
				*chip_info = curr_chip;
			return ERROR_OK;
		}
		curr_chip += 1;
	}
	return ERROR_FAIL;
}

static int artery_guess_sector_size_from_flash_size(int flash_size_kb)
{
	/* According to device DB, devices with 4096 byte sectors do not have a power of 2 kB of FLASH */
	if ((flash_size_kb & (flash_size_kb - 1)) != 0)
		return 4096;

	/* According to AT32F415 code examples, FLASH <= 128kB means 1024 bytes setor size */
	if (flash_size_kb <= 128)
		return 1024;

	/* Other devices have 2048 bytes per sectors */
	return 2048;
}

static int artery_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct artery_flash_bank *artery_bank_info = bank->driver_priv;
	uint32_t device_id, sector_size;
	uint16_t read_flash_size_in_kb;
	unsigned int num_sectors;
	unsigned int flash_size;
	const struct artery_chip_info *chip_info = 0;
	int retval;

	artery_bank_info->probed = false;

	free(bank->sectors);
	bank->num_sectors = 0;
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->num_prot_blocks = 0;
	bank->prot_blocks = NULL;

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	/* Read device ID */
	retval = target_read_u32(target, MCU_DEVICE_ID_ADDR, &device_id);
	if (retval != ERROR_OK) {
		LOG_WARNING("Cannot read device ID.");
		return retval;
	}

	/* get flash size from target. */
	retval = target_read_u16(target, FLASH_SIZE_ADDR, &read_flash_size_in_kb);
	if (retval != ERROR_OK || read_flash_size_in_kb == 0xffff || read_flash_size_in_kb == 0) {
		LOG_WARNING("Cannot read flash size.");
		return ERROR_FAIL;
	}

	/* look up chip id in known chip db */
	retval = artery_find_chip_from_id(device_id, &chip_info);
	if (retval == ERROR_OK) {
		/* known flash size matches read flash size. Trust known sector size */
		if (read_flash_size_in_kb == chip_info->flash_size_kB) {
			sector_size = chip_info->sector_size;
			LOG_INFO("Chip: %s, %" PRIi32 "kB FLASH, %" PRIi32 " bytes sectors",
						chip_info->chip_name, read_flash_size_in_kb, sector_size);
		}

		/* Known flash size does not match read flash size. Guess sector size */
		else {
			sector_size = artery_guess_sector_size_from_flash_size(read_flash_size_in_kb);
			LOG_INFO("Chip: %s, %" PRIi32 "kB FLASH expected, but %" PRIi32 \
						"kB detected. Guessing %" PRIi32 " bytes sectors",
						chip_info->chip_name, chip_info->flash_size_kB,
						read_flash_size_in_kb, sector_size);
		}
	}

	/* Unknown chip. Guess sector size */
	else {
		sector_size = artery_guess_sector_size_from_flash_size(read_flash_size_in_kb);
		LOG_INFO("Unknown chip id: %" PRIi32 ", %" PRIi32 \
					"kB FLASH detected. Guessing %" PRIi32 " bytes sectors",
					device_id, read_flash_size_in_kb, sector_size);
	}

	flash_size = read_flash_size_in_kb;
	flash_size <<= 10;

	num_sectors = flash_size / sector_size;
	if ((num_sectors * sector_size) != flash_size) {
		LOG_ERROR("Total FLASH size does not match sector size times sectors count !");
		return ERROR_FAIL;
	}

	bank->base = FLASH_BASE_ADDR;
	bank->num_sectors = num_sectors;
	bank->sectors = calloc(num_sectors, sizeof(struct flash_sector));
	bank->size = flash_size;
	for (unsigned int i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * sector_size;
		bank->sectors[i].size = sector_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
		/* Currently we simply ignore sector protection. TODO : implement protection read / write */
	}
	LOG_DEBUG("allocated %u sectors", num_sectors);

	artery_bank_info->probed = true;
	return ERROR_OK;
}

/* flash bank stm32x <base> <size> 0 0 <target#>
*/
FLASH_BANK_COMMAND_HANDLER(artery_flash_bank_command)
{
	struct artery_flash_bank *artery_bank_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	artery_bank_info = malloc(sizeof(struct artery_flash_bank));
	bank->driver_priv = artery_bank_info;

	artery_bank_info->probed = false;

	return ERROR_OK;
}

static const struct command_registration artery_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static int artery_auto_probe(struct flash_bank *bank)
{
	struct artery_flash_bank *artery_bank_info = bank->driver_priv;
	if (artery_bank_info->probed)
		return ERROR_OK;
	return artery_probe(bank);
}

static int artery_print_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct target *target = bank->target;
	uint32_t device_id;
	uint16_t flash_size_in_kb;
	uint8_t mask_version;
	int retval;
	const struct artery_chip_info *chip_info = 0;

	/* Read device ID */
	retval = target_read_u32(target, MCU_DEVICE_ID_ADDR, &device_id);
	if (retval != ERROR_OK) {
		LOG_WARNING("Cannot read device ID.");
		return retval;
	}

	/* Read revision */
	retval = target_read_u8(target, MASK_VERSION_ADDR, &mask_version);
	if (retval != ERROR_OK) {
		LOG_WARNING("Cannot read mask version.");
		return retval;
	}
	mask_version = ((mask_version >> 4) & 0x07) + 'A';

	/* Read Flash size */
	retval = target_read_u16(target, FLASH_SIZE_ADDR, &flash_size_in_kb);
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("Cannot read flash size.");
		return retval;
	}

	/* look up chip id in known chip db */
	retval = artery_find_chip_from_id(device_id, &chip_info);
	if (retval == ERROR_OK)
		command_print_sameline(cmd, "Chip: %s Rev. %c, %" PRIi32 "kB FLASH",
							   chip_info->chip_name, mask_version, flash_size_in_kb);
	else
		command_print_sameline(cmd, "Unknown chip, Id: 0x%08" PRIx32 \
							   ", Rev: %c, %" PRIi32 "kB FLASH",
							   device_id, mask_version, flash_size_in_kb);

	return ERROR_OK;
}

static int artery_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = target_read_u32(target, EFC_STS_REG, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32, status);
		if ((status & EFC_BSY_BIT) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & EFC_WRPRTFLR_BIT) {
		LOG_ERROR("Device protected");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & EFC_WRPRTFLR_BIT) {
		LOG_ERROR("Attempt to write an address that has not been erased before");
		retval = ERROR_FAIL;
	}
	return retval;
}


static int artery_unlock_flash_write(struct target *target)
{
	uint32_t ctrl;

	int retval = target_read_u32(target, EFC_CTRL_REG, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & EFC_LOCK_BIT) == 0)
		return ERROR_OK;

	/* unlock flash registers */
	retval = target_write_u32(target, EFC_FCKEY_REG, EFC_KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, EFC_FCKEY_REG, EFC_KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, EFC_CTRL_REG, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & EFC_LOCK_BIT) {
		LOG_ERROR("flash not unlocked FLASH_CTRL: 0x%" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int artery_erase(struct flash_bank *bank, unsigned int first,
						unsigned int last)
{
	struct target *target = bank->target;

	assert((first <= last) && (last < bank->num_sectors));

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval;
	retval = artery_unlock_flash_write(target);
	if (retval != ERROR_OK)
		return retval;

	/* Wait for flash not busy */
	retval = artery_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = first; i <= last; i++) {
		/* Clear status register beforehand */
		target_write_u32(target, EFC_STS_REG, EFC_PRCDN_BIT | EFC_WRPRTFLR_BIT | EFC_PRGMFLR_BIT);

		/* Set the PGERS bit in the FLASH_CTRLx register */
		retval = target_write_u32(target, EFC_CTRL_REG, EFC_PGERS_BIT);
		if (retval != ERROR_OK)
			return retval;

		/* Select the page to be erased with the FLASH_ADDRx register */
		target_addr_t eraseAddress = bank->base + bank->sectors[i].offset;
		retval = target_write_u32(target, EFC_ADDR_REG, eraseAddress);
		if (retval != ERROR_OK)
			return retval;

		/* Set the RSTR bit in the FLASH_CTRLx register */
		retval = target_write_u32(target, EFC_CTRL_REG, EFC_PGERS_BIT | EFC_RSTR_BIT);
		if (retval != ERROR_OK)
			return retval;

		/* Wait for flash not busy */
		retval = artery_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Re-lock flash */
	retval = target_write_u32(target, EFC_CTRL_REG, EFC_LOCK_BIT);
	return retval;
}

static int artery_write(struct flash_bank *bank, const uint8_t *buffer,
						uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	target_addr_t write_address = bank->base + offset;
	uint32_t bytes_written = 0;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = artery_unlock_flash_write(target);
	if (retval != ERROR_OK)
		return retval;

	/* Write byte by byte until we align to a word */
	while ((bytes_written < count) && (write_address & 0x03)) {
		/* Set the PRGM bit = 1 in FLASH_CTRL */
		retval = target_write_u32(target, EFC_CTRL_REG, EFC_PRGM_BIT);
		if (retval != ERROR_OK)
			return retval;

		/* Write byte to flash */
		retval = target_write_u8(target, write_address, buffer[bytes_written]);
		if (retval != ERROR_OK)
			return retval;

		retval = artery_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 1;
		write_address += 1;
	}

	/* First write word by word, as it provides the highest write speed */
	while (bytes_written < (count - 3)) {
		/* Set the PRGM bit = 1 in FLASH_CTRL */
		retval = target_write_u32(target, EFC_CTRL_REG, EFC_PRGM_BIT);
		if (retval != ERROR_OK)
			return retval;

		/* Write word to flash */
		retval = target_write_memory(target, write_address, 4, 1, buffer + bytes_written);
		if (retval != ERROR_OK)
			return retval;

		retval = artery_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 4;
		write_address += 4;
	}

	/* Write potential last bytes */
	while (bytes_written < count) {
		/* Set the PRGM bit = 1 in FLASH_CTRL */
		retval = target_write_u32(target, EFC_CTRL_REG, EFC_PRGM_BIT);
		if (retval != ERROR_OK)
			return retval;

		/* Write byte to flash */
		retval = target_write_u8(target, write_address, buffer[bytes_written]);
		if (retval != ERROR_OK)
			return retval;

		retval = artery_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 1;
		write_address += 1;
	}

	/* Re-lock flash */
	retval = target_write_u32(target, EFC_CTRL_REG, EFC_LOCK_BIT);
	return retval;
}

static const struct command_registration artery_command_handlers[] = {
{
	.name = "artery",
	.mode = COMMAND_ANY,
	.help = "artery flash command group",
	.usage = "",
	.chain = artery_exec_command_handlers,
},
COMMAND_REGISTRATION_DONE
};


const struct flash_driver artery_flash = {
	.name = "artery",
	.commands = artery_command_handlers,
	.flash_bank_command = artery_flash_bank_command,
	.erase = artery_erase,
	.protect = NULL,
	.write = artery_write,
	.read = default_flash_read,
	.probe = artery_probe,
	.auto_probe = artery_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = NULL,
	.info = artery_print_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

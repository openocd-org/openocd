/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/***************************************************************************
* STELLARIS is tested on LM3S811, LM3S6965
***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"

#include "stellaris.h"
#include "cortex_m3.h"

#include "flash.h"
#include "target.h"
#include "log.h"
#include "binarybuffer.h"
#include "types.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define DID0_VER(did0) ((did0>>28)&0x07)
int stellaris_register_commands(struct command_context_s *cmd_ctx);
int stellaris_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int stellaris_erase(struct flash_bank_s *bank, int first, int last);
int stellaris_protect(struct flash_bank_s *bank, int set, int first, int last);
int stellaris_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int stellaris_auto_probe(struct flash_bank_s *bank);
int stellaris_probe(struct flash_bank_s *bank);
int stellaris_protect_check(struct flash_bank_s *bank);
int stellaris_info(struct flash_bank_s *bank, char *buf, int buf_size);

int stellaris_read_part_info(struct flash_bank_s *bank);
u32 stellaris_get_flash_status(flash_bank_t *bank);
void stellaris_set_flash_mode(flash_bank_t *bank,int mode);
u32 stellaris_wait_status_busy(flash_bank_t *bank, u32 waitbits, int timeout);

int stellaris_read_part_info(struct flash_bank_s *bank);
int stellaris_handle_mass_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int stellaris_mass_erase(struct flash_bank_s *bank);

flash_driver_t stellaris_flash =
{
	.name = "stellaris",
	.register_commands = stellaris_register_commands,
	.flash_bank_command = stellaris_flash_bank_command,
	.erase = stellaris_erase,
	.protect = stellaris_protect,
	.write = stellaris_write,
	.probe = stellaris_probe,
	.auto_probe = stellaris_auto_probe,
	.erase_check = default_flash_mem_blank_check,
	.protect_check = stellaris_protect_check,
	.info = stellaris_info
};

struct {
	u32 partno;
	char *partname;
}	StellarisParts[] =
{
	{0x01,"LM3S101"},
	{0x02,"LM3S102"},
	{0x03,"LM3S1625"},
	{0x04,"LM3S1626"},
	{0x05,"LM3S1627"},
	{0x06,"LM3S1607"},
	{0x10,"LM3S1776"},
	{0x19,"LM3S300"},
	{0x11,"LM3S301"},
	{0x12,"LM3S310"},
	{0x1A,"LM3S308"},
	{0x13,"LM3S315"},
	{0x14,"LM3S316"},
	{0x17,"LM3S317"},
	{0x18,"LM3S318"},
	{0x15,"LM3S328"},
	{0x2A,"LM3S600"},
	{0x21,"LM3S601"},
	{0x2B,"LM3S608"},
	{0x22,"LM3S610"},
	{0x23,"LM3S611"},
	{0x24,"LM3S612"},
	{0x25,"LM3S613"},
	{0x26,"LM3S615"},
	{0x28,"LM3S617"},
	{0x29,"LM3S618"},
	{0x27,"LM3S628"},
	{0x38,"LM3S800"},
	{0x31,"LM3S801"},
	{0x39,"LM3S808"},
	{0x32,"LM3S811"},
	{0x33,"LM3S812"},
	/*{0x33,"LM3S2616"},*/
	{0x34,"LM3S815"},
	{0x36,"LM3S817"},
	{0x37,"LM3S818"},
	{0x35,"LM3S828"},
	{0x39,"LM3S2276"},
	{0x3A,"LM3S2776"},
	{0x43,"LM3S3651"},
	{0x44,"LM3S3739"},
	{0x45,"LM3S3749"},
	{0x46,"LM3S3759"},
	{0x48,"LM3S3768"},
	{0x49,"LM3S3748"},
	{0x50,"LM3S2678"},
	{0x51,"LM3S2110"},
	{0x52,"LM3S2739"},
	{0x53,"LM3S2651"},
	{0x54,"LM3S2939"},
	{0x55,"LM3S2965"},
	{0x56,"LM3S2432"},
	{0x57,"LM3S2620"},
	{0x58,"LM3S2950"},
	{0x59,"LM3S2412"},
	{0x5A,"LM3S2533"},
	{0x61,"LM3S8630"},
	{0x62,"LM3S8970"},
	{0x63,"LM3S8730"},
	{0x64,"LM3S8530"},
	{0x65,"LM3S8930"},
	{0x71,"LM3S6610"},
	{0x72,"LM3S6950"},
	{0x73,"LM3S6965"},
	{0x74,"LM3S6110"},
	{0x75,"LM3S6432"},
	{0x76,"LM3S6537"},
	{0x77,"LM3S6753"},
	{0x78,"LM3S6952"},
	{0x80,"LM3S2671"},
	{0x81,"LM3S5632"},
	{0x82,"LM3S6422"},
	{0x83,"LM3S6633"},
	{0x84,"LM3S2139"},
	{0x85,"LM3S2637"},
	{0x86,"LM3S8738"},
	{0x88,"LM3S8938"},
	{0x89,"LM3S6938"},
	{0x8A,"LM3S5652"},
	{0x8B,"LM3S6637"},
	{0x8C,"LM3S8933"},
	{0x8D,"LM3S8733"},
	{0x8E,"LM3S8538"},
	{0x8F,"LM3S2948"},
	{0x91,"LM3S5662"},
	{0x96,"LM3S5732"},
	{0x97,"LM3S5737"},
	{0x99,"LM3S5747"},
	{0x9A,"LM3S5752"},
	{0x9B,"LM3S5757"},
	{0x9C,"LM3S5762"},
	{0x9D,"LM3S5767"},
	{0xA0,"LM3S5739"},
	{0xA1,"LM3S6100"},
	{0xA2,"LM3S2410"},
	{0xA3,"LM3S6730"},
	{0xA4,"LM3S2730"},
	{0xA5,"LM3S6420"},
	{0xA6,"LM3S8962"},
	{0xA7,"LM3S5749"},
	{0xA8,"LM3S5769"},
	{0xA9,"LM3S5768"},
	{0xB3,"LM3S1635"},
	{0xB4,"LM3S1850"},
	{0xB5,"LM3S1960"},
	{0xB7,"LM3S1937"},
	{0xB8,"LM3S1968"},
	{0xB9,"LM3S1751"},
	{0xBA,"LM3S1439"},
	{0xBB,"LM3S1512"},
	{0xBC,"LM3S1435"},
	{0xBD,"LM3S1637"},
	{0xBE,"LM3S1958"},
	{0xBF,"LM3S1110"},
	{0xC0,"LM3S1620"},
	{0xC1,"LM3S1150"},
	{0xC2,"LM3S1165"},
	{0xC3,"LM3S1133"},
	{0xC4,"LM3S1162"},
	{0xC5,"LM3S1138"},
	{0xC6,"LM3S1332"},
	{0xC7,"LM3S1538"},
	{0xD0,"LM3S6815"},
	{0xD1,"LM3S6816"},
	{0xD2,"LM3S6915"},
	{0xD3,"LM3S6916"},
	{0xD4,"LM3S2016"},
	{0xD5,"LM3S1615"},
	{0xD6,"LM3S1616"},
	{0xD7,"LM3S8971"},
	{0xD8,"LM3S1108"},
	{0xD9,"LM3S1101"},
	{0xDA,"LM3S1608"},
	{0xDB,"LM3S1601"},
	{0xDC,"LM3S1918"},
	{0xDD,"LM3S1911"},
	{0xDE,"LM3S2108"},
	{0xDF,"LM3S2101"},
	{0xE0,"LM3S2608"},
	{0xE1,"LM3S2601"},
	{0xE2,"LM3S2918"},
	{0xE3,"LM3S2911"},
	{0xE4,"LM3S6118"},
	{0xE5,"LM3S6111"},
	{0xE6,"LM3S6618"},
	{0xE7,"LM3S6611"},
	{0xE8,"LM3S6918"},
	{0xE9,"LM3S6911"},
	{0,"Unknown part"}
};

char * StellarisClassname[5] =
{
	"Sandstorm",
	"Fury",
	"Unknown",
	"DustDevil",
	"Tempest"
};

/***************************************************************************
*	openocd command interface                                              *
***************************************************************************/

/* flash_bank stellaris <base> <size> 0 0 <target#>
 */
int stellaris_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	stellaris_flash_bank_t *stellaris_info;
	
	if (argc < 6)
	{
		LOG_WARNING("incomplete flash_bank stellaris configuration");
		return ERROR_FLASH_BANK_INVALID;
	}
	
	stellaris_info = calloc(sizeof(stellaris_flash_bank_t), 1);
	bank->base = 0x0;
	bank->driver_priv = stellaris_info;
	
	stellaris_info->target_name = "Unknown target";
	
	/* part wasn't probed for info yet */
	stellaris_info->did1 = 0;
	
	/* TODO Use an optional main oscillator clock rate in kHz from arg[6] */ 
	return ERROR_OK;
}

int stellaris_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *stm32x_cmd = register_command(cmd_ctx, NULL, "stellaris", NULL, COMMAND_ANY, "stellaris flash specific commands");
	
	register_command(cmd_ctx, stm32x_cmd, "mass_erase", stellaris_handle_mass_erase_command, COMMAND_EXEC, "mass erase device");
	return ERROR_OK;
}

int stellaris_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	int printed, device_class;
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	
	stellaris_read_part_info(bank);

	if (stellaris_info->did1 == 0)
	{
		printed = snprintf(buf, buf_size, "Cannot identify target as a Stellaris\n");
		buf += printed;
		buf_size -= printed;
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	if (DID0_VER(stellaris_info->did0) > 0)
	{
		device_class = (stellaris_info->did0>>16) & 0xFF;
	}
	else
	{
		device_class = 0;
	}	
	printed = snprintf(buf, buf_size, "\nLMI Stellaris information: Chip is class %i(%s) %s v%c.%i\n",
	  device_class, StellarisClassname[device_class], stellaris_info->target_name,
	  'A' + ((stellaris_info->did0>>8) & 0xFF), (stellaris_info->did0) & 0xFF);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "did1: 0x%8.8x, arch: 0x%4.4x, eproc: %s, ramsize:%ik, flashsize: %ik\n", 
	 stellaris_info->did1, stellaris_info->did1, "ARMV7M", (1+((stellaris_info->dc0>>16) & 0xFFFF))/4, (1+(stellaris_info->dc0 & 0xFFFF))*2);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "master clock(estimated): %ikHz, rcc is 0x%x \n", stellaris_info->mck_freq / 1000, stellaris_info->rcc);
	buf += printed;
	buf_size -= printed;

	if (stellaris_info->num_lockbits>0)
	{
		printed = snprintf(buf, buf_size, "pagesize: %i, lockbits: %i 0x%4.4x, pages in lock region: %i \n", stellaris_info->pagesize, stellaris_info->num_lockbits, stellaris_info->lockbits,stellaris_info->num_pages/stellaris_info->num_lockbits);
		buf += printed;
		buf_size -= printed;
	}
	return ERROR_OK;
}

/***************************************************************************
*	chip identification and status                                         *
***************************************************************************/

u32 stellaris_get_flash_status(flash_bank_t *bank)
{
	target_t *target = bank->target;
	u32 fmc;
	
	target_read_u32(target, FLASH_CONTROL_BASE|FLASH_FMC, &fmc);
	
	return fmc;
}

/** Read clock configuration and set stellaris_info->usec_clocks*/
 
void stellaris_read_clock_info(flash_bank_t *bank)
{
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 rcc, pllcfg, sysdiv, usesysdiv, bypass, oscsrc;
	unsigned long mainfreq;

	target_read_u32(target, SCB_BASE|RCC, &rcc);
	LOG_DEBUG("Stellaris RCC %x", rcc);
	target_read_u32(target, SCB_BASE|PLLCFG, &pllcfg);
	LOG_DEBUG("Stellaris PLLCFG %x", pllcfg);
	stellaris_info->rcc = rcc;
	
	sysdiv = (rcc>>23) & 0xF;
	usesysdiv = (rcc>>22) & 0x1;
	bypass = (rcc>>11) & 0x1;
	oscsrc = (rcc>>4) & 0x3;
	/* xtal = (rcc>>6)&0xF; */
	switch (oscsrc)
	{
		case 0:
			mainfreq = 6000000;  /* Default xtal */
			break;
		case 1:
			mainfreq = 22500000; /* Internal osc. 15 MHz +- 50% */
			break;
		case 2:
			mainfreq = 5625000;  /* Internal osc. / 4 */
			break;
		case 3:
			LOG_WARNING("Invalid oscsrc (3) in rcc register");
			mainfreq = 6000000;
			break;

		default: /* NOTREACHED */
			mainfreq = 0;
			break;
	}
	
	if (!bypass)
		mainfreq = 200000000; /* PLL out frec */
		
	if (usesysdiv)
		stellaris_info->mck_freq = mainfreq/(1+sysdiv);
	else
		stellaris_info->mck_freq = mainfreq;
	
	/* Forget old flash timing */
	stellaris_set_flash_mode(bank, 0);
}

/* Setup the timimg registers */
void stellaris_set_flash_mode(flash_bank_t *bank,int mode)
{
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	target_t *target = bank->target;

	u32 usecrl = (stellaris_info->mck_freq/1000000ul-1);
	LOG_DEBUG("usecrl = %i",usecrl);	
	target_write_u32(target, SCB_BASE|USECRL, usecrl);
}

u32 stellaris_wait_status_busy(flash_bank_t *bank, u32 waitbits, int timeout)
{
	u32 status;
	
	/* Stellaris waits for cmdbit to clear */
	while (((status = stellaris_get_flash_status(bank)) & waitbits) && (timeout-- > 0))
	{
		LOG_DEBUG("status: 0x%x", status);
		alive_sleep(1);
	}
	
	/* Flash errors are reflected in the FLASH_CRIS register */

	return status;
}

/* Send one command to the flash controller */
int stellaris_flash_command(struct flash_bank_s *bank,u8 cmd,u16 pagen) 
{
	u32 fmc;
	target_t *target = bank->target;

	fmc = FMC_WRKEY | cmd; 
	target_write_u32(target, FLASH_CONTROL_BASE|FLASH_FMC, fmc);
	LOG_DEBUG("Flash command: 0x%x", fmc);

	if (stellaris_wait_status_busy(bank, cmd, 100)) 
	{
		return ERROR_FLASH_OPERATION_FAILED;
	}		

	return ERROR_OK;
}

/* Read device id register, main clock frequency register and fill in driver info structure */
int stellaris_read_part_info(struct flash_bank_s *bank)
{
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 did0, did1, ver, fam, status;
	int i;
	
	/* Read and parse chip identification register */
	target_read_u32(target, SCB_BASE|DID0, &did0);
	target_read_u32(target, SCB_BASE|DID1, &did1);
	target_read_u32(target, SCB_BASE|DC0, &stellaris_info->dc0);
	target_read_u32(target, SCB_BASE|DC1, &stellaris_info->dc1);
	LOG_DEBUG("did0 0x%x, did1 0x%x, dc0 0x%x, dc1 0x%x", did0, did1, stellaris_info->dc0, stellaris_info->dc1);

	ver = did0 >> 28;
	if((ver != 0) && (ver != 1))
	{
		LOG_WARNING("Unknown did0 version, cannot identify target");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (did1 == 0)
	{
		LOG_WARNING("Cannot identify target as a Stellaris");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	ver = did1 >> 28;
	fam = (did1 >> 24) & 0xF;
	if(((ver != 0) && (ver != 1)) || (fam != 0))
	{
		LOG_WARNING("Unknown did1 version/family, cannot positively identify target as a Stellaris");
	}

	for (i = 0; StellarisParts[i].partno; i++)
	{
		if (StellarisParts[i].partno == ((did1 >> 16) & 0xFF))
			break;
	}
	
	stellaris_info->target_name = StellarisParts[i].partname;
	
	stellaris_info->did0 = did0;
	stellaris_info->did1 = did1;

	stellaris_info->num_lockbits = 1 + (stellaris_info->dc0 & 0xFFFF);
	stellaris_info->num_pages = 2 *(1+(stellaris_info->dc0 & 0xFFFF));
	stellaris_info->pagesize = 1024;
	bank->size = 1024 * stellaris_info->num_pages;
	stellaris_info->pages_in_lockregion = 2;
	target_read_u32(target, SCB_BASE|FMPPE, &stellaris_info->lockbits);

	/* provide this for the benefit of the higher flash driver layers */
	bank->num_sectors = stellaris_info->num_pages;
	bank->sectors = malloc(sizeof(flash_sector_t) * bank->num_sectors);
	for (i = 0; i < bank->num_sectors; i++)
	{
		bank->sectors[i].offset = i * stellaris_info->pagesize;
		bank->sectors[i].size = stellaris_info->pagesize;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	/* Read main and master clock freqency register */
	stellaris_read_clock_info(bank);
	
	status = stellaris_get_flash_status(bank);
	
	return ERROR_OK;
}

/***************************************************************************
*	flash operations                                                       *
***************************************************************************/

int stellaris_protect_check(struct flash_bank_s *bank)
{
	u32 status;
	
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stellaris_info->did1 == 0)
	{
		stellaris_read_part_info(bank);
	}

	if (stellaris_info->did1 == 0)
	{
		LOG_WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}
		
	status = stellaris_get_flash_status(bank);
	stellaris_info->lockbits = status >> 16;
	
	return ERROR_OK;
}

int stellaris_erase(struct flash_bank_s *bank, int first, int last)
{
	int banknr;
	u32 flash_fmc, flash_cris;
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	target_t *target = bank->target;
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stellaris_info->did1 == 0)
	{
		stellaris_read_part_info(bank);
	}

	if (stellaris_info->did1 == 0)
	{
		LOG_WARNING("Cannot identify target as Stellaris");
		return ERROR_FLASH_OPERATION_FAILED;
	}	
	
	if ((first < 0) || (last < first) || (last >= stellaris_info->num_pages))
	{
		return ERROR_FLASH_SECTOR_INVALID;
	}
	
	if ((first == 0) && (last == (stellaris_info->num_pages-1)))
	{
		return stellaris_mass_erase(bank);
	}
	
	/* Configure the flash controller timing */
	stellaris_read_clock_info(bank);	
	stellaris_set_flash_mode(bank,0);

	/* Clear and disable flash programming interrupts */
	target_write_u32(target, FLASH_CIM, 0);
	target_write_u32(target, FLASH_MISC, PMISC|AMISC);
	
	for (banknr = first; banknr <= last; banknr++)
	{
		/* Address is first word in page */
		target_write_u32(target, FLASH_FMA, banknr * stellaris_info->pagesize);
		/* Write erase command */
		target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_ERASE);
		/* Wait until erase complete */
		do
		{
			target_read_u32(target, FLASH_FMC, &flash_fmc);
		}
		while(flash_fmc & FMC_ERASE);

		/* Check acess violations */
		target_read_u32(target, FLASH_CRIS, &flash_cris);
		if(flash_cris & (AMASK))
		{
			LOG_WARNING("Error erasing flash page %i,  flash_cris 0x%x", banknr, flash_cris);
			target_write_u32(target, FLASH_CRIS, 0);
			return ERROR_FLASH_OPERATION_FAILED;
		}
		
		bank->sectors[banknr].is_erased = 1;
	}

	return ERROR_OK;
}

int stellaris_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	u32 fmppe, flash_fmc, flash_cris;
	int lockregion;
	
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	target_t *target = bank->target;
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if ((first < 0) || (last < first) || (last >= stellaris_info->num_lockbits))
	{
		return ERROR_FLASH_SECTOR_INVALID;
	}
	
	if (stellaris_info->did1 == 0)
	{
		stellaris_read_part_info(bank);
	}

	if (stellaris_info->did1 == 0)
	{
		LOG_WARNING("Cannot identify target as an Stellaris MCU");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	/* Configure the flash controller timing */
	stellaris_read_clock_info(bank);
	stellaris_set_flash_mode(bank, 0);

	fmppe = stellaris_info->lockbits;
	for (lockregion = first; lockregion <= last; lockregion++)
	{
		if (set)
			fmppe &= ~(1<<lockregion); 
		else
			fmppe |= (1<<lockregion); 
	}

	/* Clear and disable flash programming interrupts */
	target_write_u32(target, FLASH_CIM, 0);
	target_write_u32(target, FLASH_MISC, PMISC|AMISC);
	
	LOG_DEBUG("fmppe 0x%x",fmppe);
	target_write_u32(target, SCB_BASE|FMPPE, fmppe);
	/* Commit FMPPE */
	target_write_u32(target, FLASH_FMA, 1);
	/* Write commit command */
	/* TODO safety check, sice this cannot be undone */
	LOG_WARNING("Flash protection cannot be removed once commited, commit is NOT executed !");
	/* target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_COMT); */
	/* Wait until erase complete */
	do
	{
		target_read_u32(target, FLASH_FMC, &flash_fmc);
	}
	while(flash_fmc & FMC_COMT);

	/* Check acess violations */
	target_read_u32(target, FLASH_CRIS, &flash_cris);
	if(flash_cris & (AMASK))
	{
		LOG_WARNING("Error setting flash page protection,  flash_cris 0x%x", flash_cris);
		target_write_u32(target, FLASH_CRIS, 0);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	target_read_u32(target, SCB_BASE|FMPPE, &stellaris_info->lockbits);
	
	return ERROR_OK;
}

u8 stellaris_write_code[] = 
{
/* 
	Call with :	
	r0 = buffer address
	r1 = destination address
	r2 = bytecount (in) - endaddr (work) 
	
	Used registers:	
	r3 = pFLASH_CTRL_BASE
	r4 = FLASHWRITECMD
	r5 = #1
	r6 = bytes written
	r7 = temp reg
*/
	0x07,0x4B,			/* ldr r3,pFLASH_CTRL_BASE */
	0x08,0x4C,			/* ldr r4,FLASHWRITECMD */
	0x01,0x25,			/* movs r5, 1 */
	0x00,0x26,			/* movs r6, #0 */
/* mainloop: */
	0x19,0x60,			/* str	r1, [r3, #0] */
	0x87,0x59,			/* ldr	r7, [r0, r6] */
	0x5F,0x60,			/* str	r7, [r3, #4] */
	0x9C,0x60,			/* str	r4, [r3, #8] */
/* waitloop: */
	0x9F,0x68,			/* ldr	r7, [r3, #8] */
	0x2F,0x42,			/* tst	r7, r5 */
	0xFC,0xD1,			/* bne	waitloop */
	0x04,0x31,			/* adds	r1, r1, #4 */
	0x04,0x36,			/* adds	r6, r6, #4 */
	0x96,0x42,			/* cmp	r6, r2 */
	0xF4,0xD1,			/* bne	mainloop */
						/* exit: */
	0xFE,0xE7,			/* b exit */
/* pFLASH_CTRL_BASE: */
	0x00,0xD0,0x0F,0x40,	/* .word	0x400FD000 */
/* FLASHWRITECMD: */
	0x01,0x00,0x42,0xA4 	/* .word	0xA4420001 */
};

int stellaris_write_block(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 wcount)
{
	target_t *target = bank->target;
	u32 buffer_size = 8192;
	working_area_t *source;
	working_area_t *write_algorithm;
	u32 address = bank->base + offset;
	reg_param_t reg_params[3];
	armv7m_algorithm_t armv7m_info;
	int retval = ERROR_OK;
	
	LOG_DEBUG("(bank=%p buffer=%p offset=%08X wcount=%08X)",
			bank, buffer, offset, wcount);

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(stellaris_write_code), &write_algorithm) != ERROR_OK)
	{
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	target_write_buffer(target, write_algorithm->address, sizeof(stellaris_write_code), stellaris_write_code);

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
		LOG_DEBUG("called target_alloc_working_area(target=%p buffer_size=%08X source=%p)",
				target, buffer_size, source); 
		buffer_size /= 2;
		if (buffer_size <= 256)
		{
			/* if we already allocated the writing code, but failed to get a buffer, free the algorithm */
			if (write_algorithm)
				target_free_working_area(target, write_algorithm);
			
			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	};
	
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARMV7M_MODE_ANY;
	
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	
	while (wcount > 0)
	{
		u32 thisrun_count = (wcount > (buffer_size / 4)) ? (buffer_size / 4) : wcount;
		
		target_write_buffer(target, source->address, thisrun_count * 4, buffer);
		
		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, 4*thisrun_count);
		LOG_INFO("Algorithm flash write %i words to 0x%x, %i remaining", thisrun_count, address, wcount);
		LOG_DEBUG("Algorithm flash write %i words to 0x%x, %i remaining", thisrun_count, address, wcount);
		if ((retval = target->type->run_algorithm(target, 0, NULL, 3, reg_params, write_algorithm->address, write_algorithm->address + sizeof(stellaris_write_code)-10, 10000, &armv7m_info)) != ERROR_OK)
		{
			LOG_ERROR("error executing stellaris flash write algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
	
		buffer += thisrun_count * 4;
		address += thisrun_count * 4;
		wcount -= thisrun_count;
	}
	
	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, source);
	
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	
	return retval;
}

int stellaris_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 address = offset;
	u32 flash_cris, flash_fmc;
	u32 words_remaining = (count / 4);
	u32 bytes_remaining = (count & 0x00000003);
	u32 bytes_written = 0;
	u32 retval;
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("(bank=%p buffer=%p offset=%08X count=%08X)",
			bank, buffer, offset, count);

	if (stellaris_info->did1 == 0)
	{
		stellaris_read_part_info(bank);
	}

	if (stellaris_info->did1 == 0)
	{
		LOG_WARNING("Cannot identify target as a Stellaris processor");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	if (offset & 0x3)
	{
		LOG_WARNING("offset size must be word aligned");
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
	
	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	/* Configure the flash controller timing */	
	stellaris_read_clock_info(bank);	
	stellaris_set_flash_mode(bank, 0);
	
	/* Clear and disable flash programming interrupts */
	target_write_u32(target, FLASH_CIM, 0);
	target_write_u32(target, FLASH_MISC, PMISC|AMISC);

	/* multiple words to be programmed? */
	if (words_remaining > 0) 
	{
		/* try using a block write */
		if ((retval = stellaris_write_block(bank, buffer, offset, words_remaining)) != ERROR_OK)
		{
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			{
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */ 
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			}
			else if (retval == ERROR_FLASH_OPERATION_FAILED)
			{
				/* if an error occured, we examine the reason, and quit */
				target_read_u32(target, FLASH_CRIS, &flash_cris);
				
				LOG_ERROR("flash writing failed with CRIS: 0x%x", flash_cris);
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
		else
		{
			buffer += words_remaining * 4;
			address += words_remaining * 4;
			words_remaining = 0;
		}
	}
	
	while (words_remaining > 0)
	{
		if (!(address & 0xff))
			LOG_DEBUG("0x%x", address);
		
		/* Program one word */
		target_write_u32(target, FLASH_FMA, address);
		target_write_buffer(target, FLASH_FMD, 4, buffer);
		target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_WRITE);
		/* LOG_DEBUG("0x%x 0x%x 0x%x",address,buf_get_u32(buffer, 0, 32),FMC_WRKEY | FMC_WRITE); */
		/* Wait until write complete */
		do
		{
			target_read_u32(target, FLASH_FMC, &flash_fmc);
		} while (flash_fmc & FMC_WRITE);
		
		buffer += 4;
		address += 4;
		words_remaining--;
	}
	
	if (bytes_remaining)
	{
		u8 last_word[4] = {0xff, 0xff, 0xff, 0xff};
		int i = 0;
				
		while(bytes_remaining > 0)
		{
			last_word[i++] = *(buffer + bytes_written); 
			bytes_remaining--;
			bytes_written++;
		}
		
		if (!(address & 0xff))
			LOG_DEBUG("0x%x", address);
		
		/* Program one word */
		target_write_u32(target, FLASH_FMA, address);
		target_write_buffer(target, FLASH_FMD, 4, last_word);
		target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_WRITE);
		/* LOG_DEBUG("0x%x 0x%x 0x%x",address,buf_get_u32(buffer, 0, 32),FMC_WRKEY | FMC_WRITE); */
		/* Wait until write complete */
		do
		{
			target_read_u32(target, FLASH_FMC, &flash_fmc);
		} while (flash_fmc & FMC_WRITE);
	}
	
	/* Check access violations */
	target_read_u32(target, FLASH_CRIS, &flash_cris);
	if (flash_cris & (AMASK))
	{
		LOG_DEBUG("flash_cris 0x%x", flash_cris);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

int stellaris_probe(struct flash_bank_s *bank)
{
	/* we can't probe on an stellaris
	 * if this is an stellaris, it has the configured flash
	 */
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* stellaris_read_part_info() already takes care about error checking and reporting */
	return stellaris_read_part_info(bank);
}

int stellaris_auto_probe(struct flash_bank_s *bank)
{
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	if (stellaris_info->did1)
		return ERROR_OK;
	return stellaris_probe(bank);
}

int stellaris_mass_erase(struct flash_bank_s *bank)
{
	target_t *target = NULL;
	stellaris_flash_bank_t *stellaris_info = NULL;
	u32 flash_fmc;
	
	stellaris_info = bank->driver_priv;
	target = bank->target;
	
	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (stellaris_info->did1 == 0)
	{
		stellaris_read_part_info(bank);
	}
	
	if (stellaris_info->did1 == 0)
	{
		LOG_WARNING("Cannot identify target as Stellaris");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	/* Configure the flash controller timing */
	stellaris_read_clock_info(bank);	
	stellaris_set_flash_mode(bank, 0);

	/* Clear and disable flash programming interrupts */
	target_write_u32(target, FLASH_CIM, 0);
	target_write_u32(target, FLASH_MISC, PMISC|AMISC);
	
	target_write_u32(target, FLASH_FMA, 0);
	target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_MERASE);
	/* Wait until erase complete */
	do
	{
		target_read_u32(target, FLASH_FMC, &flash_fmc);
	}
	while (flash_fmc & FMC_MERASE);
	
	/* if device has > 128k, then second erase cycle is needed
	 * this is only valid for older devices, but will not hurt */
	if (stellaris_info->num_pages * stellaris_info->pagesize > 0x20000)
	{
		target_write_u32(target, FLASH_FMA, 0x20000);
		target_write_u32(target, FLASH_FMC, FMC_WRKEY | FMC_MERASE);
		/* Wait until erase complete */
		do
		{
			target_read_u32(target, FLASH_FMC, &flash_fmc);
		}
		while (flash_fmc & FMC_MERASE);
	}
	
	return ERROR_OK;
}

int stellaris_handle_mass_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	int i;
	
	if (argc < 1)
	{
		command_print(cmd_ctx, "stellaris mass_erase <bank>");
		return ERROR_OK;	
	}
	
	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}
	
	if (stellaris_mass_erase(bank) == ERROR_OK)
	{
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
		{
			bank->sectors[i].is_erased = 1;
		}
		
		command_print(cmd_ctx, "stellaris mass erase complete");
	}
	else
	{
		command_print(cmd_ctx, "stellaris mass erase failed");
	}
	
	return ERROR_OK;
}

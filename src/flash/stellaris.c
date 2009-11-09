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

#include "stellaris.h"
#include "armv7m.h"
#include "binarybuffer.h"


#define DID0_VER(did0) ((did0 >> 28)&0x07)

static int stellaris_read_part_info(struct flash_bank_s *bank);
static uint32_t stellaris_get_flash_status(flash_bank_t *bank);
static void stellaris_set_flash_mode(flash_bank_t *bank,int mode);
//static uint32_t stellaris_wait_status_busy(flash_bank_t *bank, uint32_t waitbits, int timeout);

static int stellaris_mass_erase(struct flash_bank_s *bank);

static struct {
	uint32_t partno;
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

static char * StellarisClassname[5] =
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
static int stellaris_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
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

	/* TODO Specify the main crystal speed in kHz using an optional
	 * argument; ditto, the speed of an external oscillator used
	 * instead of a crystal.  Avoid programming flash using IOSC.
	 */
	return ERROR_OK;
}

static int stellaris_info(struct flash_bank_s *bank, char *buf, int buf_size)
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
		device_class = (stellaris_info->did0 >> 16) & 0xFF;
	}
	else
	{
		device_class = 0;
	}
	printed = snprintf(buf,
			   buf_size,
			   "\nTI/LMI Stellaris information: Chip is "
			   "class %i (%s) %s rev %c%i\n",
			   device_class,
			   StellarisClassname[device_class],
			   stellaris_info->target_name,
			   (int)('A' + ((stellaris_info->did0 >> 8) & 0xFF)),
			   (int)((stellaris_info->did0) & 0xFF));
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf,
			   buf_size,
			   "did1: 0x%8.8" PRIx32 ", arch: 0x%4.4" PRIx32
			   ", eproc: %s, ramsize: %ik, flashsize: %ik\n",
			   stellaris_info->did1,
			   stellaris_info->did1,
			   "ARMv7M",
			   (int)((1 + ((stellaris_info->dc0 >> 16) & 0xFFFF))/4),
			   (int)((1 + (stellaris_info->dc0 & 0xFFFF))*2));
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf,
			   buf_size,
			   "master clock: %ikHz%s, "
			   "rcc is 0x%" PRIx32 ", rcc2 is 0x%" PRIx32 "\n",
			   (int)(stellaris_info->mck_freq / 1000),
			   stellaris_info->mck_desc,
			   stellaris_info->rcc,
			   stellaris_info->rcc2);
	buf += printed;
	buf_size -= printed;

	if (stellaris_info->num_lockbits > 0)
	{
		printed = snprintf(buf,
				   buf_size,
				   "pagesize: %" PRIi32 ", lockbits: %i 0x%4.4" PRIx32 ", pages in lock region: %i \n",
				   stellaris_info->pagesize,
				   stellaris_info->num_lockbits,
				   stellaris_info->lockbits,
				   (int)(stellaris_info->num_pages/stellaris_info->num_lockbits));
		buf += printed;
		buf_size -= printed;
	}
	return ERROR_OK;
}

/***************************************************************************
*	chip identification and status                                         *
***************************************************************************/

static uint32_t stellaris_get_flash_status(flash_bank_t *bank)
{
	target_t *target = bank->target;
	uint32_t fmc;

	target_read_u32(target, FLASH_CONTROL_BASE | FLASH_FMC, &fmc);

	return fmc;
}

/** Read clock configuration and set stellaris_info->usec_clocks*/

static const unsigned rcc_xtal[32] = {
	[0x00] = 1000000,		/* no pll */
	[0x01] = 1843200,		/* no pll */
	[0x02] = 2000000,		/* no pll */
	[0x03] = 2457600,		/* no pll */

	[0x04] = 3579545,
	[0x05] = 3686400,
	[0x06] = 4000000,		/* usb */
	[0x07] = 4096000,

	[0x08] = 4915200,
	[0x09] = 5000000,		/* usb */
	[0x0a] = 5120000,
	[0x0b] = 6000000,		/* (reset) usb */

	[0x0c] = 6144000,
	[0x0d] = 7372800,
	[0x0e] = 8000000,		/* usb */
	[0x0f] = 8192000,

	/* parts before DustDevil use just 4 bits for xtal spec */

	[0x10] = 10000000,		/* usb */
	[0x11] = 12000000,		/* usb */
	[0x12] = 12288000,
	[0x13] = 13560000,

	[0x14] = 14318180,
	[0x15] = 16000000,		/* usb */
	[0x16] = 16384000,
};

static void stellaris_read_clock_info(flash_bank_t *bank)
{
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	target_t *target = bank->target;
	uint32_t rcc, rcc2, pllcfg, sysdiv, usesysdiv, bypass, oscsrc;
	unsigned xtal;
	unsigned long mainfreq;

	target_read_u32(target, SCB_BASE | RCC, &rcc);
	LOG_DEBUG("Stellaris RCC %" PRIx32 "", rcc);

	target_read_u32(target, SCB_BASE | RCC2, &rcc2);
	LOG_DEBUG("Stellaris RCC2 %" PRIx32 "", rcc);

	target_read_u32(target, SCB_BASE | PLLCFG, &pllcfg);
	LOG_DEBUG("Stellaris PLLCFG %" PRIx32 "", pllcfg);

	stellaris_info->rcc = rcc;
	stellaris_info->rcc = rcc2;

	sysdiv = (rcc >> 23) & 0xF;
	usesysdiv = (rcc >> 22) & 0x1;
	bypass = (rcc >> 11) & 0x1;
	oscsrc = (rcc >> 4) & 0x3;
	xtal = (rcc >> 6) & stellaris_info->xtal_mask;

	/* NOTE: post-Sandstorm parts have RCC2 which may override
	 * parts of RCC ... with more sysdiv options, option for
	 * 32768 Hz mainfreq, PLL controls.  On Sandstorm it reads
	 * as zero, so the "use RCC2" flag is always clear.
	 */
	if (rcc2 & (1 << 31)) {
		sysdiv = (rcc2 >> 23) & 0x3F;
		bypass = (rcc2 >> 11) & 0x1;
		oscsrc = (rcc2 >> 4) & 0x7;

		/* FIXME Tempest parts have an additional lsb for
		 * fractional sysdiv (200 MHz / 2.5 == 80 MHz)
		 */
	}

	stellaris_info->mck_desc = "";

	switch (oscsrc)
	{
		case 0:				/* MOSC */
			mainfreq = rcc_xtal[xtal];
			break;
		case 1:				/* IOSC */
			mainfreq = stellaris_info->iosc_freq;
			stellaris_info->mck_desc = stellaris_info->iosc_desc;
			break;
		case 2:				/* IOSC/4 */
			mainfreq = stellaris_info->iosc_freq / 4;
			stellaris_info->mck_desc = stellaris_info->iosc_desc;
			break;
		case 3:				/* lowspeed */
			/* Sandstorm doesn't have this 30K +/- 30% osc */
			mainfreq = 30000;
			stellaris_info->mck_desc = " (±30%)";
			break;
		case 8:				/* hibernation osc */
			/* not all parts support hibernation */
			mainfreq = 32768;
			break;

		default: /* NOTREACHED */
			mainfreq = 0;
			break;
	}

	/* PLL is used if it's not bypassed; its output is 200 MHz
	 * even when it runs at 400 MHz (adds divide-by-two stage).
	 */
	if (!bypass)
		mainfreq = 200000000;

	if (usesysdiv)
		stellaris_info->mck_freq = mainfreq/(1 + sysdiv);
	else
		stellaris_info->mck_freq = mainfreq;

	/* Forget old flash timing */
	stellaris_set_flash_mode(bank, 0);
}

/* Setup the timimg registers */
static void stellaris_set_flash_mode(flash_bank_t *bank,int mode)
{
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	target_t *target = bank->target;

	uint32_t usecrl = (stellaris_info->mck_freq/1000000ul-1);
	LOG_DEBUG("usecrl = %i",(int)(usecrl));
	target_write_u32(target, SCB_BASE | USECRL, usecrl);
}

#if 0
static uint32_t stellaris_wait_status_busy(flash_bank_t *bank, uint32_t waitbits, int timeout)
{
	uint32_t status;

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
static int stellaris_flash_command(struct flash_bank_s *bank,uint8_t cmd,uint16_t pagen)
{
	uint32_t fmc;
	target_t *target = bank->target;

	fmc = FMC_WRKEY | cmd;
	target_write_u32(target, FLASH_CONTROL_BASE | FLASH_FMC, fmc);
	LOG_DEBUG("Flash command: 0x%x", fmc);

	if (stellaris_wait_status_busy(bank, cmd, 100))
	{
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}
#endif

/* Read device id register, main clock frequency register and fill in driver info structure */
static int stellaris_read_part_info(struct flash_bank_s *bank)
{
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	target_t *target = bank->target;
	uint32_t did0, did1, ver, fam, status;
	int i;

	/* Read and parse chip identification register */
	target_read_u32(target, SCB_BASE | DID0, &did0);
	target_read_u32(target, SCB_BASE | DID1, &did1);
	target_read_u32(target, SCB_BASE | DC0, &stellaris_info->dc0);
	target_read_u32(target, SCB_BASE | DC1, &stellaris_info->dc1);
	LOG_DEBUG("did0 0x%" PRIx32 ", did1 0x%" PRIx32 ", dc0 0x%" PRIx32 ", dc1 0x%" PRIx32 "",
		  did0, did1, stellaris_info->dc0, stellaris_info->dc1);

	ver = did0 >> 28;
	if ((ver != 0) && (ver != 1))
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
	if (((ver != 0) && (ver != 1)) || (fam != 0))
	{
		LOG_WARNING("Unknown did1 version/family, cannot positively identify target as a Stellaris");
	}

	/* For Sandstorm, Fury, DustDevil:  current data sheets say IOSC
	 * is 12 MHz, but some older parts have 15 MHz.  A few data sheets
	 * even give _both_ numbers!  We'll use current numbers; IOSC is
	 * always approximate.
	 *
	 * For Tempest:  IOSC is calibrated, 16 MHz
	 */
	stellaris_info->iosc_freq = 12000000;
	stellaris_info->iosc_desc = " (±30%)";
	stellaris_info->xtal_mask = 0x0f;

	switch ((did0 >> 28) & 0x7) {
	case 0:				/* Sandstorm */
		/*
		 * Current (2009-August) parts seem to be rev C2 and use 12 MHz.
		 * Parts before rev C0 used 15 MHz; some C0 parts use 15 MHz
		 * (LM3S618), but some other C0 parts are 12 MHz (LM3S811).
		 */
		if (((did0 >> 8) & 0xff) < 2) {
			stellaris_info->iosc_freq = 15000000;
			stellaris_info->iosc_desc = " (±50%)";
		}
		break;
	case 1:
		switch ((did0 >> 16) & 0xff) {
		case 1:			/* Fury */
			break;
		case 4:			/* Tempest */
			stellaris_info->iosc_freq = 16000000;	/* +/- 1% */
			stellaris_info->iosc_desc = " (±1%)";
			/* FALL THROUGH */
		case 3:			/* DustDevil */
			stellaris_info->xtal_mask = 0x1f;
			break;
		default:
			LOG_WARNING("Unknown did0 class");
		}
	default:
		break;
		LOG_WARNING("Unknown did0 version");
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
	stellaris_info->num_pages = 2 *(1 + (stellaris_info->dc0 & 0xFFFF));
	stellaris_info->pagesize = 1024;
	bank->size = 1024 * stellaris_info->num_pages;
	stellaris_info->pages_in_lockregion = 2;
	target_read_u32(target, SCB_BASE | FMPPE, &stellaris_info->lockbits);

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

static int stellaris_protect_check(struct flash_bank_s *bank)
{
	uint32_t status;

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
		LOG_WARNING("Cannot identify target as Stellaris");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	status = stellaris_get_flash_status(bank);
	stellaris_info->lockbits = status >> 16;

	return ERROR_OK;
}

static int stellaris_erase(struct flash_bank_s *bank, int first, int last)
{
	int banknr;
	uint32_t flash_fmc, flash_cris;
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

	if ((first < 0) || (last < first) || (last >= (int)stellaris_info->num_pages))
	{
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if ((first == 0) && (last == ((int)stellaris_info->num_pages-1)))
	{
		return stellaris_mass_erase(bank);
	}

	/* Configure the flash controller timing */
	stellaris_read_clock_info(bank);
	stellaris_set_flash_mode(bank,0);

	/* Clear and disable flash programming interrupts */
	target_write_u32(target, FLASH_CIM, 0);
	target_write_u32(target, FLASH_MISC, PMISC | AMISC);

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
		while (flash_fmc & FMC_ERASE);

		/* Check acess violations */
		target_read_u32(target, FLASH_CRIS, &flash_cris);
		if (flash_cris & (AMASK))
		{
			LOG_WARNING("Error erasing flash page %i,  flash_cris 0x%" PRIx32 "", banknr, flash_cris);
			target_write_u32(target, FLASH_CRIS, 0);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bank->sectors[banknr].is_erased = 1;
	}

	return ERROR_OK;
}

static int stellaris_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	uint32_t fmppe, flash_fmc, flash_cris;
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
			fmppe &= ~(1 << lockregion);
		else
			fmppe |= (1 << lockregion);
	}

	/* Clear and disable flash programming interrupts */
	target_write_u32(target, FLASH_CIM, 0);
	target_write_u32(target, FLASH_MISC, PMISC | AMISC);

	LOG_DEBUG("fmppe 0x%" PRIx32 "",fmppe);
	target_write_u32(target, SCB_BASE | FMPPE, fmppe);
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
	while (flash_fmc & FMC_COMT);

	/* Check acess violations */
	target_read_u32(target, FLASH_CRIS, &flash_cris);
	if (flash_cris & (AMASK))
	{
		LOG_WARNING("Error setting flash page protection,  flash_cris 0x%" PRIx32 "", flash_cris);
		target_write_u32(target, FLASH_CRIS, 0);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	target_read_u32(target, SCB_BASE | FMPPE, &stellaris_info->lockbits);

	return ERROR_OK;
}

static uint8_t stellaris_write_code[] =
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

static int stellaris_write_block(struct flash_bank_s *bank, uint8_t *buffer, uint32_t offset, uint32_t wcount)
{
	target_t *target = bank->target;
	uint32_t buffer_size = 8192;
	working_area_t *source;
	working_area_t *write_algorithm;
	uint32_t address = bank->base + offset;
	reg_param_t reg_params[3];
	armv7m_algorithm_t armv7m_info;
	int retval = ERROR_OK;

	LOG_DEBUG("(bank=%p buffer=%p offset=%08" PRIx32 " wcount=%08" PRIx32 "",
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
		LOG_DEBUG("called target_alloc_working_area(target=%p buffer_size=%08" PRIx32 " source=%p)",
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
		uint32_t thisrun_count = (wcount > (buffer_size / 4)) ? (buffer_size / 4) : wcount;

		target_write_buffer(target, source->address, thisrun_count * 4, buffer);

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, 4*thisrun_count);
		LOG_INFO("Algorithm flash write %" PRIi32 " words to 0x%" PRIx32 ", %" PRIi32 " remaining", thisrun_count, address, (wcount - thisrun_count));
		LOG_DEBUG("Algorithm flash write %" PRIi32 " words to 0x%" PRIx32 ", %" PRIi32 " remaining", thisrun_count, address, (wcount - thisrun_count));
		if ((retval = target_run_algorithm(target, 0, NULL, 3, reg_params, write_algorithm->address, write_algorithm->address + sizeof(stellaris_write_code)-10, 10000, &armv7m_info)) != ERROR_OK)
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

static int stellaris_write(struct flash_bank_s *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	target_t *target = bank->target;
	uint32_t address = offset;
	uint32_t flash_cris, flash_fmc;
	uint32_t words_remaining = (count / 4);
	uint32_t bytes_remaining = (count & 0x00000003);
	uint32_t bytes_written = 0;
	int retval;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("(bank=%p buffer=%p offset=%08" PRIx32 " count=%08" PRIx32 "",
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
	target_write_u32(target, FLASH_MISC, PMISC | AMISC);

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

				LOG_ERROR("flash writing failed with CRIS: 0x%" PRIx32 "", flash_cris);
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
			LOG_DEBUG("0x%" PRIx32 "", address);

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
		uint8_t last_word[4] = {0xff, 0xff, 0xff, 0xff};
		int i = 0;

		while (bytes_remaining > 0)
		{
			last_word[i++] = *(buffer + bytes_written);
			bytes_remaining--;
			bytes_written++;
		}

		if (!(address & 0xff))
			LOG_DEBUG("0x%" PRIx32 "", address);

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
		LOG_DEBUG("flash_cris 0x%" PRIx32 "", flash_cris);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

static int stellaris_probe(struct flash_bank_s *bank)
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

static int stellaris_auto_probe(struct flash_bank_s *bank)
{
	stellaris_flash_bank_t *stellaris_info = bank->driver_priv;
	if (stellaris_info->did1)
		return ERROR_OK;
	return stellaris_probe(bank);
}

static int stellaris_mass_erase(struct flash_bank_s *bank)
{
	target_t *target = NULL;
	stellaris_flash_bank_t *stellaris_info = NULL;
	uint32_t flash_fmc;

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
	target_write_u32(target, FLASH_MISC, PMISC | AMISC);

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

static int stellaris_handle_mass_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;

	if (argc < 1)
	{
		command_print(cmd_ctx, "stellaris mass_erase <bank>");
		return ERROR_OK;
	}

	flash_bank_t *bank;
	int retval = flash_command_get_bank_by_num(cmd_ctx, args[0], &bank);
	if (ERROR_OK != retval)
		return retval;

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

static int stellaris_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *stm32x_cmd = register_command(cmd_ctx, NULL, "stellaris",
			NULL, COMMAND_ANY, "stellaris flash specific commands");

	register_command(cmd_ctx, stm32x_cmd, "mass_erase",
			stellaris_handle_mass_erase_command, COMMAND_EXEC,
			"mass erase device");
	return ERROR_OK;
}


flash_driver_t stellaris_flash = {
		.name = "stellaris",
		.register_commands = &stellaris_register_commands,
		.flash_bank_command = &stellaris_flash_bank_command,
		.erase = &stellaris_erase,
		.protect = &stellaris_protect,
		.write = &stellaris_write,
		.probe = &stellaris_probe,
		.auto_probe = &stellaris_auto_probe,
		.erase_check = &default_flash_mem_blank_check,
		.protect_check = &stellaris_protect_check,
		.info = &stellaris_info,
	};

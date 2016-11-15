/***************************************************************************
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>       *
 *   Modified by Megan Wachs <megan@sifive.com> from the original stmsmi.c *
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

/* The Freedeom E SPI controller is a SPI bus controller
 * specifically designed for SPI Flash Memories on Freedom E platforms.
 * 
 * Two working modes are available:
 * - SW mode: the SPI is controlled by SW. Any custom commands can be sent
 *   on the bus. Writes are only possible in this mode.
 * - HW mode: Memory content is directly
 *   accessible in CPU memory space. CPU can read, write and execute memory
 *   content. */

/* ATTENTION:
 * To have flash memory mapped in CPU memory space, the controller
 * has have  "HW mode" enabled.
 * 1) The command "reset init" has to initialize the controller and put
 *    it in HW mode (this is actually the default out of reset for Freedom E systems).
 * 2) every command in this file have to return to prompt in HW mode. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>

/* Register offsets */

#define FESPI_REG_SCKDIV          0x00
#define FESPI_REG_SCKMODE         0x04
#define FESPI_REG_CSID            0x10
#define FESPI_REG_CSDEF           0x14
#define FESPI_REG_CSMODE          0x18

#define FESPI_REG_DCSSCK          0x28
#define FESPI_REG_DSCKCS          0x2a
#define FESPI_REG_DINTERCS        0x2c
#define FESPI_REG_DINTERXFR       0x2e

#define FESPI_REG_FMT             0x40
#define FESPI_REG_TXFIFO          0x48
#define FESPI_REG_RXFIFO          0x4c
#define FESPI_REG_TXCTRL          0x50
#define FESPI_REG_RXCTRL          0x54

#define FESPI_REG_FCTRL           0x60
#define FESPI_REG_FFMT            0x64

#define FESPI_REG_IE              0x70
#define FESPI_REG_IP              0x74

/* Fields */

#define FESPI_SCK_POL             0x1
#define FESPI_SCK_PHA             0x2

#define FESPI_FMT_PROTO(x)        ((x) & 0x3)
#define FESPI_FMT_ENDIAN(x)       (((x) & 0x1) << 2)
#define FESPI_FMT_DIR(x)          (((x) & 0x1) << 3)
#define FESPI_FMT_LEN(x)          (((x) & 0xf) << 16)

/* TXCTRL register */
#define FESPI_TXWM(x)             ((x) & 0xffff)
/* RXCTRL register */
#define FESPI_RXWM(x)             ((x) & 0xffff)

#define FESPI_IP_TXWM             0x1
#define FESPI_IP_RXWM             0x2

#define FESPI_FCTRL_EN            0x1

#define FESPI_INSN_CMD_EN         0x1
#define FESPI_INSN_ADDR_LEN(x)    (((x) & 0x7) << 1)
#define FESPI_INSN_PAD_CNT(x)     (((x) & 0xf) << 4)
#define FESPI_INSN_CMD_PROTO(x)   (((x) & 0x3) << 8)
#define FESPI_INSN_ADDR_PROTO(x)  (((x) & 0x3) << 10)
#define FESPI_INSN_DATA_PROTO(x)  (((x) & 0x3) << 12)
#define FESPI_INSN_CMD_CODE(x)    (((x) & 0xff) << 16)
#define FESPI_INSN_PAD_CODE(x)    (((x) & 0xff) << 24)

/* Values */

#define FESPI_CSMODE_AUTO         0
#define FESPI_CSMODE_HOLD         2
#define FESPI_CSMODE_OFF          3

#define FESPI_DIR_RX              0
#define FESPI_DIR_TX              1

#define FESPI_PROTO_S             0
#define FESPI_PROTO_D             1
#define FESPI_PROTO_Q             2

#define FESPI_ENDIAN_MSB          0
#define FESPI_ENDIAN_LSB          1


/* Timeout in ms */
#define FESPI_CMD_TIMEOUT   (100)
#define FESPI_PROBE_TIMEOUT (100)
#define FESPI_MAX_TIMEOUT  (3000)


#define FESPI_READ_REG(a) (_FESPI_READ_REG(a))
#define _FESPI_READ_REG(a)			\
{									\
	int __a;						\
	uint32_t __v;					\
									\
	__a = target_read_u32(target, ctrl_base + (a), &__v); \
	if (__a != ERROR_OK)			\
		return __a;					\
	__v;							\
}

#define FESPI_WRITE_REG(a, v)			\
{									\
	int __r;						\
									\
	__r = target_write_u32(target, ctrl_base + (a), (v)); \
	if (__r != ERROR_OK)			\
		return __r;					\
}

#define FESPI_DISABLE_HW_MODE()	FESPI_WRITE_REG(FESPI_REG_FCTRL, \
		FESPI_READ_REG(FESPI_REG_FCTRL) & ~FESPI_FCTRL_EN)
#define FESPI_ENABLE_HW_MODE()	FESPI_WRITE_REG(FESPI_REG_FCTRL, \
	FESPI_READ_REG(FESPI_REG_FCTRL) | FESPI_FCTRL_EN)

struct fespi_flash_bank {
  int probed;
  uint32_t ctrl_base;
  const struct flash_device *dev;
};

struct fespi_target {
  char *name;
  uint32_t tap_idcode;
  uint32_t ctrl_base;
};

//TODO !!! What is the right naming convention here?
static const struct fespi_target target_devices[] = {
	/* name,   tap_idcode, ctrl_base */
        { "Freedom E300 SPI Flash",  0x10e31913 , 0x10014000 },
	{ NULL,    0,           0          }
};

FLASH_BANK_COMMAND_HANDLER(fespi_flash_bank_command)
{
	struct fespi_flash_bank *fespi_info;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	fespi_info = malloc(sizeof(struct fespi_flash_bank));
	if (fespi_info == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = fespi_info;
	fespi_info->probed = 0;

	return ERROR_OK;
}

static int fespi_set_dir (struct flash_bank * bank, bool dir) {
  struct target *target = bank->target;
  struct fespi_flash_bank *fespi_info = bank->driver_priv;
  uint32_t ctrl_base = fespi_info->ctrl_base;

  FESPI_WRITE_REG(FESPI_REG_FMT,
		  (FESPI_READ_REG(FESPI_REG_FMT) & ~(FESPI_FMT_DIR(0xFFFFFFFF))) |
		  FESPI_FMT_DIR(dir));
  				   
  return ERROR_OK;
  
}

static int fespi_txwm_wait(struct flash_bank *bank) {
  struct target *target = bank->target;
  struct fespi_flash_bank *fespi_info = bank->driver_priv;
  uint32_t ctrl_base = fespi_info->ctrl_base;

  while (!(FESPI_READ_REG(FESPI_REG_IP) & FESPI_IP_TXWM));

  return ERROR_OK;
  
}
  
static int fespi_tx(struct flash_bank *bank, uint8_t in){
  struct target *target = bank->target;
  struct fespi_flash_bank *fespi_info = bank->driver_priv;
  uint32_t ctrl_base = fespi_info->ctrl_base;

  while ((int32_t) FESPI_READ_REG(FESPI_REG_TXFIFO) < 0);
  FESPI_WRITE_REG(FESPI_REG_TXFIFO, in);

  return ERROR_OK;
}

static uint8_t fespi_rx(struct flash_bank * bank) {
    
  struct target *target = bank->target;
  struct fespi_flash_bank *fespi_info = bank->driver_priv;
  uint32_t ctrl_base = fespi_info->ctrl_base;

  int32_t out;
  while ((out = (int32_t) FESPI_READ_REG(FESPI_REG_RXFIFO)) < 0);

  return out & 0xFF;

}

//TODO!!! Why don't we need to call this after writing?
static int fespi_wip (struct flash_bank * bank, int timeout) {
  struct target *target = bank->target;
  struct fespi_flash_bank *fespi_info = bank->driver_priv;
  uint32_t ctrl_base = fespi_info->ctrl_base;

  int64_t endtime;
  
  fespi_set_dir(bank, FESPI_DIR_RX);

  FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);
  endtime = timeval_ms() + timeout;

  fespi_tx(bank, SPIFLASH_READ_STATUS);
  fespi_rx(bank);
  
  do {
   
    alive_sleep(1);
 
    fespi_tx(bank, 0);
    if ((fespi_rx(bank) & SPIFLASH_BSY_BIT) == 0) {
      FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);
      fespi_set_dir(bank, FESPI_DIR_TX);
      return ERROR_OK;
    }
    
  } while (timeval_ms() < endtime);

  LOG_ERROR("timeout");
  return ERROR_FAIL;
  
}

static int fespi_erase_sector(struct flash_bank *bank, int sector)
{
  struct target *target = bank->target;
  struct fespi_flash_bank *fespi_info = bank->driver_priv;
  uint32_t ctrl_base = fespi_info->ctrl_base;
  int retval;
  
  fespi_tx(bank, SPIFLASH_WRITE_ENABLE);
  fespi_txwm_wait(bank);
  
  FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);
  fespi_tx(bank, fespi_info->dev->erase_cmd);
  sector = bank->sectors[sector].offset;
  fespi_tx(bank, sector >> 16);
  fespi_tx(bank, sector >> 8);
  fespi_tx(bank, sector);
  fespi_txwm_wait(bank);
  FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);

  retval = fespi_wip(bank, FESPI_MAX_TIMEOUT);
  if (retval != ERROR_OK)
    return retval;
  
  return ERROR_OK;
}

static int fespi_erase(struct flash_bank *bank, int first, int last)
{
  struct target *target = bank->target;
  struct fespi_flash_bank *fespi_info = bank->driver_priv;
  uint32_t ctrl_base = fespi_info->ctrl_base;
  int retval = ERROR_OK;
  int sector;
  
  LOG_DEBUG("%s: from sector %d to sector %d", __func__, first, last);
  
  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }
  
  if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
    LOG_ERROR("Flash sector invalid");
    return ERROR_FLASH_SECTOR_INVALID;
  }
  
  if (!(fespi_info->probed)) {
    LOG_ERROR("Flash bank not probed");
    return ERROR_FLASH_BANK_NOT_PROBED;
  }
  
  for (sector = first; sector <= last; sector++) {
    if (bank->sectors[sector].is_protected) {
      LOG_ERROR("Flash sector %d protected", sector);
      return ERROR_FAIL;
    }
  }
        
   fespi_txwm_wait(bank);
   
   /* Disable Hardware accesses*/
   FESPI_DISABLE_HW_MODE();

   /* poll WIP */
   retval = fespi_wip(bank, FESPI_PROBE_TIMEOUT);
   if (retval != ERROR_OK)
     return retval;

  for (sector = first; sector <= last; sector++) {
    retval = fespi_erase_sector(bank, sector);
    if (retval != ERROR_OK)
      break;
    keep_alive();
  }
  
  /* Switch to HW mode before return to prompt */
  FESPI_ENABLE_HW_MODE();
  return retval;
}

static int fespi_protect(struct flash_bank *bank, int set,
	int first, int last)
{
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

/* This should write something less than or equal to a  page.*/
static int fespi_write_buffer(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t len)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;

	uint32_t ii;

	//TODO!!! assert that len < page size
	
	LOG_DEBUG("%s: offset=0x%08" PRIx32 " len=0x%08" PRIx32,
		  __func__, offset, len);


	printf("%s: offset=0x%08" PRIx32 " len=0x%08" PRIx32,
	       __func__, offset, len);


	
	fespi_tx(bank, SPIFLASH_WRITE_ENABLE);
	fespi_txwm_wait(bank);

	FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);

	fespi_tx(bank, SPIFLASH_PAGE_PROGRAM);

	fespi_tx(bank, offset >> 16);
	fespi_tx(bank, offset >> 8);
	fespi_tx(bank, offset);

	for (ii = 0; ii < len; ii++) {
	  fespi_tx(bank, buffer[ii]);
	}

	fespi_txwm_wait(bank);

	FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);
		
	return ERROR_OK;
}

static int fespi_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
  struct target *target = bank->target;
  struct fespi_flash_bank *fespi_info = bank->driver_priv;
  uint32_t ctrl_base = fespi_info->ctrl_base;
  uint32_t cur_count, page_size, page_offset;
  int sector;
  int retval = ERROR_OK;
  
  LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
	    __func__, offset, count);
  
  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }
  
  if (offset + count > fespi_info->dev->size_in_bytes) {
    LOG_WARNING("Write past end of flash. Extra data discarded.");
    count = fespi_info->dev->size_in_bytes - offset;
  }
  
  /* Check sector protection */
  for (sector = 0; sector < bank->num_sectors; sector++) {
    /* Start offset in or before this sector? */
    /* End offset in or behind this sector? */
    if ((offset <
	 (bank->sectors[sector].offset + bank->sectors[sector].size))
	&& ((offset + count - 1) >= bank->sectors[sector].offset)
	&& bank->sectors[sector].is_protected) {
      LOG_ERROR("Flash sector %d protected", sector);
      return ERROR_FAIL;
    }
  }
  
  page_size = fespi_info->dev->pagesize;
  
  fespi_txwm_wait(bank);
  
  /* Disable Hardware accesses*/
  FESPI_DISABLE_HW_MODE();
  
  /* poll WIP */
  retval = fespi_wip(bank, FESPI_PROBE_TIMEOUT);
  if (retval != ERROR_OK)
    return retval;
  
  /* unaligned buffer head */
  if (count > 0 && (offset & 3) != 0) {
    cur_count = 4 - (offset & 3);
    if (cur_count > count)
      cur_count = count;
    retval = fespi_write_buffer(bank, buffer, offset,
				cur_count);
    if (retval != ERROR_OK)
      goto err;
    offset += cur_count;
    buffer += cur_count;
    count -= cur_count;
  }
  
  page_offset = offset % page_size;
  /* central part, aligned words */
  while (count >= 4) {
    /* clip block at page boundary */
    if (page_offset + count > page_size)
      cur_count = page_size - page_offset;
    else
      cur_count = count & ~3;
    
    retval = fespi_write_buffer(bank, buffer, offset,
				cur_count);
    if (retval != ERROR_OK)
      goto err;
    
    page_offset = 0;
    buffer += cur_count;
    offset += cur_count;
    count -= cur_count;
    
    keep_alive();
  }
  
  /* buffer tail */
  if (count > 0)
    retval = fespi_write_buffer(bank, buffer, offset, count);
  
 err:
  /* Switch to HW mode before return to prompt */
  FESPI_ENABLE_HW_MODE();
  return retval;
}

/* Return ID of flash device */
/* On exit, SW mode is kept */
 static int fespi_read_flash_id(struct flash_bank *bank, uint32_t *id)
 {
   struct target *target = bank->target;
   struct fespi_flash_bank *fespi_info = bank->driver_priv;
   uint32_t ctrl_base = fespi_info->ctrl_base;
   int retval;
   
   if (target->state != TARGET_HALTED) {
     LOG_ERROR("Target not halted");
     return ERROR_TARGET_NOT_HALTED;
   }
      
   fespi_txwm_wait(bank);
   
   /* Disable Hardware accesses*/
   FESPI_DISABLE_HW_MODE();

   /* poll WIP */
   retval = fespi_wip(bank, FESPI_PROBE_TIMEOUT);
   if (retval != ERROR_OK)
     return retval;

   fespi_set_dir(bank, FESPI_DIR_RX);
   
   /* Send SPI command "read ID" */
   FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);
  
   fespi_tx(bank, SPIFLASH_READ_ID);
   /* Send dummy bytes to actually read the ID.*/
   fespi_tx(bank, 0);
   fespi_tx(bank, 0);
   fespi_tx(bank, 0);
      
   /* read ID from Receive Register */
   *id = 0;
   fespi_rx(bank);
   *id = fespi_rx(bank);
   *id |= (fespi_rx(bank) << 8);
   *id |= (fespi_rx(bank) << 16);

   FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);

   fespi_set_dir(bank, FESPI_DIR_TX);
  
   return ERROR_OK;
 }

 static int fespi_probe(struct flash_bank *bank)
 {
   struct target *target = bank->target;
   struct fespi_flash_bank *fespi_info = bank->driver_priv;
   uint32_t ctrl_base;
   struct flash_sector *sectors;
   uint32_t id = 0; /* silence uninitialized warning */
   const struct fespi_target *target_device;
   int retval;
   
   if (fespi_info->probed)
     free(bank->sectors);
   fespi_info->probed = 0;
   
   for (target_device = target_devices ; target_device->name ; ++target_device)
     if (target_device->tap_idcode == target->tap->idcode)
       break;
   if (!target_device->name) {
     LOG_ERROR("Device ID 0x%" PRIx32 " is not known as FESPI capable",
	       target->tap->idcode);
     return ERROR_FAIL;
   }
   
   ctrl_base = target_device->ctrl_base;
   fespi_info->ctrl_base = ctrl_base;
   
   LOG_DEBUG("Valid FESPI on device %s at address 0x%" PRIx32,
	     target_device->name, bank->base);
   
   /* read and decode flash ID; returns in SW mode */
   // TODO!!! Pass these arguments in to the driver
   // Elsewhere this driver assumes these are set this way,
   // but should really save and restore at the entry points.
   FESPI_WRITE_REG(FESPI_REG_SCKDIV, 3);
   FESPI_WRITE_REG(FESPI_REG_TXCTRL, FESPI_TXWM(1));
   fespi_set_dir(bank, FESPI_DIR_TX);

   retval = fespi_read_flash_id(bank, &id);

   FESPI_ENABLE_HW_MODE();
   if (retval != ERROR_OK)
     return retval;
   
   fespi_info->dev = NULL;
   for (const struct flash_device *p = flash_devices; p->name ; p++)
     if (p->device_id == id) {
       fespi_info->dev = p;
       break;
     }
   
   if (!fespi_info->dev) {
     LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
     return ERROR_FAIL;
   }
   
   LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
	    fespi_info->dev->name, fespi_info->dev->device_id);
   
   /* Set correct size value */
   bank->size = fespi_info->dev->size_in_bytes;
   
   /* create and fill sectors array */
   bank->num_sectors =
     fespi_info->dev->size_in_bytes / fespi_info->dev->sectorsize;
   sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
   if (sectors == NULL) {
     LOG_ERROR("not enough memory");
     return ERROR_FAIL;
   }
   
   for (int sector = 0; sector < bank->num_sectors; sector++) {
     sectors[sector].offset = sector * fespi_info->dev->sectorsize;
     sectors[sector].size = fespi_info->dev->sectorsize;
     sectors[sector].is_erased = -1;
     sectors[sector].is_protected = 1;
   }
   
   bank->sectors = sectors;
   fespi_info->probed = 1;
   return ERROR_OK;
 }

static int fespi_auto_probe(struct flash_bank *bank)
{
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	if (fespi_info->probed)
		return ERROR_OK;
	return fespi_probe(bank);
}

static int fespi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

 static int get_fespi_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct fespi_flash_bank *fespi_info = bank->driver_priv;

	if (!(fespi_info->probed)) {
		snprintf(buf, buf_size,
			"\nFESPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nFESPI flash information:\n"
		"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
		fespi_info->dev->name, fespi_info->dev->device_id);

	return ERROR_OK;
}

struct flash_driver fespi_flash = {
	.name = "fespi",
	.flash_bank_command = fespi_flash_bank_command,
	.erase = fespi_erase,
	.protect = fespi_protect,
	.write = fespi_write,
	.read = default_flash_read,
	.probe = fespi_probe,
	.auto_probe = fespi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = fespi_protect_check,
	.info = get_fespi_info
};

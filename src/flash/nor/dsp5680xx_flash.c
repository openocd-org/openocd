/***************************************************************************
 *   Copyright (C) 2011 by Rodrigo L. Rosa                                 *
 *   rodrigorosa.LG@gmail.com                                              *
 *                                                                         *
 *   Based on a file written by:                                           *
 *   Kevin McGuire                                                         * 
 *   Marcel Wijlaars                                                       *
 *   Michael Ashton                                                        *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef DSP5680XX_FLASH_H
#define DSP5680XX_FLASH_H

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/dsp5680xx.h>

struct dsp5680xx_flash_bank {
	struct working_area *write_algorithm;
};

static int dsp5680xx_build_sector_list(struct flash_bank *bank){
  //LOG_USER("%s not implemented",__FUNCTION__);
  //return ERROR_OK;

  // sector size is 512
  // bank->num_sectors = bank->size / 512; // Bank size is actually 0x2000, but it is set much higher as part of the workaround for byte/word addressing issues.
  bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
  int i;
  for (i = 0; i < bank->num_sectors; ++i){
    bank->sectors[i].offset = 0;// not implemented.
    bank->sectors[i].size = HFM_SECTOR_SIZE;
    //offset += bank->sectors[i].size;
    bank->sectors[i].is_erased = -1;
    bank->sectors[i].is_protected = -1;
  }
  LOG_USER("%s not tested yet.",__FUNCTION__);
  return ERROR_OK;

}

// flash bank dsp5680xx 0 0 0 0 <target#>
FLASH_BANK_COMMAND_HANDLER(dsp5680xx_flash_bank_command){
  struct dsp5680xx_flash_bank *nbank;

  nbank = malloc(sizeof(struct dsp5680xx_flash_bank));

  bank->base = HFM_FLASH_BASE_ADDR;
  bank->size = HFM_SIZE; // top 4k not accessible
  bank->driver_priv = nbank;
  bank->num_sectors = HFM_SECTOR_COUNT;// This number is anything >0. not really used.
  dsp5680xx_build_sector_list(bank);

  return ERROR_OK;
}

static int dsp5680xx_flash_protect_check(struct flash_bank *bank){
  int retval = ERROR_OK;
  uint8_t protected = 0; 
  if(bank->sectors[0].is_protected == -1){
    retval = dsp5680xx_f_protect_check(bank->target,&protected);
    if(retval == ERROR_OK)
      if(protected)
	bank->sectors[0].is_protected = 1;
      else
	bank->sectors[0].is_protected = 0;
    else
      bank->sectors[0].is_protected = -1;
  }
  return retval;
}

static int dsp5680xx_flash_protect(struct flash_bank *bank, int set, int first, int last){
  int retval;
  if(set){
    retval = dsp5680xx_f_lock(bank->target);
    if(retval == ERROR_OK)
      bank->sectors[0].is_protected = 1;
  }else{    
    retval = dsp5680xx_f_unlock(bank->target);
    if(retval == ERROR_OK)
      bank->sectors[0].is_protected = 0;
  }
  return retval;
}

/*
static int dsp5680xx_write_block(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count){
  LOG_USER("%s not implemented",__FUNCTION__);
  return ERROR_OK;
}

static int dsp5680xx_write_single(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count){
  LOG_USER("%s not implemented",__FUNCTION__);
  return ERROR_OK;
}
*/

//-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
//-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
//  Flash stuff test
//-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
//-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

static int dsp5680xx_flash_write(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count){
  int retval;
  if((offset + count/2)>bank->size){
    LOG_ERROR("%s: Flash bank cannot fit data.",__FUNCTION__);
    return ERROR_FAIL;
  }
  if(offset%2){
    LOG_ERROR("%s: Writing to odd addresses not supported. This chip uses word addressing, Openocd only supports byte addressing. The workaround results in disabling writing to odd byte addresses.",__FUNCTION__);
    return ERROR_FAIL;
  }
  retval = dsp5680xx_f_wr(bank->target,  buffer, bank->base + offset/2,  count);
  if(retval == ERROR_OK)
    bank->sectors[0].is_erased = 0;
  else
    bank->sectors[0].is_erased = -1;
  return retval;
}

static int dsp5680xx_probe(struct flash_bank *bank){
  LOG_DEBUG("%s not implemented",__FUNCTION__);
  return ERROR_OK;
}

static int dsp5680xx_flash_info(struct flash_bank *bank, char *buf, int buf_size){
	snprintf(buf, buf_size, "\ndsp5680xx flash driver info:\n - Currently only full erase/lock/unlock are implemented. \n - Call with bank==0 and sector 0 to 0.\n - Protect requires arp_init-reset to complete. \n - Before removing protection the master tap must be selected, and arp_init-reset is required to complete unlocking.");
	return ERROR_OK;
}
/*
static int dsp5680xx_set_write_enable(struct target *target, int enable){
	LOG_USER("%s not implemented",__FUNCTION__);
        return ERROR_OK;
}


static int dsp5680xx_check_flash_completion(struct target* target, unsigned int timeout_ms){
  LOG_USER("%s not implemented",__FUNCTION__);
  return ERROR_OK;
}
*/

static int dsp5680xx_flash_erase(struct flash_bank * bank, int first, int last){
  int retval;
  retval = dsp5680xx_f_erase(bank->target, (uint32_t) first, (uint32_t) last);
  if(retval == ERROR_OK)
    bank->sectors[0].is_erased = 1;
  else
    bank->sectors[0].is_erased = -1;
  return retval;
}

static int dsp5680xx_flash_erase_check(struct flash_bank * bank){
  int retval = ERROR_OK;
  uint8_t erased = 0;
  if(bank->sectors[0].is_erased == -1){
    retval = dsp5680xx_f_erase_check(bank->target,&erased);
    if (retval != ERROR_OK){
      bank->sectors[0].is_erased = -1;
    }else{
      if(erased)
	bank->sectors[0].is_erased = 1;
      else
	bank->sectors[0].is_erased = 0;
    }
  }
  return retval;
}

struct flash_driver dsp5680xx_flash = {
  .name = "dsp5680xx_flash",
  .flash_bank_command = dsp5680xx_flash_bank_command,
  .erase = dsp5680xx_flash_erase,
  .protect = dsp5680xx_flash_protect,
  .write = dsp5680xx_flash_write,
  //.read = default_flash_read,
  .probe = dsp5680xx_probe,
  .auto_probe = dsp5680xx_probe,
  .erase_check = dsp5680xx_flash_erase_check,
  .protect_check = dsp5680xx_flash_protect_check,
  .info = dsp5680xx_flash_info
};
#endif // dsp5680xx_flash.h

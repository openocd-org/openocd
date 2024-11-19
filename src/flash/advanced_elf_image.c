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

#include "advanced_elf_image.h"
#include <log.h>

void advanced_elf_image_close(struct advanced_elf_image *image)
{
    if (!image)
        return;
    if (image->fileio)
    {
        fileio_close(image->fileio);
        image->fileio = NULL;        
    }
    
    if (image->sections)
    {
        free(image->sections);
        image->sections = NULL;
    }
    
    if (image->symbols)
    {
        free(image->symbols);
        image->symbols = NULL;
    }
 
    if (image->strtab)
    {
        free(image->strtab);
        image->strtab = NULL;
    }
}

int advanced_elf_image_read_section(struct advanced_elf_image *elf, int section, void *buf, size_t buf_size, size_t *done)
{
    if (!elf || section < 0 || section >= elf->num_sections || !elf->sections)
        return ERROR_COMMAND_ARGUMENT_INVALID;
    
    int retval = fileio_seek(elf->fileio, elf->sections[section].sh_offset);
    if (retval != ERROR_OK)
        return retval;
    
    retval = fileio_read(elf->fileio, buf_size, buf, done);
    return retval;
}


uint32_t advanced_elf_image_find_symbol(struct advanced_elf_image *image, const char *symbol_name)
{
    if (!image || !image->symbols || !image->strtab)
        return 0;
    
    for (int i = 0; i < image->num_symbols; i++)
    {
        if (image->symbols[i].st_name >= image->strtab_size)
            continue;
        if (!strcmp(image->strtab + image->symbols[i].st_name, symbol_name))
            return image->symbols[i].st_value;
    }
    
    return 0;
}


int advanced_elf_image_open(struct advanced_elf_image *elf, const char *URL)
{
    memset(elf, 0, sizeof(struct advanced_elf_image));
    int retval = fileio_open(&elf->fileio, URL, FILEIO_READ, FILEIO_BINARY);
    size_t done;
    if (retval != ERROR_OK)
        return retval;
    
    retval = fileio_read(elf->fileio, sizeof(elf->header), &elf->header, &done);
    if (retval != ERROR_OK)
        return retval;
    
    if (strncmp((char *)elf->header.e_ident, ELFMAG, SELFMAG) != 0) {
        LOG_ERROR("invalid ELF file, bad magic number");
        return ERROR_IMAGE_FORMAT_ERROR;
    }
    if (elf->header.e_ident[EI_CLASS] != ELFCLASS32) {
        LOG_ERROR("invalid ELF file, only 32bits files are supported");
        return ERROR_IMAGE_FORMAT_ERROR;
    }
    
    elf->num_sections = elf->header.e_shnum;
    elf->sections = (Elf32_Shdr *)calloc(elf->header.e_shnum, sizeof(Elf32_Shdr));
    retval = fileio_seek(elf->fileio, elf->header.e_shoff);
    if (retval != ERROR_OK)
        return retval;
    
    retval = fileio_read(elf->fileio, sizeof(Elf32_Shdr) * elf->header.e_shnum, elf->sections, &done);
    if (retval != ERROR_OK)
        return retval;
    
    for (int i = 0; i < elf->num_sections; i++)
        if (elf->sections[i].sh_type == 2 /*SHT_SYMTAB*/)
        {
            if (elf->sections[i].sh_entsize != sizeof(Elf32_Sym))
            {
                LOG_ERROR("Unexpected symtab entry size in %s: %d.", URL, elf->sections[i].sh_entsize);
                return ERROR_IMAGE_FORMAT_ERROR;       
            }
    
            elf->num_symbols = elf->sections[i].sh_size / elf->sections[i].sh_entsize;
            elf->symbols = calloc(elf->num_symbols, elf->sections[i].sh_entsize);
            retval = fileio_seek(elf->fileio, elf->sections[i].sh_offset);
            if (retval != ERROR_OK)
                return retval;
            
            retval = fileio_read(elf->fileio, sizeof(Elf32_Sym) * elf->num_symbols, elf->symbols, &done);
            if (retval != ERROR_OK)
                return retval;
            if (done != (sizeof(Elf32_Sym) * elf->num_symbols))
                return ERROR_IMAGE_FORMAT_ERROR;
            
            int str = elf->sections[i].sh_link;
            if (str < 0 || str >= elf->header.e_shnum || elf->sections[str].sh_type != 3 /*SHT_STRTAB*/)
            {
                LOG_ERROR("Invalid strtab link from symtab section");
                return ERROR_IMAGE_FORMAT_ERROR;       
            }
            
            elf->strtab_size = elf->sections[str].sh_size;
            elf->strtab = malloc(elf->sections[str].sh_size);
            
            retval = fileio_seek(elf->fileio, elf->sections[str].sh_offset);
            if (retval != ERROR_OK)
                return retval;
            
            retval = fileio_read(elf->fileio, elf->strtab_size, elf->strtab, &done);
            if (retval != ERROR_OK)
                return retval;
            if (done != elf->strtab_size)
                return ERROR_IMAGE_FORMAT_ERROR;

            
            break;
        }
    
    if (!elf->symbols || !elf->strtab)
    {
        LOG_ERROR("The FLASH plugin file does not contain a symbol table and cannot be loaded.");
        return ERROR_IMAGE_FORMAT_ERROR;       
    }
    
    //TODO: actually read sections
    return retval;
}

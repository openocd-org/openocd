/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Semihosting API for Espressif chips                                   *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <target/target.h>
#include <target/semihosting_common.h>
#include "esp_semihosting.h"
#include "esp_riscv.h"
#include "esp_xtensa.h"
#include <sys/stat.h>
#include <utime.h>
#include <dirent.h>
#include <helper/list.h>

#if IS_MINGW
#define mkdir(fname, mode) mkdir(fname)
#define fsync(fd) FlushFileBuffers((HANDLE)(uintptr_t)(fd))
#define link(src, dest) CreateHardLink(dest, src, NULL)
#endif

/* Time and stat structs added due to the data size mismatch and OS based differences. Problem happens from 64-bit OS
 * environments */
struct time_values {
	uint32_t tv_sec;
	uint32_t tv_nsec;
};

struct stat_ret {
	uint16_t st_dev;
	uint16_t st_ino;
	uint32_t st_mode;
	uint16_t st_nlink;
	uint16_t st_uid;
	uint16_t st_gid;
	uint16_t st_rdev;
	uint32_t st_size;
	uint32_t st_blksize;
	uint32_t st_blocks;
	struct time_values st_atim;
	struct time_values st_mtim;
	struct time_values st_ctim;
};

struct dir_map {
	DIR *dirptr;
	uint32_t id;
	struct list_head lh;
};

static struct esp_semihost_data *target_to_esp_semihost_data(struct target *target)
{
	struct xtensa *xtensa = target->arch_info;
	if (xtensa->common_magic == XTENSA_COMMON_MAGIC)
		return &target_to_esp_xtensa(target)->semihost;
	else if (xtensa->common_magic == RISCV_COMMON_MAGIC)
		return &target_to_esp_riscv(target)->semihost;
	LOG_ERROR("Unknown target arch!");
	return NULL;
}

static struct dir_map *create_new_dir_map(DIR *dirptr)
{
	struct dir_map *dir_map = (struct dir_map *)calloc(1, sizeof(*dir_map));

	if (!dir_map)
		return NULL;

	dir_map->id = UINT32_MAX;
	dir_map->dirptr = dirptr;

	return dir_map;
}

static struct dir_map *find_dir_map_list_by_id(uint32_t id, struct target *target)
{
	struct esp_semihost_data *semihost_data = target_to_esp_semihost_data(target);

	int ret = list_empty(&semihost_data->dir_map_list);
	if (!ret) {
		struct dir_map *dir_map;
		list_for_each_entry(dir_map, &semihost_data->dir_map_list, lh) {
			if (dir_map->id == id)
				return dir_map;
		}
	}
	return NULL;
}

static void add_to_dir_map_list(struct dir_map *dir_map, struct target *target)
{
	struct esp_semihost_data *semihost_data = target_to_esp_semihost_data(target);

	int ret = list_empty(&semihost_data->dir_map_list);
	if (ret) {
		dir_map->id = 0;
	} else {
		struct dir_map *temp = list_last_entry(&semihost_data->dir_map_list, struct dir_map, lh);
		dir_map->id = temp->id + 1;
	}
	list_add_tail(&dir_map->lh, &semihost_data->dir_map_list);
}

static int remove_from_dir_map_list(struct dir_map *dir_map, struct target *target)
{
	struct esp_semihost_data *semihost_data = target_to_esp_semihost_data(target);
	int ret = list_empty(&semihost_data->dir_map_list);
	if (!ret && dir_map) {
		list_del(&dir_map->lh);
		free(dir_map);
		return ERROR_OK;
	}
	return ERROR_FAIL;
}

static int clean_dir_map_list(struct target *target)
{
	struct esp_semihost_data *semihost_data = target_to_esp_semihost_data(target);

	if (!semihost_data)
		return ERROR_OK;

	int ret = list_empty(&semihost_data->dir_map_list);
	if (!ret) {
		struct dir_map *dir_map, *tmp;
		list_for_each_entry_safe(dir_map, tmp, &semihost_data->dir_map_list, lh) {
			if (dir_map) {
				ret = closedir(dir_map->dirptr);
				if (ret != ERROR_OK)
					LOG_ERROR("Closedir failed during cleanup");
				list_del(&dir_map->lh);
				free(dir_map);
			}
		}
	}
	return ERROR_OK;
}

static void fill_stat_struct(struct stat_ret *st_stat_ret, const struct stat *statbuf)
{
	st_stat_ret->st_dev = 0;
	st_stat_ret->st_ino = 0;
	st_stat_ret->st_mode = statbuf->st_mode;
	st_stat_ret->st_nlink = statbuf->st_nlink;
	st_stat_ret->st_uid = statbuf->st_uid;
	st_stat_ret->st_gid = statbuf->st_gid;
	st_stat_ret->st_rdev = statbuf->st_rdev;
	st_stat_ret->st_size = statbuf->st_size;
	st_stat_ret->st_atim.tv_nsec = 0;
	st_stat_ret->st_mtim.tv_nsec = 0;
	st_stat_ret->st_ctim.tv_nsec = 0;

#if IS_WIN32
	st_stat_ret->st_atim.tv_sec = statbuf->st_atime;
	st_stat_ret->st_mtim.tv_sec = statbuf->st_mtime;
	st_stat_ret->st_ctim.tv_sec = statbuf->st_ctime;
#else
#if IS_DARWIN
	st_stat_ret->st_blksize = statbuf->st_blksize;
	st_stat_ret->st_blocks = statbuf->st_blocks;
	st_stat_ret->st_atim.tv_sec = statbuf->st_atimespec.tv_sec;
	st_stat_ret->st_mtim.tv_sec = statbuf->st_mtimespec.tv_sec;
	st_stat_ret->st_ctim.tv_sec = statbuf->st_ctimespec.tv_sec;
#else
	st_stat_ret->st_blksize = statbuf->st_blksize;
	st_stat_ret->st_blocks = statbuf->st_blocks;
	st_stat_ret->st_atim.tv_sec = statbuf->st_atim.tv_sec;
	st_stat_ret->st_mtim.tv_sec = statbuf->st_mtim.tv_sec;
	st_stat_ret->st_ctim.tv_sec = statbuf->st_ctim.tv_sec;
#endif	/* #if IS_DARWIN */
#endif	/* #if IS_WIN32 */
}

#define D_TYPE_OFFSET 2
#define D_NAME_OFFSET 3
#define PATH_LENGTH 255

static int write_to_dirent_struct(struct target *target, uint64_t ret_addr, struct dirent *e)
{
	int retval = target_write_buffer(target, ret_addr, sizeof(uint32_t), (uint8_t *)&(e->d_ino));
	if (retval != ERROR_OK)
		return retval;

#if IS_WIN32
	int tmp = 0;
	retval = target_write_buffer(target, ret_addr + D_TYPE_OFFSET, sizeof(char), (uint8_t *)&tmp);
#else
	retval = target_write_buffer(target, ret_addr + D_TYPE_OFFSET, sizeof(char), (uint8_t *)&(e->d_type));
#endif

	if (retval != ERROR_OK)
		return retval;

	return target_write_buffer(target, ret_addr + D_NAME_OFFSET, PATH_LENGTH, (uint8_t *)&(e->d_name));
}

int esp_semihosting_post_reset(struct target *target)
{
	return clean_dir_map_list(target);
}

static int esp_semihosting_sys_seek(struct target *target, uint64_t fd, int pos, size_t whence)
{
	struct semihosting *semihosting = target->semihosting;

	semihosting->result = lseek(fd, pos, whence);
	semihosting->sys_errno = errno;
	LOG_TARGET_DEBUG(target, "lseek(%" PRIx64 ", %" PRId32 " %" PRId64 ")=%d", fd, pos, semihosting->result, errno);
	return ERROR_OK;
}

static int esp_semihosting_sys_drv_info(struct target *target, int addr, int size)
{
	struct semihosting *semihosting = target->semihosting;

	semihosting->result = -1;
	semihosting->sys_errno = EINVAL;

	uint8_t *buf = malloc(size);
	if (!buf) {
		LOG_ERROR("Memory alloc failed drv info! size:(%d)", size);
		return ERROR_FAIL;
	}
	int retval = target_read_buffer(target, addr, size, buf);
	if (retval == ERROR_OK) {
		struct esp_semihost_data *semihost_data = target_to_esp_semihost_data(target);
		semihosting->result = 0;
		semihosting->sys_errno = 0;
		semihost_data->version = le_to_h_u32(&buf[0]);
		LOG_TARGET_DEBUG(target, "semihost.version: %d", semihost_data->version);
	}
	free(buf);
	LOG_TARGET_DEBUG(target, "drv_info res=%" PRId64 " errno=%d", semihosting->result, semihosting->sys_errno);
	return retval;
}

static const char *esp_semihosting_opcode_to_str(const int opcode)
{
	switch (opcode) {
		case ESP_SEMIHOSTING_SYS_DRV_INFO:
			return "SYS_DRV_INFO";
		case ESP_SEMIHOSTING_SYS_SEEK:
			return "ESP_SYS_SEEK";
		case ESP_SEMIHOSTING_SYS_APPTRACE_INIT:
			return "APPTRACE_INIT";
		case ESP_SEMIHOSTING_SYS_DEBUG_STUBS_INIT:
			return "DEBUG_STUBS_INIT";
		case ESP_SEMIHOSTING_SYS_BREAKPOINT_SET:
			return "BP_ADD_REMOVE";
		case ESP_SEMIHOSTING_SYS_WATCHPOINT_SET:
			return "WP_ADD_REMOVE";
		case ESP_SEMIHOSTING_SYS_PANIC_REASON:
			return "SYS_PANIC_REASON";
		case ESP_SEMIHOSTING_SYS_MKDIR:
			return "SYS_MKDIR";
		case ESP_SEMIHOSTING_SYS_OPENDIR:
			return "SYS_OPENDIR";
		case ESP_SEMIHOSTING_SYS_READDIR:
			return "SYS_READDIR";
		case ESP_SEMIHOSTING_SYS_READDIR_R:
			return "SYS_READDIR_R";
		case ESP_SEMIHOSTING_SYS_SEEKDIR:
			return "SYS_SEEKDIR";
		case ESP_SEMIHOSTING_SYS_TELLDIR:
			return "SYS_TELLDIR";
		case ESP_SEMIHOSTING_SYS_CLOSEDIR:
			return "SYS_CLOSEDIR";
		case ESP_SEMIHOSTING_SYS_RMDIR:
			return "SYS_RMDIR";
		case ESP_SEMIHOSTING_SYS_ACCESS:
			return "SYS_ACCESS";
		case ESP_SEMIHOSTING_SYS_TRUNCATE:
			return "SYS_TRUNCATE";
		case ESP_SEMIHOSTING_SYS_UTIME:
			return "SYS_UTIME";
		case ESP_SEMIHOSTING_SYS_FSTAT:
			return "SYS_FSTAT";
		case ESP_SEMIHOSTING_SYS_STAT:
			return "SYS_STAT";
		case ESP_SEMIHOSTING_SYS_FSYNC:
			return "SYS_FSYNC";
		case ESP_SEMIHOSTING_SYS_LINK:
			return "SYS_LINK";
		case ESP_SEMIHOSTING_SYS_UNLINK:
			return "SYS_UNLINK";
		default:
			return "<unknown>";
	}
}

int esp_semihosting_common(struct target *target)
{
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		/* Silently ignore if the semihosting field was not set. */
		return ERROR_OK;
	}

	struct gdb_fileio_info *fileio_info = target->fileio_info;

	int retval = ERROR_NOT_IMPLEMENTED;

	/* Enough space to hold 4 long words. */
	uint8_t fields[4 * 8];

	/*
	 * By default return an error.
	 * The actual result must be set by each function
	 */
	semihosting->result = -1;
	semihosting->sys_errno = EIO;

	LOG_TARGET_DEBUG(target, "op=0x%x (%s), param=0x%" PRIx64,
		semihosting->op, esp_semihosting_opcode_to_str(semihosting->op), semihosting->param);

	switch (semihosting->op) {
	case ESP_SEMIHOSTING_SYS_DRV_INFO:
		retval = semihosting_read_fields(target, 2, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int size = semihosting_get_field(target, 1, fields);
			retval = esp_semihosting_sys_drv_info(target, addr, size);
		}
		break;

	case ESP_SEMIHOSTING_SYS_SEEK:
		retval = semihosting_read_fields(target, 3, fields);
		if (retval == ERROR_OK) {
			uint64_t fd = semihosting_get_field(target, 0, fields);
			uint32_t pos = semihosting_get_field(target, 1, fields);
			size_t whence = semihosting_get_field(target, 2, fields);
			if (semihosting->is_fileio) {
				semihosting->hit_fileio = true;
				fileio_info->identifier = "lseek";
				fileio_info->param_1 = fd;
				fileio_info->param_2 = pos;
				fileio_info->param_3 = whence;
			} else {
				retval = esp_semihosting_sys_seek(target, fd, pos, whence);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_APPTRACE_INIT:
	case ESP_SEMIHOSTING_SYS_DEBUG_STUBS_INIT:
	case ESP_SEMIHOSTING_SYS_BREAKPOINT_SET:
	case ESP_SEMIHOSTING_SYS_WATCHPOINT_SET:
		/* For the time being only riscv chips support these commands */
		return esp_riscv_semihosting(target);

	case ESP_SEMIHOSTING_SYS_PANIC_REASON:		/* 0x116 */
		/* Read pseudo exception string */
		retval = semihosting_read_fields(target, 2, fields);
		if (retval == ERROR_OK) {
			struct esp_common *esp = target_to_esp_common(target);
			if (esp) {
				esp->panic_reason.addr = semihosting_get_field(target, 0, fields);
				esp->panic_reason.len = semihosting_get_field(target, 1, fields);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_MKDIR:			/* 0x106 */
		/* Attempts to create a directory to the given path. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_MKDIR not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 3, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int mode = semihosting_get_field(target, 1, fields);
			int len = semihosting_get_field(target, 2, fields);
			char *fn;

			retval = semihosting_get_file_name(target, addr, len, &fn);
			if (retval != ERROR_OK)
				return retval;
			if (fn) {
				semihosting->result = mkdir(fn, mode);
				semihosting->sys_errno = errno;
				LOG_DEBUG("mkdir('%s, %3o')=%" PRId64, fn, mode, semihosting->result);
				free(fn);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_OPENDIR:		/* 0x107 */
		/* Opens a directory stream corresponding to the given path. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_OPENDIR not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 3, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int len = semihosting_get_field(target, 1, fields);
			int id_addr = semihosting_get_field(target, 2, fields);
			char *fn;

			retval = semihosting_get_file_name(target, addr, len, &fn);
			if (retval != ERROR_OK)
				return retval;
			if (fn) {
				DIR *dirptr = opendir(fn);
				semihosting->sys_errno = errno;
				if (dirptr) {
					struct dir_map *dir_map = create_new_dir_map(dirptr);
					if (!dir_map) {
						closedir(dirptr);
						semihosting->sys_errno = ENOMEM;
					} else {
						add_to_dir_map_list(dir_map, target);
						retval =
							target_write_buffer(target,
							id_addr,
							sizeof(uint32_t),
							(uint8_t *)&dir_map->id);
						if (retval != ERROR_OK) {
							semihosting->sys_errno = EIO;
							remove_from_dir_map_list(dir_map, target);
							closedir(dirptr);
							free(fn);
							return retval;
						}
						semihosting->result = 0;
						LOG_DEBUG("opendir('%s' -> %d)", fn, dir_map->id);
					}
				}
				free(fn);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_READDIR:		/* 0x108 */
		/* Reads the next directory entry to the given path. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_READDIR not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 4, fields);
		if (retval == ERROR_OK) {
			int ret_addr = semihosting_get_field(target, 0, fields);
			int id = semihosting_get_field(target, 1, fields);
			struct dir_map *dir_map = find_dir_map_list_by_id(id, target);

			if (!dir_map || !dir_map->dirptr) {
				LOG_ERROR("DIR* node not found in readdir function");
				semihosting->sys_errno = EBADF;
			} else {
				/* Clear errno to differentiate end of directory and the error cases. If end of the
				 * directory is reached, errno will remain same */
				errno = 0;
				struct dirent *e = readdir(dir_map->dirptr);
				semihosting->sys_errno = errno;
				if (e) {
					retval = write_to_dirent_struct(target, ret_addr, e);
					if (retval != ERROR_OK) {
						semihosting->sys_errno = EIO;
						return retval;
					}
					semihosting->result = 0;
				}
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_SEEKDIR:		/* 0x10A */
		/* Function sets the location of the directory stream dirp. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_SEEKDIR not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 2, fields);
		if (retval == ERROR_OK) {
			int id = semihosting_get_field(target, 0, fields);
			int pos = semihosting_get_field(target, 1, fields);
			struct dir_map *dir_map = find_dir_map_list_by_id(id, target);

			if (!dir_map) {
				LOG_ERROR("DIR* node not found in seekdir function");
				semihosting->sys_errno = EBADF;
			} else {
				seekdir(dir_map->dirptr, pos);
				semihosting->result = 0;
				semihosting->sys_errno = errno;
				LOG_DEBUG("seekdir(%d)=%" PRId64, id, semihosting->result);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_TELLDIR:		/* 0x10B */
		/* Returns the current location of the directory stream dirp. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_TELLDIR not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 1, fields);
		if (retval == ERROR_OK) {
			int id = semihosting_get_field(target, 0, fields);
			struct dir_map *dir_map = find_dir_map_list_by_id(id, target);

			if (!dir_map) {
				LOG_ERROR("DIR* node not found in telldir function");
				semihosting->sys_errno = EBADF;
			} else {
				semihosting->result = telldir(dir_map->dirptr);
				semihosting->sys_errno = errno;
				LOG_DEBUG("telldir(%d)= %" PRId64, id, semihosting->result);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_CLOSEDIR:		/* 0x10C */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_CLOSEDIR not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 1, fields);
		if (retval == ERROR_OK) {
			int id = semihosting_get_field(target, 0, fields);
			struct dir_map *dir_map = find_dir_map_list_by_id(id, target);

			if (!dir_map) {
				LOG_ERROR("DIR* node not found in closedir function");
				semihosting->sys_errno = EBADF;
			} else {
				semihosting->result = closedir(dir_map->dirptr);
				semihosting->sys_errno = errno;
				remove_from_dir_map_list(dir_map, target);
				LOG_DEBUG("closedir(%d)=%" PRId64, id, semihosting->result);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_RMDIR:			/* 0x10D */
		/* Removes the folder to the given path. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_RMDIR not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 2, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int len = semihosting_get_field(target, 1, fields);
			char *fn;

			retval = semihosting_get_file_name(target, addr, len, &fn);
			if (retval != ERROR_OK)
				return retval;
			if (fn) {
				semihosting->result = rmdir(fn);
				semihosting->sys_errno = errno;
				LOG_DEBUG("rmdir('%s')=%" PRId64, fn, semihosting->result);
				free(fn);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_ACCESS:		/* 0x10E */
		/* Checks user's permission in the given path. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_ACCESS not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 3, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int len = semihosting_get_field(target, 1, fields);
			int flag_values = semihosting_get_field(target, 2, fields);
			char *fn;

			retval = semihosting_get_file_name(target, addr, len, &fn);
			if (retval != ERROR_OK)
				return retval;
			if (fn) {
				semihosting->result = access(fn, flag_values);
				semihosting->sys_errno = errno;
				LOG_DEBUG("access('%s')=%" PRId64, fn, semihosting->result);
				free(fn);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_TRUNCATE:		/* 0x10F */
		/* Truncates the file given bytes long. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_TRUNCATE not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 3, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int len = semihosting_get_field(target, 1, fields);
			int trunc_len = semihosting_get_field(target, 2, fields);
			char *fn;

			retval = semihosting_get_file_name(target, addr, len, &fn);
			if (retval != ERROR_OK)
				return retval;
			if (fn) {
				semihosting->result = truncate(fn, trunc_len);
				semihosting->sys_errno = errno;
				LOG_DEBUG("truncate('%s', '%d')=%" PRId64, fn, trunc_len, semihosting->result);
				free(fn);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_UTIME:			/* 0x110 */
		/* Changes last access and modification time information with given file path. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_UTIME not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 4, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int len = semihosting_get_field(target, 1, fields);
			int actime = semihosting_get_field(target, 2, fields);
			int modtime = semihosting_get_field(target, 3, fields);
			char *fn;

			retval = semihosting_get_file_name(target, addr, len, &fn);
			if (retval != ERROR_OK)
				return retval;
			if (fn) {
				struct utimbuf times = { .actime = actime, .modtime = modtime };

				semihosting->result = utime(fn, &times);
				semihosting->sys_errno = errno;
				LOG_DEBUG("utime('%s')=%" PRId64, fn, semihosting->result);
				free(fn);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_FSTAT:			/* 0x111 */
		/* Gets information with given file number. */
		retval = semihosting_read_fields(target, 2, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int ret_addr = semihosting_get_field(target, 1, fields);
			struct stat statbuf;
			struct stat_ret stat_ret;

			if (semihosting->is_fileio) {
				semihosting->hit_fileio = true;
				fileio_info->identifier = "fstat";
				fileio_info->param_1 = addr;
				fileio_info->param_2 = ret_addr;
			} else {
				semihosting->result = fstat(addr, &statbuf);
				semihosting->sys_errno = errno;
				if (semihosting->result >= 0) {
					fill_stat_struct(&stat_ret, &statbuf);
					retval =
						target_write_buffer(target, ret_addr, sizeof(struct stat_ret),
						(uint8_t *)(&stat_ret));
					if (retval != ERROR_OK) {
						semihosting->result = -1;
						semihosting->sys_errno = EIO;
						return retval;
					}
				}
				LOG_DEBUG("fstat('%d')=%" PRId64, addr, semihosting->result);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_STAT:			/* 0x112 */
		/* Gets information with the given file path. */
		retval = semihosting_read_fields(target, 3, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int len = semihosting_get_field(target, 1, fields);
			int ret_addr = semihosting_get_field(target, 2, fields);
			char *fn;
			if (semihosting->is_fileio) {
				semihosting->hit_fileio = true;
				fileio_info->identifier = "stat";
				fileio_info->param_1 = addr;
				fileio_info->param_2 = len;
				fileio_info->param_3 = ret_addr;
			} else {
				retval = semihosting_get_file_name(target, addr, len, &fn);
				if (retval != ERROR_OK)
					return retval;
				if (fn) {
					struct stat statbuf;
					struct stat_ret stat_ret;
					semihosting->result = stat(fn, &statbuf);
					semihosting->sys_errno = errno;

					if (semihosting->result >= 0) {
						fill_stat_struct(&stat_ret, &statbuf);
						/* FIXME: endiannes issue */
						uint8_t *buff = (uint8_t *)(&stat_ret);
						retval = target_write_buffer(target, ret_addr, sizeof(struct stat_ret), buff);
						if (retval != ERROR_OK) {
							semihosting->result = -1;
							semihosting->sys_errno = EIO;
							free(fn);
							return retval;
						}
					}
					LOG_DEBUG("stat('%s')=%" PRId64, fn, semihosting->result);
					free(fn);
				}
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_FSYNC:			/* 0x113 */
		/* Synchronizes file between in-core state and storage device with given file number. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_FSYNC not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 1, fields);
		if (retval == ERROR_OK) {
			uint64_t fd = semihosting_get_field(target, 0, fields);
			semihosting->result = fsync(fd);
			semihosting->sys_errno = errno;
			LOG_DEBUG("fsync('%'" PRIu64 "')=%" PRId64, fd, semihosting->result);
		}
		break;

	case ESP_SEMIHOSTING_SYS_LINK:			/* 0x114 */
		/* Creates a new link to an existing file between given paths. */
		if (semihosting->is_fileio) {
			LOG_ERROR("SYS_LINK not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = semihosting_read_fields(target, 4, fields);
		if (retval == ERROR_OK) {
			int source_addr = semihosting_get_field(target, 0, fields);
			int source_len = semihosting_get_field(target, 1, fields);
			int dest_addr = semihosting_get_field(target, 2, fields);
			int dest_len = semihosting_get_field(target, 3, fields);
			char *source_fn, *dest_fn;

			retval = semihosting_get_file_name(target, source_addr, source_len, &source_fn);
			if (retval != ERROR_OK)
				return retval;
			retval = semihosting_get_file_name(target, dest_addr, dest_len, &dest_fn);
			if (retval != ERROR_OK) {
				free(source_fn);
				return retval;
			}
			if (source_fn && dest_fn) {
				semihosting->result = link(source_fn, dest_fn);
				semihosting->sys_errno = errno;
				LOG_DEBUG("link('%s %s')=%" PRId64, source_fn, dest_fn, semihosting->result);
				free(source_fn);
				free(dest_fn);
			}
		}
		break;

	case ESP_SEMIHOSTING_SYS_UNLINK:		/* 0x115 */
		/* Unlinks the file to the given path. */
		retval = semihosting_read_fields(target, 2, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int len = semihosting_get_field(target, 1, fields);
			char *fn;
			if (semihosting->is_fileio) {
				semihosting->hit_fileio = true;
				fileio_info->identifier = "unlink";
				fileio_info->param_1 = addr;
				fileio_info->param_2 = len;
			} else {
				retval = semihosting_get_file_name(target, addr, len, &fn);
				if (retval != ERROR_OK)
					return retval;
				if (fn) {
					semihosting->result = unlink(fn);
					semihosting->sys_errno = errno;
					LOG_DEBUG("unlink('%s')=%" PRId64, fn, semihosting->result);
					free(fn);
				}
			}
		}
		break;
	}

	return retval;
}

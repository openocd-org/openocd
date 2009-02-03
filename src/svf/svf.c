/***************************************************************************
 *	 Copyright (C) 2009 by Simon Qian									   *
 *	 SimonQian@SimonQian.com											   *
 *                                                                         *
 *	 This program is free software; you can redistribute it and/or modify  *
 *	 it under the terms of the GNU General Public License as published by  *
 *	 the Free Software Foundation; either version 2 of the License, or	   *
 *	 (at your option) any later version.								   *
 *																		   *
 *	 This program is distributed in the hope that it will be useful,	   *
 *	 but WITHOUT ANY WARRANTY; without even the implied warranty of		   *
 *	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the		   *
 *	 GNU General Public License for more details.						   *
 *																		   *
 *	 You should have received a copy of the GNU General Public License	   *
 *	 along with this program; if not, write to the						   *
 *	 Free Software Foundation, Inc.,									   *
 *	 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.			   *
 ***************************************************************************/


/* The specification for SVF is available here:
 * http://www.asset-intertech.com/support/svf.pdf
 * Below, this document is refered to as the "SVF spec".
 *
 * The specification for XSVF is available here:
 * http://www.xilinx.com/support/documentation/application_notes/xapp503.pdf
 * Below, this document is refered to as the "XSVF spec".
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "svf.h"

#include "jtag.h"
#include "command.h"
#include "log.h"
#include "time_support.h"

#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include <sys/time.h>
#include <time.h>

// SVF command
typedef enum
{
	ENDDR,
	ENDIR,
	FREQUENCY,
	HDR,
	HIR,
	PIO,
	PIOMAP,
	RUNTEST,
	SDR,
	SIR,
	STATE,
	TDR,
	TIR,
	TRST,
}svf_command_t;

const char *svf_command_name[14] =
{
	"ENDDR",
	"ENDIR",
	"FREQUENCY",
	"HDR",
	"HIR",
	"PIO",
	"PIOMAP",
	"RUNTEST",
	"SDR",
	"SIR",
	"STATE",
	"TDR",
	"TIR",
	"TRST"
};

typedef enum
{
	TRST_ON,
	TRST_OFF,
	TRST_Z,
	TRST_ABSENT
}trst_mode_t;

const char *svf_trst_mode_name[4] =
{
	"ON",
	"OFF",
	"Z",
	"ABSENT"
};

char *svf_tap_state_name[16];

#define XXR_TDI						(1 << 0)
#define XXR_TDO						(1 << 1)
#define XXR_MASK					(1 << 2)
#define XXR_SMASK					(1 << 3)
typedef struct
{
	int len;
	int data_mask;
	u8 *tdi;
	u8 *tdo;
	u8 *mask;
	u8 *smask;
}svf_xxr_para_t;

typedef struct
{
	float frequency;
	tap_state_t ir_end_state;
	tap_state_t dr_end_state;
	tap_state_t runtest_run_state;
	tap_state_t runtest_end_state;
	trst_mode_t trst_mode;

	svf_xxr_para_t hir_para;
	svf_xxr_para_t hdr_para;
	svf_xxr_para_t tir_para;
	svf_xxr_para_t tdr_para;
	svf_xxr_para_t sir_para;
	svf_xxr_para_t sdr_para;
}svf_para_t;

svf_para_t svf_para;
const svf_para_t svf_para_init =
{
//	frequency,	ir_end_state,	dr_end_state,	runtest_run_state,	runtest_end_state,	trst_mode
	0,			TAP_IDLE,		TAP_IDLE,		TAP_IDLE,			TAP_IDLE,			TRST_Z,
//	hir_para
//	{len,	data_mask,	tdi,	tdo,	mask,	smask},
	{0,		0,			NULL,	NULL,	NULL,	NULL},
//	hdr_para
//	{len,	data_mask,	tdi,	tdo,	mask,	smask},
	{0,		0,			NULL,	NULL,	NULL,	NULL},
//	tir_para
//	{len,	data_mask,	tdi,	tdo,	mask,	smask},
	{0,		0,			NULL,	NULL,	NULL,	NULL},
//	tdr_para
//	{len,	data_mask,	tdi,	tdo,	mask,	smask},
	{0,		0,			NULL,	NULL,	NULL,	NULL},
//	sir_para
//	{len,	data_mask,	tdi,	tdo,	mask,	smask},
	{0,		0,			NULL,	NULL,	NULL,	NULL},
//	sdr_para
//	{len,	data_mask,	tdi,	tdo,	mask,	smask},
	{0,		0,			NULL,	NULL,	NULL,	NULL},
};

typedef struct
{
	int line_num;		// used to record line number of the check operation
						// so more information could be printed
	int enabled;		// check is enabled or not
	int buffer_offset;	// buffer_offset to buffers
	int bit_len;		// bit length to check
}svf_check_tdo_para_t;

#define SVF_CHECK_TDO_PARA_SIZE	1024
static svf_check_tdo_para_t *svf_check_tdo_para = NULL;
static int svf_check_tdo_para_index = 0;

#define dimof(a)					(sizeof(a) / sizeof((a)[0]))

static int svf_read_command_from_file(int fd);
static int svf_check_tdo(void);
static int svf_add_check_para(u8 enabled, int buffer_offset, int bit_len);
static int svf_run_command(struct command_context_s *cmd_ctx, char *cmd_str);
static int handle_svf_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int svf_fd = 0;
static char *svf_command_buffer = NULL;
static int svf_command_buffer_size = 0;
static int svf_line_number = 1;

static jtag_tap_t *tap = NULL;
static tap_state_t last_state = TAP_RESET;

#define SVF_MAX_BUFFER_SIZE_TO_COMMIT	(4 * 1024)
static u8 *svf_tdi_buffer = NULL, *svf_tdo_buffer = NULL, *svf_mask_buffer = NULL;
static int svf_buffer_index = 0, svf_buffer_size = 0;
static int svf_quiet = 0;


int svf_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "svf", handle_svf_command,
		COMMAND_EXEC, "run svf <file>");

	return ERROR_OK;
}

void svf_free_xxd_para(svf_xxr_para_t *para)
{
	if (NULL != para)
	{
		if (para->tdi != NULL)
		{
			free(para->tdi);
			para->tdi = NULL;
		}
		if (para->tdo != NULL)
		{
			free(para->tdo);
			para->tdo = NULL;
		}
		if (para->mask != NULL)
		{
			free(para->mask);
			para->mask = NULL;
		}
		if (para->smask != NULL)
		{
			free(para->smask);
			para->smask = NULL;
		}
	}
}

static int handle_svf_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
#define SVF_NUM_OF_OPTIONS			1
	int command_num = 0, i;
	int ret = ERROR_OK;
	long long time_ago;

	if ((argc < 1) || (argc > (1 + SVF_NUM_OF_OPTIONS)))
	{
		command_print(cmd_ctx, "usage: svf <file> [quiet]");
		return ERROR_FAIL;
	}

	// parse variant
	svf_quiet = 0;
	for (i = 1; i < argc; i++)
	{
		if (!strcmp(args[i], "quiet"))
		{
			svf_quiet = 1;
		}
		else
		{
			LOG_ERROR("unknown variant for svf: %s", args[i]);

			// no need to free anything now
			return ERROR_FAIL;
		}
	}

	if ((svf_fd = open(args[0], O_RDONLY)) < 0)
	{
		command_print(cmd_ctx, "file \"%s\" not found", args[0]);

		// no need to free anything now
		return ERROR_FAIL;
	}

	LOG_USER("svf processing file: \"%s\"", args[0]);

	// get time
	time_ago = timeval_ms();

	// init
	svf_line_number = 1;
	svf_command_buffer_size = 0;

	svf_check_tdo_para_index = 0;
	svf_check_tdo_para = malloc(sizeof(svf_check_tdo_para_t) * SVF_CHECK_TDO_PARA_SIZE);
	if (NULL == svf_check_tdo_para)
	{
		LOG_ERROR("not enough memory");
		ret = ERROR_FAIL;
		goto free_all;
	}

	svf_buffer_index = 0;
	// double the buffer size
	// in case current command cannot be commited, and next command is a bit scan command
	// here is 32K bits for this big scan command, it should be enough
	// buffer will be reallocated if buffer size is not enough
	svf_tdi_buffer = (u8 *)malloc(2 * SVF_MAX_BUFFER_SIZE_TO_COMMIT);
	if (NULL == svf_tdi_buffer)
	{
		LOG_ERROR("not enough memory");
		ret = ERROR_FAIL;
		goto free_all;
	}
	svf_tdo_buffer = (u8 *)malloc(2 * SVF_MAX_BUFFER_SIZE_TO_COMMIT);
	if (NULL == svf_tdo_buffer)
	{
		LOG_ERROR("not enough memory");
		ret = ERROR_FAIL;
		goto free_all;
	}
	svf_mask_buffer = (u8 *)malloc(2 * SVF_MAX_BUFFER_SIZE_TO_COMMIT);
	if (NULL == svf_mask_buffer)
	{
		LOG_ERROR("not enough memory");
		ret = ERROR_FAIL;
		goto free_all;
	}
	svf_buffer_size = 2 * SVF_MAX_BUFFER_SIZE_TO_COMMIT;

	memcpy(&svf_para, &svf_para_init, sizeof(svf_para));
	for (i = 0; i < dimof(svf_tap_state_name); i++)
	{
		svf_tap_state_name[i] = (char *)tap_state_name(i);
	}
	// TAP_RESET
	jtag_add_tlr();

	while ( ERROR_OK == svf_read_command_from_file(svf_fd) )
	{
		if (ERROR_OK != svf_run_command(cmd_ctx, svf_command_buffer))
		{
			LOG_ERROR("fail to run command at line %d", svf_line_number);
			ret = ERROR_FAIL;
			break;
		}
		command_num++;
	}
	if (ERROR_OK != jtag_execute_queue())
	{
		ret = ERROR_FAIL;
	}
	else if (ERROR_OK != svf_check_tdo())
	{
		ret = ERROR_FAIL;
	}

	// print time
	command_print(cmd_ctx, "%d ms used", timeval_ms() - time_ago);

free_all:

	close(svf_fd);
	svf_fd = 0;

	// free buffers
	if (svf_command_buffer)
	{
		free(svf_command_buffer);
		svf_command_buffer = NULL;
		svf_command_buffer_size = 0;
	}
	if (svf_check_tdo_para)
	{
		free(svf_check_tdo_para);
		svf_check_tdo_para = NULL;
		svf_check_tdo_para_index = 0;
	}
	if (svf_tdi_buffer)
	{
		free(svf_tdi_buffer);
		svf_tdi_buffer = NULL;
	}
	if (svf_tdo_buffer)
	{
		free(svf_tdo_buffer);
		svf_tdo_buffer = NULL;
	}
	if (svf_mask_buffer)
	{
		free(svf_mask_buffer);
		svf_mask_buffer = NULL;
	}
	svf_buffer_index = 0;
	svf_buffer_size = 0;

	svf_free_xxd_para(&svf_para.hdr_para);
	svf_free_xxd_para(&svf_para.hir_para);
	svf_free_xxd_para(&svf_para.tdr_para);
	svf_free_xxd_para(&svf_para.tir_para);
	svf_free_xxd_para(&svf_para.sdr_para);
	svf_free_xxd_para(&svf_para.sir_para);

	if (ERROR_OK == ret)
	{
		command_print(cmd_ctx, "svf file programmed successfully for %d commands", command_num);
	}
	else
	{
		command_print(cmd_ctx, "svf file programmed failed");
	}

	return ret;
}

#define SVFP_CMD_INC_CNT			1024
static int svf_read_command_from_file(int fd)
{
	char ch, *tmp_buffer = NULL;
	int cmd_pos = 0, cmd_ok = 0, slash = 0, comment = 0;

	while (!cmd_ok && (read(fd, &ch, 1) > 0) )
	{
		switch(ch)
		{
		case '!':
			slash = 0;
			comment = 1;
			break;
		case '/':
			if (++slash == 2)
			{
				comment = 1;
			}
			break;
		case ';':
			slash = 0;
			if (!comment)
			{
				cmd_ok = 1;
			}
			break;
		case '\n':
			svf_line_number++;
		case '\r':
			slash = 0;
			comment = 0;
			break;
		default:
			if (!comment)
			{
				if (cmd_pos >= svf_command_buffer_size - 1)
				{
					tmp_buffer = (char*)malloc(svf_command_buffer_size + SVFP_CMD_INC_CNT);		// 1 more byte for '\0'
					if (NULL == tmp_buffer)
					{
						LOG_ERROR("not enough memory");
						return ERROR_FAIL;
					}
					if (svf_command_buffer_size > 0)
					{
						memcpy(tmp_buffer, svf_command_buffer, svf_command_buffer_size);
					}
					if (svf_command_buffer != NULL)
					{
						free(svf_command_buffer);
					}
					svf_command_buffer = tmp_buffer;
					svf_command_buffer_size += SVFP_CMD_INC_CNT;
					tmp_buffer = NULL;
				}
				svf_command_buffer[cmd_pos++] = (char)toupper(ch);
			}
			break;
		}
	}

	if (cmd_ok)
	{
		svf_command_buffer[cmd_pos] = '\0';
		return ERROR_OK;
	}
	else
	{
		return ERROR_FAIL;
	}
}

static int svf_parse_cmd_string(char *str, int len, char **argus, int *num_of_argu)
{
	int pos = 0, num = 0, space_found = 1;

	while (pos < len)
	{
		switch(str[pos])
		{
		case '\n':
		case '\r':
		case '!':
		case '/':
			LOG_ERROR("fail to parse svf command");
			return ERROR_FAIL;
			break;
		case ' ':
			space_found = 1;
			str[pos] = '\0';
			break;
		default:
			if (space_found)
			{
				argus[num++] = &str[pos];
				space_found = 0;
			}
			break;
		}
		pos++;
	}

	*num_of_argu = num;

	return ERROR_OK;
}

static int svf_tap_state_is_stable(tap_state_t state)
{
	return ((TAP_RESET == state) || (TAP_IDLE == state) || (TAP_DRPAUSE == state) || (TAP_IRPAUSE == state));
}

static int svf_tap_state_is_valid(tap_state_t state)
{
	return ((state >= 0) && (state < sizeof(svf_tap_state_name)));
}

static int svf_find_string_in_array(char *str, char **strs, int num_of_element)
{
	int i;

	for (i = 0; i < num_of_element; i++)
	{
		if (!strcmp(str, strs[i]))
		{
			return i;
		}
	}
	return 0xFF;
}

static int svf_adjust_array_length(u8 **arr, int orig_bit_len, int new_bit_len)
{
	int new_byte_len = (new_bit_len + 7) >> 3;

	if ((NULL == *arr) || (((orig_bit_len + 7) >> 3) < ((new_bit_len + 7) >> 3)))
	{
		if (*arr != NULL)
		{
			free(*arr);
			*arr = NULL;
		}
		*arr = (u8*)malloc(new_byte_len);
		if (NULL == *arr)
		{
			LOG_ERROR("not enough memory");
			return ERROR_FAIL;
		}
		memset(*arr, 0, new_byte_len);
	}
	return ERROR_OK;
}

static int svf_copy_hexstring_to_binary(char *str, u8 **bin, int orig_bit_len, int bit_len)
{
	int i, str_len = strlen(str), str_byte_len = (bit_len + 3) >> 2, loop_cnt;
	u8 ch, need_write = 1;

	if (ERROR_OK != svf_adjust_array_length(bin, orig_bit_len, bit_len))
	{
		LOG_ERROR("fail to adjust length of array");
		return ERROR_FAIL;
	}

	if (str_byte_len > str_len)
	{
		loop_cnt = str_byte_len;
	}
	else
	{
		loop_cnt = str_len;
	}

	for (i = 0; i < loop_cnt; i++)
	{
		if (i < str_len)
		{
			ch = str[str_len - i - 1];
			if ((ch >= '0') && (ch <= '9'))
			{
				ch = ch - '0';
			}
			else if ((ch >= 'A') && (ch <= 'F'))
			{
				ch = ch - 'A' + 10;
			}
			else
			{
				LOG_ERROR("invalid hex string");
				return ERROR_FAIL;
			}
		}
		else
		{
			ch = 0;
		}

		// check valid
		if (i >= str_byte_len)
		{
			// all data written, other data should be all '0's and needn't to be written
			need_write = 0;
			if (ch != 0)
			{
				LOG_ERROR("value execede length");
				return ERROR_FAIL;
			}
		}
		else if (i == (str_byte_len - 1))
		{
			// last data byte, written if valid
			if ((ch & ~((1 << (bit_len - 4 * i)) - 1)) != 0)
			{
				LOG_ERROR("value execede length");
				return ERROR_FAIL;
			}
		}

		if (need_write)
		{
			// write bin
			if (i % 2)
			{
				// MSB
				(*bin)[i / 2] |= ch << 4;
			}
			else
			{
				// LSB
				(*bin)[i / 2] = 0;
				(*bin)[i / 2] |= ch;
			}
		}
	}

	return ERROR_OK;
}

static int svf_check_tdo(void)
{
	int i, j, byte_len, index;

	for (i = 0; i < svf_check_tdo_para_index; i++)
	{
		if (svf_check_tdo_para[i].enabled)
		{
			byte_len = (svf_check_tdo_para[i].bit_len + 7) >> 3;
			index = svf_check_tdo_para[i].buffer_offset;
			for (j = 0; j < byte_len; j++)
			{
				if ((svf_tdi_buffer[index + j] & svf_mask_buffer[index + j]) != svf_tdo_buffer[index + j])
				{
					LOG_ERROR("tdo check error at line %d, read = 0x%X, want = 0x%X, mask = 0x%X",
								svf_check_tdo_para[i].line_num,
								(*(int*)(svf_tdi_buffer + index)) & ((1 << svf_check_tdo_para[i].bit_len) - 1),
								(*(int*)(svf_tdo_buffer + index)) & ((1 << svf_check_tdo_para[i].bit_len) - 1),
								(*(int*)(svf_mask_buffer + index)) & ((1 << svf_check_tdo_para[i].bit_len) - 1));
					return ERROR_FAIL;
				}
			}
		}
	}
	svf_check_tdo_para_index = 0;

	return ERROR_OK;
}

static int svf_add_check_para(u8 enabled, int buffer_offset, int bit_len)
{
	if (svf_check_tdo_para_index >= SVF_CHECK_TDO_PARA_SIZE)
	{
		LOG_ERROR("toooooo many operation undone");
		return ERROR_FAIL;
	}

	svf_check_tdo_para[svf_check_tdo_para_index].line_num = svf_line_number;
	svf_check_tdo_para[svf_check_tdo_para_index].bit_len = bit_len;
	svf_check_tdo_para[svf_check_tdo_para_index].enabled = enabled;
	svf_check_tdo_para[svf_check_tdo_para_index].buffer_offset = buffer_offset;
	svf_check_tdo_para_index++;

	return ERROR_OK;
}

static int svf_execute_tap(void)
{
	if (ERROR_OK != jtag_execute_queue())
	{
		return ERROR_FAIL;
	}
	else if (ERROR_OK != svf_check_tdo())
	{
		return ERROR_FAIL;
	}

	svf_buffer_index = 0;

	return ERROR_OK;
}

// not good to use this
extern jtag_command_t** jtag_get_last_command_p(void);
extern void* cmd_queue_alloc(size_t size);
extern jtag_command_t **last_comand_pointer;

static int svf_run_command(struct command_context_s *cmd_ctx, char *cmd_str)
{
	char *argus[256], command;
	int num_of_argu = 0, i;

	// tmp variable
	int i_tmp;

	// not good to use this
	jtag_command_t **last_cmd;

	// for RUNTEST
	int run_count;
	float min_time, max_time;
	// for XXR
	svf_xxr_para_t *xxr_para_tmp;
	u8 **pbuffer_tmp;
	scan_field_t field;
	// for STATE
	tap_state_t *path = NULL, state;

	if (!svf_quiet)
	{
		LOG_USER("%s", svf_command_buffer);
	}

	if (ERROR_OK != svf_parse_cmd_string(cmd_str, strlen(cmd_str), argus, &num_of_argu))
	{
		return ERROR_FAIL;
	}

	command = svf_find_string_in_array(argus[0], (char **)svf_command_name, dimof(svf_command_name));
	switch(command)
	{
	case ENDDR:
	case ENDIR:
		if (num_of_argu != 2)
		{
			LOG_ERROR("invalid parameter of %s", argus[0]);
			return ERROR_FAIL;
		}
		i_tmp = svf_find_string_in_array(argus[1], (char **)svf_tap_state_name, dimof(svf_tap_state_name));
		if (svf_tap_state_is_stable(i_tmp))
		{
			if (command == ENDIR)
			{
				svf_para.ir_end_state = i_tmp;
				LOG_DEBUG("\tir_end_state = %s", svf_tap_state_name[svf_para.ir_end_state]);
			}
			else
			{
				svf_para.dr_end_state = i_tmp;
				LOG_DEBUG("\tdr_end_state = %s", svf_tap_state_name[svf_para.dr_end_state]);
			}
		}
		else
		{
			LOG_ERROR("%s is not valid state", argus[1]);
			return ERROR_FAIL;
		}
		break;
	case FREQUENCY:
		if ((num_of_argu != 1) && (num_of_argu != 3))
		{
			LOG_ERROR("invalid parameter of %s", argus[0]);
			return ERROR_FAIL;
		}
		if (1 == num_of_argu)
		{
			// TODO: set jtag speed to full speed
			svf_para.frequency = 0;
		}
		else
		{
			if (strcmp(argus[2], "HZ"))
			{
				LOG_ERROR("HZ not found in FREQUENCY command");
				return ERROR_FAIL;
			}
			if (ERROR_OK != svf_execute_tap())
			{
				return ERROR_FAIL;
			}
			svf_para.frequency = atof(argus[1]);
			// TODO: set jtag speed to
			if (svf_para.frequency > 0)
			{
				command_run_linef(cmd_ctx, "jtag_khz %d", (int)svf_para.frequency / 1000);
				LOG_DEBUG("\tfrequency = %f", svf_para.frequency);
			}
		}
		break;
	case HDR:
		xxr_para_tmp = &svf_para.hdr_para;
		goto XXR_common;
	case HIR:
		xxr_para_tmp = &svf_para.hir_para;
		goto XXR_common;
	case TDR:
		xxr_para_tmp = &svf_para.tdr_para;
		goto XXR_common;
	case TIR:
		xxr_para_tmp = &svf_para.tir_para;
		goto XXR_common;
	case SDR:
		xxr_para_tmp = &svf_para.sdr_para;
		goto XXR_common;
	case SIR:
		xxr_para_tmp = &svf_para.sir_para;
		goto XXR_common;
		XXR_common:
		// XXR length [TDI (tdi)] [TDO (tdo)][MASK (mask)] [SMASK (smask)]
		if ((num_of_argu > 10) || (num_of_argu % 2))
		{
			LOG_ERROR("invalid parameter of %s", argus[0]);
			return ERROR_FAIL;
		}
		i_tmp = xxr_para_tmp->len;
		xxr_para_tmp->len = atoi(argus[1]);
		LOG_DEBUG("\tlength = %d", xxr_para_tmp->len);
		xxr_para_tmp->data_mask = 0;
		for (i = 2; i < num_of_argu; i += 2)
		{
			if ((strlen(argus[i + 1]) < 3) || (argus[i + 1][0] != '(') || (argus[i + 1][strlen(argus[i + 1]) - 1] != ')'))
			{
				LOG_ERROR("data section error");
				return ERROR_FAIL;
			}
			argus[i + 1][strlen(argus[i + 1]) - 1] = '\0';
			// TDI, TDO, MASK, SMASK
			if (!strcmp(argus[i], "TDI"))
			{
				// TDI
				pbuffer_tmp = &xxr_para_tmp->tdi;
				xxr_para_tmp->data_mask |= XXR_TDI;
			}
			else if (!strcmp(argus[i], "TDO"))
			{
				// TDO
				pbuffer_tmp = &xxr_para_tmp->tdo;
				xxr_para_tmp->data_mask |= XXR_TDO;
			}
			else if (!strcmp(argus[i], "MASK"))
			{
				// MASK
				pbuffer_tmp = &xxr_para_tmp->mask;
				xxr_para_tmp->data_mask |= XXR_MASK;
			}
			else if (!strcmp(argus[i], "SMASK"))
			{
				// SMASK
				pbuffer_tmp = &xxr_para_tmp->smask;
				xxr_para_tmp->data_mask |= XXR_SMASK;
			}
			else
			{
				LOG_ERROR("unknow parameter: %s", argus[i]);
				return ERROR_FAIL;
			}
			if (ERROR_OK != svf_copy_hexstring_to_binary(&argus[i + 1][1], pbuffer_tmp, i_tmp, xxr_para_tmp->len))
			{
				LOG_ERROR("fail to parse hex value");
				return ERROR_FAIL;
			}
			LOG_DEBUG("\t%s = 0x%X", argus[i], (**(int**)pbuffer_tmp) & ((1 << (xxr_para_tmp->len)) - 1));
		}
		// If a command changes the length of the last scan of the same type and the MASK parameter is absent,
		// the mask pattern used is all cares
		if (!(xxr_para_tmp->data_mask & XXR_MASK) && (i_tmp != xxr_para_tmp->len))
		{
			// MASK not defined and length changed
			if (ERROR_OK != svf_adjust_array_length(&xxr_para_tmp->mask, i_tmp, xxr_para_tmp->len))
			{
				LOG_ERROR("fail to adjust length of array");
				return ERROR_FAIL;
			}
			buf_set_ones(xxr_para_tmp->mask, xxr_para_tmp->len);
		}
		// do scan if necessary
		if (SDR == command)
		{
			// check buffer size first, reallocate if necessary
			i = svf_para.hdr_para.len + svf_para.sdr_para.len + svf_para.tdr_para.len;
			if ((svf_buffer_size - svf_buffer_index) < ((i + 7) >> 3))
			{
#if 1
				// simply print error message
				LOG_ERROR("buffer is not enough, report to author");
				return ERROR_FAIL;
#else
				u8 *buffer_tmp;

				// reallocate buffer
				buffer_tmp = (u8 *)malloc(svf_buffer_index + ((i + 7) >> 3));
				if (NULL == buffer_tmp)
				{
					LOG_ERROR("not enough memory");
					return ERROR_FAIL;
				}
				memcpy(buffer_tmp, svf_tdi_buffer, svf_buffer_index);
				// svf_tdi_buffer isn't NULL here
				free(svf_tdi_buffer);
				svf_tdi_buffer = buffer_tmp;

				buffer_tmp = (u8 *)malloc(svf_buffer_index + ((i + 7) >> 3));
				if (NULL == buffer_tmp)
				{
					LOG_ERROR("not enough memory");
					return ERROR_FAIL;
				}
				memcpy(buffer_tmp, svf_tdo_buffer, svf_buffer_index);
				// svf_tdo_buffer isn't NULL here
				free(svf_tdo_buffer);
				svf_tdo_buffer = buffer_tmp;

				buffer_tmp = (u8 *)malloc(svf_buffer_index + ((i + 7) >> 3));
				if (NULL == buffer_tmp)
				{
					LOG_ERROR("not enough memory");
					return ERROR_FAIL;
				}
				memcpy(buffer_tmp, svf_mask_buffer, svf_buffer_index);
				// svf_mask_buffer isn't NULL here
				free(svf_mask_buffer);
				svf_mask_buffer = buffer_tmp;

				buffer_tmp = NULL;
				svf_buffer_size = svf_buffer_index + ((i + 7) >> 3);
#endif
			}

			// assemble dr data
			i = 0;
			buf_set_buf(svf_para.hdr_para.tdi, 0, &svf_tdi_buffer[svf_buffer_index], i, svf_para.hdr_para.len);
			i += svf_para.hdr_para.len;
			buf_set_buf(svf_para.sdr_para.tdi, 0, &svf_tdi_buffer[svf_buffer_index], i, svf_para.sdr_para.len);
			i += svf_para.sdr_para.len;
			buf_set_buf(svf_para.tdr_para.tdi, 0, &svf_tdi_buffer[svf_buffer_index], i, svf_para.tdr_para.len);
			i += svf_para.tdr_para.len;

			// add check data
			if (svf_para.sdr_para.data_mask & XXR_TDO)
			{
				// assemble dr mask data
				i = 0;
				buf_set_buf(svf_para.hdr_para.mask, 0, &svf_mask_buffer[svf_buffer_index], i, svf_para.hdr_para.len);
				i += svf_para.hdr_para.len;
				buf_set_buf(svf_para.sdr_para.mask, 0, &svf_mask_buffer[svf_buffer_index], i, svf_para.sdr_para.len);
				i += svf_para.sdr_para.len;
				buf_set_buf(svf_para.tdr_para.mask, 0, &svf_mask_buffer[svf_buffer_index], i, svf_para.tdr_para.len);
				i += svf_para.tdr_para.len;
				// assemble dr check data
				i = 0;
				buf_set_buf(svf_para.hdr_para.tdo, 0, &svf_tdo_buffer[svf_buffer_index], i, svf_para.hdr_para.len);
				i += svf_para.hdr_para.len;
				buf_set_buf(svf_para.sdr_para.tdo, 0, &svf_tdo_buffer[svf_buffer_index], i, svf_para.sdr_para.len);
				i += svf_para.sdr_para.len;
				buf_set_buf(svf_para.tdr_para.tdo, 0, &svf_tdo_buffer[svf_buffer_index], i, svf_para.tdr_para.len);
				i += svf_para.tdr_para.len;

				svf_add_check_para(1, svf_buffer_index, i);
			}
			else
			{
				svf_add_check_para(0, svf_buffer_index, i);
			}
			field.tap = tap;
			field.num_bits = i;
			field.out_value = &svf_tdi_buffer[svf_buffer_index];
			field.out_mask = NULL;
			field.in_value = &svf_tdi_buffer[svf_buffer_index];
			field.in_check_value = NULL;
			field.in_check_mask = NULL;
			field.in_handler = NULL;
			field.in_handler_priv = NULL;
			jtag_add_plain_dr_scan(1, &field, svf_para.dr_end_state);

			svf_buffer_index += (i + 7) >> 3;
			last_state = svf_para.dr_end_state;
		}
		else if (SIR == command)
		{
			// check buffer size first, reallocate if necessary
			i = svf_para.hir_para.len + svf_para.sir_para.len + svf_para.tir_para.len;
			if ((svf_buffer_size - svf_buffer_index) < ((i + 7) >> 3))
			{
#if 1
				// simply print error message
				LOG_ERROR("buffer is not enough, report to author");
				return ERROR_FAIL;
#else
				u8 *buffer_tmp;

				// reallocate buffer
				buffer_tmp = (u8 *)malloc(svf_buffer_index + ((i + 7) >> 3));
				if (NULL == buffer_tmp)
				{
					LOG_ERROR("not enough memory");
					return ERROR_FAIL;
				}
				memcpy(buffer_tmp, svf_tdi_buffer, svf_buffer_index);
				// svf_tdi_buffer isn't NULL here
				free(svf_tdi_buffer);
				svf_tdi_buffer = buffer_tmp;

				buffer_tmp = (u8 *)malloc(svf_buffer_index + ((i + 7) >> 3));
				if (NULL == buffer_tmp)
				{
					LOG_ERROR("not enough memory");
					return ERROR_FAIL;
				}
				memcpy(buffer_tmp, svf_tdo_buffer, svf_buffer_index);
				// svf_tdo_buffer isn't NULL here
				free(svf_tdo_buffer);
				svf_tdo_buffer = buffer_tmp;

				buffer_tmp = (u8 *)malloc(svf_buffer_index + ((i + 7) >> 3));
				if (NULL == buffer_tmp)
				{
					LOG_ERROR("not enough memory");
					return ERROR_FAIL;
				}
				memcpy(buffer_tmp, svf_mask_buffer, svf_buffer_index);
				// svf_mask_buffer isn't NULL here
				free(svf_mask_buffer);
				svf_mask_buffer = buffer_tmp;

				buffer_tmp = NULL;
				svf_buffer_size = svf_buffer_index + ((i + 7) >> 3);
#endif
			}

			// assemble ir data
			i = 0;
			buf_set_buf(svf_para.hir_para.tdi, 0, &svf_tdi_buffer[svf_buffer_index], i, svf_para.hir_para.len);
			i += svf_para.hir_para.len;
			buf_set_buf(svf_para.sir_para.tdi, 0, &svf_tdi_buffer[svf_buffer_index], i, svf_para.sir_para.len);
			i += svf_para.sir_para.len;
			buf_set_buf(svf_para.tir_para.tdi, 0, &svf_tdi_buffer[svf_buffer_index], i, svf_para.tir_para.len);
			i += svf_para.tir_para.len;

			// add check data
			if (svf_para.sir_para.data_mask & XXR_TDO)
			{
				// assemble dr mask data
				i = 0;
				buf_set_buf(svf_para.hir_para.mask, 0, &svf_mask_buffer[svf_buffer_index], i, svf_para.hir_para.len);
				i += svf_para.hir_para.len;
				buf_set_buf(svf_para.sir_para.mask, 0, &svf_mask_buffer[svf_buffer_index], i, svf_para.sir_para.len);
				i += svf_para.sir_para.len;
				buf_set_buf(svf_para.tir_para.mask, 0, &svf_mask_buffer[svf_buffer_index], i, svf_para.tir_para.len);
				i += svf_para.tir_para.len;
				// assemble dr check data
				i = 0;
				buf_set_buf(svf_para.hir_para.tdo, 0, &svf_tdo_buffer[svf_buffer_index], i, svf_para.hir_para.len);
				i += svf_para.hir_para.len;
				buf_set_buf(svf_para.sir_para.tdo, 0, &svf_tdo_buffer[svf_buffer_index], i, svf_para.sir_para.len);
				i += svf_para.sir_para.len;
				buf_set_buf(svf_para.tir_para.tdo, 0, &svf_tdo_buffer[svf_buffer_index], i, svf_para.tir_para.len);
				i += svf_para.tir_para.len;

				svf_add_check_para(1, svf_buffer_index, i);
			}
			else
			{
				svf_add_check_para(0, svf_buffer_index, i);
			}
			field.tap = tap;
			field.num_bits = i;
			field.out_value = &svf_tdi_buffer[svf_buffer_index];
			field.out_mask = NULL;
			field.in_value = &svf_tdi_buffer[svf_buffer_index];
			field.in_check_value = NULL;
			field.in_check_mask = NULL;
			field.in_handler = NULL;
			field.in_handler_priv = NULL;
			jtag_add_plain_ir_scan(1, &field, svf_para.ir_end_state);

			svf_buffer_index += (i + 7) >> 3;
			last_state = svf_para.ir_end_state;
		}
		break;
	case PIO:
	case PIOMAP:
		LOG_ERROR("PIO and PIOMAP are not supported");
		return ERROR_FAIL;
		break;
	case RUNTEST:
		// RUNTEST [run_state] run_count run_clk [min_time SEC [MAXIMUM max_time SEC]] [ENDSTATE end_state]
		// RUNTEST [run_state] min_time SEC [MAXIMUM max_time SEC] [ENDSTATE end_state]
		if ((num_of_argu < 3) && (num_of_argu > 11))
		{
			LOG_ERROR("invalid parameter of %s", argus[0]);
			return ERROR_FAIL;
		}
		// init
		run_count = 0;
		min_time = 0;
		max_time = 0;
		i = 1;
		// run_state
		i_tmp = svf_find_string_in_array(argus[i], (char **)svf_tap_state_name, dimof(svf_tap_state_name));
		if (svf_tap_state_is_valid(i_tmp))
		{
			if (svf_tap_state_is_stable(i_tmp))
			{
				svf_para.runtest_run_state = i_tmp;

				// When a run_state is specified, the new  run_state becomes the default end_state
				svf_para.runtest_end_state = i_tmp;
				LOG_DEBUG("\trun_state = %s", svf_tap_state_name[svf_para.runtest_run_state]);
				i++;
			}
			else
			{
				LOG_ERROR("%s is not valid state", svf_tap_state_name[i_tmp]);
				return ERROR_FAIL;
			}
		}
		// run_count run_clk
		if (((i + 2) <= num_of_argu) && strcmp(argus[i + 1], "SEC"))
		{
			if (!strcmp(argus[i + 1], "TCK"))
			{
				// clock source is TCK
				run_count = atoi(argus[i]);
				LOG_DEBUG("\trun_count@TCK = %d", run_count);
			}
			else
			{
				LOG_ERROR("%s not supported for clock", argus[i + 1]);
				return ERROR_FAIL;
			}
			i += 2;
		}
		// min_time SEC
		if (((i + 2) <= num_of_argu) && !strcmp(argus[i + 1], "SEC"))
		{
			min_time = atof(argus[i]);
			LOG_DEBUG("\tmin_time = %fs", min_time);
			i += 2;
		}
		// MAXIMUM max_time SEC
		if (((i + 3) <= num_of_argu) && !strcmp(argus[i], "MAXIMUM") && !strcmp(argus[i + 2], "SEC"))
		{
			max_time = atof(argus[i + 1]);
			LOG_DEBUG("\tmax_time = %fs", max_time);
			i += 3;
		}
		// ENDSTATE end_state
		if (((i + 2) <= num_of_argu) && !strcmp(argus[i], "ENDSTATE"))
		{
			i_tmp = svf_find_string_in_array(argus[i + 1], (char **)svf_tap_state_name, dimof(svf_tap_state_name));
			if (svf_tap_state_is_stable(i_tmp))
			{
				svf_para.runtest_end_state = i_tmp;
				LOG_DEBUG("\tend_state = %s", svf_tap_state_name[svf_para.runtest_end_state]);
			}
			else
			{
				LOG_ERROR("%s is not valid state", svf_tap_state_name[i_tmp]);
				return ERROR_FAIL;
			}
			i += 2;
		}
		// calculate run_count
		if ((0 == run_count) && (min_time > 0))
		{
			run_count = min_time * svf_para.frequency;
		}
		// all parameter should be parsed
		if (i == num_of_argu)
		{
			if (run_count > 0)
			{
				// run_state and end_state is checked to be stable state
				// TODO: do runtest
#if 1
				// enter into run_state if necessary
				if (last_state != svf_para.runtest_run_state)
				{
					last_cmd = jtag_get_last_command_p();
					*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
					last_comand_pointer = &((*last_cmd)->next);
					(*last_cmd)->next = NULL;
					(*last_cmd)->type = JTAG_STATEMOVE;
					(*last_cmd)->cmd.statemove = cmd_queue_alloc(sizeof(statemove_command_t));
					(*last_cmd)->cmd.statemove->end_state = svf_para.runtest_run_state;

					cmd_queue_end_state = cmd_queue_cur_state = (*last_cmd)->cmd.statemove->end_state;
				}

				// call jtag_add_clocks
				jtag_add_clocks(run_count);

				if (svf_para.runtest_end_state != svf_para.runtest_run_state)
				{
					// move to end_state
					last_cmd = jtag_get_last_command_p();
					*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
					last_comand_pointer = &((*last_cmd)->next);
					(*last_cmd)->next = NULL;
					(*last_cmd)->type = JTAG_STATEMOVE;
					(*last_cmd)->cmd.statemove = cmd_queue_alloc(sizeof(statemove_command_t));
					(*last_cmd)->cmd.statemove->end_state = svf_para.runtest_end_state;

					cmd_queue_end_state = cmd_queue_cur_state = (*last_cmd)->cmd.statemove->end_state;
				}
				last_state = svf_para.runtest_end_state;
#else
				if (svf_para.runtest_run_state != TAP_IDLE)
				{
					// RUNTEST can only executed in TAP_IDLE
					LOG_ERROR("cannot runtest in %s state", svf_tap_state_name[svf_para.runtest_run_state]);
					return ERROR_FAIL;
				}

				jtag_add_runtest(run_count, svf_para.runtest_end_state);
#endif
			}
		}
		else
		{
			LOG_ERROR("fail to parse parameter of RUNTEST, %d out of %d is parsed", i, num_of_argu);
			return ERROR_FAIL;
		}
		break;
	case STATE:
		// STATE [pathstate1 [pathstate2 ...[pathstaten]]] stable_state
		if (num_of_argu < 2)
		{
			LOG_ERROR("invalid parameter of %s", argus[0]);
			return ERROR_FAIL;
		}
		if (num_of_argu > 2)
		{
			// STATE pathstate1 ... stable_state
			path = (tap_state_t *)malloc((num_of_argu - 1) * sizeof(tap_state_t));
			if (NULL == path)
			{
				LOG_ERROR("not enough memory");
				return ERROR_FAIL;
			}
			num_of_argu--;		// num of path
			i_tmp = 1;			// path is from patameter 1
			for (i = 0; i < num_of_argu; i++)
			{
				path[i] = svf_find_string_in_array(argus[i_tmp++], (char **)svf_tap_state_name, dimof(svf_tap_state_name));
				if (!svf_tap_state_is_valid(path[i]))
				{
					LOG_ERROR("%s is not valid state", svf_tap_state_name[path[i]]);
					return ERROR_FAIL;
				}
				if (TAP_RESET == path[i])
				{
					if (i > 0)
					{
						jtag_add_pathmove(i, path);
					}
					jtag_add_tlr();
					num_of_argu -= i + 1;
					i = -1;
				}
			}
			if (num_of_argu > 0)
			{
				// execute last path if necessary
				if (svf_tap_state_is_stable(path[num_of_argu - 1]))
				{
					// last state MUST be stable state
					// TODO: call path_move
					jtag_add_pathmove(num_of_argu, path);
					last_state = path[num_of_argu - 1];
					LOG_DEBUG("\tmove to %s by path_move", svf_tap_state_name[path[num_of_argu - 1]]);
				}
				else
				{
					LOG_ERROR("%s is not valid state", svf_tap_state_name[path[num_of_argu - 1]]);
					return ERROR_FAIL;
				}
			}
			// no need to keep this memory, in jtag_add_pathmove, path will be duplicated
			if (NULL != path)
			{
				free(path);
				path = NULL;
			}
		}
		else
		{
			// STATE stable_state
			state = svf_find_string_in_array(argus[1], (char **)svf_tap_state_name, dimof(svf_tap_state_name));
			if (svf_tap_state_is_stable(state))
			{
				// TODO: move to state
				last_cmd = jtag_get_last_command_p();
				*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
				last_comand_pointer = &((*last_cmd)->next);
				(*last_cmd)->next = NULL;
				(*last_cmd)->type = JTAG_STATEMOVE;
				(*last_cmd)->cmd.statemove = cmd_queue_alloc(sizeof(statemove_command_t));
				(*last_cmd)->cmd.statemove->end_state = state;

				cmd_queue_end_state = cmd_queue_cur_state = (*last_cmd)->cmd.statemove->end_state;
				last_state = state;

				LOG_DEBUG("\tmove to %s by state_move", svf_tap_state_name[state]);
			}
			else
			{
				LOG_ERROR("%s is not valid state", svf_tap_state_name[state]);
				return ERROR_FAIL;
			}
		}
		break;
	case TRST:
		// TRST trst_mode
		if (num_of_argu != 2)
		{
			LOG_ERROR("invalid parameter of %s", argus[0]);
			return ERROR_FAIL;
		}
		if (svf_para.trst_mode != TRST_ABSENT)
		{
			if (ERROR_OK != svf_execute_tap())
			{
				return ERROR_FAIL;
			}
			i_tmp = svf_find_string_in_array(argus[1], (char **)svf_trst_mode_name, dimof(svf_trst_mode_name));
			switch (i_tmp)
			{
			case TRST_ON:
				last_state = TAP_RESET;
				jtag_add_reset(1, 0);
				break;
			case TRST_Z:
			case TRST_OFF:
				jtag_add_reset(0, 0);
				break;
			case TRST_ABSENT:
				break;
			default:
				LOG_ERROR("unknown TRST mode: %s", argus[1]);
				return ERROR_FAIL;
			}
			svf_para.trst_mode = i_tmp;
			LOG_DEBUG("\ttrst_mode = %s", svf_trst_mode_name[svf_para.trst_mode]);
		}
		else
		{
			LOG_ERROR("can not accpet TRST command if trst_mode is ABSENT");
			return ERROR_FAIL;
		}
		break;
	default:
		LOG_ERROR("invalid svf command: %s", argus[0]);
		return ERROR_FAIL;
		break;
	}

	if (debug_level >= LOG_LVL_DEBUG)
	{
		// for convenient debugging, execute tap if possible
		if ((svf_buffer_index > 0) && \
			(((command != STATE) && (command != RUNTEST)) || \
			((command == STATE) && (num_of_argu == 2))))
		{
			if (ERROR_OK != svf_execute_tap())
			{
				return ERROR_FAIL;
			}

			// output debug info
			if ((SIR == command) || (SDR == command))
			{
				// in debug mode, data is from index 0
				LOG_DEBUG("\tTDO read = 0x%X", (*(int*)svf_tdi_buffer) & ((1 << (svf_check_tdo_para[0].bit_len)) - 1));
			}
		}
	}
	else
	{
		// for fast executing, execute tap if necessary
		// half of the buffer is for the next command
		if (((svf_buffer_index >= SVF_MAX_BUFFER_SIZE_TO_COMMIT) || (svf_check_tdo_para_index >= SVF_CHECK_TDO_PARA_SIZE / 2)) && \
			(((command != STATE) && (command != RUNTEST)) || \
			((command == STATE) && (num_of_argu == 2))))
		{
			return svf_execute_tap();
		}
	}

	return ERROR_OK;
}

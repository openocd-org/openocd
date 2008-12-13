/***************************************************************************
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   Copyright (C) 2008 Duane Ellis                                        *
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

#include "replacements.h"
#include "target.h"
#include "target_request.h"

#include "log.h"
#include "configuration.h"
#include "binarybuffer.h"
#include "jtag.h"
#include "flash.h"

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

#include <sys/time.h>
#include <time.h>

#include <time_support.h>

#include <fileio.h>
#include <image.h>

static int new_int_array_element(Jim_Interp * interp, const char *varname, int idx, u32 val)
{
	char *namebuf;
	Jim_Obj *nameObjPtr, *valObjPtr;
	int result;

	namebuf = alloc_printf("%s(%d)", varname, idx);
	if (!namebuf)
		return JIM_ERR;

	nameObjPtr = Jim_NewStringObj(interp, namebuf, -1);
	valObjPtr = Jim_NewIntObj(interp, val);
	if (!nameObjPtr || !valObjPtr)
	{
		free(namebuf);
		return JIM_ERR;
	}

	Jim_IncrRefCount(nameObjPtr);
	Jim_IncrRefCount(valObjPtr);
	result = Jim_SetVariable(interp, nameObjPtr, valObjPtr);
	Jim_DecrRefCount(interp, nameObjPtr);
	Jim_DecrRefCount(interp, valObjPtr);
	free(namebuf);
	/* printf("%s(%d) <= 0%08x\n", varname, idx, val); */
	return result;
}

static int jim_mem2array(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	target_t *target;
	command_context_t *context;
	long l;
	u32 width;
	u32 len;
	u32 addr;
	u32 count;
	u32 v;
	const char *varname;
	u8 buffer[4096];
	int  i, n, e, retval;

	/* argv[1] = name of array to receive the data
	 * argv[2] = desired width
	 * argv[3] = memory address
	 * argv[4] = count of times to read
	 */
	if (argc != 5) {
		Jim_WrongNumArgs(interp, 1, argv, "varname width addr nelems");
		return JIM_ERR;
	}
	varname = Jim_GetString(argv[1], &len);
	/* given "foo" get space for worse case "foo(%d)" .. add 20 */

	e = Jim_GetLong(interp, argv[2], &l);
	width = l;
	if (e != JIM_OK) {
		return e;
	}

	e = Jim_GetLong(interp, argv[3], &l);
	addr = l;
	if (e != JIM_OK) {
		return e;
	}
	e = Jim_GetLong(interp, argv[4], &l);
	len = l;
	if (e != JIM_OK) {
		return e;
	}
	switch (width) {
		case 8:
			width = 1;
			break;
		case 16:
			width = 2;
			break;
		case 32:
			width = 4;
			break;
		default:
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings( interp, Jim_GetResult(interp), "Invalid width param, must be 8/16/32", NULL );
			return JIM_ERR;
	}
	if (len == 0) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: zero width read?", NULL);
		return JIM_ERR;
	}
	if ((addr + (len * width)) < addr) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: addr + len - wraps to zero?", NULL);
		return JIM_ERR;
	}
	/* absurd transfer size? */
	if (len > 65536) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: absurd > 64K item request", NULL);
		return JIM_ERR;
	}

	if ((width == 1) ||
		((width == 2) && ((addr & 1) == 0)) ||
		((width == 4) && ((addr & 3) == 0))) {
		/* all is well */
	} else {
		char buf[100];
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		sprintf(buf, "mem2array address: 0x%08x is not aligned for %d byte reads", addr, width);
		Jim_AppendStrings(interp, Jim_GetResult(interp), buf , NULL);
		return JIM_ERR;
	}

	context = Jim_GetAssocData(interp, "context");
	if (context == NULL)
	{
		LOG_ERROR("mem2array: no command context");
		return JIM_ERR;
	}
	target = get_current_target(context);
	if (target == NULL)
	{
		LOG_ERROR("mem2array: no current target");
		return JIM_ERR;
	}

	/* Transfer loop */

	/* index counter */
	n = 0;
	/* assume ok */
	e = JIM_OK;
	while (len) {
		/* Slurp... in buffer size chunks */

		count = len; /* in objects.. */
		if (count > (sizeof(buffer)/width)) {
			count = (sizeof(buffer)/width);
		}

		retval = target->type->read_memory( target, addr, width, count, buffer );
		if (retval != ERROR_OK) {
			/* BOO !*/
			LOG_ERROR("mem2array: Read @ 0x%08x, w=%d, cnt=%d, failed", addr, width, count);
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: cannot read memory", NULL);
			e = JIM_ERR;
			len = 0;
		} else {
			v = 0; /* shut up gcc */
			for (i = 0 ;i < count ;i++, n++) {
				switch (width) {
					case 4:
						v = target_buffer_get_u32(target, &buffer[i*width]);
						break;
					case 2:
						v = target_buffer_get_u16(target, &buffer[i*width]);
						break;
					case 1:
						v = buffer[i] & 0x0ff;
						break;
				}
				new_int_array_element(interp, varname, n, v);
			}
			len -= count;
		}
	}

	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));

	return JIM_OK;
}

static int get_int_array_element(Jim_Interp * interp, const char *varname, int idx, u32 *val)
{
	char *namebuf;
	Jim_Obj *nameObjPtr, *valObjPtr;
	int result;
	long l;

	namebuf = alloc_printf("%s(%d)", varname, idx);
	if (!namebuf)
		return JIM_ERR;

	nameObjPtr = Jim_NewStringObj(interp, namebuf, -1);
	if (!nameObjPtr)
	{
		free(namebuf);
		return JIM_ERR;
	}

	Jim_IncrRefCount(nameObjPtr);
	valObjPtr = Jim_GetVariable(interp, nameObjPtr, JIM_ERRMSG);
	Jim_DecrRefCount(interp, nameObjPtr);
	free(namebuf);
	if (valObjPtr == NULL)
		return JIM_ERR;

	result = Jim_GetLong(interp, valObjPtr, &l);
	/* printf("%s(%d) => 0%08x\n", varname, idx, val); */
	*val = l;
	return result;
}

static int jim_array2mem(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	target_t *target;
	command_context_t *context;
	long l;
	u32 width;
	u32 len;
	u32 addr;
	u32 count;
	u32 v;
	const char *varname;
	u8 buffer[4096];
	int  i, n, e, retval;

	/* argv[1] = name of array to get the data
	 * argv[2] = desired width
	 * argv[3] = memory address
	 * argv[4] = count to write
	 */
	if (argc != 5) {
		Jim_WrongNumArgs(interp, 1, argv, "varname width addr nelems");
		return JIM_ERR;
	}
	varname = Jim_GetString(argv[1], &len);
	/* given "foo" get space for worse case "foo(%d)" .. add 20 */

	e = Jim_GetLong(interp, argv[2], &l);
	width = l;
	if (e != JIM_OK) {
		return e;
	}

	e = Jim_GetLong(interp, argv[3], &l);
	addr = l;
	if (e != JIM_OK) {
		return e;
	}
	e = Jim_GetLong(interp, argv[4], &l);
	len = l;
	if (e != JIM_OK) {
		return e;
	}
	switch (width) {
		case 8:
			width = 1;
			break;
		case 16:
			width = 2;
			break;
		case 32:
			width = 4;
			break;
		default:
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings( interp, Jim_GetResult(interp), "Invalid width param, must be 8/16/32", NULL );
			return JIM_ERR;
	}
	if (len == 0) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "array2mem: zero width read?", NULL);
		return JIM_ERR;
	}
	if ((addr + (len * width)) < addr) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "array2mem: addr + len - wraps to zero?", NULL);
		return JIM_ERR;
	}
	/* absurd transfer size? */
	if (len > 65536) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "array2mem: absurd > 64K item request", NULL);
		return JIM_ERR;
	}

	if ((width == 1) ||
		((width == 2) && ((addr & 1) == 0)) ||
		((width == 4) && ((addr & 3) == 0))) {
		/* all is well */
	} else {
		char buf[100];
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		sprintf(buf, "array2mem address: 0x%08x is not aligned for %d byte reads", addr, width);
		Jim_AppendStrings(interp, Jim_GetResult(interp), buf , NULL);
		return JIM_ERR;
	}

	context = Jim_GetAssocData(interp, "context");
	if (context == NULL)
	{
		LOG_ERROR("array2mem: no command context");
		return JIM_ERR;
	}
	target = get_current_target(context);
	if (target == NULL)
	{
		LOG_ERROR("array2mem: no current target");
		return JIM_ERR;
	}

	/* Transfer loop */

	/* index counter */
	n = 0;
	/* assume ok */
	e = JIM_OK;
	while (len) {
		/* Slurp... in buffer size chunks */

		count = len; /* in objects.. */
		if (count > (sizeof(buffer)/width)) {
			count = (sizeof(buffer)/width);
		}

		v = 0; /* shut up gcc */
		for (i = 0 ;i < count ;i++, n++) {
			get_int_array_element(interp, varname, n, &v);
			switch (width) {
			case 4:
				target_buffer_set_u32(target, &buffer[i*width], v);
				break;
			case 2:
				target_buffer_set_u16(target, &buffer[i*width], v);
				break;
			case 1:
				buffer[i] = v & 0x0ff;
				break;
			}
		}
		len -= count;

		retval = target->type->write_memory(target, addr, width, count, buffer);
		if (retval != ERROR_OK) {
			/* BOO !*/
			LOG_ERROR("array2mem: Write @ 0x%08x, w=%d, cnt=%d, failed", addr, width, count);
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: cannot read memory", NULL);
			e = JIM_ERR;
			len = 0;
		}
	}

	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));

	return JIM_OK;
}

int Jim_Command_drscan(Jim_Interp *interp, int argc, Jim_Obj *const *args)
{
	int retval;
	scan_field_t *fields;
	int num_fields;
	int field_count = 0;
	int i, e;
	long device;

	/* args[1] = device
	 * args[2] = num_bits
	 * args[3] = hex string
	 * ... repeat num bits and hex string ...
	 */
	if ((argc < 4) || ((argc % 2)!=0))
	{
		Jim_WrongNumArgs(interp, 1, args, "<device> <num_bits1> <value1> <num_bits2> <value2> ...");
		return JIM_ERR;
	}

	for (i = 2; i < argc; i+=2)
	{
		long bits;

		e = Jim_GetLong(interp, args[i], &bits);
		if (e != JIM_OK)
			return e;
	}

	e = Jim_GetLong(interp, args[1], &device);
	if (e != JIM_OK)
		return e;

	num_fields=(argc-2)/2;
	fields = malloc(sizeof(scan_field_t) * num_fields);
	for (i = 2; i < argc; i+=2)
	{
		long bits;
		int len;
		const char *str;

		Jim_GetLong(interp, args[i], &bits);
		str = Jim_GetString(args[i+1], &len);

		fields[field_count].device = device;
		fields[field_count].num_bits = bits;
		fields[field_count].out_value = malloc(CEIL(bits, 8));
		str_to_buf(str, len, fields[field_count].out_value, bits, 0);
		fields[field_count].out_mask = NULL;
		fields[field_count].in_value = fields[field_count].out_value;
		fields[field_count].in_check_mask = NULL;
		fields[field_count].in_check_value = NULL;
		fields[field_count].in_handler = NULL;
		fields[field_count++].in_handler_priv = NULL;
	}

	jtag_add_dr_scan(num_fields, fields, -1);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
	{
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "drscan: jtag execute failed", NULL);
		return JIM_ERR;
	}

	field_count=0;
	Jim_Obj *list = Jim_NewListObj(interp, NULL, 0);
	for (i = 2; i < argc; i+=2)
	{
		long bits;
		char *str;

		Jim_GetLong(interp, args[i], &bits);
		str = buf_to_str(fields[field_count].in_value, bits, 16);
		free(fields[field_count].out_value);

		Jim_ListAppendElement(interp, list, Jim_NewStringObj(interp, str, strlen(str)));
		free(str);
		field_count++;
	}

	Jim_SetResult(interp, list);

	free(fields);

	return JIM_OK;
}

static int jim_flash_banks(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	flash_bank_t *p;

	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "no arguments to flash_banks command");
		return JIM_ERR;
	}

	if (!flash_banks)
	{
		return JIM_ERR;
	}

	Jim_Obj *list=Jim_NewListObj(interp, NULL, 0);
	for (p = flash_banks; p; p = p->next)
	{
		Jim_Obj *elem=Jim_NewListObj(interp, NULL, 0);

		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "name", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, p->driver->name, -1));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "base", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->base));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "size", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->size));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "bus_width", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->bus_width));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "chip_width", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->chip_width));

		Jim_ListAppendElement(interp, list, elem);
	}

	Jim_SetResult(interp, list);

	return JIM_OK;
}

int tclapi_register_commands()
{
	register_jim("ocd_mem2array", jim_mem2array, "read memory and return as a TCL array for script processing");
	register_jim("ocd_array2mem", jim_array2mem, "convert a TCL array to memory locations and write the values");
	register_jim("drscan", Jim_Command_drscan, "execute DR scan <device> <num_bits> <value> <num_bits1> <value2> ...");
	register_jim("ocd_flash_banks", jim_flash_banks, "return information about the flash banks");

}

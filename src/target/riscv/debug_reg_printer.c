// SPDX-License-Identifier: GPL-2.0-or-later

#include <stdio.h>
#include <inttypes.h>
#include <assert.h>
#include <stdarg.h>

#include "debug_reg_printer.h"

static unsigned int get_len_or_sprintf(char *buf, unsigned int curr, const char *format, ...)
{
	assert(format);
	va_list args;
	int length;

	va_start(args, format);
	if (buf)
		length = vsprintf(buf + curr, format, args);
	else
		length = vsnprintf(NULL, 0, format, args);
	va_end(args);
	assert(length >= 0);
	return (unsigned int)length;
}

static unsigned int print_number(char *buf, unsigned int offset, uint64_t value)
{
	const char * const format = value > 9 ? "0x%" PRIx64 : "%" PRIx64;

	return get_len_or_sprintf(buf, offset, format, value);
}

static unsigned int riscv_debug_reg_field_value_to_s(char *buf, unsigned int offset,
		const char * const *field_value_names, uint64_t field_value)
{
	const char * const field_value_name = field_value_names ?
		field_value_names[field_value] :
		NULL;

	if (!field_value_name)
		return print_number(buf, offset, field_value);
	return get_len_or_sprintf(buf, offset, "%s", field_value_name);
}

static unsigned int riscv_debug_reg_field_to_s(char *buf, unsigned int offset,
		riscv_debug_reg_field_info_t field, riscv_debug_reg_ctx_t context,
		uint64_t field_value)
{
	const unsigned int name_len = get_len_or_sprintf(buf, offset, "%s=", field.name);

	return name_len + riscv_debug_reg_field_value_to_s(buf, offset + name_len,
			field.values, field_value);
}

static uint64_t riscv_debug_reg_field_value(riscv_debug_reg_field_info_t field, uint64_t value)
{
	assert(field.msb < 64);
	assert(field.msb >= field.lsb);
	const uint64_t trailing_ones_mask = (uint64_t)(-1) >> (63 - field.msb);
	return (value & trailing_ones_mask) >> field.lsb;
}

static unsigned int riscv_debug_reg_fields_to_s(char *buf, unsigned int offset,
	struct riscv_debug_reg_field_list_t (*get_next)(riscv_debug_reg_ctx_t contex),
	riscv_debug_reg_ctx_t context, uint64_t value,
	enum riscv_debug_reg_show show)
{
	unsigned int curr = offset;
	curr += get_len_or_sprintf(buf, curr, " {");
	char *separator = "";
	for (struct riscv_debug_reg_field_list_t list; get_next; get_next = list.get_next) {
		list = get_next(context);

		uint64_t field_value = riscv_debug_reg_field_value(list.field, value);

		if (show == RISCV_DEBUG_REG_SHOW_ALL ||
				(show == RISCV_DEBUG_REG_HIDE_UNNAMED_0 &&
					(field_value != 0 ||
						(list.field.values && list.field.values[0]))) ||
				(show == RISCV_DEBUG_REG_HIDE_ALL_0 && field_value != 0)) {
			curr += get_len_or_sprintf(buf, curr, separator);
			curr += riscv_debug_reg_field_to_s(buf, curr, list.field, context,
							field_value);
			separator = " ";
		}
	}
	curr += get_len_or_sprintf(buf, curr, "}");
	return curr - offset;
}

unsigned int riscv_debug_reg_to_s(char *buf, enum riscv_debug_reg_ordinal reg_ordinal,
		riscv_debug_reg_ctx_t context, uint64_t value,
		enum riscv_debug_reg_show show)
{
	unsigned int length = 0;

	riscv_debug_reg_info_t reg = get_riscv_debug_reg_info(reg_ordinal);

	length += get_len_or_sprintf(buf, length, "%s=", reg.name);
	length += print_number(buf, length, value);

	if (reg.get_fields_head)
		length += riscv_debug_reg_fields_to_s(buf, length,
				reg.get_fields_head, context, value, show);

	if (buf)
		buf[length] = '\0';
	return length;
}

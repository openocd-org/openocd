/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_MINIDRIVER_H
#define OPENOCD_JTAG_MINIDRIVER_H

/**
 * @page jtagminidriver JTAG Mini-Driver
 *
 * The JTAG minidriver interface allows the definition of alternate
 * interface functions, instead of the built-in asynchronous driver
 * module that is used by the standard JTAG interface drivers.
 *
 * In addition to the functions defined in the @c minidriver.h file, the
 * @c jtag_minidriver.h file must declare the following functions (or
 * define static inline versions of them):
 * - jtag_add_callback
 * - jtag_add_callback4
 *
 * The following core functions are declared in this file for use by
 * the minidriver and do @b not need to be defined by an implementation:
 * - default_interface_jtag_execute_queue()
 */

/* this header will be provided by the minidriver implementation, */
/* and it may provide additional declarations that must be defined. */
#include <jtag/drivers/minidriver_imp.h>

int interface_jtag_add_ir_scan(struct jtag_tap *active,
		const struct scan_field *fields,
		enum tap_state endstate);
int interface_jtag_add_plain_ir_scan(
		int num_bits, const uint8_t *out_bits, uint8_t *in_bits,
		enum tap_state endstate);

int interface_jtag_add_dr_scan(struct jtag_tap *active,
		int num_fields, const struct scan_field *fields,
		enum tap_state endstate);
int interface_jtag_add_plain_dr_scan(
		int num_bits, const uint8_t *out_bits, uint8_t *in_bits,
		enum tap_state endstate);

int interface_jtag_add_tlr(void);
int interface_jtag_add_pathmove(unsigned int num_states, const enum tap_state *path);
int interface_jtag_add_runtest(unsigned int num_cycles, enum tap_state endstate);

int interface_add_tms_seq(unsigned int num_bits,
		const uint8_t *bits, enum tap_state state);

/**
 * This drives the actual srst and trst pins. srst will always be 0
 * if jtag_reset_config & RESET_SRST_PULLS_TRST != 0 and ditto for
 * trst.
 *
 * the higher level jtag_add_reset will invoke jtag_add_tlr() if
 * appropriate
 */
int interface_jtag_add_reset(int trst, int srst);
int interface_jtag_add_sleep(uint32_t us);
int interface_jtag_add_clocks(unsigned int num_cycles);
int interface_jtag_execute_queue(void);

/**
 * Calls the interface callback to execute the queue.  This routine
 * is used by the JTAG driver layer and should not be called directly.
 */
int default_interface_jtag_execute_queue(void);

#endif /* OPENOCD_JTAG_MINIDRIVER_H */

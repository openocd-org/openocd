/***************************************************************************
 *   Copyright (C) 2005, 2007 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007 by Vincent Palatin                                 *
 *   vincent.palatin_openocd@m4x.org                                       *
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
#ifndef ETM_H
#define ETM_H

#include "trace.h"
#include "arm_jtag.h"
#include "armv4_5.h"

struct image_s;

/* ETM registers (JTAG protocol) */
enum
{
	ETM_CTRL = 0x00,
	ETM_CONFIG = 0x01,
	ETM_TRIG_EVENT = 0x02,
	ETM_ASIC_CTRL = 0x03,
	ETM_STATUS = 0x04,
	ETM_SYS_CONFIG = 0x05,
	ETM_TRACE_RESOURCE_CTRL = 0x06,
	ETM_TRACE_EN_CTRL2 = 0x07,
	ETM_TRACE_EN_EVENT = 0x08,
	ETM_TRACE_EN_CTRL1 = 0x09,
	/* optional FIFOFULL */
	ETM_FIFOFULL_REGION = 0x0a,
	ETM_FIFOFULL_LEVEL = 0x0b,
	/* viewdata support */
	ETM_VIEWDATA_EVENT = 0x0c,
	ETM_VIEWDATA_CTRL1 = 0x0d,
	ETM_VIEWDATA_CTRL2 = 0x0e,	/* optional */
	ETM_VIEWDATA_CTRL3 = 0x0f,
	/* N pairs of ADDR_{COMPARATOR,ACCESS} registers */
	ETM_ADDR_COMPARATOR_VALUE = 0x10,
	ETM_ADDR_ACCESS_TYPE = 0x20,
	/* N pairs of DATA_COMPARATOR_{VALUE,MASK} registers */
	ETM_DATA_COMPARATOR_VALUE = 0x30,
	ETM_DATA_COMPARATOR_MASK = 0x40,
	/* N quads of COUNTER_{RELOAD_{VALUE,EVENT},ENABLE,VALUE} registers */
	ETM_COUNTER_RELOAD_VALUE = 0x50,
	ETM_COUNTER_ENABLE = 0x54,
	ETM_COUNTER_RELOAD_EVENT = 0x58,
	ETM_COUNTER_VALUE = 0x5c,
	/* 6 sequencer event transitions */
	ETM_SEQUENCER_EVENT = 0x60,
	ETM_SEQUENCER_STATE = 0x67,
	/* N triggered outputs */
	ETM_EXTERNAL_OUTPUT = 0x68,
	/* N task contexts */
	ETM_CONTEXTID_COMPARATOR_VALUE = 0x6c,
	ETM_CONTEXTID_COMPARATOR_MASK = 0x6f,
};

typedef struct etm_reg_s
{
	uint32_t value;
	const struct etm_reg_info *reg_info;
	arm_jtag_t *jtag_info;
} etm_reg_t;

typedef enum
{
	/* Port width */
	ETM_PORT_4BIT		= 0x00,
	ETM_PORT_8BIT		= 0x10,
	ETM_PORT_16BIT		= 0x20,
	ETM_PORT_WIDTH_MASK	= 0x70,
	/* Port modes */
	ETM_PORT_NORMAL    = 0x00000,
	ETM_PORT_MUXED     = 0x10000,
	ETM_PORT_DEMUXED   = 0x20000,
	ETM_PORT_MODE_MASK = 0x30000,
	/* Clocking modes */
	ETM_PORT_FULL_CLOCK = 0x0000,
	ETM_PORT_HALF_CLOCK = 0x1000,
	ETM_PORT_CLOCK_MASK = 0x1000,
} etm_portmode_t;

typedef enum
{
	/* Data trace */
	ETMV1_TRACE_NONE	 = 0x00,
	ETMV1_TRACE_DATA     = 0x01,
	ETMV1_TRACE_ADDR     = 0x02,
	ETMV1_TRACE_MASK     = 0x03,
	/* ContextID */
	ETMV1_CONTEXTID_NONE = 0x00,
	ETMV1_CONTEXTID_8    = 0x10,
	ETMV1_CONTEXTID_16   = 0x20,
	ETMV1_CONTEXTID_32   = 0x30,
	ETMV1_CONTEXTID_MASK = 0x30,
	/* Misc */
	ETMV1_CYCLE_ACCURATE = 0x100,
	ETMV1_BRANCH_OUTPUT = 0x200
} etmv1_tracemode_t;

/* forward-declare ETM context */
struct etm_context_s;

typedef struct etm_capture_driver_s
{
	char *name;
	int (*register_commands)(struct command_context_s *cmd_ctx);
	int (*init)(struct etm_context_s *etm_ctx);
	trace_status_t (*status)(struct etm_context_s *etm_ctx);
	int (*read_trace)(struct etm_context_s *etm_ctx);
	int (*start_capture)(struct etm_context_s *etm_ctx);
	int (*stop_capture)(struct etm_context_s *etm_ctx);
} etm_capture_driver_t;

enum
{
	ETMV1_TRACESYNC_CYCLE = 0x1,
	ETMV1_TRIGGER_CYCLE = 0x2,
};

typedef struct etmv1_trace_data_s
{
	uint8_t pipestat;	/* bits 0-2 pipeline status */
	uint16_t packet;		/* packet data (4, 8 or 16 bit) */
	int flags;		/* ETMV1_TRACESYNC_CYCLE, ETMV1_TRIGGER_CYCLE */
} etmv1_trace_data_t;

/* describe a trace context
 * if support for ETMv2 or ETMv3 is to be implemented,
 * this will have to be split into version independent elements
 * and a version specific part
 */
typedef struct etm_context_s
{
	target_t *target;		/* target this ETM is connected to */
	reg_cache_t *reg_cache;		/* ETM register cache */
	etm_capture_driver_t *capture_driver;	/* driver used to access ETM data */
	void *capture_driver_priv;	/* capture driver private data */
	uint32_t trigger_percent;	/* how much trace buffer to fill after trigger */
	trace_status_t capture_status;	/* current state of capture run */
	etmv1_trace_data_t *trace_data;	/* trace data */
	uint32_t trace_depth;		/* number of cycles to be analyzed, 0 if no data available */
	etm_portmode_t portmode;	/* normal, multiplexed or demultiplexed */
	etmv1_tracemode_t tracemode;	/* type of info trace contains */
	armv4_5_state_t core_state;	/* current core state */
	struct image_s *image;		/* source for target opcodes */
	uint32_t pipe_index;		/* current trace cycle */
	uint32_t data_index;		/* cycle holding next data packet */
	bool data_half;			/* port half on a 16 bit port */
	bool pc_ok;			/* full PC has been acquired */
	bool ptr_ok;			/* whether last_ptr is valid */
	uint8_t bcd_vers;		/* e.g. 0x13 == ETMv1.3 */
	uint32_t config;		/* cache of ETM_CONFIG value */
	uint32_t current_pc;		/* current program counter */
	uint32_t last_branch;		/* last branch address output */
	uint32_t last_branch_reason;	/* type of last branch encountered */
	uint32_t last_ptr;		/* address of the last data access */
	uint32_t last_instruction;	/* index of last executed (to calc timings) */
} etm_context_t;

/* PIPESTAT values */
typedef enum
{
	STAT_IE = 0x0,
	STAT_ID = 0x1,
	STAT_IN = 0x2,
	STAT_WT = 0x3,
	STAT_BE = 0x4,
	STAT_BD = 0x5,
	STAT_TR = 0x6,
	STAT_TD = 0x7
} etmv1_pipestat_t;

/* branch reason values */
typedef enum
{
	BR_NORMAL  = 0x0, /* Normal PC change : periodic synchro (ETMv1.1) */
	BR_ENABLE  = 0x1, /* Trace has been enabled */
	BR_RESTART = 0x2, /* Trace restarted after a FIFO overflow */
	BR_NODEBUG = 0x3, /* ARM has exited for debug state */
	BR_PERIOD  = 0x4, /* Peridioc synchronization point (ETM >= v1.2)*/
	BR_RSVD5   = 0x5, /* reserved */
	BR_RSVD6   = 0x6, /* reserved */
	BR_RSVD7   = 0x7, /* reserved */
} etmv1_branch_reason_t;

reg_cache_t* etm_build_reg_cache(target_t *target,
		arm_jtag_t *jtag_info, etm_context_t *etm_ctx);

int etm_setup(target_t *target);

int etm_register_commands(struct command_context_s *cmd_ctx);

#define ERROR_ETM_INVALID_DRIVER	(-1300)
#define ERROR_ETM_PORTMODE_NOT_SUPPORTED	(-1301)
#define ERROR_ETM_CAPTURE_INIT_FAILED	(-1302)
#define ERROR_ETM_ANALYSIS_FAILED	(-1303)

#endif /* ETM_H */

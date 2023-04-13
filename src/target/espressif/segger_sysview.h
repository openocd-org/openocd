/* SPDX-License-Identifier: BSD-1-Clause */
/* SPDX-FileCopyrightText: (c) 1995-2021 SEGGER Microcontroller GmbH. All rights reserved. */
/* SPDX-FileContributor: 2023 Espressif Systems (Shanghai) CO LTD */

/*
* The contend below is extracted from files SEGGER_SYSVIEW.h and SEGGER_SYSVIEW_Int.h in:
* https://www.segger.com/downloads/systemview/systemview_target_src
* SystemView version: 3.42
*/

#ifndef OPENOCD_TARGET_SEGGER_SYSVIEW_H
#define OPENOCD_TARGET_SEGGER_SYSVIEW_H

#define SYSVIEW_EVTID_NOP                 0	/* Dummy packet. */
#define SYSVIEW_EVTID_OVERFLOW            1
#define SYSVIEW_EVTID_ISR_ENTER           2
#define SYSVIEW_EVTID_ISR_EXIT            3
#define SYSVIEW_EVTID_TASK_START_EXEC     4
#define SYSVIEW_EVTID_TASK_STOP_EXEC      5
#define SYSVIEW_EVTID_TASK_START_READY    6
#define SYSVIEW_EVTID_TASK_STOP_READY     7
#define SYSVIEW_EVTID_TASK_CREATE         8
#define SYSVIEW_EVTID_TASK_INFO           9
#define SYSVIEW_EVTID_TRACE_START         10
#define SYSVIEW_EVTID_TRACE_STOP          11
#define SYSVIEW_EVTID_SYSTIME_CYCLES      12
#define SYSVIEW_EVTID_SYSTIME_US          13
#define SYSVIEW_EVTID_SYSDESC             14
#define SYSVIEW_EVTID_USER_START          15
#define SYSVIEW_EVTID_USER_STOP           16
#define SYSVIEW_EVTID_IDLE                17
#define SYSVIEW_EVTID_ISR_TO_SCHEDULER    18
#define SYSVIEW_EVTID_TIMER_ENTER         19
#define SYSVIEW_EVTID_TIMER_EXIT          20
#define SYSVIEW_EVTID_STACK_INFO          21
#define SYSVIEW_EVTID_MODULEDESC          22

#define SYSVIEW_EVTID_INIT                24
#define SYSVIEW_EVTID_NAME_RESOURCE       25
#define SYSVIEW_EVTID_PRINT_FORMATTED     26
#define SYSVIEW_EVTID_NUMMODULES          27
#define SYSVIEW_EVTID_END_CALL            28
#define SYSVIEW_EVTID_TASK_TERMINATE      29

#define SYSVIEW_EVTID_EX                  31
//
// SystemView extended events. Sent with ID 31.
//
#define SYSVIEW_EVTID_EX_MARK             0
#define SYSVIEW_EVTID_EX_NAME_MARKER      1
#define SYSVIEW_EVTID_EX_HEAP_DEFINE      2
#define SYSVIEW_EVTID_EX_HEAP_ALLOC       3
#define SYSVIEW_EVTID_EX_HEAP_ALLOC_EX    4
#define SYSVIEW_EVTID_EX_HEAP_FREE        5

#define SYSVIEW_SYNC_LEN                  10

#define SYSVIEW_EVENT_ID_MAX             (200)

//
// Commands that Host can send to target
//
enum {
	SEGGER_SYSVIEW_COMMAND_ID_START = 1,
	SEGGER_SYSVIEW_COMMAND_ID_STOP,
	SEGGER_SYSVIEW_COMMAND_ID_GET_SYSTIME,
	SEGGER_SYSVIEW_COMMAND_ID_GET_TASKLIST,
	SEGGER_SYSVIEW_COMMAND_ID_GET_SYSDESC,
	SEGGER_SYSVIEW_COMMAND_ID_GET_NUMMODULES,
	SEGGER_SYSVIEW_COMMAND_ID_GET_MODULEDESC,
	SEGGER_SYSVIEW_COMMAND_ID_HEARTBEAT = 127,
	// Extended commands: Commands >= 128 have a second parameter
	SEGGER_SYSVIEW_COMMAND_ID_GET_MODULE = 128
};

/* Minimum compatible SEGGER SystemView tool version */
#define SYSVIEW_MIN_VER_STRING			"SEGGER SystemViewer V2.42"

#endif

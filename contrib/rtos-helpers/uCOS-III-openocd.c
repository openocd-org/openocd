// SPDX-License-Identifier: Apache-2.0

/*
 * The original version of this file did not reported any license nor
 * copyright, but the author clearly stated that:
 * "This file should be linked along with the [uC/OS-III user's] project
 *  to enable RTOS support for uC/OS-III."
 * Such statement implies the willing to have this file's license compatible
 * with the license Apache 2.0 of uC/OS-III.
 */

/*
 * uC/OS-III does not provide a fixed layout for OS_TCB, which makes it
 * impossible to determine the appropriate offsets within the structure
 * unaided. A priori knowledge of offsets based on os_dbg.c is tied to a
 * specific release and thusly, brittle. The constants defined below
 * provide the necessary information OpenOCD needs to provide support in
 * the most robust manner possible.
 *
 * This file should be linked along with the project to enable RTOS
 * support for uC/OS-III.
 */

#include <os.h>

#if OS_CFG_DBG_EN == 0
#error "OS_CFG_DBG_EN is required to enable RTOS support for OpenOCD"
#endif

#define OFFSET_OF(type, member) ((CPU_SIZE_T)&(((type *)0)->member))

#ifdef __GNUC__
#define USED __attribute__((used))
#else
#define USED
#endif

const CPU_SIZE_T USED openocd_OS_TCB_StkPtr_offset = OFFSET_OF(OS_TCB, StkPtr);
const CPU_SIZE_T USED openocd_OS_TCB_NamePtr_offset = OFFSET_OF(OS_TCB, NamePtr);
const CPU_SIZE_T USED openocd_OS_TCB_TaskState_offset = OFFSET_OF(OS_TCB, TaskState);
const CPU_SIZE_T USED openocd_OS_TCB_Prio_offset = OFFSET_OF(OS_TCB, Prio);
const CPU_SIZE_T USED openocd_OS_TCB_DbgPrevPtr_offset = OFFSET_OF(OS_TCB, DbgPrevPtr);
const CPU_SIZE_T USED openocd_OS_TCB_DbgNextPtr_offset = OFFSET_OF(OS_TCB, DbgNextPtr);

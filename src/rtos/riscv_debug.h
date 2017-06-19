#ifndef RTOS__RISCV_H
#define RTOS__RISCV_H

#include "rtos.h"

struct riscv_rtos {
	/* The index into the thread list used to handle */
	int qs_thread_info_offset;
};

int riscv_update_threads(struct rtos *rtos);

#endif

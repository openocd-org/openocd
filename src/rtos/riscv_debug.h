#ifndef RTOS__RISCV_H
#define RTOS__RISCV_H

struct riscv_rtos {
	/* The index into the thread list used to handle */
	int qs_thread_info_offset;
};

#endif

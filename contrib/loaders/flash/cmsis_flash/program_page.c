#include <stdint.h>


struct circular_buffer {
	volatile uint32_t wp;
	volatile uint32_t rp;
	uint8_t data[];
};

struct ram_params {
	uint32_t work_area;
	uint32_t fifo_end;
	uint32_t flash_addr;
	uint32_t num_pages;
	uint32_t page_size;
	uint32_t program_page_p;
};

__attribute__((noreturn, naked, flatten))
void write(const struct ram_params *const params)
{
	struct circular_buffer *work_area = (struct circular_buffer *)params->work_area;
	const uint32_t page_size = params->page_size;
	const uint32_t fifo_end = params->fifo_end;
	uint32_t num_pages = params->num_pages;
	uint32_t flash_addr = params->flash_addr;
	int (*program_page)(uint32_t addr, uint32_t size, uint32_t buffer) = (void *)params->program_page_p;

	while (num_pages) {
		/* Wait for some data in the FIFO */
		while (work_area->rp == work_area->wp);

		int result = program_page(flash_addr, page_size, work_area->rp);
		if (result) {
			work_area->rp = 0;
			__asm volatile ("mov r0, %[value]\n\t"
			"bkpt 0"  :  : [value] "r" (result) : );
		}

		flash_addr += page_size;

		uint32_t read_ptr = work_area->rp;
		read_ptr += page_size;
		if (read_ptr >= fifo_end)
			read_ptr = ((uint32_t)&work_area->data);

		work_area->rp = read_ptr;
		num_pages--;
	}

	for (;;)
		__asm("bkpt 0");
}

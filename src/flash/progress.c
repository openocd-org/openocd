#include <string.h>

#include "progress.h"
#include "helper/log.h"

#define MAX_BAR_WIDTH   32
static char sbuf[MAX_BAR_WIDTH + 1];
static size_t prev_bar_width;
static const char *op_type;
static size_t total_size;

static const char *progress_str[] = {
	"Erasing    ",
	"Programming",
	"Verifying  ",
	"Blank Check",
};


void progress_init(size_t total, enum progress_type type)
{
	if(!op_type)
		op_type = progress_str[type];

	total_size = total;
	prev_bar_width = 0;
}

void progress_sofar(size_t sofar)
{
	size_t bar_width;
	size_t pc;

	assert(sofar <= total_size);

	pc = total_size ? 100 * sofar / total_size : 100;
	bar_width = pc * MAX_BAR_WIDTH / 100;
	if (bar_width > MAX_BAR_WIDTH)
		bar_width = MAX_BAR_WIDTH;

	if (prev_bar_width != bar_width) {
		memset(sbuf, ' ', sizeof(sbuf) - 1);
		sbuf[sizeof(sbuf) - 1] = 0;

		for (size_t i = 0; i < bar_width; i++)
			sbuf[i] = '#';

		//LOG_USER_N("\r[%3zu%%] [%s] [ %s ]", pc, sbuf, op_type);
    LOG_USER_N("\r[ %s ] [%s] [%3zu%%]", op_type, sbuf, pc);
		prev_bar_width = bar_width;
	}
}

void progress_left(size_t left)
{
	progress_sofar(total_size - left);
}

void progress_done(int result)
{
	if (result == ERROR_OK) {
		progress_sofar(total_size);
		LOG_USER_N("\n");
	}

	total_size = 0;
	op_type = NULL;
}

#include <stddef.h>
#include <stdbool.h>

enum progress_type {
	ERASING,
	PROGRAMMING,
	VERIFYING,
	BLANKCHECK,
};

void progress_init(size_t total, enum progress_type type);
void progress_sofar(size_t sofar);
void progress_left(size_t left);
void progress_done(int result);

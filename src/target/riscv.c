#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"

static int riscv_poll(struct target *target)
{
    return 0;
}

struct target_type riscv_target = {
    .name = "riscv",

    .poll = riscv_poll,
};

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "riscv.h"
#include "program.h"
#include "helper/log.h"

#include "asm.h"
#include "encoding.h"

riscv_addr_t riscv_program_gal(struct riscv_program *p, riscv_addr_t addr);
int riscv_program_lah(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr);
int riscv_program_lal(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr);

/* Program interface. */
int riscv_program_init(struct riscv_program *p, struct target *target)
{
	LOG_DEBUG("riscv_program_init: p=0x%p", p);

	memset(p, 0, sizeof(*p));
	p->target = target;
	p->instruction_count = 0;
	p->data_count = 0;
	p->writes_memory = 0;
	p->target_xlen = riscv_xlen(target);
	for (size_t i = 0; i < RISCV_REGISTER_COUNT; ++i) {
		p->writes_xreg[i] = 0;
		p->in_use[i] = 0;
	}

	for(size_t i = 0; i < RISCV_MAX_DEBUG_BUFFER_SIZE; ++i)
		p->debug_buffer[i] = -1;

	if (riscv_debug_buffer_enter(target, p) != ERROR_OK) {
		LOG_ERROR("unable to write progam buffer enter code");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int riscv_program_exec(struct riscv_program *p, struct target *t)
{
	if (riscv_debug_buffer_leave(t, p) != ERROR_OK) {
		LOG_ERROR("unable to write program buffer exit code");
		return ERROR_FAIL;
	}

	riscv_reg_t saved_registers[GDB_REGNO_XPR31 + 1];
	for (size_t i = GDB_REGNO_XPR0 + 1; i <= GDB_REGNO_XPR31; ++i) {
		if (p->writes_xreg[i]) {
			LOG_DEBUG("Saving register %d as used by program", (int)i);
			saved_registers[i] = riscv_get_register(t, i);
		}
	}

	if (p->writes_memory && (riscv_program_fence(p) != ERROR_OK)) {
		LOG_ERROR("Unable to write fence");
		for(size_t i = 0; i < riscv_debug_buffer_size(p->target); ++i)
			LOG_ERROR("ram[%02x]: DASM(0x%08lx) [0x%08lx]", (int)i, (long)p->debug_buffer[i], (long)p->debug_buffer[i]);
		abort();
		return ERROR_FAIL;
	}

	if (riscv_program_ebreak(p) != ERROR_OK) {
		LOG_ERROR("Unable to write ebreak");
		for(size_t i = 0; i < riscv_debug_buffer_size(p->target); ++i)
			LOG_ERROR("ram[%02x]: DASM(0x%08lx) [0x%08lx]", (int)i, (long)p->debug_buffer[i], (long)p->debug_buffer[i]);
		abort();
		return ERROR_FAIL;
	}

	for (size_t i = 0; i < riscv_debug_buffer_size(p->target); ++i) {
		LOG_DEBUG("Executing program 0x%p: debug_buffer[%02x] = DASM(0x%08lx)", p, (int)i, (long)p->debug_buffer[i]);
		if (i <= p->instruction_count || i >= riscv_debug_buffer_size(p->target) - p->data_count)
			riscv_write_debug_buffer(t, i, p->debug_buffer[i]);
	}

	if (riscv_execute_debug_buffer(t) != ERROR_OK) {
		LOG_DEBUG("Unable to execute program 0x%p", p);
		return ERROR_FAIL;
	}

	for (size_t i = 0; i < riscv_debug_buffer_size(p->target); ++i)
		if (i >= riscv_debug_buffer_size(p->target) - p->data_count)
			p->debug_buffer[i] = riscv_read_debug_buffer(t, i);

	for (size_t i = GDB_REGNO_XPR0; i <= GDB_REGNO_XPR31; ++i)
		if (p->writes_xreg[i])
			riscv_set_register(t, i, saved_registers[i]);

	return ERROR_OK;
}

riscv_addr_t riscv_program_alloc_data(struct riscv_program *p, size_t bytes)
{
	LOG_DEBUG("allocating %d bytes of data", (int)bytes);

	riscv_addr_t addr = 
		riscv_debug_buffer_addr(p->target) 
		+ riscv_debug_buffer_size(p->target) * sizeof(p->debug_buffer[0])
		- p->data_count * sizeof(p->debug_buffer[0])
		- bytes;
	while (addr % bytes != 0) addr--;

	riscv_addr_t ptop =
		riscv_debug_buffer_addr(p->target)
		+ p->instruction_count * sizeof(p->debug_buffer[0]);

	if (addr <= ptop) {
		LOG_DEBUG("unable to allocate %d bytes", (int)bytes);
		return RISCV_PROGRAM_ALLOC_FAIL;
	}

	LOG_DEBUG("allocated %d bytes at 0x%08lx", (int)bytes, (long)addr);
	p->data_count = 
		+ riscv_debug_buffer_size(p->target)
		- (addr - riscv_debug_buffer_addr(p->target)) / sizeof(p->debug_buffer[0]);
	return addr;
}

riscv_addr_t riscv_program_alloc_x(struct riscv_program *p)
{
	return riscv_program_alloc_data(p, p->target_xlen / 8);
}

riscv_addr_t riscv_program_alloc_d(struct riscv_program *p)
{
	return riscv_program_alloc_data(p, 8);
}

riscv_addr_t riscv_program_alloc_w(struct riscv_program *p)
{
	return riscv_program_alloc_data(p, 4);
}

riscv_addr_t riscv_program_alloc_h(struct riscv_program *p)
{
	return riscv_program_alloc_data(p, 2);
}

riscv_addr_t riscv_program_alloc_b(struct riscv_program *p)
{
	return riscv_program_alloc_data(p, 1);
}

riscv_insn_t riscv_program_read_ram(struct riscv_program *p, riscv_addr_t addr)
{
	if (addr < riscv_debug_buffer_addr(p->target))
		return -1;
	if ((size_t)addr > riscv_debug_buffer_addr(p->target) + (riscv_debug_buffer_size(p->target) * sizeof(p->debug_buffer[0])))
		return -1;

	int off = (addr - riscv_debug_buffer_addr(p->target)) / sizeof(p->debug_buffer[0]);
	return p->debug_buffer[off];
}

void riscv_program_write_ram(struct riscv_program *p, riscv_addr_t addr, uint64_t d)
{
	if (addr < riscv_debug_buffer_addr(p->target))
		return;
	if ((size_t)addr > riscv_debug_buffer_addr(p->target) + (riscv_debug_buffer_size(p->target) * sizeof(p->debug_buffer[0])))
		return;

	int off = (addr - riscv_debug_buffer_addr(p->target)) / sizeof(p->debug_buffer[0]);
	p->debug_buffer[off] = d;
}

int riscv_program_swr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	p->writes_memory = 1;
	return riscv_program_insert(p, sw(d, b, offset));
}

int riscv_program_shr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	p->writes_memory = 1;
	return riscv_program_insert(p, sh(d, b, offset));
}

int riscv_program_sbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	p->writes_memory = 1;
	return riscv_program_insert(p, sb(d, b, offset));
}

int riscv_program_lwr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	p->writes_memory = 1;
	return riscv_program_insert(p, lw(d, b, offset));
}

int riscv_program_lhr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	p->writes_memory = 1;
	return riscv_program_insert(p, lh(d, b, offset));
}

int riscv_program_lbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	p->writes_memory = 1;
	return riscv_program_insert(p, lb(d, b, offset));
}

int riscv_program_lx(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	switch (p->target_xlen) {
	case 64:  return riscv_program_ld(p, d, addr);
	case 32:  return riscv_program_lw(p, d, addr);
	}

	LOG_ERROR("unknown xlen %d", p->target_xlen);
	abort();
	return -1;
}

int riscv_program_ld(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	enum gdb_regno t = riscv_program_gah(p, addr) == 0 ? GDB_REGNO_X0 : d;
	if (riscv_program_lah(p, d, addr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(p, ld(d, t, riscv_program_gal(p, addr))) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

int riscv_program_lw(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	enum gdb_regno t = riscv_program_gah(p, addr) == 0 ? GDB_REGNO_X0 : d;
	if (riscv_program_lah(p, d, addr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(p, lw(d, t, riscv_program_gal(p, addr))) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

int riscv_program_lh(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	enum gdb_regno t = riscv_program_gah(p, addr) == 0 ? GDB_REGNO_X0 : d;
	if (riscv_program_lah(p, d, addr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(p, lh(d, t, riscv_program_gal(p, addr))) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

int riscv_program_lb(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	enum gdb_regno t = riscv_program_gah(p, addr) == 0 ? GDB_REGNO_X0 : d;
	if (riscv_program_lah(p, t, addr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(p, lb(d, t, riscv_program_gal(p, addr))) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

int riscv_program_sx(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	switch (p->target_xlen) {
	case 64:  return riscv_program_sd(p, d, addr);
	case 32:  return riscv_program_sw(p, d, addr);
	}

	LOG_ERROR("unknown xlen %d", p->target_xlen);
	abort();
	return -1;
}

int riscv_program_sd(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	enum gdb_regno t = riscv_program_gah(p, addr) == 0
		? GDB_REGNO_X0
		: riscv_program_gettemp(p);
	if (riscv_program_lah(p, t, addr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(p, sd(d, t, riscv_program_gal(p, addr))) != ERROR_OK)
		return ERROR_FAIL;
	riscv_program_puttemp(p, t);
	p->writes_memory = true;
	return ERROR_OK;
}

int riscv_program_sw(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	enum gdb_regno t = riscv_program_gah(p, addr) == 0
		? GDB_REGNO_X0
		: riscv_program_gettemp(p);
	if (riscv_program_lah(p, t, addr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(p, sw(d, t, riscv_program_gal(p, addr))) != ERROR_OK)
		return ERROR_FAIL;
	riscv_program_puttemp(p, t);
	p->writes_memory = true;
	return ERROR_OK;
}

int riscv_program_sh(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	enum gdb_regno t = riscv_program_gah(p, addr) == 0
		? GDB_REGNO_X0
		: riscv_program_gettemp(p);
	if (riscv_program_lah(p, t, addr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(p, sh(d, t, riscv_program_gal(p, addr))) != ERROR_OK)
		return ERROR_FAIL;
	riscv_program_puttemp(p, t);
	p->writes_memory = true;
	return ERROR_OK;
}

int riscv_program_sb(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	enum gdb_regno t = riscv_program_gah(p, addr) == 0
		? GDB_REGNO_X0
		: riscv_program_gettemp(p);
	if (riscv_program_lah(p, t, addr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(p, sb(d, t, riscv_program_gal(p, addr))) != ERROR_OK)
		return ERROR_FAIL;
	riscv_program_puttemp(p, t);
	p->writes_memory = true;
	return ERROR_OK;
}

int riscv_program_csrr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0);
	return riscv_program_insert(p, csrrs(d, GDB_REGNO_X0, csr - GDB_REGNO_CSR0));
}

int riscv_program_csrw(struct riscv_program *p, enum gdb_regno s, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0);
	return riscv_program_insert(p, csrrw(GDB_REGNO_X0, s, csr - GDB_REGNO_CSR0));
}

int riscv_program_csrrw(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0);
	return riscv_program_insert(p, csrrw(d, s, csr - GDB_REGNO_CSR0));
}

int riscv_program_fence_i(struct riscv_program *p)
{
	return riscv_program_insert(p, fence_i());
}

int riscv_program_fence(struct riscv_program *p)
{
	return riscv_program_insert(p, fence());
}

int riscv_program_ebreak(struct riscv_program *p)
{
	return riscv_program_insert(p, ebreak());
}

int riscv_program_lui(struct riscv_program *p, enum gdb_regno d, int32_t u)
{
	return riscv_program_insert(p, lui(d, u));
}

int riscv_program_addi(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, int16_t u)
{
	return riscv_program_insert(p, addi(d, s, u));
}

int riscv_program_fsd(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	assert(d >= GDB_REGNO_FPR0);
	assert(d <= GDB_REGNO_FPR31);
	enum gdb_regno t = riscv_program_gah(p, addr) == 0
		? GDB_REGNO_X0
		: riscv_program_gettemp(p);
	if (riscv_program_lah(p, t, addr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(p, fsd(d - GDB_REGNO_FPR0, t, riscv_program_gal(p, addr))) != ERROR_OK)
		return ERROR_FAIL;
	riscv_program_puttemp(p, t);
	p->writes_memory = true;
	return ERROR_OK;
}

int riscv_program_fld(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	assert(d >= GDB_REGNO_FPR0);
	assert(d <= GDB_REGNO_FPR31);
	enum gdb_regno t = riscv_program_gah(p, addr) == 0 ? GDB_REGNO_X0 : d;
	if (riscv_program_lah(p, t, addr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_insert(p, fld(d - GDB_REGNO_FPR0, t, riscv_program_gal(p, addr))) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

int riscv_program_li(struct riscv_program *p, enum gdb_regno d, riscv_reg_t c)
{
	if (riscv_program_lui(p, d, c >> 12) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_addi(p, d, d, c & 0xFFF) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

int riscv_program_dont_restore_register(struct riscv_program *p, enum gdb_regno r)
{
	assert(r < RISCV_REGISTER_COUNT);
	p->writes_xreg[r] = 0;
	return ERROR_OK;
}

int riscv_program_do_restore_register(struct riscv_program *p, enum gdb_regno r)
{
	assert(r < RISCV_REGISTER_COUNT);
	p->writes_xreg[r] = 1;
	return ERROR_OK;
}

void riscv_program_reserve_register(struct riscv_program *p, enum gdb_regno r)
{
	assert(r < RISCV_REGISTER_COUNT);
	assert(p->in_use[r] == 0);
	p->in_use[r] = 1;
}

enum gdb_regno riscv_program_gettemp(struct riscv_program *p)
{
	for (size_t i = GDB_REGNO_S0; i <= GDB_REGNO_XPR31; ++i) {
		if (p->in_use[i]) continue;

		riscv_program_do_restore_register(p, i);
		p->in_use[i] = 1;
		return i;
	}

	LOG_ERROR("You've run out of temporary registers.  This is impossible.");
	abort();
}

void riscv_program_puttemp(struct riscv_program *p, enum gdb_regno r)
{
	assert(r < RISCV_REGISTER_COUNT);
	p->in_use[r] = 0;
}

/* Helper functions. */
riscv_addr_t riscv_program_gah(struct riscv_program *p, riscv_addr_t addr)
{
	return addr >> 12;
}

riscv_addr_t riscv_program_gal(struct riscv_program *p, riscv_addr_t addr)
{
	return ((addr > 0) ? 1 : 0) * (abs(addr) & 0x7FF);
}

int riscv_program_lah(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	riscv_addr_t ah = riscv_program_gah(p, addr);
	if (ah == 0)
		return ERROR_OK;
	return riscv_program_lui(p, d, ah);
}

int riscv_program_lal(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr)
{
	riscv_addr_t al = riscv_program_gal(p, addr);
	if (al == 0)
		return ERROR_OK;
	return riscv_program_addi(p, d, d, al);
}

int riscv_program_insert(struct riscv_program *p, riscv_insn_t i)
{
	LOG_DEBUG("instruction_count: %d (p=0x%p)", (int)p->instruction_count, p);

	if (p->instruction_count + p->data_count + 1 > riscv_debug_buffer_size(p->target)) {
		LOG_DEBUG("Unable to insert instruction:");
		LOG_DEBUG("  instruction_count=%d", (int)p->instruction_count);
		LOG_DEBUG("  data_count       =%d", (int)p->data_count);
		LOG_DEBUG("  buffer size      =%d", (int)riscv_debug_buffer_size(p->target));
		return ERROR_FAIL;
	}

	LOG_DEBUG("PROGBUF[%d] = DASM(0x%08x) [0x%08x]", (int)p->instruction_count, i, i);
	p->debug_buffer[p->instruction_count] = i;
	p->instruction_count++;
	return ERROR_OK;
}

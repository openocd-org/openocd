/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_OPCODES_H
#define OPENOCD_TARGET_RISCV_OPCODES_H

#include "encoding.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#define ZERO	0
#define T0      5
#define S0      8
#define S1      9

#define MAX_GPR_NUM 31
#define MAX_FPR_NUM 31
#define MAX_VREG_NUM 31
#define MAX_CSR_NUM 4095

#define MIN_INT12 (-0x800)
#define MAX_INT12 0x7ff

#define MIN_INT13 (-0x1000)
#define MAX_INT13 0xfff

#define MIN_INT21 (-0x100000)
#define MAX_INT21 0xfffff

#define MAX_UINT5 0x1f
#define MAX_UINT11 0x7ff
#define MAX_UINT12 0xfff

static uint32_t bits(uint32_t value, unsigned int hi, unsigned int lo)
{
	return (value >> lo) & (((uint32_t)1 << (hi + 1 - lo)) - 1);
}

static uint32_t bit(uint32_t value, unsigned int b)
{
	return (value >> b) & 1;
}

static uint32_t inst_rd(uint32_t r) __attribute__ ((unused));
static uint32_t inst_rd(uint32_t r)
{
	return bits(r, 4, 0) << 7;
}

static uint32_t inst_rs1(uint32_t r) __attribute__ ((unused));
static uint32_t inst_rs1(uint32_t r)
{
	return bits(r, 4, 0) << 15;
}

static uint32_t inst_rs2(uint32_t r) __attribute__ ((unused));
static uint32_t inst_rs2(uint32_t r)
{
	return bits(r, 4, 0) << 20;
}

static uint32_t imm_i(uint32_t imm) __attribute__ ((unused));
static uint32_t imm_i(uint32_t imm)
{
	return bits(imm, 11, 0) << 20;
}

static uint32_t imm_s(uint32_t imm) __attribute__ ((unused));
static uint32_t imm_s(uint32_t imm)
{
	return (bits(imm, 4, 0) << 7) | (bits(imm, 11, 5) << 25);
}

static uint32_t imm_b(uint32_t imm) __attribute__ ((unused));
static uint32_t imm_b(uint32_t imm)
{
	return (bit(imm, 11) << 7) | (bits(imm, 4, 1) << 8) | (bits(imm, 10, 5) << 25) | (bit(imm, 12) << 31);
}

static uint32_t imm_u(uint32_t imm) __attribute__ ((unused));
static uint32_t imm_u(uint32_t imm)
{
	return bits(imm, 31, 12) << 12;
}

static uint32_t imm_j(uint32_t imm) __attribute__ ((unused));
static uint32_t imm_j(uint32_t imm)
{
	return (bits(imm, 19, 12) << 12) | (bit(imm, 11) << 20) | (bits(imm, 10, 1) << 21) | (bit(imm, 20) << 31);
}

static uint32_t jal(unsigned int rd, int32_t imm) __attribute__ ((unused));
static uint32_t jal(unsigned int rd, int32_t imm)
{
	assert(rd <= MAX_GPR_NUM);
	assert((imm >= MIN_INT21) && (imm <= MAX_INT21));
	assert((imm & 1) == 0);

	return imm_j((uint32_t)imm) | inst_rd(rd) | MATCH_JAL;
}

static uint32_t csrsi(unsigned int csr, uint8_t imm) __attribute__ ((unused));
static uint32_t csrsi(unsigned int csr, uint8_t imm)
{
	assert(csr <= MAX_CSR_NUM);
	assert(imm <= MAX_UINT5);

	return imm_i(csr) | inst_rs1(imm) | MATCH_CSRRSI;
}

static uint32_t sw(unsigned int src, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t sw(unsigned int src, unsigned int base, int16_t offset)
{
	assert(src <= MAX_GPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_s((uint16_t)offset) | inst_rs2(src) | inst_rs1(base) | MATCH_SW;
}

static uint32_t sd(unsigned int src, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t sd(unsigned int src, unsigned int base, int16_t offset)
{
	assert(src <= MAX_GPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_s((uint16_t)offset) | inst_rs2(src) | inst_rs1(base) | MATCH_SD;
}

static uint32_t sh(unsigned int src, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t sh(unsigned int src, unsigned int base, int16_t offset)
{
	assert(src <= MAX_GPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_s((uint16_t)offset) | inst_rs2(src) | inst_rs1(base) | MATCH_SH;
}

static uint32_t sb(unsigned int src, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t sb(unsigned int src, unsigned int base, int16_t offset)
{
	assert(src <= MAX_GPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_s((uint16_t)offset) | inst_rs2(src) | inst_rs1(base) | MATCH_SB;
}

static uint32_t ld(unsigned int rd, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t ld(unsigned int rd, unsigned int base, int16_t offset)
{
	assert(rd <= MAX_GPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_i((uint16_t)offset) | inst_rs1(base) | inst_rd(rd) | MATCH_LD;
}

static uint32_t lw(unsigned int rd, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t lw(unsigned int rd, unsigned int base, int16_t offset)
{
	assert(rd <= MAX_GPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_i((uint16_t)offset) | inst_rs1(base) | inst_rd(rd) | MATCH_LW;
}

static uint32_t lh(unsigned int rd, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t lh(unsigned int rd, unsigned int base, int16_t offset)
{
	assert(rd <= MAX_GPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_i((uint16_t)offset) | inst_rs1(base) | inst_rd(rd) | MATCH_LH;
}

static uint32_t lb(unsigned int rd, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t lb(unsigned int rd, unsigned int base, int16_t offset)
{
	assert(rd <= MAX_GPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_i((uint16_t)offset) | inst_rs1(base) | inst_rd(rd) | MATCH_LB;
}

static uint32_t csrw(unsigned int source, unsigned int csr) __attribute__ ((unused));
static uint32_t csrw(unsigned int source, unsigned int csr)
{
	assert(source <= MAX_GPR_NUM);
	assert(csr <= MAX_CSR_NUM);

	return imm_i(csr) | inst_rs1(source) | MATCH_CSRRW;
}

static uint32_t addi(unsigned int dest, unsigned int src, int16_t imm) __attribute__ ((unused));
static uint32_t addi(unsigned int dest, unsigned int src, int16_t imm)
{
	assert(dest <= MAX_GPR_NUM);
	assert(src <= MAX_GPR_NUM);
	assert((imm >= MIN_INT12) && (imm <= MAX_INT12));

	return imm_i((uint16_t)imm) | inst_rs1(src) | inst_rd(dest) | MATCH_ADDI;
}

static uint32_t csrr(unsigned int rd, unsigned int csr) __attribute__ ((unused));
static uint32_t csrr(unsigned int rd, unsigned int csr)
{
	assert(rd <= MAX_GPR_NUM);
	assert(csr <= MAX_CSR_NUM);

	return imm_i(csr) | inst_rd(rd) | MATCH_CSRRS;
}

static uint32_t csrrs(unsigned int rd, unsigned int rs, unsigned int csr) __attribute__ ((unused));
static uint32_t csrrs(unsigned int rd, unsigned int rs, unsigned int csr)
{
	assert(rd <= MAX_GPR_NUM);
	assert(rs <= MAX_GPR_NUM);
	assert(csr <= MAX_CSR_NUM);

	return imm_i(csr) | inst_rs1(rs) | inst_rd(rd) | MATCH_CSRRS;
}

static uint32_t csrrw(unsigned int rd, unsigned int rs, unsigned int csr) __attribute__ ((unused));
static uint32_t csrrw(unsigned int rd, unsigned int rs, unsigned int csr)
{
	assert(rd <= MAX_GPR_NUM);
	assert(rs <= MAX_GPR_NUM);
	assert(csr <= MAX_CSR_NUM);

	return imm_i(csr) | inst_rs1(rs) | inst_rd(rd) | MATCH_CSRRW;
}

static uint32_t csrrci(unsigned int rd, uint8_t zimm, unsigned int csr) __attribute__ ((unused));
static uint32_t csrrci(unsigned int rd, uint8_t zimm, unsigned int csr)
{
	assert(rd <= MAX_GPR_NUM);
	assert(zimm <= MAX_UINT5);
	assert(csr <= MAX_CSR_NUM);

	return imm_i(csr) | inst_rs1(zimm) | inst_rd(rd) | MATCH_CSRRCI;
}

static uint32_t csrrsi(unsigned int rd, uint8_t zimm, unsigned int csr) __attribute__ ((unused));
static uint32_t csrrsi(unsigned int rd, uint8_t zimm, unsigned int csr)
{
	assert(rd <= MAX_GPR_NUM);
	assert(zimm <= MAX_UINT5);
	assert(csr <= MAX_CSR_NUM);

	return imm_i(csr) | inst_rs1(zimm) | inst_rd(rd) | MATCH_CSRRSI;
}

static uint32_t fsw(unsigned int src, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t fsw(unsigned int src, unsigned int base, int16_t offset)
{
	assert(src <= MAX_FPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_s((uint16_t)offset) | inst_rs2(src) | inst_rs1(base) | MATCH_FSW;
}

static uint32_t fsd(unsigned int src, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t fsd(unsigned int src, unsigned int base, int16_t offset)
{
	assert(src <= MAX_FPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_s((uint16_t)offset) | inst_rs2(src) | inst_rs1(base) | MATCH_FSD;
}

static uint32_t flw(unsigned int dest, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t flw(unsigned int dest, unsigned int base, int16_t offset)
{
	assert(dest <= MAX_FPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_i((uint16_t)offset) | inst_rs1(base) | inst_rd(dest) | MATCH_FLW;
}

static uint32_t fld(unsigned int dest, unsigned int base, int16_t offset) __attribute__ ((unused));
static uint32_t fld(unsigned int dest, unsigned int base, int16_t offset)
{
	assert(dest <= MAX_FPR_NUM);
	assert(base <= MAX_GPR_NUM);
	assert((offset >= MIN_INT12) && (offset <= MAX_INT12));

	return imm_i((uint16_t)offset) | inst_rs1(base) | inst_rd(dest) | MATCH_FLD;
}

static uint32_t fmv_x_w(unsigned int dest, unsigned int src) __attribute__ ((unused));
static uint32_t fmv_x_w(unsigned int dest, unsigned int src)
{
	assert(dest <= MAX_GPR_NUM);
	assert(src <= MAX_FPR_NUM);

	return inst_rs1(src) | inst_rd(dest) | MATCH_FMV_X_W;
}

static uint32_t fmv_x_d(unsigned int dest, unsigned int src) __attribute__ ((unused));
static uint32_t fmv_x_d(unsigned int dest, unsigned int src)
{
	assert(dest <= MAX_GPR_NUM);
	assert(src <= MAX_FPR_NUM);

	return inst_rs1(src) | inst_rd(dest) | MATCH_FMV_X_D;
}

static uint32_t fmv_w_x(unsigned int dest, unsigned int src) __attribute__ ((unused));
static uint32_t fmv_w_x(unsigned int dest, unsigned int src)
{
	assert(dest <= MAX_FPR_NUM);
	assert(src <= MAX_GPR_NUM);

	return inst_rs1(src) | inst_rd(dest) | MATCH_FMV_W_X;
}

static uint32_t fmv_d_x(unsigned int dest, unsigned int src) __attribute__ ((unused));
static uint32_t fmv_d_x(unsigned int dest, unsigned int src)
{
	assert(dest <= MAX_FPR_NUM);
	assert(src <= MAX_GPR_NUM);

	return inst_rs1(src) | inst_rd(dest) | MATCH_FMV_D_X;
}

static uint32_t ebreak(void) __attribute__ ((unused));
static uint32_t ebreak(void)
{
	return MATCH_EBREAK;
}
static uint32_t ebreak_c(void) __attribute__ ((unused));
static uint32_t ebreak_c(void)
{
	return MATCH_C_EBREAK;
}

static uint32_t wfi(void) __attribute__ ((unused));
static uint32_t wfi(void) { return MATCH_WFI; }

static uint32_t fence_i(void) __attribute__ ((unused));
static uint32_t fence_i(void)
{
	return MATCH_FENCE_I;
}

static uint32_t lui(unsigned int dest, uint32_t imm) __attribute__ ((unused));
static uint32_t lui(unsigned int dest, uint32_t imm)
{
	assert(dest <= MAX_GPR_NUM);
	assert(bits(imm, 11, 0) == 0);

	return imm_u(imm) | inst_rd(dest) | MATCH_LUI;
}

static uint32_t xori(unsigned int dest, unsigned int src, int16_t imm) __attribute__ ((unused));
static uint32_t xori(unsigned int dest, unsigned int src, int16_t imm)
{
	assert(dest <= MAX_GPR_NUM);
	assert(src <= MAX_GPR_NUM);
	assert((imm >= MIN_INT12) && (imm <= MAX_INT12));

	return imm_i((uint16_t)imm) | inst_rs1(src) | inst_rd(dest) | MATCH_XORI;
}

static uint32_t srli(unsigned int dest, unsigned int src, uint8_t shamt) __attribute__ ((unused));
static uint32_t srli(unsigned int dest, unsigned int src, uint8_t shamt)
{
	assert(dest <= MAX_GPR_NUM);
	assert(src <= MAX_GPR_NUM);
	assert(shamt <= MAX_UINT5);

	return inst_rs2(shamt) | inst_rs1(src) | inst_rd(dest) | MATCH_SRLI;
}

static uint32_t fence_rw_rw(void) __attribute__((unused));
static uint32_t fence_rw_rw(void)
{
	/* fence rw,rw */
	return MATCH_FENCE | 0x3300000;
}

static uint32_t auipc(unsigned int dest) __attribute__((unused));
static uint32_t auipc(unsigned int dest)
{
	assert(dest <= MAX_GPR_NUM);

	return MATCH_AUIPC | inst_rd(dest);
}

static uint32_t vsetvli(unsigned int dest, unsigned int src, uint16_t vtypei) __attribute__((unused));
static uint32_t vsetvli(unsigned int dest, unsigned int src, uint16_t vtypei)
{
	assert(dest <= MAX_GPR_NUM);
	assert(src <= MAX_GPR_NUM);
	assert(vtypei <= MAX_UINT11);

	return (bits(vtypei, 10, 0) << 20) | inst_rs1(src) | inst_rd(dest) | MATCH_VSETVLI;
}

static uint32_t vsetvl(unsigned int rd, unsigned int rs1, unsigned int rs2) __attribute__((unused));
static uint32_t vsetvl(unsigned int rd, unsigned int rs1, unsigned int rs2)
{
	assert(rd <= MAX_GPR_NUM);
	assert(rs1 <= MAX_GPR_NUM);
	assert(rs2 <= MAX_GPR_NUM);

	return inst_rd(rd) | inst_rs1(rs1) | inst_rs2(rs2) | MATCH_VSETVL;
}

static uint32_t vmv_x_s(unsigned int rd, unsigned int vs2) __attribute__((unused));
static uint32_t vmv_x_s(unsigned int rd, unsigned int vs2)
{
	assert(rd <= MAX_GPR_NUM);
	assert(vs2 <= MAX_VREG_NUM);

	return inst_rs2(vs2) | inst_rd(rd) | MATCH_VMV_X_S;
}

static uint32_t vmv_s_x(unsigned int vd, unsigned int rs2) __attribute__((unused));
static uint32_t vmv_s_x(unsigned int vd, unsigned int rs2)
{
	assert(vd <= MAX_VREG_NUM);
	assert(rs2 <= MAX_GPR_NUM);

	return inst_rs1(rs2) | inst_rd(vd) | MATCH_VMV_S_X;
}

static uint32_t vslide1down_vx(unsigned int vd, unsigned int vs2,
		unsigned int rs1, bool vm) __attribute__((unused));
static uint32_t vslide1down_vx(unsigned int vd, unsigned int vs2,
		unsigned int rs1, bool vm)
{
	assert(vd <= MAX_VREG_NUM);
	assert(vs2 <= MAX_VREG_NUM);
	assert(rs1 <= MAX_GPR_NUM);

	return (vm ? (1u << 25) : 0u) | inst_rs2(vs2) | inst_rs1(rs1) | inst_rd(vd) | MATCH_VSLIDE1DOWN_VX;
}

#endif /* OPENOCD_TARGET_RISCV_OPCODES_H */

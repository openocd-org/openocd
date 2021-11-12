/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "encoding.h"

#define ZERO	0
#define T0      5
#define S0      8
#define S1      9

static uint32_t bits(uint32_t value, unsigned int hi, unsigned int lo)
{
	return (value >> lo) & ((1 << (hi+1-lo)) - 1);
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

static uint32_t jal(unsigned int rd, uint32_t imm) __attribute__ ((unused));
static uint32_t jal(unsigned int rd, uint32_t imm)
{
	return imm_j(imm) | inst_rd(rd) | MATCH_JAL;
}

static uint32_t csrsi(unsigned int csr, uint16_t imm) __attribute__ ((unused));
static uint32_t csrsi(unsigned int csr, uint16_t imm)
{
	return imm_i(csr) | inst_rs1(imm) | MATCH_CSRRSI;
}

static uint32_t sw(unsigned int src, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t sw(unsigned int src, unsigned int base, uint16_t offset)
{
	return imm_s(offset) | inst_rs2(src) | inst_rs1(base) | MATCH_SW;
}

static uint32_t sd(unsigned int src, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t sd(unsigned int src, unsigned int base, uint16_t offset)
{
	return imm_s(offset) | inst_rs2(src) | inst_rs1(base) | MATCH_SD;
}

static uint32_t sh(unsigned int src, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t sh(unsigned int src, unsigned int base, uint16_t offset)
{
	return imm_s(offset) | inst_rs2(src) | inst_rs1(base) | MATCH_SH;
}

static uint32_t sb(unsigned int src, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t sb(unsigned int src, unsigned int base, uint16_t offset)
{
	return imm_s(offset) | inst_rs2(src) | inst_rs1(base) | MATCH_SB;
}

static uint32_t ld(unsigned int rd, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t ld(unsigned int rd, unsigned int base, uint16_t offset)
{
	return imm_i(offset) | inst_rs1(base) | inst_rd(rd) | MATCH_LD;
}

static uint32_t lw(unsigned int rd, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t lw(unsigned int rd, unsigned int base, uint16_t offset)
{
	return imm_i(offset) | inst_rs1(base) | inst_rd(rd) | MATCH_LW;
}

static uint32_t lh(unsigned int rd, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t lh(unsigned int rd, unsigned int base, uint16_t offset)
{
	return imm_i(offset) | inst_rs1(base) | inst_rd(rd) | MATCH_LH;
}

static uint32_t lb(unsigned int rd, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t lb(unsigned int rd, unsigned int base, uint16_t offset)
{
	return imm_i(offset) | inst_rs1(base) | inst_rd(rd) | MATCH_LB;
}

static uint32_t csrw(unsigned int source, unsigned int csr) __attribute__ ((unused));
static uint32_t csrw(unsigned int source, unsigned int csr)
{
	return imm_i(csr) | inst_rs1(source) | MATCH_CSRRW;
}

static uint32_t addi(unsigned int dest, unsigned int src, uint16_t imm) __attribute__ ((unused));
static uint32_t addi(unsigned int dest, unsigned int src, uint16_t imm)
{
	return imm_i(imm) | inst_rs1(src) | inst_rd(dest) | MATCH_ADDI;
}

static uint32_t csrr(unsigned int rd, unsigned int csr) __attribute__ ((unused));
static uint32_t csrr(unsigned int rd, unsigned int csr)
{
	return imm_i(csr) | inst_rd(rd) | MATCH_CSRRS;
}

static uint32_t csrrs(unsigned int rd, unsigned int rs, unsigned int csr) __attribute__ ((unused));
static uint32_t csrrs(unsigned int rd, unsigned int rs, unsigned int csr)
{
	return imm_i(csr) | inst_rs1(rs) | inst_rd(rd) | MATCH_CSRRS;
}

static uint32_t csrrw(unsigned int rd, unsigned int rs, unsigned int csr) __attribute__ ((unused));
static uint32_t csrrw(unsigned int rd, unsigned int rs, unsigned int csr)
{
	return imm_i(csr) | inst_rs1(rs) | inst_rd(rd) | MATCH_CSRRW;
}

static uint32_t csrrci(unsigned int rd, unsigned int zimm, unsigned int csr) __attribute__ ((unused));
static uint32_t csrrci(unsigned int rd, unsigned int zimm, unsigned int csr)
{
	return imm_i(csr) | inst_rs1(zimm) | inst_rd(rd) | MATCH_CSRRCI;
}

static uint32_t csrrsi(unsigned int rd, unsigned int zimm, unsigned int csr) __attribute__ ((unused));
static uint32_t csrrsi(unsigned int rd, unsigned int zimm, unsigned int csr)
{
	return imm_i(csr) | inst_rs1(zimm) | inst_rd(rd) | MATCH_CSRRSI;
}

static uint32_t fsw(unsigned int src, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t fsw(unsigned int src, unsigned int base, uint16_t offset)
{
	return imm_s(offset) | inst_rs2(src) | inst_rs1(base) | MATCH_FSW;
}

static uint32_t fsd(unsigned int src, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t fsd(unsigned int src, unsigned int base, uint16_t offset)
{
	return imm_s(offset) | inst_rs2(src) | inst_rs1(base) | MATCH_FSD;
}

static uint32_t flw(unsigned int dest, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t flw(unsigned int dest, unsigned int base, uint16_t offset)
{
	return imm_i(offset) | inst_rs1(base) | inst_rd(dest) | MATCH_FLW;
}

static uint32_t fld(unsigned int dest, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t fld(unsigned int dest, unsigned int base, uint16_t offset)
{
	return imm_i(offset) | inst_rs1(base) | inst_rd(dest) | MATCH_FLD;
}

static uint32_t fmv_x_w(unsigned dest, unsigned src) __attribute__ ((unused));
static uint32_t fmv_x_w(unsigned dest, unsigned src)
{
	return inst_rs1(src) | inst_rd(dest) | MATCH_FMV_X_W;
}

static uint32_t fmv_x_d(unsigned dest, unsigned src) __attribute__ ((unused));
static uint32_t fmv_x_d(unsigned dest, unsigned src)
{
	return inst_rs1(src) | inst_rd(dest) | MATCH_FMV_X_D;
}

static uint32_t fmv_w_x(unsigned dest, unsigned src) __attribute__ ((unused));
static uint32_t fmv_w_x(unsigned dest, unsigned src)
{
	return inst_rs1(src) | inst_rd(dest) | MATCH_FMV_W_X;
}

static uint32_t fmv_d_x(unsigned dest, unsigned src) __attribute__ ((unused));
static uint32_t fmv_d_x(unsigned dest, unsigned src)
{
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
	return imm_u(imm) | inst_rd(dest) | MATCH_LUI;
}

/*
static uint32_t csrci(unsigned int csr, uint16_t imm) __attribute__ ((unused));
static uint32_t csrci(unsigned int csr, uint16_t imm)
{
  return (csr << 20) |
    (bits(imm, 4, 0) << 15) |
    MATCH_CSRRCI;
}

static uint32_t li(unsigned int dest, uint16_t imm) __attribute__ ((unused));
static uint32_t li(unsigned int dest, uint16_t imm)
{
	return addi(dest, 0, imm);
}

static uint32_t fsd(unsigned int src, unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t fsd(unsigned int src, unsigned int base, uint16_t offset)
{
  return (bits(offset, 11, 5) << 25) |
    (bits(src, 4, 0) << 20) |
    (base << 15) |
    (bits(offset, 4, 0) << 7) |
    MATCH_FSD;
}

static uint32_t ori(unsigned int dest, unsigned int src, uint16_t imm) __attribute__ ((unused));
static uint32_t ori(unsigned int dest, unsigned int src, uint16_t imm)
{
  return (bits(imm, 11, 0) << 20) |
    (src << 15) |
    (dest << 7) |
    MATCH_ORI;
}

static uint32_t nop(void) __attribute__ ((unused));
static uint32_t nop(void)
{
  return addi(0, 0, 0);
}
*/

static uint32_t xori(unsigned int dest, unsigned int src, uint16_t imm) __attribute__ ((unused));
static uint32_t xori(unsigned int dest, unsigned int src, uint16_t imm)
{
	return imm_i(imm) | inst_rs1(src) | inst_rd(dest) | MATCH_XORI;
}

static uint32_t srli(unsigned int dest, unsigned int src, uint8_t shamt) __attribute__ ((unused));
static uint32_t srli(unsigned int dest, unsigned int src, uint8_t shamt)
{
	return inst_rs2(shamt) | inst_rs1(src) | inst_rd(dest) | MATCH_SRLI;
}

static uint32_t fence(void) __attribute__((unused));
static uint32_t fence(void)
{
	return MATCH_FENCE;
}

static uint32_t auipc(unsigned int dest) __attribute__((unused));
static uint32_t auipc(unsigned int dest)
{
	return MATCH_AUIPC | inst_rd(dest);
}

static uint32_t vsetvli(unsigned int dest, unsigned int src, uint16_t imm) __attribute__((unused));
static uint32_t vsetvli(unsigned int dest, unsigned int src, uint16_t imm)
{
	return (bits(imm, 10, 0) << 20) | inst_rs1(src) | inst_rd(dest) | MATCH_VSETVLI;
}

static uint32_t vsetvl(unsigned int rd, unsigned int rs1, unsigned int rs2) __attribute__((unused));
static uint32_t vsetvl(unsigned int rd, unsigned int rs1, unsigned int rs2)
{
	return inst_rd(rd) | inst_rs1(rs1) | inst_rs2(rs2) | MATCH_VSETVL;
}

static uint32_t vmv_x_s(unsigned int rd, unsigned int vs2) __attribute__((unused));
static uint32_t vmv_x_s(unsigned int rd, unsigned int vs2)
{
	return inst_rs2(vs2) | inst_rd(rd) | MATCH_VMV_X_S;
}

static uint32_t vmv_s_x(unsigned int vd, unsigned int vs2) __attribute__((unused));
static uint32_t vmv_s_x(unsigned int vd, unsigned int rs1)
{
	return inst_rs1(rs1) | inst_rd(vd) | MATCH_VMV_S_X;
}

static uint32_t vslide1down_vx(unsigned int vd, unsigned int vs2,
		unsigned int rs1, unsigned int vm) __attribute__((unused));
static uint32_t vslide1down_vx(unsigned int vd, unsigned int vs2,
		unsigned int rs1, unsigned int vm)
{
	return ((vm & 1) << 25) | inst_rs2(vs2) | inst_rs1(rs1) | inst_rd(vd) | MATCH_VSLIDE1DOWN_VX;
}


/***************************************************************************
 *   Copyright (C) 2015 by Alamy Liu                                       *
 *   alamy.liu@gmail.com                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef ARMV8_OPCODES_H
#define ARMV8_OPCODES_H


/*
 * ------------------------------------------------------------------
 */

/* C4.2.1 Compare & Branch (immediate) */
#define A64_OPCODE_COMPARE_BRANCH(sf, op, imm19, Xt) \
	((   (sf) &     0b1) << 31) | \
	(          (0b011010 << 25) | \
	((   (op) &     0b1) << 24) | \
	(((imm19) & 0x7FFFF) <<  5) | \
	((   (Xt) & 0b11111) <<  0) )

/* C4.2.2 Conditional branch (immediate) */
#define A64_OPCODE_CONDITIONAL_BRANCH(o1, imm19, o0, cond) \
	(         (0b0101010 << 25) | \
	((   (o1) &     0b1) << 24) | \
	(((imm19) & 0x7FFFF) <<  5) | \
	((   (o0) &     0b1) <<  4) | \
	(( (cond) &  0b1111) <<  0) )

/* C4.2.3 Exception generation */
#define A64_OPCODE_EXCEPTION(Opc, Imm16, Op2, LL) \
	(       (0b11010100 << 24) | \
	((  (Opc) &  0b111) << 21) | \
	(((Imm16) & 0xFFFF) <<  5) | \
	((  (Op2) &  0b111) <<  2) | \
	((   (LL) &   0b11) <<  0) )


/* C5.2
 * Op0==0b00: architectural hints, barriers and CLREX, and PSTATE access
 * Op0==0b01, cache maintenance, TLB maintenance, and address translation instructions
 * Op0==0b10, Moves to and from debug and trace System registers
 * Op0==0b11, Moves to and from non-debug System registers and Special-purpose registers
 */

/* C5.2.2 System instruction class encoding

   |31   28|27   24|23   20|19   16|15   12|11    8|7   5 4|3     0|
   |1 1 0 1 0 1 0 1 0 0|L|Op0| Op1 |  CRn  |  CRm  | Op2 |    Xt   |

	L
	0	Transfer to system register		W, MSR: 0xd51..., Op0=0b0x
	1	Transfer from system register	R, MRS: 0xd53..., Op0=0b1x
 */
#define	A64_OPCODE_SYSTEM(L, Op0, Op1, CRn, CRm, Op2, Xt) \
	(    (0b1101010100 << 22) | \
	((  (L) &     0b1) << 21) | \
	(((Op0) &    0b11) << 19) | \
	(((Op1) &   0b111) << 16) | \
	(((CRn) &  0b1111) << 12) | \
	(((CRm) &  0b1111) <<  8) | \
	(((Op2) &   0b111) <<  5) | \
	(( (Xt) & 0b11111) <<  0) )


/* C5.2.3: Op0 == 0b00
 *	Architectural hint, barriers and CLREX, and PSTATE access
 */
#define	A64_OPCODE_ARCHITECTURAL_HINT(Op6_0) \
		A64_OPCODE_SYSTEM(	0b0 /* L */, \
							0b00 /* Op0 */, \
							0b011 /* Op1 */, \
							0b0010 /* CRn */, \
							(Op6_0) >> 3 /* CRm */, \
							(Op6_0) & 0b111 /* Op2 */, \
							0b11111 /* Xt */)

#define	A64_OPCODE_NOP		A64_OPCODE_ARCHITECTURAL_HINT(0b0000000)
#define	A64_OPCODE_YIELD	A64_OPCODE_ARCHITECTURAL_HINT(0b0000001)
#define	A64_OPCODE_WFE		A64_OPCODE_ARCHITECTURAL_HINT(0b0000010)
#define	A64_OPCODE_WFI		A64_OPCODE_ARCHITECTURAL_HINT(0b0000011)
#define	A64_OPCODE_SEV		A64_OPCODE_ARCHITECTURAL_HINT(0b0000100)
#define	A64_OPCODE_SEVL		A64_OPCODE_ARCHITECTURAL_HINT(0b0000101)


#define A64_OPCODE_BARRIES_CLREX(CRm, Op2) \
		A64_OPCODE_SYSTEM(	0b0 /* L */, \
							0b00 /* Op0 */, \
							0b011 /* Op1 */, \
							0b0011 /* CRn */, \
							CRm /* CRm */, \
							Op2 /* Op2 */, \
							0b11111 /* Xt */)

#define A64_OPCODE_CLREX	A64_OPCODE_BARRIES_CLREX(0, 0b010)	/* CRm is ignored */
#define A64_OPCODE_DSB(t)	A64_OPCODE_BARRIES_CLREX(t, 0b100)	/* CRm sets the option type */
#define A64_OPCODE_DMB(t)	A64_OPCODE_BARRIES_CLREX(t, 0b101)	/* CRm sets the option type */
#define A64_OPCODE_ISB		A64_OPCODE_BARRIES_CLREX(0, 0b110)	/* CRm is ignored */


#define A64_OPCODE_PSTATE_FIELDS(Op1, Imm4, Op2) \
		A64_OPCODE_SYSTEM(	0b0 /* L */, \
							0b00 /* Op0 */, \
							Op1 /* Op1 */, \
							0b0100 /* CRn */, \
							Imm4 /* CRm */, \
							Op2 /* Op2 */, \
							0b11111 /* Xt */)

#define A64_OPCODE_MSR_SPSel_Imm(Imm)	A64_OPCODE_PSTATE_FIELDS(0b000, Imm, 0b101)
#define A64_OPCODE_MSR_DAIFSet_Imm(Imm)	A64_OPCODE_PSTATE_FIELDS(0b011, Imm, 0b110)
#define A64_OPCODE_MSR_DAIFClr_Imm(Imm)	A64_OPCODE_PSTATE_FIELDS(0b011, Imm, 0b111)


/* C5.2.4: Op0 == 0b01
 *	cache maintenance, TLB maintenance, and address translation instructions
 */
#define A64_OPCODE_CACHE_Xt(Op1, CRm, Op2, Xt) \
		A64_OPCODE_SYSTEM(	0b0 /* L */, \
							0b01 /* Op0 */, \
							Op1 /* Op1 */, \
							0b0111 /* CRn */, \
							CRm /* CRm */, \
							Op2 /* Op2 */, \
							Xt /* Xt */)

#define A64_OPCODE_CACHE(Op1, CRm, Op2) \
		A64_OPCODE_CACHE_Xt(Op1, CRm, Op2, 0b11111 /* Xt */)

#define A64_OPCODE_AT_Xt(Op1, Op2, Xt) \
		A64_OPCODE_CACHE_Xt(Op1, 0b1000 /* CRm */, Op2, Xt)

#define A64_OPCODE_TLB_Xt(Op1, CRm, Op2, Xt) \
		A64_OPCODE_SYSTEM(	0b0 /* L */, \
							0b01 /* Op0 */, \
							Op1 /* Op1 */, \
							0b1000 /* CRn */, \
							CRm /* CRm */, \
							Op2 /* Op2 */, \
							Xt /* Xt */)

#define A64_OPCODE_TLB(Op1, CRm, Op2) \
		A64_OPCODE_TLB_Xt(Op1, CRm, Op2, 0b11111 /* Xt */)


#define A64_OPCODE_IC_IALLUIS			A64_OPCODE_CACHE   (0, 1, 0)
#define A64_OPCODE_IC_IALLU				A64_OPCODE_CACHE   (0, 5, 0)
#define A64_OPCODE_IC_IVAU(Xt)			A64_OPCODE_CACHE_Xt(3, 5, 1, Xt)

#define A64_OPCODE_DC_IVAC(Xt)			A64_OPCODE_CACHE_Xt(0, 6, 1, Xt)
#define A64_OPCODE_DC_ISW(Xt)			A64_OPCODE_CACHE_Xt(0, 6, 2, Xt)
#define A64_OPCODE_DC_CSW(Xt)			A64_OPCODE_CACHE_Xt(0,10, 2, Xt)
#define A64_OPCODE_DC_CISW(Xt)			A64_OPCODE_CACHE_Xt(0,14, 2, Xt)
#define A64_OPCODE_DC_CVAC(Xt)			A64_OPCODE_CACHE_Xt(3,10, 1, Xt)
#define A64_OPCODE_DC_CVAU(Xt)			A64_OPCODE_CACHE_Xt(3,11, 1, Xt)
#define A64_OPCODE_DC_CIVAC(Xt)			A64_OPCODE_CACHE_Xt(3,14, 1, Xt)
#define A64_OPCODE_DC_ZVA(Xt)			A64_OPCODE_CACHE_Xt(3, 4, 1, Xt)

#define A64_OPCODE_AT_S1E1R(Xt)			A64_OPCODE_AT_Xt(0, 0, Xt)
#define A64_OPCODE_AT_S1E1W(Xt)			A64_OPCODE_AT_Xt(0, 1, Xt)
#define A64_OPCODE_AT_S1E0R(Xt)			A64_OPCODE_AT_Xt(0, 2, Xt)
#define A64_OPCODE_AT_S1E0W(Xt)			A64_OPCODE_AT_Xt(0, 3, Xt)
#define A64_OPCODE_AT_S1E2R(Xt)			A64_OPCODE_AT_Xt(4, 0, Xt)
#define A64_OPCODE_AT_S1E2W(Xt)			A64_OPCODE_AT_Xt(4, 1, Xt)
#define A64_OPCODE_AT_S12E1R(Xt)		A64_OPCODE_AT_Xt(4, 4, Xt)
#define A64_OPCODE_AT_S12E1W(Xt)		A64_OPCODE_AT_Xt(4, 5, Xt)
#define A64_OPCODE_AT_S12E0R(Xt)		A64_OPCODE_AT_Xt(4, 6, Xt)
#define A64_OPCODE_AT_S12E0W(Xt)		A64_OPCODE_AT_Xt(4, 7, Xt)
#define A64_OPCODE_AT_S1E3R(Xt)			A64_OPCODE_AT_Xt(6, 0, Xt)
#define A64_OPCODE_AT_S1E3W(Xt)			A64_OPCODE_AT_Xt(6, 1, Xt)

#define A64_OPCODE_TLBI_VMALLE1IS		A64_OPCODE_TLB(0, 3, 0)
#define A64_OPCODE_TLBI_VAE1IS(Xt)		A64_OPCODE_TLB(0, 3, 1, Xt)
#define A64_OPCODE_TLBI_ASIDE1IS(Xt)	A64_OPCODE_TLB(0, 3, 2, Xt)
#define A64_OPCODE_TLBI_VAAE1IS(Xt)		A64_OPCODE_TLB(0, 3, 3, Xt)
#define A64_OPCODE_TLBI_VALE1IS(Xt)		A64_OPCODE_TLB(0, 3, 5, Xt)
#define A64_OPCODE_TLBI_VAALE1IS(Xt)	A64_OPCODE_TLB(0, 3, 7, Xt)
#define A64_OPCODE_TLBI_VMALLE1			A64_OPCODE_TLB(0, 7, 0)
#define A64_OPCODE_TLBI_VAE1(Xt)		A64_OPCODE_TLB(0, 7, 1, Xt)
#define A64_OPCODE_TLBI_ASIDE1(Xt)		A64_OPCODE_TLB(0, 7, 2, Xt)
#define A64_OPCODE_TLBI_VAAE1(Xt)		A64_OPCODE_TLB(0, 7, 3, Xt)
#define A64_OPCODE_TLBI_VALE1(Xt)		A64_OPCODE_TLB(0, 7, 5, Xt)
#define A64_OPCODE_TLBI_VAALE1(Xt)		A64_OPCODE_TLB(0, 7, 7, Xt)

#define A64_OPCODE_TLBI_IPAS2E1IS(Xt)	A64_OPCODE_TLB(4, 0, 1, Xt)
#define A64_OPCODE_TLBI_IPAS2LE1IS(Xt)	A64_OPCODE_TLB(4, 0, 5, Xt)
#define A64_OPCODE_TLBI_ALLE2IS			A64_OPCODE_TLB(4, 3, 0)
#define A64_OPCODE_TLBI_VAE2IS(Xt)		A64_OPCODE_TLB(4, 3, 1, Xt)
#define A64_OPCODE_TLBI_ALLE1IS			A64_OPCODE_TLB(4, 3, 4)
#define A64_OPCODE_TLBI_VALE2IS(Xt)		A64_OPCODE_TLB(4, 3, 5, Xt)
#define A64_OPCODE_TLBI_VMALLS12E1IS	A64_OPCODE_TLB(4, 3, 6)
#define A64_OPCODE_TLBI_IPAS2E1(Xt)		A64_OPCODE_TLB(4, 4, 1, Xt)
#define A64_OPCODE_TLBI_IPAS2LE1(Xt)	A64_OPCODE_TLB(4, 4, 5, Xt)
#define A64_OPCODE_TLBI_ALLE2			A64_OPCODE_TLB(4, 7, 0)
#define A64_OPCODE_TLBI_VAE2(Xt)		A64_OPCODE_TLB(4, 7, 1, Xt)
#define A64_OPCODE_TLBI_ALLE1			A64_OPCODE_TLB(4, 7, 4)
#define A64_OPCODE_TLBI_VALE2(Xt)		A64_OPCODE_TLB(4, 7, 5, Xt)
#define A64_OPCODE_TLBI_VMALLS12E1		A64_OPCODE_TLB(4, 7, 6)

#define A64_OPCODE_TLBI_ALLE3IS			A64_OPCODE_TLB(6, 3, 0)
#define A64_OPCODE_TLBI_VAE3IS(Xt)		A64_OPCODE_TLB(6, 3, 1, Xt)
#define A64_OPCODE_TLBI_VALE3IS(Xt)		A64_OPCODE_TLB(6, 3, 5, Xt)
#define A64_OPCODE_TLBI_ALLE3			A64_OPCODE_TLB(6, 7, 0)
#define A64_OPCODE_TLBI_VAE3(Xt)		A64_OPCODE_TLB(6, 7, 1, Xt)
#define A64_OPCODE_TLBI_VALE3(Xt)		A64_OPCODE_TLB(6, 7, 5, Xt)


/* C5.2.5: Op0 == 0b10
 *	Moves to and from debug and trace System registers
 *	MSR <System register>, Xt	; Write to System register
 *	MRS Xt, <System register>	; Read from System register
 */
#define A64_OPCODE_DBG_MSR_MRS(L, Op1, CRn, CRm, Op2, Xt) \
		A64_OPCODE_SYSTEM(	L /* L */, \
							0b10 /* Op0 */, \
							Op1 /* Op1 */, \
							CRn /* CRn */, \
							CRm /* CRm */, \
							Op2 /* Op2 */, \
							Xt /* Xt */)

#define A64_OPCODE_DBG_MSR(Op1, CRn, CRm, Op2, Xt) \
		A64_OPCODE_DBG_MSR_MRS(0b0 /* L */, Op1, CRn, CRm, Op2, Xt)

#define A64_OPCODE_DBG_MRS(Op1, CRn, CRm, Op2, Xt) \
		A64_OPCODE_DBG_MSR_MRS(0b1 /* L */, Op1, CRn, CRm, Op2, Xt)

#define A64_OPCODE_MSR_OSDTRRX_EL1(Xt)		A64_OPCODE_DBG_MSR(0, 0, 0, 2, Xt)
#define A64_OPCODE_MRS_OSDTRRX_EL1(Xt)		A64_OPCODE_DBG_MRS(0, 0, 0, 2, Xt)
#define A64_OPCODE_MSR_MDCCINT_EL1(Xt)		A64_OPCODE_DBG_MSR(0, 0, 2, 0, Xt)
#define A64_OPCODE_MRS_MDCCINT_EL1(Xt)		A64_OPCODE_DBG_MRS(0, 0, 2, 0, Xt)
#define A64_OPCODE_MSR_MDSCR_EL1(Xt)		A64_OPCODE_DBG_MSR(0, 0, 2, 2, Xt)
#define A64_OPCODE_MRS_MDSCR_EL1(Xt)		A64_OPCODE_DBG_MRS(0, 0, 2, 2, Xt)
#define A64_OPCODE_MSR_OSDTRTX_EL1(Xt)		A64_OPCODE_DBG_MSR(0, 0, 3, 2, Xt)
#define A64_OPCODE_MRS_OSDTRTX_EL1(Xt)		A64_OPCODE_DBG_MRS(0, 0, 3, 2, Xt)
#define A64_OPCODE_MSR_OSECCR_EL1(Xt)		A64_OPCODE_DBG_MSR(0, 0, 6, 2, Xt)
#define A64_OPCODE_MRS_OSECCR_EL1(Xt)		A64_OPCODE_DBG_MRS(0, 0, 6, 2, Xt)

#define A64_OPCODE_MSR_DBGBVR_EL1(n, Xt)	A64_OPCODE_DBG_MSR(0, 0, n, 4, Xt)
#define A64_OPCODE_MRS_DBGBVR_EL1(n, Xt)	A64_OPCODE_DBG_MRS(0, 0, n, 4, Xt)
#define A64_OPCODE_MSR_DBGBCR_EL1(n, Xt)	A64_OPCODE_DBG_MSR(0, 0, n, 5, Xt)
#define A64_OPCODE_MRS_DBGBCR_EL1(n, Xt)	A64_OPCODE_DBG_MRS(0, 0, n, 5, Xt)
#define A64_OPCODE_MSR_DBGWVR_EL1(n, Xt)	A64_OPCODE_DBG_MSR(0, 0, n, 6, Xt)
#define A64_OPCODE_MRS_DBGWVR_EL1(n, Xt)	A64_OPCODE_DBG_MRS(0, 0, n, 6, Xt)
#define A64_OPCODE_MSR_DBGWCR_EL1(n, Xt)	A64_OPCODE_DBG_MSR(0, 0, n, 7, Xt)
#define A64_OPCODE_MRS_DBGWCR_EL1(n, Xt)	A64_OPCODE_DBG_MRS(0, 0, n, 7, Xt)

#define A64_OPCODE_MRS_MDRAR_EL1(Xt)		A64_OPCODE_DBG_MRS(0, 1, 0, 0, Xt)	/* RO */
#define A64_OPCODE_MSR_OSLAR_EL1(Xt)		A64_OPCODE_DBG_MSR(0, 1, 0, 4, Xt)	/* WO */
#define A64_OPCODE_MRS_OSLSR_EL1(Xt)		A64_OPCODE_DBG_MRS(0, 1, 1, 4, Xt)	/* RO */
#define A64_OPCODE_MSR_OSDLR_EL1(Xt)		A64_OPCODE_DBG_MSR(0, 1, 3, 4, Xt)
#define A64_OPCODE_MRS_OSDLR_EL1(Xt)		A64_OPCODE_DBG_MRS(0, 1, 3, 4, Xt)
#define A64_OPCODE_MSR_DBGPRCR_EL1(Xt)		A64_OPCODE_DBG_MSR(0, 1, 4, 4, Xt)
#define A64_OPCODE_MRS_DBGPRCR_EL1(Xt)		A64_OPCODE_DBG_MRS(0, 1, 4, 4, Xt)

#define A64_OPCODE_MSR_DBGCLAIMSET_EL1(Xt)	A64_OPCODE_DBG_MSR(0, 7, 8, 6, Xt)
#define A64_OPCODE_MRS_DBGCLAIMSET_EL1(Xt)	A64_OPCODE_DBG_MRS(0, 7, 8, 6, Xt)
#define A64_OPCODE_MSR_DBGCLAIMCLR_EL1(Xt)	A64_OPCODE_DBG_MSR(0, 7, 9, 6, Xt)
#define A64_OPCODE_MRS_DBGCLAIMCLR_EL1(Xt)	A64_OPCODE_DBG_MRS(0, 7, 9, 6, Xt)
#define A64_OPCODE_MRS_DBGAUTHSTATUS_EL1(Xt)	A64_OPCODE_DBG_MRS(0, 7,14, 6, Xt)	/* RO */

#define A64_OPCODE_MRS_MDCCSR_EL0(Xt)		A64_OPCODE_DBG_MRS(3, 0, 1, 0, Xt)	/* RO */
#define A64_OPCODE_MSR_DBGDTR_EL0(Xt)		A64_OPCODE_DBG_MSR(3, 0, 4, 0, Xt)	/* DCC <-- Xt */
#define A64_OPCODE_MRS_DBGDTR_EL0(Xt)		A64_OPCODE_DBG_MRS(3, 0, 4, 0, Xt)	/* Xt <-- DCC */
#define A64_OPCODE_MRS_DBGDTRRX_EL0(Xt)		A64_OPCODE_DBG_MRS(3, 0, 5, 0, Xt)	/* RO */
#define A64_OPCODE_MSR_DBGDTRTX_EL0(Xt)		A64_OPCODE_DBG_MSR(3, 0, 5, 0, Xt)	/* WO */

#define A64_OPCODE_MSR_DBGVCR32_EL2(Xt)		A64_OPCODE_DBG_MSR(4, 0, 7, 0, Xt)
#define A64_OPCODE_MRS_DBGVCR32_EL2(Xt)		A64_OPCODE_DBG_MRS(4, 0, 7, 0, Xt)


/* C5.2.5: Op0 == 0b11
 *	Moves to and from non-debug System registers and Special-purpose registers
 *	MSR <System register>, Xt	; Write to System register
 *	MRS Xt, <System register>	; Read from System register
 */
#define A64_OPCODE_NDBG_SPCL_MSR_MRS(L, Op1, CRn, CRm, Op2, Xt) \
		A64_OPCODE_SYSTEM(	L /* L */, \
							0b11 /* Op0 */, \
							Op1 /* Op1 */, \
							CRn /* CRn */, \
							CRm /* CRm */, \
							Op2 /* Op2 */, \
							Xt /* Xt */)

#define A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt) \
		A64_OPCODE_NDBG_SPCL_MSR_MRS(0b0 /* L */, Op1, CRn, CRm, Op2, Xt)

#define A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt) \
		A64_OPCODE_NDBG_SPCL_MSR_MRS(0b1 /* L */, Op1, CRn, CRm, Op2, Xt)

#define A64_OPCODE_SPCL_MSR(Op1, CRm, Op2, Xt) \
		A64_OPCODE_NDBG_SPCL_MSR_MRS(0b0 /* L */, Op1, 0b0100 /* CRn */, CRm, Op2, Xt)

#define A64_OPCODE_SPCL_MRS(Op1, CRm, Op2, Xt) \
		A64_OPCODE_NDBG_SPCL_MSR_MRS(0b1 /* L */, Op1, 0b0100 /* CRn */, CRm, Op2, Xt)

#if 0	/* Too much to defined, later */
#define A64_OPCODE_MSR_reg_EL0(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_reg_EL0(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_reg_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_reg_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_reg_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_reg_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)

#define A64_OPCODE_MRS_MIDR_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_MPIDR_EL1(Xt)	A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_REVIDR_EL1(Xt)	A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */

#define T32_OPCODE_MRS_PFR0_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_PFR1_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_DFR0_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_AFR0_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_MMFR0_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_MMFR1_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_MMFR2_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_MMFR3_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_ISAR0_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_ISAR1_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_ISAR2_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_ISAR3_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_ISAR4_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_ISAR5_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_MMFR4_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_MVFR0_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_MVFR1_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define T32_OPCODE_MRS_MVFR2_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */

#define A64_OPCODE_MRS_AA64PFR0_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_AA64PFR1_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_AA64DFR0_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_AA64DFR1_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_AA64AFR0_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_AA64AFR1_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_AA64ISAR0_EL1(Xt)	A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_AA64ISAR1_EL1(Xt)	A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_AA64MMFR0_EL1(Xt)	A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_AA64MMFR1_EL1(Xt)	A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */

#define A64_OPCODE_MSR_SCTLR_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_SCTLR_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_ACTLR_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_ACTLR_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_CPACR_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_CPACR_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_TTBR0_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_TTBR0_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_TTBR1_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_TTBR1_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_TCR_EL1(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_TCR_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)

#define A64_OPCODE_MSR_AFSR0_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_AFSR0_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_AFSR1_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_AFSR1_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_ESR_EL1(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_ESR_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_FAR_EL1(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_FAR_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_PAR_EL1(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_PAR_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_MAIR_EL1(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_MAIR_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_AMAIR_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_AMAIR_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)

#define A64_OPCODE_MSR_VBAR_EL1(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_VBAR_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_RVBAR_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_RVBAR_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_RMR_EL1(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_RMR_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_ISR_EL1(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_ISR_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_CONTEXTIDR_EL1(Xt)	A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_CONTEXTIDR_EL1(Xt)	A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_TPIDR_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_TPIDR_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_CCSIDR_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_CLIDR_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_AIDR_EL1(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MSR_CSSELR_EL1(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_CSSELR_EL1(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_CTR_EL0(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MRS_DCZID_EL0(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MSR_TPIDR_EL0(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_TPIDR_EL0(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_TPIDRRO_EL0(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_TPIDRRO_EL0(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_VPIDR_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_VPIDR_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_VMPIDR_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_VMPIDR_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)

#define A64_OPCODE_MSR_SCTLR_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_SCTLR_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_ACTLR_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_ACTLR_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_HCR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_HCR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_MDCR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_MDCR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_CPTR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_CPTR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_HSTR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_HSTR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_HACR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_HACR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_TTBR0_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_TTBR0_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_TCR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_TCR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_VTTBR_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_VTTBR_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_VTCR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_VTCR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_AFSR0_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_AFSR0_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_AFSR1_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_AFSR1_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_ESR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_ESR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_FAR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_FAR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_HPFAR_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_HPFAR_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_MAIR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_MAIR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_AMAIR_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_AMAIR_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_VBAR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_VBAR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_RVBAR_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)	/* RO */
#define A64_OPCODE_MSR_RMR_EL2(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_RMR_EL2(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_TPIDR_EL2(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_TPIDR_EL2(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)

#define A64_OPCODE_MSR_SCTLR_EL3(Xt)		A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_SCTLR_EL3(Xt)		A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_reg_EL3(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_reg_EL3(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_reg_EL3(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_reg_EL3(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_reg_EL3(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_reg_EL3(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_reg_EL3(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_reg_EL3(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_reg_EL3(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_reg_EL3(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_reg_EL3(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_reg_EL3(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MSR_reg_EL3(Xt)			A64_OPCODE_NDBG_MSR(Op1, CRn, CRm, Op2, Xt)
#define A64_OPCODE_MRS_reg_EL3(Xt)			A64_OPCODE_NDBG_MRS(Op1, CRn, CRm, Op2, Xt)

...

#endif	/* Too much to defined, later */

#define A64_OPCODE_MSR_SPSR_EL1(Xt)		A64_OPCODE_SPCL_MSR(0, 0, 0, Xt)
#define A64_OPCODE_MRS_SPSR_EL1(Xt)		A64_OPCODE_SPCL_MRS(0, 0, 0, Xt)
#define A64_OPCODE_MSR_ELR_EL1(Xt)		A64_OPCODE_SPCL_MSR(0, 0, 1, Xt)
#define A64_OPCODE_MRS_ELR_EL1(Xt)		A64_OPCODE_SPCL_MRS(0, 0, 1, Xt)
#define A64_OPCODE_MSR_SP_EL0(Xt)		A64_OPCODE_SPCL_MSR(0, 1, 0, Xt)
#define A64_OPCODE_MRS_SP_EL0(Xt)		A64_OPCODE_SPCL_MRS(0, 1, 0, Xt)
#define A64_OPCODE_MSR_SPSel(Xt)		A64_OPCODE_SPCL_MSR(0, 2, 0, Xt)
#define A64_OPCODE_MRS_SPSel(Xt)		A64_OPCODE_SPCL_MRS(0, 2, 0, Xt)
#define A64_OPCODE_MSR_CurrentEL(Xt)	A64_OPCODE_SPCL_MSR(0, 2, 2, Xt)
#define A64_OPCODE_MRS_CurrentEL(Xt)	A64_OPCODE_SPCL_MRS(0, 2, 2, Xt)

#define A64_OPCODE_MSR_NZCV(Xt)			A64_OPCODE_SPCL_MSR(3, 2, 0, Xt)
#define A64_OPCODE_MRS_NZCV(Xt)			A64_OPCODE_SPCL_MRS(3, 2, 0, Xt)
#define A64_OPCODE_MSR_DAIF(Xt)			A64_OPCODE_SPCL_MSR(3, 2, 1, Xt)
#define A64_OPCODE_MRS_DAIF(Xt)			A64_OPCODE_SPCL_MRS(3, 2, 1, Xt)
#define A64_OPCODE_MSR_FPCR(Xt)			A64_OPCODE_SPCL_MSR(3, 4, 0, Xt)
#define A64_OPCODE_MRS_FPCR(Xt)			A64_OPCODE_SPCL_MRS(3, 4, 0, Xt)
#define A64_OPCODE_MSR_FPSR(Xt)			A64_OPCODE_SPCL_MSR(3, 4, 1, Xt)
#define A64_OPCODE_MRS_FPSR(Xt)			A64_OPCODE_SPCL_MRS(3, 4, 1, Xt)
#define A64_OPCODE_MSR_DSPSR_EL0(Xt)	A64_OPCODE_SPCL_MSR(3, 5, 0, Xt)
#define A64_OPCODE_MRS_DSPSR_EL0(Xt)	A64_OPCODE_SPCL_MRS(3, 5, 0, Xt)
#define A64_OPCODE_MSR_DLR_EL0(Xt)		A64_OPCODE_SPCL_MSR(3, 5, 1, Xt)
#define A64_OPCODE_MRS_DLR_EL0(Xt)		A64_OPCODE_SPCL_MRS(3, 5, 1, Xt)

#define A64_OPCODE_MSR_SPSR_EL2(Xt)		A64_OPCODE_SPCL_MSR(4, 0, 0, Xt)
#define A64_OPCODE_MRS_SPSR_EL2(Xt)		A64_OPCODE_SPCL_MRS(4, 0, 0, Xt)
#define A64_OPCODE_MSR_ELR_EL2(Xt)		A64_OPCODE_SPCL_MSR(4, 0, 1, Xt)
#define A64_OPCODE_MRS_ELR_EL2(Xt)		A64_OPCODE_SPCL_MRS(4, 0, 1, Xt)
#define A64_OPCODE_MSR_SP_EL1(Xt)		A64_OPCODE_SPCL_MSR(4, 1, 0, Xt)
#define A64_OPCODE_MRS_SP_EL1(Xt)		A64_OPCODE_SPCL_MRS(4, 1, 0, Xt)
#define A64_OPCODE_MSR_SPSR_irq(Xt)		A64_OPCODE_SPCL_MSR(4, 3, 0, Xt)
#define A64_OPCODE_MRS_SPSR_irq(Xt)		A64_OPCODE_SPCL_MRS(4, 3, 0, Xt)
#define A64_OPCODE_MSR_SPSR_abt(Xt)		A64_OPCODE_SPCL_MSR(4, 3, 1, Xt)
#define A64_OPCODE_MRS_SPSR_abt(Xt)		A64_OPCODE_SPCL_MRS(4, 3, 1, Xt)
#define A64_OPCODE_MSR_SPSR_und(Xt)		A64_OPCODE_SPCL_MSR(4, 3, 2, Xt)
#define A64_OPCODE_MRS_SPSR_und(Xt)		A64_OPCODE_SPCL_MRS(4, 3, 2, Xt)
#define A64_OPCODE_MSR_SPSR_fiq(Xt)		A64_OPCODE_SPCL_MSR(4, 3, 3, Xt)
#define A64_OPCODE_MRS_SPSR_fiq(Xt)		A64_OPCODE_SPCL_MRS(4, 3, 3, Xt)

#define A64_OPCODE_MSR_SPSR_EL3(Xt)		A64_OPCODE_SPCL_MSR(6, 0, 0, Xt)
#define A64_OPCODE_MRS_SPSR_EL3(Xt)		A64_OPCODE_SPCL_MRS(6, 0, 0, Xt)
#define A64_OPCODE_MSR_ELR_EL3(Xt)		A64_OPCODE_SPCL_MSR(6, 0, 1, Xt)
#define A64_OPCODE_MRS_ELR_EL3(Xt)		A64_OPCODE_SPCL_MRS(6, 0, 1, Xt)
#define A64_OPCODE_MSR_SP_EL2(Xt)		A64_OPCODE_SPCL_MSR(6, 1, 0, Xt)
#define A64_OPCODE_MRS_SP_EL2(Xt)		A64_OPCODE_SPCL_MRS(6, 1, 0, Xt)


/* C6.6.29: BRK : Software Breakpoint */
#define A64_OPCODE_BRK(imm16) \
		((0b11010100 << 24) | (0b001 << 21) | \
		(((imm16) & 0xFFFF) << 5) | (0b00000))

/* C6.6.62: DSB (Data Synchronization Barrier) */
#define A64_OPCODE_DSB_SY		A64_OPCODE_DSB(0b1111)
#define	A64_OPCODE_DSB_ST		A64_OPCODE_DSB(0b1110)
#define	A64_OPCODE_DSB_LD		A64_OPCODE_DSB(0b1101)
#define	A64_OPCODE_DSB_ISH		A64_OPCODE_DSB(0b1011)
#define	A64_OPCODE_DSB_ISHST	A64_OPCODE_DSB(0b1010)
#define	A64_OPCODE_DSB_ISHLD	A64_OPCODE_DSB(0b1001)
#define	A64_OPCODE_DSB_NSH		A64_OPCODE_DSB(0b0111)
#define	A64_OPCODE_DSB_NSHST	A64_OPCODE_DSB(0b0110)
#define	A64_OPCODE_DSB_NSHLD	A64_OPCODE_DSB(0b0101)
#define	A64_OPCODE_DSB_OSH		A64_OPCODE_DSB(0b0011)
#define	A64_OPCODE_DSB_OSHST	A64_OPCODE_DSB(0b0010)
#define	A64_OPCODE_DSB_OSHLD	A64_OPCODE_DSB(0b0001)


/* C6.6.131: MSR(register) */
#define A64_OPCODE_MSR(Op0, Op1, CRn, CRm, Op2, Xt) \
		A64_OPCODE_SYSTEM(	0b0 /* L */, \
							Op0 /* Op0 */, \
							Op1 /* Op1 */, \
							CRn /* CRn */, \
							CRm /* CRm */, \
							Op2 /* Op2 */, \
							Xt /* Xt */)

/* C6.6.129: MRS */
#define A64_OPCODE_MRS(Op0, Op1, CRn, CRm, Op2, Xt) \
		A64_OPCODE_SYSTEM(	0b1 /* L */, \
							Op0 /* Op0 */, \
							Op1 /* Op1 */, \
							CRn /* CRn */, \
							CRm /* CRm */, \
							Op2 /* Op2 */, \
							Xt /* Xt */)


#endif /* ARMV8_OPCODES_H */

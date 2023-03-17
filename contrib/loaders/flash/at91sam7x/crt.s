/* SPDX-License-Identifier: BSD-3-Clause */

/****************************************************************************
*  Copyright (c) 2006 by Michael Fischer. All rights reserved.
****************************************************************************
*
*  History:
*
*  18.12.06  mifi   First Version
*                   The hardware initialization is based on the startup file
*                   crtat91sam7x256_rom.S from NutOS 4.2.1.
*                   Therefore partial copyright by egnite Software GmbH.
****************************************************************************/

/*
 * Some defines for the program status registers
 */
   ARM_MODE_USER  = 0x10      /* Normal User Mode                             */
   ARM_MODE_FIQ   = 0x11      /* FIQ Fast Interrupts Mode                     */
   ARM_MODE_IRQ   = 0x12      /* IRQ Standard Interrupts Mode                 */
   ARM_MODE_SVC   = 0x13      /* Supervisor Interrupts Mode                   */
   ARM_MODE_ABORT = 0x17      /* Abort Processing memory Faults Mode          */
   ARM_MODE_UNDEF = 0x1B      /* Undefined Instructions Mode                  */
   ARM_MODE_SYS   = 0x1F      /* System Running in Privileged Operating Mode  */
   ARM_MODE_MASK  = 0x1F

   I_BIT          = 0x80      /* disable IRQ when I bit is set */
   F_BIT          = 0x40      /* disable IRQ when I bit is set */

/*
 * Register Base Address
 */
   AIC_BASE         = 0xFFFFF000
   AIC_EOICR_OFF    = 0x130
   AIC_IDCR_OFF     = 0x124

   RSTC_MR          = 0xFFFFFD08
   RSTC_KEY         = 0xA5000000
   RSTC_URSTEN      = 0x00000001

   WDT_BASE         = 0xFFFFFD40
   WDT_MR_OFF       = 0x00000004
   WDT_WDDIS        = 0x00008000

   MC_BASE          = 0xFFFFFF00
   MC_FMR_OFF       = 0x00000060
   MC_FWS_1FWS      = 0x00480100

   .section .vectors,"ax"
   .code 32

/****************************************************************************/
/*               Vector table and reset entry                               */
/****************************************************************************/
_vectors:
   ldr pc, ResetAddr    /* Reset                 */
   ldr pc, UndefAddr    /* Undefined instruction */
   ldr pc, SWIAddr      /* Software interrupt    */
   ldr pc, PAbortAddr   /* Prefetch abort        */
   ldr pc, DAbortAddr   /* Data abort            */
   ldr pc, ReservedAddr /* Reserved              */
   ldr pc, IRQAddr      /* IRQ interrupt         */
   ldr pc, FIQAddr      /* FIQ interrupt         */


ResetAddr:     .word ResetHandler
UndefAddr:     .word UndefHandler
SWIAddr:       .word SWIHandler
PAbortAddr:    .word PAbortHandler
DAbortAddr:    .word DAbortHandler
ReservedAddr:  .word 0
IRQAddr:       .word IRQHandler
FIQAddr:       .word FIQHandler

   .ltorg

   .section .init, "ax"
   .code 32

   .global ResetHandler
   .global ExitFunction
   .extern main
/****************************************************************************/
/*                           Reset handler                                  */
/****************************************************************************/
ResetHandler:
   /*
    * The watchdog is enabled after processor reset. Disable it.
    */
   ldr   r1, =WDT_BASE
   ldr   r0, =WDT_WDDIS
   str   r0, [r1, #WDT_MR_OFF]


   /*
    * Enable user reset: assertion length programmed to 1ms
    */
   ldr   r0, =(RSTC_KEY | RSTC_URSTEN | (4 << 8))
   ldr   r1, =RSTC_MR
   str   r0, [r1, #0]


   /*
    * Use 2 cycles for flash access.
    */
   ldr   r1, =MC_BASE
   ldr   r0, =MC_FWS_1FWS
   str   r0, [r1, #MC_FMR_OFF]


   /*
    * Disable all interrupts. Useful for debugging w/o target reset.
    */
   ldr   r1, =AIC_BASE
   mvn   r0, #0
   str   r0, [r1, #AIC_EOICR_OFF]
   str   r0, [r1, #AIC_IDCR_OFF]


   /*
    * Setup a stack for each mode
    */
   msr   CPSR_c, #ARM_MODE_UNDEF | I_BIT | F_BIT   /* Undefined Instruction Mode */
   ldr   sp, =__stack_und_end

   msr   CPSR_c, #ARM_MODE_ABORT | I_BIT | F_BIT   /* Abort Mode */
   ldr   sp, =__stack_abt_end

   msr   CPSR_c, #ARM_MODE_FIQ | I_BIT | F_BIT     /* FIQ Mode */
   ldr   sp, =__stack_fiq_end

   msr   CPSR_c, #ARM_MODE_IRQ | I_BIT | F_BIT     /* IRQ Mode */
   ldr   sp, =__stack_irq_end

   msr   CPSR_c, #ARM_MODE_SVC | I_BIT | F_BIT     /* Supervisor Mode */
   ldr   sp, =__stack_svc_end


   /*
    * Clear .bss section
    */
   ldr   r1, =__bss_start
   ldr   r2, =__bss_end
   ldr   r3, =0
bss_clear_loop:
   cmp   r1, r2
   strne r3, [r1], #+4
   bne   bss_clear_loop


   /*
    * Jump to main
    */
   mrs   r0, cpsr
   bic   r0, r0, #I_BIT | F_BIT     /* Enable FIQ and IRQ interrupt */
   msr   cpsr, r0

   mov   r0, #0 /* No arguments */
   mov   r1, #0 /* No arguments */
   ldr   r2, =main
   mov   lr, pc
   bx    r2     /* And jump... */

ExitFunction:
   nop
   nop
   nop
   b ExitFunction


/****************************************************************************/
/*                         Default interrupt handler                        */
/****************************************************************************/

UndefHandler:
   b UndefHandler

SWIHandler:
   b SWIHandler

PAbortHandler:
   b PAbortHandler

DAbortHandler:
   b DAbortHandler

IRQHandler:
   b IRQHandler

FIQHandler:
   b FIQHandler

   .weak ExitFunction
   .weak UndefHandler, PAbortHandler, DAbortHandler
   .weak IRQHandler, FIQHandler

   .ltorg
/*** EOF ***/

/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 *   Copyright (C) 2017 Ake Rehnman
 *   ake.rehnman(at)gmail.com
 */
;;
;; erase check memory code
;;
 .org 0x0
;; start address
 start_addr: .byte 0x00
       .word 0x8000
;; byte count
 byte_cnt: .byte 0x00
       .word 0x8000
;
; SP must point to start_addr on entry
; first relocate start_addr to the location
; we are running at
start:
	ldw X,SP
	ldw .cont+2,X
	ldw X,(start_addr+1,SP)	;start addr
	ldw Y,(byte_cnt+1,SP)	;count
	ld A,#0xff
;
; if count == 0 return
.L1:
	tnzw Y
	jrne .decrcnt	;continue if low word != 0
	tnz (byte_cnt,SP)	;high byte
	jreq .exit	;goto exit
;
; decrement count (byte_cnt)
.decrcnt:
	tnzw Y	;low word count
	jrne .decr1
	dec (byte_cnt,SP)	;high byte
.decr1:
	decw Y;	decr low word
;
; first check if [start_addr] is 0xff
.cont:
	ldf A, [start_addr.e]
	cp A,#0xff
	jrne .exit ;exit if not 0xff
;
; increment start_addr (addr)
	incw X
	jrne .L1
	inc (start_addr,SP)	;increment high byte
	jra .L1
;
.exit:
	ldw (start_addr+1,SP),X	;start addr
	ldw (byte_cnt+1,SP),Y	;count
	break

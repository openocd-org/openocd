; SPDX-License-Identifier: GPL-2.0-or-later
;****************************************************************************
;	File : USBJmpTb.a51														*
;	Contents : Interruptions vector configuration.                    		*
;	Based on openULINK project code by: Martin Schmoelzer.					*
;	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.				*
;	<aboudjelida@nanoxplore.com>											*
;	<ahmederrachedbjld@gmail.com>											*
;****************************************************************************
.module JUMPTABLE

.globl USB_AutoVector
.globl USB_Jump_Table

.globl _sudav_isr, _sof_isr, _sutok_isr, _suspend_isr, _usbreset_isr, _highspeed_isr, _ep0ack_isr, _stub_isr, _ep0in_isr, _ep0out_isr, _ep1in_isr, _ep1out_isr, _ep2_isr, _ep4_isr, _ep6_isr, _ep8_isr, _ibn_isr
.globl _ep0pingnak_isr, _ep1pingnak_isr, _ep2pingnak_isr, _ep4pingnak_isr, _ep6pingnak_isr, _ep8pingnak_isr, _errorlimit_isr, _stub_isr, _stub_isr, _stub_isr, _ep2piderror_isr, _ep4piderror_isr, _ep6piderror_isr, _ep8piderror_isr
.globl _ep2pflag_isr, _ep4pflag_isr, _ep6pflag_isr, _ep8pflag_isr, _ep2eflag_isr, _ep4eflag_isr, _ep6eflag_isr, _ep8eflag_isr, _ep2fflag_isr, _ep4fflag_isr, _ep6fflag_isr, _ep8fflag_isr, _gpifcomplete_isr, _gpifwaveform_isr

;--------------------------------------------------------------------------;
; Interrupt Vectors                                                        ;
;--------------------------------------------------------------------------;
.area   USB_JV (ABS,OVR)   ; Absolute, Overlay
.org    0x43               ; USB interrupt (INT2) jumps here
USB_AutoVector = #. + 2
    ljmp  USB_Jump_Table    ; Autovector will replace byte 45

;--------------------------------------------------------------------------;
; USB Jump Table                                                           ;
;--------------------------------------------------------------------------;
.area  USB_JT (ABS)        ; Absolute placement
.org   0x0200              ; Place jump table at 0x0200

USB_Jump_Table:            ; autovector jump table
    ljmp  _sudav_isr       ; (00) Setup Data Available
    .db 0
    ljmp  _sof_isr         ; (04) Start of Frame
    .db 0
    ljmp  _sutok_isr       ; (08) Setup Data Loading
    .db 0
    ljmp  _suspend_isr     ; (0C) Global Suspend
    .db 0
    ljmp  _usbreset_isr    ; (10) USB Reset
    .db 0
    ljmp  _highspeed_isr   ; (14) Entered High Speed
    .db 0
    ljmp  _ep0ack_isr      ; (18) EP0ACK
    .db 0
    ljmp  _stub_isr        ; (1C) Reserved
    .db 0
    ljmp  _ep0in_isr       ; (20) EP0 In
    .db 0
    ljmp  _ep0out_isr      ; (24) EP0 Out
    .db 0
    ljmp  _ep1in_isr       ; (28) EP1 In
    .db 0
    ljmp  _ep1out_isr      ; (2C) EP1 Out
    .db 0
    ljmp  _ep2_isr         ; (30) EP2 In/Out
    .db 0
    ljmp  _ep4_isr         ; (34) EP4 In/Out
    .db 0
    ljmp  _ep6_isr         ; (38) EP6 In/Out
    .db 0
    ljmp  _ep8_isr         ; (3C) EP8 In/Out
    .db 0
    ljmp  _ibn_isr         ; (40) IBN
    .db 0
    ljmp  _stub_isr        ; (44) Reserved
    .db 0
    ljmp  _ep0pingnak_isr  ; (48) EP0 PING NAK
    .db 0
    ljmp  _ep1pingnak_isr  ; (4C) EP1 PING NAK
    .db 0
    ljmp  _ep2pingnak_isr  ; (50) EP2 PING NAK
    .db 0
    ljmp  _ep4pingnak_isr  ; (54) EP4 PING NAK
    .db 0
    ljmp  _ep6pingnak_isr  ; (58) EP6 PING NAK
    .db 0
    ljmp  _ep8pingnak_isr  ; (5C) EP8 PING NAK
    .db 0
    ljmp  _errorlimit_isr  ; (60) Error Limit
    .db 0
    ljmp  _stub_isr        ; (64) Reserved
    .db 0
    ljmp  _stub_isr        ; (68) Reserved
    .db 0
    ljmp  _stub_isr        ; (6C) Reserved
    .db 0
    ljmp  _ep2piderror_isr ; (70) EP2 ISO Pid Sequence Error
    .db 0
    ljmp  _ep4piderror_isr ; (74) EP4 ISO Pid Sequence Error
    .db 0
    ljmp  _ep6piderror_isr ; (78) EP6 ISO Pid Sequence Error
    .db 0
    ljmp  _ep8piderror_isr ; (7C) EP8 ISO Pid Sequence Error
    .db 0
    ljmp  _ep2pflag_isr    ; (80) EP2 Programmable Flag
    .db 0
    ljmp  _ep4pflag_isr    ; (84) EP4 Programmable Flag
    .db 0
    ljmp  _ep6pflag_isr    ; (88) EP6 Programmable Flag
    .db 0
    ljmp  _ep8pflag_isr    ; (8C) EP8 Programmable Flag
    .db 0
    ljmp  _ep2eflag_isr    ; (90) EP2 Empty Flag
    .db 0
    ljmp  _ep4eflag_isr    ; (94) EP4 Empty Flag
    .db 0
    ljmp  _ep6eflag_isr    ; (98) EP6 Empty Flag
    .db 0
    ljmp  _ep8eflag_isr    ; (9C) EP8 Empty Flag
    .db 0
    ljmp  _ep2fflag_isr    ; (A0) EP2 Full Flag
    .db 0
    ljmp  _ep4fflag_isr    ; (A4) EP4 Full Flag
    .db 0
    ljmp  _ep6fflag_isr    ; (A8) EP6 Full Flag
    .db 0
    ljmp  _ep8fflag_isr    ; (AC) EP8 Full Flag
    .db 0
    ljmp  _gpifcomplete_isr    ; (B0) GPIF Operation Complete
    .db 0
    ljmp  _gpifwaveform_isr    ; (B4) GPIF Waveform
    .db 0

;--------------------------------------------------------------------------;
;    Copyright (C) 2011-2013 by Martin Schmoelzer                          ;
;    <martin.schmoelzer@student.tuwien.ac.at>                              ;
;                                                                          ;
;    This program is free software; you can redistribute it and/or modify  ;
;    it under the terms of the GNU General Public License as published by  ;
;    the Free Software Foundation; either version 2 of the License, or     ;
;    (at your option) any later version.                                   ;
;                                                                          ;
;    This program is distributed in the hope that it will be useful,       ;
;    but WITHOUT ANY WARRANTY; without even the implied warranty of        ;
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         ;
;    GNU General Public License for more details.                          ;
;                                                                          ;
;    You should have received a copy of the GNU General Public License     ;
;    along with this program.  If not, see <http://www.gnu.org/licenses/>. ;
;--------------------------------------------------------------------------;

.module JUMPTABLE
.globl USB_AutoVector
.globl USB_Jump_Table

;--------------------------------------------------------------------------;
; Interrupt Vectors                                                        ;
;--------------------------------------------------------------------------;
.area   USB_JV (ABS,OVR)   ; Absolute, Overlay
.org    0x43               ; USB interrupt (INT2) jumps here
USB_AutoVector = #. + 2
    ljmp  USB_Jump_Table

;--------------------------------------------------------------------------;
; USB Jump Table                                                           ;
;--------------------------------------------------------------------------;
.area  USB_JT (ABS)        ; Absolute placement
.org   0x1B00              ; Place jump table at 0x1B00

USB_Jump_Table:            ; autovector jump table
    ljmp  _sudav_isr       ; Setup Data Available
    .db 0
    ljmp  _sof_isr         ; Start of Frame
    .db 0
    ljmp  _sutok_isr       ; Setup Data Loading
    .db 0
    ljmp  _suspend_isr     ; Global Suspend
    .db 0
    ljmp  _usbreset_isr    ; USB Reset
    .db 0
    ljmp  _ibn_isr         ; IN Bulk NAK interrupt
    .db 0
    ljmp  _ep0in_isr       ; Endpoint 0 IN
    .db 0
    ljmp  _ep0out_isr      ; Endpoint 0 OUT
    .db 0
    ljmp  _ep1in_isr       ; Endpoint 1 IN
    .db 0
    ljmp  _ep1out_isr      ; Endpoint 1 OUT
    .db 0
    ljmp  _ep2in_isr       ; Endpoint 2 IN
    .db 0
    ljmp  _ep2out_isr      ; Endpoint 2 OUT
    .db 0
    ljmp  _ep3in_isr       ; Endpoint 3 IN
    .db 0
    ljmp  _ep3out_isr      ; Endpoint 3 OUT
    .db 0
    ljmp  _ep4in_isr       ; Endpoint 4 IN
    .db 0
    ljmp  _ep4out_isr      ; Endpoint 4 OUT
    .db 0
    ljmp  _ep5in_isr       ; Endpoint 5 IN
    .db 0
    ljmp  _ep5out_isr      ; Endpoint 5 OUT
    .db 0
    ljmp  _ep6in_isr       ; Endpoint 6 IN
    .db 0
    ljmp  _ep6out_isr      ; Endpoint 6 OUT
    .db 0
    ljmp  _ep7in_isr       ; Endpoint 7 IN
    .db 0
    ljmp  _ep7out_isr      ; Endpoint 7 OUT
    .db 0

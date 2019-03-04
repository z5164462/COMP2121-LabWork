;
; Lab03- A.asm
;
; Created: 4/03/2019 2:54:54 PM
; Author : andre
;


; Replace with your application code


.include "m2560def.inc"



.def letter = r16
.def len = r17
.dseg
.org 0x200
	rev_string: .byte 4

.cseg
	rjmp start
	string: .db "abc",0

start:
	ldi ZL, low(string<<1)	;load address of first character
	ldi ZH, High(string<<1)
	
	ldi YL, low(RAMEND)		;initialise stack pointer
	ldi YH, high(RAMEND)
	out SPL, YL
	out SPH, YH
	
	clr len
loop_start:
	lpm letter, Z+
	cpi letter, 0
	breq end_loop
	push letter
	inc len
	rjmp loop_start

end_loop:


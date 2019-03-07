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
.def counter = r18

.dseg
.org 0x300
	rev_string: .byte 4

.cseg
	rjmp start
	string: .db "Implement a program that loads a null-terminated string from program memory and pushes it onto the stack, then writes out the reversed string into data memory. The stack pointer must be initialized before use.",0

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
	clr counter

	ldi XL, low(rev_string)
	ldi XH, high(rev_string)
	
	;add XL, len
	;adiw X, 1
rev_loop:	
	pop letter
	cp counter, len
	breq end_rev_loop
	st X+, letter
	;ld r22, X+
	inc counter
	rjmp rev_loop
	
end_rev_loop:
	ldi letter, 0
	st X+, letter
end:
	rjmp end
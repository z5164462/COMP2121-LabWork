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
loop_start:	; loop for loading string
	lpm letter, Z+	; load character from program memory
	cpi letter, 0	; check its not null terminator (0)
	breq end_loop
	push letter		; if not '0' push onto stack
	inc len			; increment length of string
	rjmp loop_start
	
end_loop:
	clr counter					; counter to check all letters have been stored

	ldi XL, low(rev_string)		; space to store the reversed string
	ldi XH, high(rev_string)
	
	;add XL, len
	;adiw X, 1
rev_loop:		
	pop letter					; pop letter from stack
	cp counter, len				; check that all letters havent been popped yet
	breq end_rev_loop
	st X+, letter				; store the letter in the allocated space and increment array pointer
	;ld r22, X+
	inc counter
	rjmp rev_loop
	
end_rev_loop:
	ldi letter, 0				; add a null terminator to the end of the string
	st X+, letter
end:
	rjmp end
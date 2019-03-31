;
; Lab05 - A.asm
;
; Created: 20/03/2019 9:02:50 PM
; Author : andre
;

.include "m2560def.inc"

.equ floor = 6
.dseg

.cseg

.org 0x0000 ; reset adress
rjmp RESET



; Replace with your application code
RESET:
	ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

	ser r16
	out DDRC, r16
	out DDRG, r16
	ldi r17, 0b01010101
	ldi r18, 0b110001
	out PORTC, r17
	out PORTG, r18
	ldi r16,  (2<<ISC00)
	sts EICRA, r16

	in r16, EIMSK
	ori r16, (1<<INT0)
	out EIMSK, r16
	
	sei
	jmp main 


main:
	ldi r16, floor
	clr r15	; counter
	clr XL	; output
	clr XH
loop:
	cp r15, r16
	breq end
	lsl XL
	inc XL
	brcs grtr8
	rjmp end_x
grtr8:
	lsl XH
	inc XH
end_x:	
	inc r15
	rjmp loop

end:
	out PORTC, XL
	out PORTG, XH


exit:
	rjmp exit

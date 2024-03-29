;
; Lab05 - A.asm
;
; Created: 20/03/2019 9:02:50 PM
; Author : andre
;

.include "m2560def.inc"
.def temp = r16
.def zero = r3
.def floor = r20
.def direction = r21

.dseg
Count:
	.byte 2
Seconds:
	.byte 2
.cseg

.org 0x0000 ; reset adress
rjmp RESET

.org OVF0addr
	jmp Timer0OVF

; Replace with your application code
RESET:
	ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

	clr zero
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
	
	ldi temp, 0b00000000
	out TCCR0A, temp
	ldi temp, 0b00000010
	out TCCR0B, temp
	ldi temp, 1<<TOIE0
	sts TIMSK0, temp


	sei
	jmp main 



Timer0OVF:
	in temp, SREG
	push temp
	push YH
	push YL
	push r25
	push r24

	lds r24, Count
	lds r25, Count + 1
	adiw r25:r24, 1
	
	cpi r24, low(7812)
	ldi temp, high(7812)
	cpc r25, temp
	brne Not_second
	lds r24, Seconds
	lds r25, Seconds+1


	adiw r25:r24, 1
	sts Seconds, r24
	sts Seconds+1, r25
	sts Count, zero
	sts Count+1, zero
	rjmp End_I

Not_second:
	sts Count, r24 
	sts Count+1, r25 

End_I:
	pop r25
	pop r24
	pop YL
	pop YH
	pop temp
	out SREG, temp
	reti

main:
	ldi floor, 1 ;this is the FlOoR <-------------------------------
	ldi direction, 1
	rcall show_floor
wait_loop:
	lds r24, Seconds
	cpi r24, 2
	brne wait_loop
	sts Seconds, zero
	sts Seconds+1, zero
	cpi floor, 10
	brne next
	ldi direction, -1
	rjmp move
next:
	cpi floor, 1
	brne move
	ldi direction, 1
move:
	add floor, direction
	rcall show_floor
	rjmp wait_loop



	
	
end_main:	
	rjmp end_main




;input r20 = floor, output X = binary representation
show_floor:
;prologue
	push YL
	push YH
	push floor
	push r15
	push XL
	push XH


	clr r15	; counter
	clr XL	; output
	clr XH
loop:
	cp r15, floor
	breq end_show_floor
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


;epilogue
end_show_floor:
	out PORTG, XH
	out PORTC, XL

pop XH
pop XL
pop r15
pop floor
pop YH
pop YL
ret








exit:
	rjmp exit

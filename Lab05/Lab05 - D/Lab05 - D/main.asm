;
; Lab05 - A.asm
;
; Created: 20/03/2019 9:02:50 PM
; Author : andre
;

.include "m2560def.inc"
.def temp = r16
.def zero = r3
.def one = r4
.def floor = r20
.def direction = r21
.def req_floor = r22

.equ clock_speed = 200//782

.macro clear
	sts @0, zero
	sts @0+1, zero
.endmacro

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


Requests:
	.db 4,7,9,2,1
; Replace with your application code
RESET:
	ldi r16, low(RAMEND)	; Init stack frame
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

	clr zero				; zero
	clr one					
	inc one					; one
	ser r16
	out DDRC, r16			; Enabling Timer interrupt
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
	in temp, SREG		; stack frame for timer interrupt handler
	push temp
	push YH
	push YL
	push r25
	push r24

	lds r24, Count		; increment count
	lds r25, Count + 1
	adiw r25:r24, 1
	
	cpi r24, low(clock_speed)	; compare with clock speed to check if 1/10 of second has passed
	ldi temp, high(clock_speed)
	cpc r25, temp
	brne Not_second
	lds r24, Seconds			; increment seconds every 1/10 of second
	lds r25, Seconds+1


	adiw r25:r24, 1
	sts Seconds, r24
	sts Seconds+1, r25
	clear Count
	rjmp End_I

Not_second:
	sts Count, r24 
	sts Count+1, r25 

End_I:
; Epilogue
	pop r25
	pop r24
	pop YL
	pop YH
	pop temp
	out SREG, temp
	reti

main:
	ldi floor, 1 ;this is the FlooR <-------------------------------
	ldi ZL, low(Requests<<1)
	ldi ZH, high(Requests<<1)
	lpm req_floor, Z		; Load requested floor

	ldi direction, 1
	rcall show_floor
	ldi r18, 1


wait_loop:


	cpi direction, 0
	breq choose_direction


check_count:
	lds r24, Seconds
	cp floor, req_floor		; compare current floor with requested floor
	breq stop_here			; stop here if current = requested
	cpi r24, 20				; check for 2 seconds
	brne wait_loop
	rjmp choose_direction			; moving after 2 seconds

stop_here:
	cp r24, r18				; compare flash counter with timer
	brne no_flash
	push floor				; push floor to use in comparison later
	ldi temp, 1
	cpse one, temp			; check if one is 1
	subi floor, 1
	neg one
	rcall show_floor
	pop floor
	subi r18, -1

no_flash:	
	cpi r24, 50				; check for 5 seconds
	brne wait_loop
	mov floor, req_floor	; put required floor back into floor for comparison
	adiw Z, 1
	lpm req_floor, Z		; load requested floor


/*	cpi floor, 10			
	brne next
	ldi direction, -1
	rjmp move
next:
	cpi floor, 1
	brne move
	ldi direction, 1
move:*/
choose_direction:
	clear Seconds			; clear seconds after comparison passed
	ldi r18, 1				; reset flash counter
	cpi floor, 1
	breq go_up
	cpi floor, 10
	breq go_down
	cp req_floor, floor
	brlt go_down
	cp floor, req_floor
	brlt go_up 

	rjmp move
go_up:
	ldi direction, 1
	rjmp move
go_down:
	ldi direction, -1
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

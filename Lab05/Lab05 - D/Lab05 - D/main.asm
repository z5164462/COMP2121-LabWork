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

.equ clock_speed = 782
.equ wait_speed = 1

.macro clear
	sts @0, zero
	sts @0+1, zero
.endmacro

.dseg
Count:
	.byte 2
Seconds:
	.byte 2
Moving_flag:
	.byte 1
Debounce:
	.byte 1
Wait_duration:
	.byte 1

.cseg


.org 0x0000 ; reset adress
	jmp RESET

.org INT0addr
	jmp EXT_INT0
.org INT1addr
	jmp EXT_INT1

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
	ori r16, (1<<INT1)
	out EIMSK, r16
	
	ldi temp, 0b00000000
	out TCCR0A, temp
	ldi temp, 0b00000010
	out TCCR0B, temp
	ldi temp, 1<<TOIE0
	sts TIMSK0, temp


	sei
	jmp main 

EXT_INT0:
	push temp
	in temp, SREG
	push temp
	push r24
	push r25
	
	lds temp, Moving_flag
	cpi temp, 1
	breq INT0_END

	lds r24, Seconds
	lds r25, Debounce
	subi r25, -1
	cp r25, r24
	brge INT0_END

	ldi temp, 1
	out PORTG, temp
/*	ldi temp, 0
	sts Wait_duration, temp*/

INT0_END:
	sts Debounce, r24
	pop r25
	pop r24
	pop temp
	out SREG, temp
	pop temp
	reti


EXT_INT1:
	push temp
	in temp, SREG
	push temp
	
	lds temp, Moving_flag
	cpi temp, 1
	breq INT1_END
	
	lds r24, Seconds
	lds r25, Debounce
	subi r25, -1
	cp r25, r24
	brge INT1_END
	
/*
	lds temp, Wait_duration
	subi temp, -100
	sts Wait_duration, temp*/
	ldi temp, 2
	out PORTG, temp

INT1_END:
	pop temp
	out SREG, temp
	pop temp
	reti




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
	ldi r18, wait_speed
	ldi temp, 1
	sts Moving_flag, temp
	ldi temp, 30
	sts Wait_duration, temp


wait_loop:


	cpi direction, 0
	breq choose_direction


check_count:
	lds r24, Seconds
	cp floor, req_floor		; compare current floor with requested floor
	breq stop_here			; stop here if current = requested
	cpi r24, 20				; check for 2 seconds
	brlt wait_loop
	rjmp choose_direction			; moving after 2 seconds

stop_here:
	clr temp
	sts Moving_flag, temp
	cp r24, r18				; compare flash counter with timer
	brne no_flash
	push floor				; push floor to use in comparison later
	ldi temp, 1
	cpse one, temp			; check if one is 1
	subi floor, 1
	neg one
	rcall show_floor
	pop floor
	subi r18, -wait_speed

no_flash:
	lds temp, Wait_duration	
	cp r24, temp			; check for wait_duration (default is 5 sec unless extended)
	brlt wait_loop
	mov floor, req_floor	; put required floor back into floor for comparison
	adiw Z, 1
	lpm req_floor, Z		; load requested floor
	ldi temp, 1
	sts Moving_flag, temp


choose_direction:
	clear Seconds			; clear seconds after comparison passed
	ldi temp, 30
	sts Wait_duration, temp
	ldi r18, wait_speed				; reset flash counter
	cpi floor, 1
	breq go_up
	cpi floor, 30
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

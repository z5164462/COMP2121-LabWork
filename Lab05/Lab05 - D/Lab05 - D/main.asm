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
.def data = r23



.equ clock_speed = 782
.equ wait_speed = 1

//from LCD example

.macro do_lcd_command
	ldi r16, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro
.macro do_lcd_data
	mov r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro


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
	.byte 2

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

//from LCD-example LCD setup
ser r16
	out DDRF, r16
	out DDRA, r16
	clr r16
	out PORTF, r16
	out PORTA, r16

	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off?
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink

	ldi r17, 48
	do_lcd_data r17
	ldi r17,'e'
	do_lcd_data r17
	ldi r17, 'l'
	do_lcd_data r17
	do_lcd_data r17
	ldi r17, 'o'
	do_lcd_data r17

	ldi r17, 10
	do_lcd_data r17

	ldi r17, 48
	do_lcd_data r17
	ldi r17,'e'
	do_lcd_data r17
	ldi r17, 'l'
	do_lcd_data r17
	do_lcd_data r17
	ldi r17, 'o'
	do_lcd_data r17





	sei
	jmp main 

//-------------------------------------------------------------

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

	clr YL
	clr YH



INT0_END:
	sts Debounce, r24
	pop r25
	pop r24
	pop temp
	out SREG, temp
	pop temp
	reti
//-------------------------------------------------------------

EXT_INT1:
	push temp
	in temp, SREG
	push temp
	push r24
	push r25
	
	lds temp, Moving_flag
	cpi temp, 1
	breq INT1_END

	lds r24, Seconds
	lds r25, Debounce+1
	subi r25, -1
	cp r25, r24
	brge INT1_END


	adiw Y, 30
	ldi temp, 'E'
	do_lcd_data temp



INT1_END:
	sts Debounce+1, r24
	pop r25
	pop r24
	pop temp
	out SREG, temp
	pop temp
	reti


//-------------------------------------------------------------

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

//-------------------------------------------------------------



main:
	ldi floor, 3 ;this is the FlooR <-------------------------------
	ldi ZL, low(Requests<<1)
	ldi ZH, high(Requests<<1)
	lpm req_floor, Z		; Load requested floor

	ldi direction, 1
	rcall show_floor
	ldi r18, wait_speed
	ldi r19, 0
	ldi temp, 1
	sts Moving_flag, temp
	ldi YL, 50
	ldi YH, 0
	do_lcd_command 0b00000001
	mov temp, floor
	subi temp, -48

wait_loop:

/*	lds r24, Seconds
	lds r25, Seconds+1
	adiw r25:r24, 1
	sts Seconds, r24
	sts Seconds+1, r25*/




check_count:
	lds XL, Seconds
	lds XH, Seconds+1
	cp floor, req_floor		; compare current floor with requested floor
	breq stop_here			; stop here if current = requested
	cpi XL, 20				; check for 2 seconds
	brlt wait_loop
	rjmp choose_direction			; moving after 2 seconds

stop_here:




	clr temp
	sts Moving_flag, temp
	cp XL, r18				; compare flash counter with timer
	cpc XH, r19
	brne no_flash
	push floor				; push floor to use in comparison later
	ldi temp, 1
	cpse one, temp			; check if one is 1
	subi floor, 1
	neg one
	rcall show_floor
	pop floor
	subi r18, -wait_speed
	sbci r19, -wait_speed

no_flash:

	cp XL, YL
	cpc XH, YH			; check for wait_duration (default is 5 sec unless extended)
	brlt wait_loop
	mov floor, req_floor	; put required floor back into floor for comparison
	adiw Z, 1
	lpm req_floor, Z		; load requested floor
	ldi temp, 1
	sts Moving_flag, temp


choose_direction:
	clear Seconds			; clear seconds after comparison passed
	
	ldi YL, 50
	ldi YH, 0

	
	ldi r18, wait_speed				; reset flash counter
	ldi r19, 0
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
	clr XL
	clr XH



	add floor, direction
	do_lcd_command 0b00000001
	mov temp, floor
	subi temp, -48
	do_lcd_data temp

	rcall show_floor
	rjmp wait_loop



	
	
end_main:	
	rjmp end_main

//-------------------------------------------------------------


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


//LCD-example functions

.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

.macro lcd_set
	sbi PORTA, @0
.endmacro
.macro lcd_clr
	cbi PORTA, @0
.endmacro

;
; Send a command to the LCD (r16)
;

lcd_command:
	out PORTF, r16
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	ret

lcd_data:
	out PORTF, r16
	lcd_set LCD_RS
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	lcd_clr LCD_RS
	ret

lcd_wait:
	push r16
	clr r16
	out DDRF, r16
	out PORTF, r16
	lcd_set LCD_RW
lcd_wait_loop:
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	in r16, PINF
	lcd_clr LCD_E
	sbrc r16, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser r16
	out DDRF, r16
	pop r16
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret






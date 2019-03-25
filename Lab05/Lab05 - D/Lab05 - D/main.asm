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

/*;taken from lectures

; Register data stores value to be written to the LCD; Port D is output and connects to LCD; Port A controls the LCD.; Assume all other labels are pre-defined.
.macro lcd_write_com 
	out PORTD, data; set the data port's value up
	clr temp
	out PORTA, temp; RS = 0, RW = 0 for a command write
	nop; delay to meet timing (Set up time)
	sbi PORTA, LCD_E; turn on the enable pin
	nop; delay to meet timing (Enable pulse width)
	nop
	nop
	cbi PORTA, LCD_E; turn off the enable	pin
	nop; delay to meet timing (Enable cycle time)
	nop
	nop
.endmacro

.macro lcd_write_data
	out PORTD, data; set the data port's value up
	ldi temp, 1<<LCD_RS
	out PORTA, temp; RS = 0, RW = 0 for a command write
	nop; delay to meet timing (Set up time)
	sbi PORTA, LCD_E; turn on the enable pin
	nop; delay to meet timing (Enable pulse width)
	nop
	nop
	cbi PORTA, LCD_E; turn off the enable	pin
	nop; delay to meet timing (Enable cycle time)
	nop
	nop
.endmacro

.macro lcd_wait_busy
	clr temp
	out DDRD, temp; Make PORTD be an input port for now
	out PORTD, temp
	ldi temp, 1 << LCD_RW
	out PORTA, temp; RS = 0, RW = 1 for a command port read
busy_loop:
	nop; delay to meet set-up time)
	sbi PORTA, LCD_E; turn on the enable pin
	nop; delay to meet timing (Data delay time)
	nop
	nop
	in temp, PIND; read value from LCD
	cbi PORTA, LCD_E; turn off the enable pin
	sbrc temp, LCD_BF; if the busy flag is set
	rjmp busy_loop; repeat command read
	clr temp; else
	out PORTA, temp; turn off read mode,
	ser temp; 
	out DDRD, temp; make PORTD an output port again
.endmacro

.macro delay
loop:
	subi ZL, 1
	sbci ZH, 0
	nop
	nop
	nop
	nop
	brne loop; taken branch takes two cycles.  ; one loop time is 8 cycles = ~1.08us
.endmacro



*/


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

/*//from lectures -- Set up LCD

ldi ZL, low(15000); delay (>15ms)
ldi ZH, high(15000)
delay; Function set command with N = 1 and F = 0; for 2 line display and 5*7 font. The 1st command
ldi data, LCD_FUNC_SET | (1 << LCD_N) 
lcd_write_com 
ldi ZL, low(4100); delay (>4.1 ms)
ldi ZH, high(4100)
delay
lcd_write_com ; 2nd Function set command

ldi ZL, low(100); delay (>100 ns)
ldi ZH, high(100)
delay
lcd_write_com ; 3rd Function set command
lcd_write_com ; Final Function set command
lcd_wait_busy; Wait until the LCD is ready
ldi data, LCD_DISP_OFF
lcd_write_com; Turn Display off
lcd_wait_busy; Wait until the LCD is ready
ldi data, LCD_DISP_CLR
lcd_write_com; Clear Display


lcd_wait_busy; Wait until the LCD is ready; Entry set command with I/D = 1 and S = 0
; Set Entry mode: Increment = yes and Shift = no
ldi data, LCD_ENTRY_SET | (1 << LCD_ID) 
lcd_write_com
lcd_wait_busy; Wait until the LCD is ready
; Display On command with C = 1 and B = 0
ldi data, LCD_DISP_ON | (1 << LCD_C)
lcd_write_com*/


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
	

wait_loop:

/*	lds r24, Seconds
	lds r25, Seconds+1
	adiw r25:r24, 1
	sts Seconds, r24
	sts Seconds+1, r25*/

	cpi direction, 0
	breq choose_direction


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








exit:
	rjmp exit

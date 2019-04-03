;
; Assignment V1.asm
;
; Created: 31/03/2019 9:58:14 AM
; Author : andrew fleming z5164462 anirudh rami z5164466
;


; Replace with your application code

;initial definitions and assignments

.dseg
Queue_len:
	.byte 1

/*
input_array?
floor_array?
*/


.cseg

.org 0x0000
jmp RESET

.def current_floor = r16
.def next_floor = r17
.def input_value = r18
.def lift_status = r19
.def temp1 = r20
.def temp2 = r21 //NOTE: temp1 and temp2 will not reliably hold the data given to them

/*
lift_status is a status register with the bits as flags
b0 = Stopped? 
b1 = Going up?
b2 = Doors Open?
b3 = Doors Opening?
b4 = Floors to come?
b5 = Flash on?
b6 = Emergency?
b7 = Halted?
*/
.equ stopped =		0b00000001
.equ goingUp =		0b00000010
.equ doorsOpen =	0b00000100
.equ opening =		0b00001000
.equ floorsToCome = 0b00010000
.equ flashOn =		0b00100000
.equ emergency =	0b01000000
.equ halted =		0b10000000

//LCD interface constants
.equ PORTLDIR = 0xF0			; 0xF0 = 0b11110000 -> Setting PORTA 7:4 as output and 3:0 as input
.equ INITCOLMASK = 0XEF			; 0xEF = 0b11101111 -> Mask to decide which column is selected
.equ INITROWMASK = 0x01			; 0x01 = 0b00000001 -> Mask to check which row is selected
.equ ROWMASK = 0x0F				; 0x0F = 0b00001111 -> To get keyboard output value using an AND operation
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4
.equ l_one = 0b10000000
.equ l_two = 0b11000000







.def TimerL = r28	;YL
.def TimerH = r29	;YH


//Flag checking register  usage check_register_bit stopped
//								breq action
.macro check_register_bit
	mov temp1, lift_status
	andi temp1, @0
	cp temp1, @0
.end macro

//LCD MACROS
.macro do_lcd_command						// LCD commands
	ldi temp1, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro

.macro do_lcd_command_reg					// LCD commands, with registers
	mov temp1, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro

.macro change_line							// change line and cursor position on line
	ldi temp1, @0
	cpi temp1, 2
	breq line_two
	ldi temp2, l_one
	ori temp2, @1
	do_lcd_command_reg temp2
	jmp end_cl
line_two:
	ldi temp2, l_two
	ori temp2, @1
	do_lcd_command_reg temp2
end_cl:
.endmacro

.macro write								// write immediate data to LCD screen
	ldi temp1, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro

.macro write_reg							// write register data to LCD screen
	mov temp1, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro

.macro clear_disp							// clear the LCD Display
	do_lcd_command 0b00000001
.endmacro

.macro lcd_set								// set bit in PORTA
	sbi PORTA, @0
.endmacro
.macro lcd_clr								// clear bit in PORTA
	cbi PORTA, @0
.endmacro















// Function to insert input floor into list 
// parameters Address of queue (X), input_floor (arg1), current_floor (global), direction (b1 of r20, global)
// will set next_floor (global), direction (b1 of r20, global)



insert_request:

//prologue

push SREG

push r7
//r16 current_floor global
//r17 next_floor return value
push r18	//parameter input_value
//r19 lift_status global and return value
push r20	//temp1
push r22

push XL
push XH




clr r7 // counter = 0

cp current_floor, input_value //if the current floor is the input floor, break to end
breq end_insert_request
lds r22, Queue_len

check_register_bit goingUp	// check the goingUp bit
breq down_search	// if zero, sort down
rjmp up_search		//else sort up

//r17 input floor
//r16 len -->r22
//r18 counter --> r7
//r19 ith number
//r20 current floor


//r7 counter r22 len

up_search:
	cp input_value, current_floor			; if input floor < current floor, jump to up_descending_loop, else up_ascending_loop
	brlt up_descending_loop
up_ascending_loop:
	cp r7, r22			; compare counter to len (check if end of list reached)
	breq end_search
	ld temp1, X			; load floor from output array
	cp r17, temp1			; check if input floor already exists
	breq end_insert_request	; quit if it does
	brlo insert_start	; if input floor lower than ith floor, insert 
	cp temp1, current_floor			; compare ith floor to current floor
	brlo insert_start	; if ith < current, insert
	adiw X, 1			; increment output array
	inc r7				; increment counter
	rjmp up_ascending_loop

up_descending_loop:
	cp r7, r22			; compare counter to len (check if end of list reached)
	breq end_search		
	ld temp1, X			; load floor from output array
	cp input_value, temp1			; check if input floor already exists
	breq end_insert_request	; quit if it does
	cp temp1, current_floor			; compare ith floor to current floor
	brlo down_search	; if input floor < current floor and ith floor < current floor jmp to down search
	adiw X, 1			; increment output array
	inc r7				; increment counter
	rjmp up_descending_loop



down_search:
	cp current_floor, input_value			; compare current floor < input floor
	brlt down_ascending_loop
down_descending_loop:
	cp r7, r22			; compare counter to len (check if end of list reached)
	breq end_search		
	ld temp1, X			; load floor from output array
	cp temp1, r17			; check if input floor already exists
	breq end_insert_request	; quit if it does
	brlo insert_start	; if current floor < input floor insert here
	cp current_floor, temp1			; compare current floor to ith floor
	brlo insert_start	
	adiw X, 1			; increment output array
	inc r7				; increment counter
	rjmp down_descending_loop

down_ascending_loop:
	cp r7, r22			; compare counter to len (check if end of list reached)
	breq end_search		
	ld temp1, X			; load floor from output array
	cp temp1, input_value			; check if input floor already exists
	breq end_insert_request	; quit if it does
	cp current_floor, temp1			; compare ith floor to current floor
	brlo up_search		; if ith floor > current and input floor > current floor, jmp to upsearch
	adiw X, 1			; increment output array
	inc r7				; increment counter
	rjmp down_ascending_loop

end_search:
	rjmp insert_start

insert_start:	
	inc r22	;len++			;length of list is now longer
	

insert_loop:
	cp r22, r7				; comparison of index and list length to check if the end of the list has been reached
	breq end_insert_loop	
	st X+, input_value
	mov input_value, temp1
	ld temp1, X
	inc r7
	rjmp insert_loop

end_insert_loop:
end_insert_request:

	sts Queue_len, r22		;store new length back in memory

	;epilogue
	pop XH
	pop XL
	pop r22
	pop r20
	pop r18
	pop r7
	pop SREG
	ret



//LCD Functions

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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// FUNCTIONS
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\
// show_floor - input r22 = floor, output X = binary representation
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\

show_floor:
;prologue
;	push YL
;	push YH
	push floor
	push counter

	clr counter	
	clr XL	; output
	clr XH
loop:
	cp counter, floor
	breq end_show_floor
	lsl XL
	inc XL
	brcs grtr8
	rjmp end_x
grtr8:
	lsl XH
	inc XH
end_x:	
	inc counter
	rjmp loop


;epilogue
end_show_floor:
	out PORTG, XH
	out PORTC, XL

pop counter
pop floor
;pop YH
;pop YL
ret

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\
// convert_to_ascii - input X = floor in binary, output r7 = ascii val
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\

convert_to_ascii:
;prologue
	push floor

	clr r25
	mov r25, floor
	subi r25, -'0'

end_convert:
;epilogue
	pop floor
	ret
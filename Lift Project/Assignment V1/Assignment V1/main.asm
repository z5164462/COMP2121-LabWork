//
// Assignment V1.asm
//
// Created: 31/03/2019 9:58:14 AM
// Author : andrew fleming z5164462 anirudh ramia z5164466
//


// Replace with your application code

//initial definitions and assignments

.dseg
Queue_len:
	.byte 1

Count:
	.byte 2
Seconds:
	.byte 2
Debounce1:
	.byte 1
Debounce2:
	.byte 1
Wait_duration:
	.byte 2
Flash_wait:
	.byte 2
/*
input_array?
floor_array?
*/


.cseg


.def zero = r3
.def one = r4
.def counter = r7
.def current_floor = r16
.def next_floor = r17
.def input_value = r18
.def lift_status = r19
.def temp1 = r20
.def temp2 = r21 //NOTE: temp1 and temp2 will not reliably hold the data given to them
.def arg1 = r22

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
.equ flashUp =		0b00100000  //Flash with all bits, as opposed to the flash with fewer bits on, on the flashDown stroke
.equ emergency =	0b01000000
.equ halted =		0b10000000

//LCD interface constants
.equ PORTLDIR = 0xF0			// 0xF0 = 0b11110000 -> Setting PORTA 7:4 as output and 3:0 as input
.equ INITCOLMASK = 0XEF			// 0xEF = 0b11101111 -> Mask to decide which column is selected
.equ INITROWMASK = 0x01			// 0x01 = 0b00000001 -> Mask to check which row is selected
.equ ROWMASK = 0x0F				// 0x0F = 0b00001111 -> To get keyboard output value using an AND operation
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4
.equ l_one = 0b10000000
.equ l_two = 0b11000000

//clock and flash constants

.equ clock_speed = 781
.equ wait_speed = 1


.def wait_durationL = r28	//YL
.def wait_durationH = r29	//YH


//Flag checking register  usage check_register_bit stopped
//								breq action
.macro check_register_bit
	mov temp1, lift_status
	andi temp1, @0
	cpi temp1, @0
.endmacro

//Clear word macro
.macro clear
	sts @0, zero
	sts @0+1, zero
.endmacro

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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// INTERRUPTS
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\

//-----------------------------
// |  C3   |  C2   |  C1   |  C0   |  R3   |  R2   |  R1   |  R0   |
// |  PL0  |  PL1  |  PL2  |  PL3  |  PL4  |  PL5  |  PL6  |  PL7  |


.org 0x0000 // reset adress
	jmp RESET

.org INT0addr
	jmp EXT_INT0
.org INT1addr
	jmp EXT_INT1

.org OVF0addr
	jmp Timer0OVF


//CSEG MEMORY STORAGE

Requests:
		.db 1,3,5,7,9,0


RESET:
	ldi temp1, low(RAMEND)	// Init stack frame
	out SPL, temp1
	ldi r16, high(RAMEND)
	out SPH, temp1

	clr zero				// zero
	clr one					
	inc one					// one

	ldi temp1, PORTLDIR		//p7-4 outputs, p3-0 inputs
	sts DDRL, temp1
	ser temp1				//send 1 to p3-0 to activate pull up resistors
	sts PORTL, temp1

	out DDRC, temp1			//LED Lower
	out DDRG, temp1			//LED Higher
	out DDRF, temp1			//LCD data
	out DDRA, temp1			//LCD control
	clr temp1
	out PORTF, temp1		
	out PORTA, temp1
	out DDRD, temp1			//Buttons?
	ldi temp1, 0b01010101	//LED testing
	ldi temp2, 0b110001
	out PORTC, temp1		//LED lower
	out PORTG, temp2		//LED higher
	ldi temp1,  0b00001010	//falling edges for interrupts 1 and 0
	sts EICRA, temp1		



	in temp1, EIMSK
	ori temp1, (1<<INT0)
	ori temp1, (1<<INT1)
	out EIMSK, temp1
	
	ldi temp1, 0b00000000
	out TCCR0A, temp1
	ldi temp1, 0b00000010
	out TCCR0B, temp1
	ldi temp1, 1<<TOIE0
	sts TIMSK0, temp1

//from LCD-example LCD setup
	ser temp1
	out DDRF, temp1
	out DDRA, temp1
	clr temp1
	out PORTF, temp1
	out PORTA, temp1

	do_lcd_command 0b00111000 // 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 // 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 // 2x5x7
	do_lcd_command 0b00111000 // 2x5x7
	do_lcd_command 0b00001000 // display off?
	do_lcd_command 0b00000001 // clear display
	do_lcd_command 0b00000110 // increment, no display shift
	do_lcd_command 0b00001110 // Cursor on, bar, no blink

	ldi temp2, 'H'
	write_reg temp2
	ldi temp2,'e'
	write_reg temp2
	ldi temp2, 'l'
	write_reg temp2
	write_reg temp2
	ldi temp2, 'o'
	write_reg temp2
	sei
	jmp main 




EXT_INT0:

	
	check_register_bit stopped  //if not stopped, ignore close door
	brne INT0_END

	check_register_bit opening //if doors opening, ignore close door
	breq INT0_END

//in theory, we should be able to test: check_register_bit, doorsOpen / brne INT0_END

	lds temp2, Debounce1
	cpi temp2, 0
	brne INT0_END

	ldi temp2, 100			//set debounce counter to 100
	sts Debounce1, temp2
	
	
	clr wait_durationL					//set the wait duration to 0
	clr wait_durationH
	sts Wait_duration, wait_durationL
	sts Wait_duration+1, wait_durationH




INT0_END:
	pop temp2
	out SREG, temp2
	pop temp2
	reti



EXT_INT1:
	push temp2
	in temp2, SREG
	push temp2
	
	check_register_bit stopped
	brne INT1_END

	lds temp2, Debounce2
	cpi temp2, 0
	brne INT1_END

	ldi temp2, 100
	sts Debounce2, temp2

	adiw wait_durationH:wait_durationL, 30 //not as simple as this, if closing, stopp closing, if held while waiting, extend until not pressed.
	sts Wait_duration, wait_durationL
	sts Wait_duration+1, wait_durationH

INT1_END:
	pop temp2
	out SREG, temp2
	pop temp2
	reti

Timer0OVF:
	in temp1, SREG		// stack frame for timer interrupt handler
	push temp1
	push r25
	push r24

	lds r24, Count		// increment count
	lds r25, Count + 1
	adiw r25:r24, 1
	
	cpi r24, low(clock_speed)	// compare with clock speed to check if 1/10 of second has passed
	ldi temp1, high(clock_speed)
	cpc r25, temp1
	brne Not_second
	lds r24, Seconds			// increment seconds every 1/10 of second
	lds r25, Seconds+1


	lds temp1, Debounce1		// decrement Debounce counter for INT0
	dec temp1
	sts Debounce1, temp1

	lds temp1, Debounce2		// decrement Debounce counter for INT1
	dec temp1
	sts Debounce2, temp1

	adiw r25:r24, 1
	sts Seconds, r24
	sts Seconds+1, r25
	clear Count
	rjmp End_I

Not_second:
	sts Count, r24 
	sts Count+1, r25 

End_I:
// Epilogue
	pop r25
	pop r24
	pop temp1
	out SREG, temp1
	reti

// Function to insert input floor into list 
// parameters Address of queue (X), input_floor (arg1), current_floor (global), direction (b1 of r20, global)
// will set next_floor (global), direction (b1 of r20, global)




main:
	rjmp main










insert_request:

//prologue
push temp2
in temp2 , SREG
push temp2

push counter
//r16 current_floor global
//r17 next_floor return value
push r18	//parameter input_value
//r19 lift_status global 
push r20	//temp1
push r22

push XL
push XH




clr counter // counter = 0

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
	cp input_value, current_floor			// if input floor < current floor, jump to up_descending_loop, else up_ascending_loop
	brlt up_descending_loop
up_ascending_loop:
	cp counter, r22			// compare counter to len (check if end of list reached)
	breq end_search
	ld temp1, X			// load floor from output array
	cp r17, temp1			// check if input floor already exists
	breq end_insert_request	// quit if it does
	brlo insert_start	// if input floor lower than ith floor, insert 
	cp temp1, current_floor			// compare ith floor to current floor
	brlo insert_start	// if ith < current, insert
	adiw X, 1			// increment output array
	inc counter				// increment counter
	rjmp up_ascending_loop

up_descending_loop:
	cp counter, r22			// compare counter to len (check if end of list reached)
	breq end_search		
	ld temp1, X			// load floor from output array
	cp input_value, temp1			// check if input floor already exists
	breq end_insert_request	// quit if it does
	cp temp1, current_floor			// compare ith floor to current floor
	brlo down_search	// if input floor < current floor and ith floor < current floor jmp to down search
	adiw X, 1			// increment output array
	inc counter				// increment counter
	rjmp up_descending_loop



down_search:
	cp current_floor, input_value			// compare current floor < input floor
	brlt down_ascending_loop
down_descending_loop:
	cp counter, r22			// compare counter to len (check if end of list reached)
	breq end_search		
	ld temp1, X			// load floor from output array
	cp temp1, r17			// check if input floor already exists
	breq end_insert_request	// quit if it does
	brlo insert_start	// if current floor < input floor insert here
	cp current_floor, temp1			// compare current floor to ith floor
	brlo insert_start	
	adiw X, 1			// increment output array
	inc counter				// increment counter
	rjmp down_descending_loop

down_ascending_loop:
	cp counter, r22			// compare counter to len (check if end of list reached)
	breq end_search		
	ld temp1, X			// load floor from output array
	cp temp1, input_value			// check if input floor already exists
	breq end_insert_request	// quit if it does
	cp current_floor, temp1			// compare ith floor to current floor
	brlo up_search		// if ith floor > current and input floor > current floor, jmp to upsearch
	adiw X, 1			// increment output array
	inc counter				// increment counter
	rjmp down_ascending_loop

end_search:
	rjmp insert_start

insert_start:	
	inc r22	//len++			//length of list is now longer
	

insert_loop:
	cp r22, counter				// comparison of index and list length to check if the end of the list has been reached
	breq end_insert_loop	
	st X+, input_value
	mov input_value, temp1
	ld temp1, X
	inc counter
	rjmp insert_loop

end_insert_loop:
end_insert_request:

	sts Queue_len, r22		//store new length back in memory

	//epilogue
	pop XH
	pop XL
	pop r22
	pop r20
	pop r18
	pop counter
	pop temp2
	out SREG, temp2
	pop temp2
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
// 4 cycles per iteration - setup/call-return overhead

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
//prologue
//	push YL
//	push YH
	push current_floor
	push counter

	clr counter	
	clr XL	// output
	clr XH
loop:
	cp counter, current_floor
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


//epilogue
end_show_floor:
	out PORTG, XH
	out PORTC, XL

pop counter
pop current_floor

ret

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\
// convert_to_ascii - parameters input_value, output r25:r24 = ascii val
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\

convert_to_ascii:
//prologue
	push arg1
	push temp1
	clr r24
	clr r25
	mov r24, arg1
	cpi temp1, 10
	breq ZEROE
	brlt DIGIT
ZEROE:					//ie 10
	ldi r25, '1'
	ldi r24, '0'
	rjmp end_convert
DIGIT:					//ie 07
	ldi r25, '0'
	subi r24, -'0'
	rjmp end_convert  
SYMBOL:
//TODO with predefined codes
LETTER:
//Can use Hex for A->D

end_convert:
//epilogue
	pop arg1
	ret


//flashing function
//inputs current_floor(global), lift_status(global)

flash_LED:
push temp1
push temp2
lds temp1, Flash_wait		//value of second counter when flash is next toggled
lds temp2, Flash_wait+1


check_register_bit flashUp
breq flashTrue
rjmp flashFalse	

flashTrue:
	rcall show_floor
	cbr lift_status, flashUp
	rjmp end_flash_LED
flashFalse:
	push current_floor
	dec current_floor
	rcall show_floor
	pop current_floor
	sbr lift_status, flashUp

end_flash_LED:
pop temp2
pop temp1
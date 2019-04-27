;
; Assignment V1.asm
;
; Created: 31/03/2019 9:58:14 AM
; Author : andrew fleming z5164462 anirudh ramia z5164466
;

/*Assumptions !!!!

The Emergency call cannot be cancelled until the lift reaches the first floor
and has completed the opening and closing routinues

Any button press length is counted as a hold for the hold open doors duration
*/


; Replace with your application code

;initial definitions and assignments
.include "m2560def.inc"

.dseg
Queue_len:
    .byte 1
Queue:
	.byte 10
Count:
    .byte 2
Seconds:
    .byte 2
Wait_duration:
    .byte 2
/*Flash_wait:
    .byte 2*/

/*
input_array?
floor_array?
*/

//REGISTER_DEFINITIONS
.cseg


.def zero = r3
.def one = r4
.def flip_flash = r5
.def counter = r7
.def ret1 = r10
.def debounce1 = r11
.def debounce2 = r12
.def lift_status = r13
.def old_floor = r14 
.def current_floor = r16
.def requested_floor = r17
.def input_value = r18

.def temp1 = r20
.def temp2 = r21 //NOTE: temp1 and temp2 will not reliably hold the data given to them
.def arg1 = r22
.def arg2 = r23


/*
lift_status is a status register with the bits as flags
b0 = Stopped?
b1 = Going up?
b2 = Doors Open?
b3 = Doors Opening?
b4 = Doors Closing?
b5 = Flash on?
b6 = Emergency?
b7 = Held?
*/
.equ stopped =   		0b00000001    // is lift stopped?
.equ goingUp =   		0b00000010    // is lift going up?
.equ doorsOpen =		0b00000100    // are the doors open?
.equ opening =   		0b00001000    // are the doors opening?
.equ closing =   		0b00010000    // are the doors closing? if no to open, opening and closing, they closed
.equ flashing =   		0b00100000  // is the LED bar flashing
.equ emergency =		0b01000000    // is there an emergency
.equ held =   			0b10000000    // is the lift held

//LCD interface constants
.equ PORTLDIR = 0xF0   		 ; 0xF0 = 0b11110000 -> Setting PORTA 7:4 as output and 3:0 as input
.equ INITCOLMASK = 0XEF   		 ; 0xEF = 0b11101111 -> Mask to decide which column is selected
.equ INITROWMASK = 0x01   		 ; 0x01 = 0b00000001 -> Mask to check which row is selected
.equ ROWMASK = 0x0F   			 ; 0x0F = 0b00001111 -> To get keyboard output value using an AND operation
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4
.equ l_one = 0b10000000
.equ l_two = 0b11000000

//clock and flash constants

.equ clock_speed = 781
.equ wait_speed = 1





//Flag checking register  usage check_register_bit stopped
//   							 breq action
.macro check_register_bit
	push temp1
    mov temp1, lift_status
    andi temp1, @0
    cpi temp1, @0
	pop temp1
.endmacro

.macro clear_register_bit
	push temp1
    mov temp1, lift_status
	cbr temp1, @0
	mov lift_status, temp1
	pop temp1
.endmacro

.macro set_register_bit
	push temp1
    mov temp1, lift_status
	sbr temp1, @0
	mov lift_status, temp1
	pop temp1
.endmacro


//Clear word macro
.macro clear
    sts @0, zero
    sts @0+1, zero
.endmacro

//LCD MACROS
.macro do_lcd_command   					 // LCD commands
    ldi temp1, @0
    rcall lcd_command
    rcall lcd_wait
.endmacro

.macro do_lcd_command_reg   				 // LCD commands, with registers
    mov temp1, @0
    rcall lcd_command
    rcall lcd_wait
.endmacro

.macro change_line   						 // change line and cursor position on line
	push temp1
	push temp2
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
	pop temp2
	pop temp1
.endmacro

.macro write   							 // write immediate data to LCD screen
    push temp1
	ldi temp1, @0
    rcall lcd_data
    rcall lcd_wait
	pop temp1
.endmacro

.macro write_reg   						 // write register data to LCD screen
    push temp1
	mov temp1, @0
    rcall lcd_data
    rcall lcd_wait
	pop temp1
.endmacro

.macro clear_disp   						 // clear the LCD Display
    do_lcd_command 0b00000001
.endmacro

.macro lcd_set   							 // set bit in PORTA
    sbi PORTA, @0
.endmacro
.macro lcd_clr   							 // clear bit in PORTA
    cbi PORTA, @0
.endmacro
.macro set_motor_speed
	push temp1
	ldi temp1, @0
	sts OCR3BL, temp1
	clr temp1
	sts OCR3BH, temp1	
	pop temp1
.endmacro


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// INTERRUPTS
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\

//-----------------------------
// |  C3   |  C2   |  C1   |  C0   |  R3   |  R2   |  R1   |  R0   |
// |  PL0  |  PL1  |  PL2  |  PL3  |  PL4  |  PL5  |  PL6  |  PL7  |


.org 0x0000 ; reset adress
    jmp RESET

.org INT0addr
    jmp EXT_INT0
.org INT1addr
    jmp EXT_INT1

.org OVF0addr
    jmp Timer0OVF
.org OVF1addr
	jmp Timer1OVF
.org 0x30


//CSEG MEMORY STORAGE

/*requests:
   	 .db 3,2,6*/
divisors:
	 .dw  10, 1

RESET:
    ldi temp1, low(RAMEND)    ; Init stack frame
    out SPL, temp1
    ldi temp1, high(RAMEND)
    out SPH, temp1


	clear_register_bit stopped	;0
	set_register_bit goingUp	;1
	clear_register_bit doorsOpen	;0
	clear_register_bit opening	;0
	clear_register_bit closing	;0
	clear_register_bit flashing	;0
	clear_register_bit emergency	;0
	clear_register_bit held		;0

	ldi current_floor, 1
	

    clr zero   			 ; zero
    clr one   				 
    inc one   				 ; one
	clr flip_flash
	clr ret1

 	ser temp1
	mov debounce1, temp1
	mov debounce2, temp1

	sts Queue_len, zero

    ldi temp1, PORTLDIR   	 ;p7-4 outputs, p3-0 inputs
	ldi temp2, 0b00001111;send 1 to p3-0 to activate pull up resistors
    sts DDRL, temp1
	sts PORTL, temp1

	nop
 			 

	ser temp1
    out DDRC, temp1   		 ;LED Lower
    out DDRG, temp1   		 ;LED Higher
	clr temp1
    out DDRD, temp1   		 ;Buttons?
/*    ldi temp1, 0b01010101    ;LED testing
    ldi temp2, 0
    out PORTC, temp1   	 ;LED lower
    out PORTG, temp2   	 ;LED higher*/

	ldi temp1, 0b00010000
	out DDRE, temp1

    ldi temp1, 0b00101010    ;falling edges for interrupts 2, 1 and 0
    sts EICRA, temp1   	 



    in temp1, EIMSK
	ori temp1, (1<<INT0)
	ori temp1, (1<<INT1)
    ori temp1, (1<<INT2)
    out EIMSK, temp1
    
    ldi temp1, 0b00000000
    out TCCR0A, temp1
    ldi temp1, 0b00000010
    out TCCR0B, temp1
    ldi temp1, 1<<TOIE0
    sts TIMSK0, temp1

	ldi temp1, 0b00000000
    sts TCCR1A, temp1
    ldi temp1, 0b00000010 //8 prescaler
    sts TCCR1B, temp1
    ldi temp1, 1<<TOIE1|1<<ICIE1
    sts TIMSK1, temp1


	ldi temp1, (1<<CS30)
	sts TCCR3B, temp1
	ldi temp1, (1<<COM3B1) | (1<<WGM30)
	sts TCCR3A, temp1
	
//from LCD-example LCD setup
    ser temp1

    out DDRF, temp1
    out DDRA, temp1

    clr temp1
    out PORTF, temp1
    out PORTA, temp1

	

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
	lcd_set 1
	clear_disp
	write 'C'
	write 'u'
	write 'r'
	write 'r'
	write 'e'
	write 'n'
	write 't'
	write ' '
	write 'f'
	write 'l'
	write 'o'
	write 'o'
	write 'r'
	lcd_clr 1

	change_line 2, 0
	
	write 'N'
	write 'e'
	write 'x'
	write 't'
	write ' '
	write 's'
	write 't'
	write 'o'
	write 'p'

	change_line 2, 13
	write 'S'

	rcall show_floor
    sei
    jmp main




EXT_INT0:

	check_register_bit doorsOpen
	brne INT0_END
	clr debounce1

INT0_END:
    reti



EXT_INT1:
	
	check_register_bit closing
	breq RE_OPEN
	check_register_bit doorsOpen
	breq HELD_OPEN
	check_register_bit held
	breq LET_CLOSE
	rjmp INT1_END
RE_OPEN:

	clr debounce2
	rjmp INT1_END
HELD_OPEN:

	clr debounce2
	rjmp INT1_END
LET_CLOSE:

	clr debounce2
	rjmp INT1_END
INT1_END:

    reti

Timer0OVF:
	push temp1
    in temp1, SREG   	 ; stack frame for timer interrupt handler
    push temp1
    push r25
    push r24

    lds r24, Count   	 ; increment count
    lds r25, Count + 1
    adiw r25:r24, 1
    
    cpi r24, low(clock_speed)    ; compare with clock speed to check if 1/10 of second has passed
    ldi temp1, high(clock_speed)
    cpc r25, temp1
    brne Not_second
    lds r24, Seconds   		 ; increment seconds every 1/10 of second
    lds r25, Seconds+1


/*    lds temp1, Debounce1   	 ; decrement Debounce counter for INT0
    dec temp1
    sts Debounce1, temp1

    lds temp1, Debounce2   	 ; decrement Debounce counter for INT1
    dec temp1
    sts Debounce2, temp1*/

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
    pop temp1
    out SREG, temp1
	pop temp1
    reti


Timer1OVF:
	push temp1
	in temp1, SREG
	push temp1
	push XL
	push XH

	//out PORTC, debounce1
	ser temp1
	cp debounce1, temp1
	breq DB2
	ldi temp1, 3
	cp debounce1, temp1
	brge action1
	inc debounce1
	//lcd_set 1	
	rjmp  DB2
action1:
	//lcd_clr 1
	check_register_bit doorsOpen
	brne END_DB1
	clear_register_bit doorsOpen
	set_register_bit closing
	clear Seconds
	//out PORTG, one

END_DB1:
	ser temp1
	mov debounce1, temp1

DB2:
	ser temp1
	cp debounce2, temp1
	brne valid_interrupt2
	rjmp End_timer1
valid_interrupt2:
	ldi temp1, 3
	cp debounce2, temp1
	brge action2
	inc debounce2
	rjmp End_Timer1
action2:
	check_register_bit closing
	breq RE_OPEN_ACTION
	check_register_bit held
	breq LET_CLOSE_ACTION
	check_register_bit doorsOpen
	breq HELD_OPEN_ACTION
	rjmp END_DB2
RE_OPEN_ACTION:
	clear_register_bit closing
	set_register_bit opening
	clear Seconds
	rjmp END_DB2
HELD_OPEN_ACTION:
	//lcd_set 1
	//out PORTG, one
	set_register_bit held
	ldi temp1, 0b00101110    ;falling edges for interrupts 2 and 0  RISING edge for interrupt 1
    sts EICRA, temp1
	rjmp END_DB2 	
LET_CLOSE_ACTION:
	//lcd_set 1
	clear_register_bit held
	clear_register_bit doorsOpen
	out PORTG, one
	set_register_bit closing
    ldi temp1, 0b00101010    ;falling edges for interrupts 2, 1 and 0
    sts EICRA, temp1 
	clear Seconds

END_DB2:
	lcd_clr 1
	ser temp1
	mov debounce2, temp1
	

End_Timer1:
	pop XH
	pop XL
	pop temp1
	out SREG, temp1
	pop temp1
	reti

// Function to insert input floor into list
// parameters Address of queue (X), input_floor (arg1), current_floor (global), direction (b1 of r20, global)
// will set requested_floor (global), direction (b1 of r20, global)




//MAIN:
main:
/*	lds r24, Seconds   		 ; increment seconds every 1/10 of second
    lds r25, Seconds+1		 ;FOR DEBUGGING ONLY
    adiw r25:r24, 5
    sts Seconds, r24
    sts Seconds+1, r25*/
// ---------------------------------------- SCANNING THE KEYPAD \/

	change_line 1, 14
	mov arg1, current_floor
	rcall convert_to_ascii
	change_line 2, 10
	mov arg1, requested_floor
	rcall convert_to_ascii
	change_line 2, 14
	lds arg1, Seconds
	lsr arg1
	lsr arg1
	lsr arg1
	rcall convert_to_ascii
/*	lds arg1, Queue_len
	rcall convert_to_ascii*/
	//display current_floor and requested_floor on LCD
	//requested floor = 0 in Reset (TODO)

	rcall scan				//reading the queue
/*	mov arg1, ret1 
	rcall convert_to_ascii*/ //DEBUGGING
	mov input_value, ret1

/*	if input_value == *
		emergency func*/
	cpi input_value, '*'
	brne add_to_queue
	rcall emergency_func
add_to_queue:
	rcall insert_request
	
// ---------------------------------------- SCANNING THE KEYPAD	/\

// ---------------------------------------- CHECKING STOPPED \/

	check_register_bit stopped		//already stopped
	brne read_queue
	rjmp stop_here

// ---------------------------------------- CHECKING STOPPED /\

// ---------------------------------------- DISPLAYING THE CURRENT FLOOR AND REQUESTED FLOOR \/
read_queue:
	rcall show_floor
/*	ldi temp1, 0
	out PORTG, temp1*/
	
// ---------------------------------------- DISPLAYING THE CURRENT FLOOR AND REQUESTED FLOOR /\

// ---------------------------------------- READING THE QUEUE \/
	ldi requested_floor, 0		//assume queue empty	
	lds temp1, Queue_len		//is the queue empty?
	cpi temp1, 0
	breq to_main


	lds requested_floor, Queue
	cp current_floor, requested_floor	//on correct floor?
	brne check_direction
				
	set_register_bit stopped			//first detection of requested_floor
	set_register_bit opening
	rjmp stop_here
// ---------------------------------------- READING THE QUEUE /\
to_main:
	jmp main
// ---------------------------------------- MOVING BETWEEN FLOORS \/
check_direction:
	brlt direction_up
	brge direction_down
	
direction_up:
	set_register_bit goingUp
	rjmp moving
direction_down:
	clear_register_bit goingUp
	rjmp moving

moving:
	lds r24, Seconds		//2 Seconds passed?
	lds r25, Seconds+1
	cpi r24, 20
	brlt to_main

	clear Seconds
	check_register_bit goingUp
	breq moving_up
	rjmp moving_down


moving_up:
	ldi temp1, 10				//don't increment past 10
	cpse current_floor, temp1
	inc current_floor			//move up a floor
	rcall show_floor
	rjmp main

moving_down:
	ldi temp1, 1
	cpse current_floor, temp1
	dec current_floor
	rcall show_floor
	rjmp main
// ---------------------------------------- MOVING BETWEEN FLOORS /\


// ---------------------------------------- STOPPING AT THE FLOOR \/
stop_here:
	check_register_bit opening
	breq opening_sequence
	check_register_bit doorsOpen
	breq doors_open_sequence
	check_register_bit closing
	breq to_closing_sequence
	rjmp end_main

opening_sequence:
/*	ldi temp1, 1
	out PORTG, temp1*/
	rcall LED_flash
	set_motor_speed 0x2A
	lds r24, Seconds		//1 Second passed?
	lds r25, Seconds+1
	cpi r24, 10
	brge opening_done
	rjmp main
opening_done:
	clear Seconds
	clear_register_bit opening
	set_register_bit doorsOpen
	rjmp main

to_closing_sequence:
	jmp closing_sequence
to_main2:
	jmp main

doors_open_sequence:
/*	ldi temp1, 2
	out PORTG, temp1*/	
	set_motor_speed 0
	lds r24, Seconds		//3 seconds passed?
	lds r25, Seconds+1
	//mov arg1, r24			//LED_flash needs the time as an argument
	rcall LED_flash
	check_register_bit held
	breq to_main2
	cpi r24, 30
	brge doors_open_done
	rjmp main
doors_open_done:
	clear Seconds
	clear_register_bit doorsOpen
	set_register_bit closing	
	rjmp main

closing_sequence:
/*	ldi temp1, 3
	out PORTG, temp1*/
	rcall LED_flash
	set_motor_speed 0x8A
	lds r24, Seconds		//1 Second passed?
	lds r25, Seconds+1
	cpi r24, 10				
	brge closing_done
	rjmp main
closing_done:
	clear Seconds
	//out PORTG, zero //bug testing board

	clear_register_bit closing
	clear_register_bit stopped
	set_motor_speed 0
	rcall shuffle_queue
	rjmp main
// ---------------------------------------- STOPPING AT THE FLOOR /\

// ---------------------------------------- ERROR HANDLING \/
end_main:
	ldi temp1, 0b11001100
	out PORTC, temp1
	rjmp end_main
// ---------------------------------------- ERROR HANDLING /\

// ---------------------------------------- EMERGENCY FUNCTION \/




emergency_func:

	push temp1
	
	in temp1, SREG
	push temp1
	
	lds temp1, Count
	push temp1
	
	lds temp1, Count+1
	push temp1

	lds temp1, Seconds
	push temp1
	
	lds temp1, Seconds+1
	push temp1
	
	push old_floor
	push current_floor
	push r24
	push r25	

	clear_disp
	clear Seconds
	write 'E'
	write 'm'
	write 'e'
	write 'r'
	write 'g'
	write 'e'
	write 'n'
	write 'c'
	write 'y'
	change_line 2, 0
	write 'C'
	write 'a'
	write 'l'
	write 'l'
	write ' '
	write '0'
	write '0'
	write '0'


	mov old_floor, current_floor	   // for restoring original floor

drop_floor_loop:
	rcall Strobe_flash
	rcall show_floor
	cpi current_floor, 1
	breq drop_floor_end
	lds r24, Seconds
	lds r25, Seconds+1
	mov arg1, r24
	cpi r24, 20
	brlt drop_floor_loop
	dec current_floor
	clear Seconds
	rjmp drop_floor_loop
drop_floor_end:

emergency_opening:
	rcall strobe_flash
	rcall LED_flash
	rcall scan
	ldi temp1, '*'
	cp ret1, temp1
	brne continue_e_opening
	clear_register_bit emergency //emergency cancelled
continue_e_opening:

	set_motor_speed 0x2A
	lds r24, Seconds		//1 Second passed?
	lds r25, Seconds+1
	cpi r24, 10
	brge emergency_opening_done
	rjmp emergency_opening
emergency_opening_done:
	clear Seconds


emergency_doors_open:
	rcall strobe_flash
	rcall LED_flash
	rcall scan
	ldi temp1, '*'
	cp ret1, temp1
	brne continue_e_doors_open
	clear_register_bit emergency
	clear Seconds
	rjmp emergency_closing
continue_e_doors_open:
	set_motor_speed 0
	lds r24, Seconds		//3 seconds passed?
	lds r25, Seconds+1
	cpi r24, 30
	brge emergency_doors_open_done
	rjmp emergency_doors_open
emergency_doors_open_done:
	clear Seconds

emergency_closing:
	rcall strobe_flash
	rcall LED_flash
	rcall scan
	ldi temp1, '*'
	cp ret1, temp1
	brne continue_e_closing
	clear_register_bit emergency
continue_e_closing:
	set_motor_speed 0x8A
	lds r24, Seconds		//1 Second passed?
	lds r25, Seconds+1
	cpi r24, 10				
	brge emergency_closing_done
	rjmp emergency_closing
emergency_closing_done:
	clear Seconds

emergency_halt_loop:
	rcall strobe_flash
	rcall show_floor
	rcall scan
	ldi temp1, '*'
	cp ret1, temp1	
	brne continue_e_halt
	clear_register_bit emergency
continue_e_halt:	
	check_register_bit emergency
	breq emergency_halt_loop
	clear Seconds

restore_floor:
	rcall show_floor
	cp current_floor, old_floor
	breq restore_floor_end
	lds r24, Seconds
	lds r25, Seconds+1
	cpi r24, 20
	brlt restore_floor
	inc current_floor
	clear Seconds
	rjmp restore_floor

restore_floor_end:
	clear_disp
	write 'C'
	write 'u'
	write 'r'
	write 'r'
	write 'e'
	write 'n'
	write 't'
	write ' '
	write 'f'
	write 'l'
	write 'o'
	write 'o'
	write 'r'
	lcd_clr 1

	change_line 2, 0
	
	write 'N'
	write 'e'
	write 'x'
	write 't'
	write ' '
	write 's'
	write 't'
	write 'o'
	write 'p'

	change_line 2, 13
	write 'S'
	
	pop r25
	pop r24
	pop current_floor
	pop old_floor

	pop temp1
	sts Seconds+1, temp1

	pop temp1
	sts Seconds, temp1
	
	pop temp1
	sts Count+1, temp1
 	
	pop temp1
	sts Count, temp1

	pop temp1
	out SREG, temp1

	pop temp1
	
	ret

	


	




	


	


	



// ---------------------------------------- EMERGENCY FUNCTION /\


// ---------------------------------------- LCD_FUNCTIONS \/
lcd_command:
    out PORTF, temp1
    rcall sleep_1ms
    lcd_set LCD_E
    rcall sleep_1ms
    lcd_clr LCD_E
    rcall sleep_1ms
    ret

lcd_data:
    out PORTF, temp1
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

.equ F_CPU = 160000		//edited from 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
    push r24
    push r25
    ldi r25, high(DELAY_1MS)
    ldi r24, low(DELAY_1MS)

delayloop_1ms:
    sbiw r25:r24, 1	 //DEBUGGING
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

pause:
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	reti

// ---------------------------------------- LCD_FUNCTIONS

// ---------------------------------------- FUNCTIONS


//INSERT_REQUEST

insert_request:

//prologue
push temp2
in temp2 , SREG
push temp2

push counter
push r16
push r17
//r16 current_floor global
//r17 requested_floor return value

push r18    //parameter input_value
//r19 lift_status global
push r19
push r20    //temp1
push r22
push arg1
push arg2

push XL
push XH

ldi XL, low(Queue)
ldi XH, high(Queue)
lds r22, Queue_len

clr ret1

cpi input_value, 11
brlt under_11
jmp end_insert_loop
under_11:
cpi input_value, 1
brge valid_insert
jmp end_insert_loop
valid_insert:

clr counter // counter = 0
lds r22, Queue_len
cp current_floor, input_value //if the current floor is the input floor, break to end
brne input_continue
jmp end_insert_loop
input_continue:
mov ret1, input_value



check_register_bit goingUp    // check the goingUp bit
breq up_search    // if 1, sort up
rjmp down_search 	 //else sort down



//r7 counter r22 len

up_search:
    cp input_value, current_floor   		 ; if input floor < current floor, jump to up_descending_loop, else up_ascending_loop
    brlt up_descending_loop
up_ascending_loop:
    cp counter, r22   		 ; compare counter to len (check if end of list reached)
    breq end_search
    ld temp1, X   		 ; load floor from output array
    cp input_value, temp1   		 ; check if input floor already exists
    breq jumping_to_end_insert    ; quit if it does
    brlo insert_start    ; if input floor lower than ith floor, insert
    cp temp1, current_floor   		 ; compare ith floor to current floor
    brlo insert_start    ; if ith < current, insert
    adiw X, 1   		 ; increment output array
    inc counter   			 ; increment counter
    rjmp up_ascending_loop

up_descending_loop:
    cp counter, r22   		 ; compare counter to len (check if end of list reached)
    breq end_search   	 
    ld temp1, X   		 ; load floor from output array
    cp input_value, temp1   		 ; check if input floor already exists
    breq end_insert_loop    ; quit if it does
    cp temp1, current_floor   		 ; compare ith floor to current floor
    brlo down_search    ; if input floor < current floor and ith floor < current floor jmp to down search
    adiw X, 1   		 ; increment output array
    inc counter   			 ; increment counter
    rjmp up_descending_loop

jumping_to_end_insert:
rjmp end_insert_loop

down_search:
    cp current_floor, input_value   		 ; compare current floor < input floor
    brlt down_ascending_loop
down_descending_loop:
    cp counter, r22   		 ; compare counter to len (check if end of list reached)
    breq end_search   	 
    ld temp1, X   		 ; load floor from output array
    cp temp1, input_value   		 ; check if input floor already exists
    breq end_insert_loop    ; quit if it does
    brlo insert_start    ; if current floor < input floor insert here
    cp current_floor, temp1   		 ; compare current floor to ith floor
    brlo insert_start    
    adiw X, 1   		 ; increment output array
    inc counter   			 ; increment counter
    rjmp down_descending_loop

down_ascending_loop:
    cp counter, r22   		 ; compare counter to len (check if end of list reached)
    breq end_search   	 
    ld temp1, X   		 ; load floor from output array
    cp temp1, input_value   		 ; check if input floor already exists
    breq end_insert_loop    ; quit if it does
    cp current_floor, temp1   		 ; compare ith floor to current floor
    brlo up_search   	 ; if ith floor > current and input floor > current floor, jmp to upsearch
    adiw X, 1   		 ; increment output array
    inc counter   			 ; increment counter
    rjmp down_ascending_loop

end_search:
    rjmp insert_start

insert_start:    
    inc r22    ;len++   		 ;length of list is now longer
/*	push arg1
	mov arg1, input_value
	rcall convert_to_ascii
	pop arg1*/


insert_loop:
    cp r22, counter   			 ; comparison of index and list length to check if the end of the list has been reached
    breq end_insert_loop    
    ld temp1, X
	st X+, input_value
    mov input_value, temp1
    inc counter
    rjmp insert_loop

end_insert_loop:


    sts Queue_len, r22   	 ;store new length back in memory

    ;epilogue
    pop XH
    pop XL
	pop arg2
	pop arg1
    pop r22
    pop r20
	pop r19
    pop r18
	pop r17
	pop r16
    pop counter
    pop temp2
    out SREG, temp2
    pop temp2
    ret


//SHOW_FLOOR:
show_floor:
;prologue
;    push YL
;    push YH

    push current_floor
    push counter
	push XL
	push XH

    clr counter    
    clr XL    ; output
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


;epilogue
end_show_floor:
    out PORTG, XH
    out PORTC, XL

pop XH
pop XL
pop counter
pop current_floor

ret



//CONVERT_TO_ASCII
convert_to_ascii:
	;prologueg
	push r7
	push r8
	push r9
	push r16
	push r17
	push r18
	push r19
	push r20
	push arg1
	push arg2
	push r24
	push r25
	push ZL
	push ZH



	clr arg2 ///THIS IS ONLY FOR WHEN NUMBERS LESS THAN 255

	;change_line 2, 14
start:
	ldi ZL, low(divisors<<1)
	ldi ZH, high(divisors<<1)
	ldi r20, 0x30		// ascii value for zero
	clr r8
	clr r9
convert_loop:
	ldi temp2, 2
	cp r9, temp2
	breq end_convert
	rjmp divide

	// r19:r18 hold the dividend (numerator)
divide:
    lpm r16, Z+  //divisor (denominator)
	lpm r17, Z+ //^^^
	ldi r24, 0  //quotient
	ldi r25, 0	//^^^
loop_start:
	cpi r16, 0  // not dividing by zero
	breq end_divide
	cp arg1, r16 //check dividend !< divisor i.e. you can still minus
	cpc arg2, r17
	brlo end_loop 
	sub arg1, r16  //dividend = dividend - divisor 
	sbc arg2, r17
	adiw r25:r24, 1 //quotient++	
    rjmp loop_start
end_loop:
	movw r19:r18, arg2:arg1
end_divide:
	
	//r23:r22 holds the remainder, r25:r24 holds the quotient

	movw arg2:arg1, r19:r18 //the remainder moves to the dividend to be divided again
	inc r9
	cpi r24, 0
	//breq check_zero
	mov r7, r20	//r7 holds ASCII val for '0'
	add r7, r24 //r7 holds ASCII val for '0' + remainder

	//st X+, r7 //store ASCII val in next part of data memory

	write_reg r7
	inc r8
	rjmp convert_loop
check_zero:
	cp r8, zero
	breq convert_loop
	write '0'

	rjmp convert_loop



end_convert:
;epilogue
	pop ZH
	pop ZL
	pop r25
	pop r24
	pop arg2
	pop arg1
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	pop r9
	pop r8
	pop r7
	ret



//FLASH_LED
/*//inputs current_floor(global), lift_status(global)

flash_LED:
push temp1
push temp2
lds temp1, Flash_wait   	 //value of second counter when flash is next toggled
lds temp2, Flash_wait+1


check_register_bit flashing
breq flashTrue
rjmp flashFalse    

flashTrue:
    rcall show_floor
    clear_register_bit flashing
    rjmp end_flash_LED
flashFalse:
    push current_floor
    dec current_floor
    rcall show_floor
    pop current_floor
    set_register_bit flashing

end_flash_LED:
pop temp2
pop temp1
ret*/


/*LED_flash:	//Reads in the seconds value in arg1
	push temp1
	in temp1, SREG
	push temp1
	push current_floor
    push arg1
	ror arg1 //rotates the time into the carry
    brcs LED_up	//if old LSB was 1, i.e. odd turn strobe on
    brcc LED_down	//if old LSB was 0, i.e even turn strobe off

LED_up:
    rcall show_floor		//set PORTA bit to 1
	rjmp LED_end
LED_down:
    dec current_floor
	rcall show_floor
	rjmp LED_end
LED_end:
    pop arg1
	pop current_floor
	pop temp1
	out SREG, temp1
	pop temp1
	ret*/

LED_flash:
	push temp1
	in temp1, SREG
	push temp1
	push current_floor
	
	check_register_bit flashing
	breq LED_up
	rjmp LED_down

LED_up:
	rcall show_floor
	clear_register_bit flashing
	rjmp LED_END
LED_down:
	dec current_floor
	rcall show_floor
	set_register_bit flashing

LED_end:
	pop current_floor
	pop temp1
	out SREG, temp1
	pop temp1
	ret



//STROBE_FLASH
Strobe_flash:	//Reads in the seconds value in X
	push temp1
	in temp1, SREG
	push temp1
	cp flip_flash, one
	breq strobe_on	
	rjmp strobe_off

strobe_on:
    lcd_set 1		//set PORTA bit to 1
	dec flip_flash
	rjmp strobe_end
strobe_off:
    lcd_clr 1
	inc flip_flash
	rjmp strobe_end
Strobe_end:
    pop arg1
	pop temp1
	out SREG, temp1
	pop temp1
	ret
	
//SCAN				//return value in arg1
scan:
    push r16 // row
    push r17 // col
    push r18 // rmask // r18 will be used to return the input_value
    push r19 // cmask
    push temp1
    push temp2 
    push arg1 // floor
    push arg2 // floor2
    ldi r19, INITCOLMASK   	 ; load column mask to scan a column
    clr r17


	clr ret1
colloop:
    cpi r17, 4   				 ; check if all columns scanned
    breq scan_end   		 ; restart scan if all cols scanned
    sts PORTL, r19   		 ; scan a column (sts used instead of out since PORTL is in extended I/O space)
    ldi temp1, 0xFF   			 ; slow down scan operation (???? WHY ????)

delay:
    dec temp1
    brne delay

    lds arg1, PINL   		;load current status of PORTL pins(lds must be used instead of in)
    andi arg1, ROWMASK   		 ; and the PINL register with row mask
    cpi arg1, 0xF   			 ; check if any row low
    breq nextcol   			 ; if temp is all 1s (i.e 0xF), then there are now lows
   							 ; if there is a low, find which row it is
    ldi r18, INITROWMASK   	 ; load Row mask
    clr r16

rowloop:
    cpi r16, 4   			; if all rows scanned, jump to next column
    breq nextcol
    mov temp2, arg1  			 
    and temp2, r18   		 ; mask the input with row mask
    breq show   				; if the bit is clear, a key has been pressed
   							 ; eg if a key in row 1 is pressed, temp2 = XXXX1101
   							 ; rmask should equal 00000010
   							 ; when AND is used, result is 00000000 -> button pressed
    inc r16   				 ; move to next row
    lsl r18   				 ; left shift mask to check next row
    jmp rowloop

nextcol:   					 ; jump to next column when row scan over
    lsl r19   				 ; left shift mask to check next col
    inc r17
    jmp colloop

show:
	
    cpi r17, 3   			; if column = 3, a key in column 3 is pressed, which is a 									letter key
    breq scan_end   				 ; we dont need to deal with this for this lab, so go to n_a

    cpi r16, 3   				 ; if row = 3, a key in row 3 has been pressed, which is any 								special character or 0
    breq check_bottom_row				 ; zero is the only key we are worried about, so go to check_zero
	
	mov arg1, r16				; move row to floor
	lsl arg1					; multiply by 2
	add arg1, r16				; add row again, to multiply row by 3
	add arg1, r17				; add col
	subi arg1, -1				; add 1
	jmp end_show

check_bottom_row:
	cpi r17, 0
	breq asterisk

	cpi r17, 1
	ldi arg1, 10
	breq end_show

	jmp scan_end

asterisk:
	ldi arg1, '*'

	
end_show:

	mov ret1, arg1


scan_end:
/*	ldi temp1, 3		//ONLY FOR DEBUGGING
	mov ret1, temp1*/
/*	cpse ret1, zero
	out PORTG, one*/
	pop arg2
	pop arg1
	pop temp2
	pop temp1
	pop r19
	pop r18
	pop r17
	pop r16

	ret
																		
//FUNCTION shuffle_queue

shuffle_queue:
	push ZL
	push ZH
	push temp1
	push temp2
	push arg1
	push arg2
	push counter
	
	ldi ZL, low(Queue)
	ldi ZH, high(Queue)

	lds temp1, Queue_len
	add ZL, temp1
	adc ZH, zero

	sbiw Z, 1
	clr counter
	clr arg1
shuffle_loop:
	//Z points to the end of the queue
	lds temp1, Queue_len
	cp counter, temp1
	breq shuffle_loop_end
	ld arg2, Z
	st Z, arg1
	mov arg1, arg2
	inc counter
	sbiw Z, 1
	rjmp shuffle_loop
shuffle_loop_end:
	lds temp1, Queue_len
	dec temp1
	sts Queue_len, temp1


	pop counter
	pop arg2
	pop arg1
	pop temp2
	pop temp1
	pop ZH	
 	pop ZL

	ret




	


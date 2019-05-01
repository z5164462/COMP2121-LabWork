;
; Assignment V1.asm
;
; Created: 31/03/2019 9:58:14 AM
; Author : andrew fleming z5164462 anirudh ramia z5164466
;

/*Assumptions !!!!

The Emergency call cannot be cancelled until the lift reaches the first floor
and has completed the opening and closing routinues

The doors for the emergency will be open for 7 seconds
If asterisk is pressed during emergency, during opening, doors Open and closing, the lift will cancel the emergency as soon as the doors have closed


Any button press length is counted as a hold for the hold open doors duration
*/


; Replace with your application code

; initial definitions and assignments
.include "m2560def.inc"

; Data Memory allocations
.dseg
Queue_len:
    .byte 1
Queue:
	.byte 10
Count:
    .byte 2
Seconds:
    .byte 2


//Register definitions
.cseg


.def zero = r3					; holds the value zero
.def one = r4					; holds the value one
.def flip_flash = r5			; used to increment the counter for the LED and strobe flashing
.def counter = r7				; generic counter
.def ret1 = r10					; used as the return value for functions
.def debounce1 = r11			; used as the debounce counter for INT0
.def debounce2 = r12			; used as the debounce counter for INT1
.def lift_status = r13			; set of lift status flag, see below
.def old_floor = r14 			; used to restore the floor from the emergency function
.def current_floor = r16		; holds the current floor
.def requested_floor = r17		; holds the requested floor
.def input_value = r18			; takes the input value from the keypad
.def temp1 = r20				; temporary register 1
.def temp2 = r21				; temporary register 2
.def arg1 = r22					; function argument register 1
.def arg2 = r23					; function argument register 2


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


.equ stopped =   		0b00000001		; is lift stopped?
.equ goingUp =   		0b00000010		; is lift going up?
.equ doorsOpen =		0b00000100		; are the doors open?
.equ opening =   		0b00001000		; are the doors opening?
.equ closing =   		0b00010000		; are the doors closing? if no to open, opening and closing, they closed
.equ flashing =   		0b00100000		; is the LED bar flashing
.equ emergency =		0b01000000		; is there an emergency
.equ held =   			0b10000000		; is the lift held

//LCD interface constants
.equ PORTLDIR = 0xF0   			; 0xF0 = 0b11110000 -> Setting PORTA 7:4 as output and 3:0 as input
.equ INITCOLMASK = 0XEF   		; 0xEF = 0b11101111 -> Mask to decide which column is selected
.equ INITROWMASK = 0x01   		; 0x01 = 0b00000001 -> Mask to check which row is selected
.equ ROWMASK = 0x0F   			; 0x0F = 0b00001111 -> To get keyboard output value using an AND operation
.equ LCD_RS = 7					; LCD bit numbers
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4
.equ l_one = 0b10000000			; Used to set LCD to line one
.equ l_two = 0b11000000			; and line two


.equ clock_speed = 781			; 781 clock cycles ~= 0.1 seconds


// Lift Status Macros

//Lift Flag checking  
//usage check_register_bit flagName
//	breq action
.macro check_register_bit
	push temp1
    mov temp1, lift_status
    andi temp1, @0
    cpi temp1, @0
	pop temp1
.endmacro

//Lift Flag clearing
//usage clear_register_bit flagName
.macro clear_register_bit
	push temp1
    mov temp1, lift_status
	cbr temp1, @0
	mov lift_status, temp1
	pop temp1
.endmacro

//Lift Flag setting
//usage set_register_bit flagName
.macro set_register_bit
	push temp1
    mov temp1, lift_status
	sbr temp1, @0
	mov lift_status, temp1
	pop temp1
.endmacro


//Clear Word macro
.macro clear
    sts @0, zero
    sts @0+1, zero
.endmacro

//LCD MACROS
.macro do_lcd_command   					 ; LCD commands
    ldi temp1, @0
    rcall lcd_command
    rcall lcd_wait
.endmacro

.macro do_lcd_command_reg   				 ; LCD commands, with registers
    mov temp1, @0
    rcall lcd_command
    rcall lcd_wait
.endmacro

.macro change_line   						 ; change line and cursor position on line
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

.macro write   								; write immediate data to LCD screen
    push temp1
	ldi temp1, @0
    rcall lcd_data
    rcall lcd_wait
	pop temp1
.endmacro

.macro write_reg   							; write register data to LCD screen
    push temp1
	mov temp1, @0
    rcall lcd_data
    rcall lcd_wait
	pop temp1
.endmacro

.macro clear_disp   						 ; clear the LCD Display
    do_lcd_command 0b00000001
.endmacro

.macro lcd_set   							 ; set bit in PORTA
    sbi PORTA, @0
.endmacro
.macro lcd_clr   							 ; clear bit in PORTA
    cbi PORTA, @0
.endmacro

//NOTE lcd_set 1 and lcd_clr 1 are also used to control the strobe light as the strobe is connected to Port A Pin 1

//Set the PWM trigger level, which controls the motor speed
.macro set_motor_speed
	push temp1
	ldi temp1, @0
	sts OCR3BL, temp1
	clr temp1
	sts OCR3BH, temp1	
	pop temp1
.endmacro


//Interrupt vectors

.org 0x0000			; reset address
    jmp RESET
.org INT0addr		; interrupt 1 address
    jmp EXT_INT0
.org INT1addr		; interrupt 2 address
    jmp EXT_INT1
.org OVF0addr		; timer 0 interrupt
    jmp Timer0OVF
.org OVF1addr
	jmp Timer1OVF	; timer 1 interrupt
.org 0x30


divisors:			; used to convert 2 digit numbers to ascii
	 .dw  10, 1


// ---------------------------------------- RESET \/

RESET:
    ldi temp1, low(RAMEND)    ; Init stack frame
    out SPL, temp1
    ldi temp1, high(RAMEND)
    out SPH, temp1


	clear_register_bit stopped		; 0
	set_register_bit goingUp		; 1
	clear_register_bit doorsOpen	; 0
	clear_register_bit opening		; 0
	clear_register_bit closing		; 0
	clear_register_bit flashing		; 0
	clear_register_bit emergency	; 0
	clear_register_bit held			; 0

	ldi current_floor, 1
	

    clr zero   				; zero
    clr one   				 
    inc one   				; one
	clr flip_flash			
	clr ret1

 	ser temp1				; set debounce1 and debounce2 to max value
	mov debounce1, temp1	; NOTE the interrupt counter only counts when its not the max value
	mov debounce2, temp1

	sts Queue_len, zero

    ldi temp1, PORTLDIR   	; p7-4 outputs, p3-0 inputs
	ldi temp2, 0b00001111	; send 1 to p3-0 to activate pull up resistors
    sts DDRL, temp1
	sts PORTL, temp1

 			 

	ser temp1
    out DDRC, temp1   			; LED Lower
    out DDRG, temp1   			; LED Higher
	
	clr temp1
    out DDRD, temp1   			; Buttons

	ldi temp1, 0b00010000
	out DDRE, temp1				; PWM generator

    ldi temp1, 0b00101010		; falling edges for interrupts 2, 1 and 0
    sts EICRA, temp1   										



    in temp1, EIMSK				; enable Interrupts 0,1,2
	ori temp1, (1<<INT0)
	ori temp1, (1<<INT1)
    ori temp1, (1<<INT2)
    out EIMSK, temp1
    
    ldi temp1, 0b00000000				; Normal mode for clock 0
    out TCCR0A, temp1
    ldi temp1, 0b00000010				; 8 Prescaler
    out TCCR0B, temp1
    ldi temp1, 1<<TOIE0					; enable Timer interrupt
    sts TIMSK0, temp1

	ldi temp1, 0b00000000				; Normal mode for clock 1
    sts TCCR1A, temp1
    ldi temp1, 0b00000010				; 8 prescaler
    sts TCCR1B, temp1
    ldi temp1, 1<<TOIE1|1<<ICIE1		; enable Timer interrupt
    sts TIMSK1, temp1

	ldi temp1, (1<<COM3B1) | (1<<WGM30)	; toggle OC3B on compare match, PWM Phase and frequency correct
	sts TCCR3A, temp1
	ldi temp1, (1<<CS30)				; no prescaling
	sts TCCR3B, temp1

	
//from LCD-example LCD setup
    ser temp1
    out DDRF, temp1				; set Port A and Port F as outputs				
    out DDRA, temp1

    clr temp1
    out PORTF, temp1
    out PORTA, temp1

    do_lcd_command 0b00111000	; 2x5x7
    rcall sleep_5ms
    do_lcd_command 0b00111000	; 2x5x7
	rcall sleep_1ms
    do_lcd_command 0b00111000	; 2x5x7
    do_lcd_command 0b00111000	; 2x5x7
    do_lcd_command 0b00001000	; display off?
    do_lcd_command 0b00000001	; clear display
    do_lcd_command 0b00000110	; increment, no display shift
    do_lcd_command 0b00001110	; Cursor on, bar, no blink

	clear_disp					; setup initial display
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

	rcall show_floor
    sei

    jmp main

// ---------------------------------------- RESET /\


// ---------------------------------------- PUSH BUTTON 0 Interrupt Handler \/

EXT_INT0:
	check_register_bit doorsOpen	; only accept button press if doors are open
	brne INT0_END
	clr debounce1					; clear the debounce counter
INT0_END:
    reti

// ---------------------------------------- PUSH BUTTON 0 Interrupt Handler /\





// ---------------------------------------- PUSH BUTTON 1 Interrupt Handler \/

EXT_INT1:
	check_register_bit closing		; accept button press if doors are closing
	breq RE_OPEN
	check_register_bit doorsOpen	; accept button press if doors are opening
	breq HELD_OPEN
	check_register_bit held			; accept button release if button is held
	breq LET_CLOSE
	rjmp INT1_END
RE_OPEN:
	clr debounce2					; clear debounce counter
	rjmp INT1_END
HELD_OPEN:
	clr debounce2
	rjmp INT1_END
LET_CLOSE:
	clr debounce2
	rjmp INT1_END
INT1_END:
    reti

// ---------------------------------------- PUSH BUTTON 1 Interrupt Handler /\

// ---------------------------------------- TIMER0 Interrupt Handler \/
Timer0OVF:
	push temp1
    in temp1, SREG   				; stack frame for timer interrupt handler
    push temp1
    push r25
    push r24

    lds r24, Count   				; increment count
    lds r25, Count + 1
    adiw r25:r24, 1
    
    cpi r24, low(clock_speed)		; compare with clock speed to check if 1/10 of second has passed
    ldi temp1, high(clock_speed)
    cpc r25, temp1
    brne Not_second
    

	lds r24, Seconds   				; increment seconds every 1/10 of second
    lds r25, Seconds+1
    adiw r25:r24, 1
    sts Seconds, r24
    sts Seconds+1, r25
    
	clear Count						; clear Count
    rjmp End_I

Not_second:							; increment Count
    sts Count, r24
    sts Count+1, r25

End_I:
    pop r25
    pop r24
    pop temp1
    out SREG, temp1
	pop temp1
    reti

// ---------------------------------------- TIMER0 Interrupt Handler /\


// ---------------------------------------- TIMER1 Interrupt Handler \/
Timer1OVF:
	push temp1
	in temp1, SREG
	push temp1
	push XL
	push XH

	ldi temp1, 5
	cp temp1, flip_flash				; check if the flash counter is up to 5
	breq invert_flash					; if yes then invert flashing
	rjmp count_flash
invert_flash:
		check_register_bit flashing		; if the flashing flag is on turn it off
		breq flash_to_zero				; if the flashing flag is off turn it on
		rjmp flash_to_one
	flash_to_zero:
		clear_register_bit flashing
		rjmp invert_end
	flash_to_one:
		set_register_bit flashing
	invert_end:
		clr flip_flash
		rjmp flash_continue
count_flash:							; else increment flash counter (flip_flash)
	inc flip_flash	
flash_continue:

	ser temp1							; check if debounce1 is set to max, i.e has not been triggered
	cp debounce1, temp1
	breq DB2
	ldi temp1, 3						; check if it has been 3 clock cycles after the last trigger
	cp debounce1, temp1
	brge action1
	inc debounce1						; if not increment debounce1
	rjmp  DB2
action1:
	check_register_bit doorsOpen		; if doors are open
	brne END_DB1
	clear_register_bit doorsOpen		; clear the doorsOpen flag
	set_register_bit closing			; set the closing flag
	clear Seconds

END_DB1:
	ser temp1							; set debounce1 back to max, mark as untriggered
	mov debounce1, temp1

DB2:									
	ser temp1							; check if debounce2 is set to max, i.e has not been triggered
	cp debounce2, temp1
	brne valid_interrupt2
	rjmp End_timer1
valid_interrupt2:
	ldi temp1, 3						; check if it has been 3 clock cycles after the last trigger
	cp debounce2, temp1
	brge action2
	inc debounce2						; if not increment debounce2
	rjmp End_Timer1
action2:
	check_register_bit closing			; if currently closing, re-open
	breq RE_OPEN_ACTION
	check_register_bit held				; if button is currently held, let close
	breq LET_CLOSE_ACTION
	check_register_bit doorsOpen		; if doors are open, hold them open
	breq HELD_OPEN_ACTION
	rjmp END_DB2
RE_OPEN_ACTION:
	clear_register_bit closing			; to re-open, clear the closing flag
	set_register_bit opening			; and set the opening flag
	clear Seconds
	rjmp END_DB2
HELD_OPEN_ACTION:
										; to start holding the door
	set_register_bit held				; set the held flag and wait for a rising edge
	ldi temp1, 0b00101110				; falling edges for interrupts 2 and 0  RISING edge for interrupt 1
    sts EICRA, temp1
	rjmp END_DB2 	
LET_CLOSE_ACTION:
										; to stop holding the door
	clear_register_bit held				; clear the held flag
	clear_register_bit doorsOpen		; clear the doors open flag
	set_register_bit closing			; set the closing flag
    ldi temp1, 0b00101010				; falling edges for interrupts 2, 1 and 0
    sts EICRA, temp1 
	clear Seconds

END_DB2:
	lcd_clr 1
	ser temp1							; set debounce2 to max, to mark as untriggered
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

// ---------------------------------------- TIMER1 Interrupt Handler /\



//MAIN:
main:

// ---------------------------------------- SCANNING THE KEYPAD \/

	change_line 1, 14						
	mov arg1, current_floor					; write current floor on display
	rcall convert_to_ascii
	change_line 2, 10						; write requested floor on display
	mov arg1, requested_floor
	rcall convert_to_ascii
/*	change_line 2, 14
	lds arg1, Seconds
	rcall convert_to_ascii*/
	rcall scan								; read keypad 

	mov input_value, ret1					; move the return value from the keypad to input_value


	cpi input_value, '*'					; check if input value is an asterisk
	brne add_to_queue						; if its not, its a digit and add to request queue
	set_register_bit emergency				; if it is, set emergency bit
	rcall emergency_func					; and call emergency function
add_to_queue:
	rcall insert_request					; for numbers, call insert request to add to queue
	
// ---------------------------------------- SCANNING THE KEYPAD	/\

// ---------------------------------------- CHECKING STOPPED \/

	check_register_bit stopped				; check if lift is stopped
	brne read_queue							; if not, read queue and move floors
	rjmp stop_here							; else stop at floor

// ---------------------------------------- CHECKING STOPPED /\

// ---------------------------------------- DISPLAYING THE CURRENT FLOOR AND REQUESTED FLOOR \/
read_queue:
	rcall show_floor						; display floor

	
// ---------------------------------------- DISPLAYING THE CURRENT FLOOR AND REQUESTED FLOOR /\

// ---------------------------------------- READING THE QUEUE \/
	
	ldi requested_floor, 0					; assume queue empty
	lds temp1, Queue_len					; is the queue empty?
	cpi temp1, 0							
	breq to_main							; if queue_len is 0, go to main

	
	lds requested_floor, Queue				; load first floor from queue to requested floor
	cp current_floor, requested_floor		; on correct floor?
	brne check_direction
				
	set_register_bit stopped				; first detection of requested_floor
	set_register_bit opening				; door is being opened on floor
	rjmp stop_here
// ---------------------------------------- READING THE QUEUE /\
to_main:
	jmp main
// ---------------------------------------- MOVING BETWEEN FLOORS \/
check_direction:
	brlt direction_up						; check if lift is moving up
	brge direction_down						; or down
	
direction_up:
	set_register_bit goingUp				; if going up, set goingup bit and jump to move
	rjmp moving

direction_down:
	clear_register_bit goingUp				; if going down, clear goingup bit and jump to move
	rjmp moving

moving:
	lds r24, Seconds						; load seconds
	lds r25, Seconds+1
	cpi r24, 20								; if not stopping, wait 2 seconds at each floor before moving
	ldi temp1, high(0)
	cpc r25, temp1 

	brlt to_main
	//out PORTG, one
	clear Seconds
	check_register_bit goingUp				; check going up bit to decide direction
	breq moving_up
	rjmp moving_down


moving_up:
	ldi temp1, 10							; don't increment past 10
	cpse current_floor, temp1
	inc current_floor						; move up a floor
	rcall show_floor						; update display
	rjmp main

moving_down:
	ldi temp1, 1							; dont decrement below 1
	cpse current_floor, temp1
	dec current_floor						; move down a floor
	rcall show_floor						; update display
	rjmp main
// ---------------------------------------- MOVING BETWEEN FLOORS /\


// ---------------------------------------- STOPPING AT THE FLOOR \/
stop_here:
	check_register_bit opening				; check if door should open 
	breq opening_sequence					; if it should, start opening sequence
	check_register_bit doorsOpen			; if door is open, check if door is finished opening
	breq doors_open_sequence				; if it is, jump to doors open sequence
	check_register_bit closing				; check if door should close
	breq to_closing_sequence				; if it should, jump to closing sequence
	rjmp end_main

opening_sequence:
	rcall LED_flash							; flash LED to show doors opening
	set_motor_speed 0x4A					; set open door motor speed
	lds r24, Seconds						; load seconds
	lds r25, Seconds+1
	cpi r24, 10								; door takes 1 second to open
	ldi temp1, high(0)
	cpc r25, temp1
	brge opening_done						; when 1 second has passed, go to opening done
	rjmp main

opening_done:
	clear Seconds
	clear_register_bit opening				; opening is finished
	set_register_bit doorsOpen				; door is open
	rjmp main

to_closing_sequence:
	jmp closing_sequence					; go to closing sequence
to_main2:
	jmp main

doors_open_sequence:
	
	set_motor_speed 0						; when door is open, turn off motor
	lds r24, Seconds						; load seconds
	lds r25, Seconds+1
	//mov arg1, r24			//LED_flash needs the time as an argument
	rcall LED_flash							; flash LED while door is open
	check_register_bit held					; check if button is held
	breq to_main2							; if it is jump to main and restart loop
	cpi r24, 30								; wait 3 seconds on floor with door open
//	ldi temp1, high(0)
//	cpc r25, temp1
	brge doors_open_done					; after 3 seoncds go to doors open done
	rjmp main
doors_open_done:
	clear Seconds
	clear_register_bit doorsOpen			; lift has waited 3 seconds on floor, so clear doorsOpen bit
	set_register_bit closing				; and set closing bit to close door
	rjmp main

closing_sequence:

	rcall LED_flash							; flash LED while closing
	set_motor_speed 0xAA					; set door closing speed
	lds r24, Seconds		 				; load seconds
	lds r25, Seconds+1
	cpi r24, 10								; door takes 1 second to close
	ldi temp1, high(0)
	cpc r25, temp1				
	brge closing_done						; after 1 second go to closing done
	rjmp main
closing_done:
	clear Seconds


	clear_register_bit closing				; clear closing bit since door has closed
	clear_register_bit stopped				; clear stopped bit since lift is ready to move again
	set_motor_speed 0						; turn off motor after door closed
	rcall shuffle_queue						; call shuffle queue. shuffle queue removes visited floors and reshuffles
	rjmp main								; queue so that the start of the queue is the next floor
// ---------------------------------------- STOPPING AT THE FLOOR /\


// ---------------------------------------- ERROR HANDLING \/
end_main:
	ldi temp1, 0b11001100
	out PORTC, temp1
	rjmp end_main
// ---------------------------------------- ERROR HANDLING /\
//
//		 ____					 ___   _____ _____	 ____			 __
//	   ||	   ||	|	||\   |	/	\	 |	   |	/	 \	||\   |	/  \
//	   ||__	   ||	|   || \  | |		 |	   |   |	  | || \  |	\__
//	   ||	   ||	|   ||  \ | |		 |	   |   |	  | ||  \ |	   \
//	   ||	   ||___/	||   \|	\___/	 |	 __|__	\____/	||   \|	\__/	
//		
//
// ---------------------------------------- EMERGENCY FUNCTION \/

emergency_func:
;prologue
	push temp1
	
	in temp1, SREG							; push SREG so that it isnt changed after the function
	push temp1
	
	lds temp1, Count						; load Count and Seconds
	push temp1
	
	lds temp1, Count+1
	push temp1

	lds temp1, Seconds
	push temp1
	
	lds temp1, Seconds+1
	push temp1
	
	push old_floor							; push old_floor, to be restored after emergency is over
	push current_floor
	push r24
	push r25	

	clear_disp								; clear display to write out emergency message
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
	lcd_set 1

	mov old_floor, current_floor		; for restoring original floor

	check_register_bit opening			; check if doors are opening
	breq initial_e_close				;
	check_register_bit doorsOpen		; or if doors are opened when emergency called
	breq initial_e_close
	rjmp drop_floor_loop

initial_e_close:						; close door if opened when emergency called
	rcall LED_flash						; LED flash to show door closing
	set_motor_speed 0xAA				; door closing speed
	lds r24, Seconds
	lds r25, Seconds+1
	cpi r24, low(10)					; wait 1 second before closing
	brge initial_e_close_done
	rjmp initial_e_close

initial_e_close_done:
	set_motor_speed 0					; turn motor off when door closed

drop_floor_loop:						; start emergency routine
	rcall Strobe_flash					; make strobe light flash
	rcall show_floor					; show current floor
	cpi current_floor, 1				; check if lift has reached bottom floor
	breq drop_floor_end					; if it has then go to end
	lds r24, Seconds					; load seconds
	lds r25, Seconds+1		
	mov arg1, r24						; move seconds into arg1
	cpi r24, 20							; if 2 seconds has not passed yet, go to the top of the loop
	brlt drop_floor_loop
	dec current_floor					; after 2 seconds, go down a floor
	clear Seconds						; clear seconds word
	rjmp drop_floor_loop				; go back to top of the loop

drop_floor_end:
;~~~~~~~~~~~~~
emergency_opening:						; door opening routine
	rcall strobe_flash					; make strobe light flash
	rcall LED_flash						; flash LED to show door opening
	rcall scan							; scan keypad
	ldi temp1, '*'						
	cp ret1, temp1						; check if button pressed is '*'
	brne continue_e_opening				; if asterisk not pressed continue opening door

	clear_register_bit emergency		; emergency cancelled if '*' pressed

continue_e_opening:
	set_motor_speed 0x4A				; run motor to simulate door opening
	lds r24, Seconds					; load seconds
	lds r25, Seconds+1
	cpi r24, 10
	brge emergency_opening_done			; 1 Second passed?
	rjmp emergency_opening

emergency_opening_done:
	clear Seconds


emergency_doors_open:					
	rcall strobe_flash					; make strobe lights flash
	rcall LED_flash						; flash LED to show door opening
	rcall scan							; scan keypad
	ldi temp1, '*'
	cp ret1, temp1						; check if asterisk is pressed
	brne continue_e_doors_open
	clear_register_bit emergency		; clear bit if emergency is over
	clear Seconds
	rjmp emergency_closing

continue_e_doors_open:
	set_motor_speed 0					; set motor speed to 0 once door is open
	lds r24, Seconds					; 3 seconds passed?
	lds r25, Seconds+1
	cpi r24, 70							; wait 7 seconds
	brge emergency_doors_open_done		; jump to door open end
	rjmp emergency_doors_open

emergency_doors_open_done:
	clear Seconds

emergency_closing:
	rcall strobe_flash					; make strobe lights flash
	rcall LED_flash						; make LED flash to show door closing
	rcall scan							; scan keypad
	ldi temp1, '*'
	cp ret1, temp1						; check if asterisk pressed
	brne continue_e_closing

	clear_register_bit emergency

continue_e_closing:						
	set_motor_speed 0xAA				; run motor at different speed to simulate door closing
	lds r24, Seconds					; 1 Second passed?
	lds r25, Seconds+1
	cpi r24, 10							
	brge emergency_closing_done
	rjmp emergency_closing
emergency_closing_done:
	clear Seconds
	set_motor_speed 0					; set motor speed to 0 once door is closed




emergency_halt_loop:
	rcall strobe_flash					; make strobe light flash
	rcall show_floor					; show floor
	rcall scan							; scan keypad
	ldi temp1, '*'
	cp ret1, temp1
	brne continue_e_halt
	clear_register_bit emergency
continue_e_halt:	
	check_register_bit emergency
	brne restore_floor					; restore to original floor when emergency is over
	rjmp emergency_halt_loop			; keep running emergency until asterisk pressed again

restore_floor:
	clear Seconds						; write message to show returning to floor
	clear_disp
	write 'R'
	write 'E'
	write 'T'
	write 'U'
	write 'R'
	write 'N'
	write 'I'
	write 'N'
	write 'G'

	change_line 2,0
	write 'T'
	write 'O'
	write ' '
	write 'F'
	write 'L'
	write 'O'
	write 'O'
	write 'R'
	write ' '
	mov arg1, old_floor					; move old_floor to arg 1
	rcall convert_to_ascii				; and call convert to show floor

	lcd_clr 1

restore_floor_loop:
	rcall show_floor					; show floor
	cp current_floor, old_floor			; check if old floor has been reached
	breq restore_floor_end				; jump to end if it has
	lds r24, Seconds
	lds r25, Seconds+1
	cpi r24, 20							; wait 2 seconds between each floor
	brlt restore_floor_loop				; jump to top of loop if not 2 seconds yet
	inc current_floor					; move up one floor after 2 seconds
	clear Seconds						; reset timer to count 2 seconds again
	rjmp restore_floor_loop

restore_floor_end:
;epilogue
	clear_disp							; write normal message of current and next floors
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
	
	pop r25								; pop registers used in function
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

.equ F_CPU = 160000		;edited from 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
    push r24
    push r25
    ldi r25, high(DELAY_1MS)
    ldi r24, low(DELAY_1MS)

delayloop_1ms:
    sbiw r25:r24, 1	 ; DEBUGGING ~~~~~~~~~~~~~~~~~
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
;prologue
push temp2
in temp2 , SREG		; push SREG so as to not change it in the function
push temp2

push counter
push r16	; r16 current_floor global
push r17	; r17 requested_floor return value
push r18    ; parameter input_value
push r19	; r19 lift_status global
push r20    ; temp1
push r22
push arg1
push arg2

push XL
push XH

ldi XL, low(Queue)							; load queue
ldi XH, high(Queue)
lds r22, Queue_len

clr ret1

cpi input_value, 11							; used just in case a number greater than 10 is in input_value somehow
brlt under_11
jmp end_insert_loop							; if its greater than 11, jump to end_insert

under_11:
cpi input_value, 1							; if the input value is greater than or equal to 0,
brge valid_insert							; then it is a valid_insert
jmp end_insert_loop							; else jump to end

valid_insert:
clr counter									; counter = 0
lds r22, Queue_len							; load queue length
cp current_floor, input_value				; if the current floor is the input floor, break to end
brne input_continue							; else continue
jmp end_insert_loop

input_continue:
mov ret1, input_value						; move the input value into the return register

check_register_bit goingUp					; check the goingUp bit
breq up_search								; if 1, sort up
rjmp down_search 							; else sort down



; r7 counter // r22 len

up_search:
    cp input_value, current_floor   		; if input floor < current floor, jump to up_descending_loop, else up_ascending_loop
    brlt up_descending_loop
up_ascending_loop:
    cp counter, r22   						; compare counter to len (check if end of list reached)
    breq end_search
    ld temp1, X   							; load floor from output array
    cp input_value, temp1   				; check if input floor already exists
    breq jumping_to_end_insert				; quit if it does
    brlo insert_start						; if input floor lower than ith floor, insert
    cp temp1, current_floor   				; compare ith floor to current floor
    brlo insert_start						; if ith < current, insert
    adiw X, 1   							; increment output array
    inc counter   							; increment counter
    rjmp up_ascending_loop

up_descending_loop:
    cp counter, r22   						; compare counter to len (check if end of list reached)
    breq end_search   	 
    ld temp1, X   							; load floor from output array
    cp input_value, temp1   				; check if input floor already exists
    breq end_insert_loop					; quit if it does
    cp temp1, current_floor   				; compare ith floor to current floor
    brlo down_search						; if input floor < current floor and ith floor < current floor jmp to down search
    adiw X, 1   							; increment output array
    inc counter   							; increment counter
    rjmp up_descending_loop

jumping_to_end_insert:
	rjmp end_insert_loop

down_search:
    cp current_floor, input_value   		; compare current floor < input floor
    brlt down_ascending_loop
down_descending_loop:
    cp counter, r22   						; compare counter to len (check if end of list reached)
    breq end_search   	 
    ld temp1, X   							; load floor from output array
    cp temp1, input_value   				; check if input floor already exists
    breq end_insert_loop					; quit if it does
    brlo insert_start						; if current floor < input floor insert here
    cp current_floor, temp1   				; compare current floor to ith floor
    brlo insert_start    
    adiw X, 1   							; increment output array
    inc counter   							; increment counter
    rjmp down_descending_loop

down_ascending_loop:
    cp counter, r22   						; compare counter to len (check if end of list reached)
    breq end_search   	 
    ld temp1, X   							; load floor from output array
    cp temp1, input_value   				; check if input floor already exists
    breq end_insert_loop					; quit if it does
    cp current_floor, temp1   				; compare ith floor to current floor
    brlo up_search   						; if ith floor > current and input floor > current floor, jmp to upsearch
    adiw X, 1   							; increment output array
    inc counter   							; increment counter
    rjmp down_ascending_loop

end_search:
    rjmp insert_start

insert_start:    
    inc r22    ;len++   					;length of list is now longer



insert_loop:
    cp r22, counter   						; comparison of index and list length to check if the end of the list has been reached
    breq end_insert_loop    
    ld temp1, X								; load address of queue
	st X+, input_value						; store the value into the queue
    mov input_value, temp1					;
    inc counter
    rjmp insert_loop

end_insert_loop:

    sts Queue_len, r22   					; store new length back in memory

    ;epilogue
    pop XH									; pop registers that were used in the function
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
    push current_floor			; push registers being used
    push counter
	push XL		
	push XH

    clr counter    
    clr XL    ; output
    clr XH

loop:
    cp counter, current_floor	; check if counter is equal to current floor
    breq end_show_floor
    lsl XL						; left shift XL and then increment to show floor
    inc XL						; |_|_|_|_|_|_|_|_| -> lsl, inc -> |_|_|_|_|_|_|_|*|
    brcs grtr8					; |_|_|_|_|_|_|_|*| -> lsl -> |_|_|_|_|_|_|*|_| -> inc -> |_|_|_|_|_|_|*|*| etc.
    rjmp end_x					; if there is a carry, that means all 8 bits of XL are filled, meaning number is bigger than 8
grtr8:
    lsl XH						; lsl XH for higher bit of LED to show numbers bigger than 8
    inc XH						; |_|_|_|_|_|_|_|*| - |*|*|*|*|*|*|*|*| (9)
end_x:    
    inc counter					; increment counter
    rjmp loop


; epilogue
end_show_floor:
    out PORTG, XH				; push the higher part of the LEDs to PORTG (highest two LEDs)
    out PORTC, XL				; push lower part of LEDs to PORTC (bottom 8 LEDs)
								
	pop XH						; pop registers that have been used
	pop XL
	pop counter
	pop current_floor
	
	ret

// CONVERT_TO_ASCII

convert_to_ascii:
	; prologue
	push r7					; register used to write digit
	push r8					; ???
	push r9					; ???
	push r16				; divisor-low
	push r17				; divisor-high
	push r18				; dividend-low
	push r19				; dividend-high
	push r20				; used to store ascii value of zero
	push arg1				; arg1 and arg2 store the value
	push arg2				; that is being converted to ascii
	push r24				; quotient-low
	push r25				; quotient-high
	push ZL					; Word Z is used to load
	push ZH					; the divisors from memory

	clr arg2 ; THIS IS ONLY FOR WHEN NUMBERS LESS THAN 255

start:
	ldi ZL, low(divisors<<1)
	ldi ZH, high(divisors<<1)
	ldi r20, 0x30			; ascii value for zero
	clr r8
	clr r9
convert_loop:
	ldi temp2, 2
	cp r9, temp2
	breq end_convert
	rjmp divide

	; r19:r18 hold the dividend (numerator)
divide:
    lpm r16, Z+				; divisor (denominator)
	lpm r17, Z+				; ^^^
	ldi r24, 0				; quotient
	ldi r25, 0				; ^^^
loop_start:
	cpi r16, 0				; not dividing by zero
	breq end_divide
	cp arg1, r16			; check dividend !< divisor i.e. you can still minus
	cpc arg2, r17
	brlo end_loop 
	sub arg1, r16			; dividend = dividend - divisor 
	sbc arg2, r17
	adiw r25:r24, 1			; quotient++	
    rjmp loop_start
end_loop:
	movw r19:r18, arg2:arg1
end_divide:
	
	; r23:r22 holds the remainder, r25:r24 holds the quotient

	movw arg2:arg1, r19:r18 ; the remainder moves to the dividend to be divided again
	inc r9
	cpi r24, 0
	; breq check_zero
	mov r7, r20				; r7 holds ASCII val for '0'
	add r7, r24				; r7 holds ASCII val for '0' + remainder



	write_reg r7			; write r7 to LCD
	inc r8
	rjmp convert_loop
check_zero:
	cp r8, zero				; if r8 = 0, then nothing has been written to LCD yet
	breq convert_loop		; if it has been written, then write 0, eg for "10"
	write '0'

	rjmp convert_loop



end_convert:
;epilogue
	pop ZH		; pop registers used in function
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

LED_flash:
	push temp1						; push registers that are being used in	the function
	in temp1, SREG					; put SREG into temp1 to make sure SREG isnt changed when this function is called
	push temp1
	push current_floor	
	
	check_register_bit flashing		; check the flashing bit
	breq LED_up						; if the bit is 1, then go to LED_up
	rjmp LED_down					; if the bit is 0, then go to LED_down

LED_up:
	rcall show_floor				; show floor as is if bit is 1
	rjmp LED_END
LED_down:
	dec current_floor				; decrement the current floor if bit is 0, THEN call show floor
	rcall show_floor
									; this creates the effect that the LED is flashing by alternatively showing the (floor)
									; and the (floor-1), e.g:
LED_end:							; |*|*|*|*|*|_|_|_|_|_|
	pop current_floor				; |*|*|*|*|_|_|_|_|_|_|
	pop temp1						; |*|*|*|*|*|_|_|_|_|_|
	out SREG, temp1					; |*|*|*|*|_|_|_|_|_|_|
	pop temp1						; |*|*|*|*|*|_|_|_|_|_|
	ret



//STROBE_FLASH
Strobe_flash:	//Reads in the seconds value in X
	push temp1
	in temp1, SREG
	push temp1
	check_register_bit flashing
	breq strobe_on	
	rjmp strobe_off

strobe_on:
    lcd_set 1		//set PORTA bit to 1
	rjmp strobe_end
strobe_off:
    lcd_clr 1
	rjmp strobe_end
Strobe_end:
	pop temp1
	out SREG, temp1
	pop temp1
	ret


//-----------------------------
// |  C3   |  C2   |  C1   |  C0   |  R3   |  R2   |  R1   |  R0   |
// |  PL0  |  PL1  |  PL2  |  PL3  |  PL4  |  PL5  |  PL6  |  PL7  |


//SCAN						; return value in arg1
scan:
;prologue
    push r16				; row
    push r17				; col
    push r18				; rmask --- r18 will be used to return the input_value
    push r19				; cmask
    push temp1
    push temp2 
    push arg1				; floor
    push arg2				; floor2
    ldi r19, INITCOLMASK   	; load column mask to scan a column
    clr r17

	clr ret1
colloop:
    cpi r17, 4   			; check if all columns scanned
    breq scan_end   		; restart scan if all cols scanned
    sts PORTL, r19   		; scan a column (sts used instead of out since PORTL is in extended I/O space)
    ldi temp1, 0xFF   		; slow down scan operation (???? WHY ????)

delay:
    dec temp1
    brne delay

    lds arg1, PINL   		; load current status of PORTL pins(lds must be used instead of in)
    andi arg1, ROWMASK   	; and the PINL register with row mask
    cpi arg1, 0xF   		; check if any row low
    breq nextcol   			; if temp is all 1s (i.e 0xF), then there are now lows
   							; if there is a low, find which row it is
    ldi r18, INITROWMASK   	; load Row mask
    clr r16

rowloop:
    cpi r16, 4   			; if all rows scanned, jump to next column
    breq nextcol
    mov temp2, arg1  			 
    and temp2, r18   		; mask the input with row mask
    breq show   			; if the bit is clear, a key has been pressed
   							; eg if a key in row 1 is pressed, temp2 = XXXX1101
   							; rmask should equal 00000010
   							; when AND is used, result is 00000000 -> button pressed
    inc r16   				; move to next row
    lsl r18   				; left shift mask to check next row
    jmp rowloop

nextcol:   					; jump to next column when row scan over
    lsl r19   				; left shift mask to check next col
    inc r17
    jmp colloop

show:
	
    cpi r17, 3   			; if column = 3, a key in column 3 is pressed, which is a letter key
    breq scan_end   		; we dont need to deal with this for this lab, so go to n_a

    cpi r16, 3   			; if row = 3, a key in row 3 has been pressed, which is any special character or 0
    breq check_bottom_row	; zero is the only key we are worried about, so go to check_zero
	
	mov arg1, r16			; move row to floor
	lsl arg1				; multiply by 2
	add arg1, r16			; add row again, to multiply row by 3
	add arg1, r17			; add col
	subi arg1, -1			; add 1
	jmp end_show

check_bottom_row:
	cpi r17, 0				; if column = 0, asterisk is pressed
	breq asterisk

	cpi r17, 1				; if column = 1, 0 is pressed
	ldi arg1, 10			; therefore floor 10 has been selected
	breq end_show			

	jmp scan_end

asterisk:
	ldi arg1, '*'			; move asterisk into arg1 to be returned to main

	
end_show:

	mov ret1, arg1			; move arg1 into ret1 to be returned


scan_end:
;epilogue
	pop arg2				; pop used registers
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



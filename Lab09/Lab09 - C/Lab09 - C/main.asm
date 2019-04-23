;
; Lab09 - C.asm
;
; Created: 23/04/2019 11:37:25 AM
; Author : andre
;


;

.include "m2560def.inc"

.equ PORTLDIR = 0xF0   		 ; 0xF0 = 0b11110000 -> Setting PORTA 7:4 as output and 3:0 as input
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4
.equ l_one = 0b10000000
.equ l_two = 0b11000000

.equ clock_speed = 781

.def zero = r3
.def one = r4
.def temp1 = r16
.def temp2 = r17
.def arg1 = r22
.def arg2 = r23
.def motor_speed = r15
//Clear word macro
.macro clear
    sts @0, zero
    sts @0+1, zero
.endmacro

//LCD MACROS
.macro do_lcd_command   					 // LCD commands
	push temp1
    ldi temp1, @0
    rcall lcd_command
    rcall lcd_wait
	pop temp1
.endmacro

.macro do_lcd_command_reg   				 // LCD commands, with registers
	push temp1
    mov temp1, @0
    rcall lcd_command
    rcall lcd_wait
	pop temp1
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
	ldi temp1, @0
	sts OCR3BL, temp1
	clr temp1
	sts OCR3BH, temp1	
.endmacro

.dseg
Count:
    .byte 2
Seconds:
    .byte 2
Revs:
	.byte 2
Speed:
	.byte 2
/*Debounce0:
	.byte 2
Debounce1:
	.byte 2*/





.cseg



.org 0x0000
jmp RESET

.org INT0addr
jmp EXT_INT0
.org INT1addr
jmp EXT_INT1
.org INT2addr
jmp EXT_INT2


.org OVF0addr
    jmp Timer0OVF



RESET:
    ldi temp1, low(RAMEND)    ; Init stack frame
    out SPL, temp1
    ldi temp1, high(RAMEND)
    out SPH, temp1

	

    clr zero   			 ; zero
    clr one   				 
    inc one   				 ; one


/*
    ldi temp1, PORTLDIR   	 ;p7-4 outputs, p3-0 inputs
	ldi temp2, 0b00001111;send 1 to p3-0 to activate pull up resistors
    sts DDRL, temp1
	sts PORTL, temp1*/

	nop
 			 
	//outputs
	ser temp1
    out DDRC, temp1   		 ;LED Lower
    out DDRG, temp1   		 ;LED Higher
	
	//inputs
	clr temp1
    out DDRD, temp1   		 ;Buttons?


	ldi temp1, 0b00010000
	
	out DDRE, temp1

/*    ldi temp1, 0b01010101    ;LED testing
    ldi temp2, 0
    out PORTC, temp1   	 ;LED lower
    out PORTG, temp2   	 ;LED higher*/

    ldi temp1,  0b000001010    ;falling edges for interrupts 0,1,2
    sts EICRA, temp1   	 


/*	ldi temp1, 0xFF
	sts Debounce0, temp1
	sts Debounce0+1, temp1
	sts Debounce1, temp1
	sts Debounce1+1, temp1*/


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

	set_motor_speed 0xFF


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



	

	clear Speed
	clear Revs
	clear Seconds
    sei
	

    jmp main	

EXT_INT0:
/*	push temp2
	in temp2, SREG
	push temp2
    

//in theory, we should be able to test: check_register_bit, doorsOpen / brne INT0_END
	//set_motor_speed 0
	ldi temp1, 0b11110000
	out PORTC, temp1
	clear Debounce0
INT0_END:
    pop temp2
    out SREG, temp2
    pop temp2*/
    reti

EXT_INT1:
/*	clear Debounce1*/
	reti


EXT_INT2:
	push XL
	push XH
	push YL
	push YH
	lds XL, Speed
	lds XH, Speed+1
	lds YL, Revs
	lds YH, Revs+1
	adiw X,1
	cpi XL, 0xFF
	cpi XH, 0xFF
	brlt no_reset
	adiw Y, 1
	clear Speed
no_reset:


	sts Speed, XL
	sts Speed+1, XH
	sts Revs, YL
	sts Revs+1, YH
	pop YH
	pop YL
	pop XH
	pop XL
	reti

//---- TIMER
Timer0OVF:
    in temp1, SREG   	 ; stack frame for timer interrupt handler
    push temp1
	push temp2
    push r25
    push r24
	push YL
	push YH

    lds r24, Count   	 ; increment count
    lds r25, Count + 1
    adiw r25:r24, 1







    
    cpi r24, low(clock_speed)    ; compare with clock speed to check if 1/10 of second has passed
    ldi temp1, high(clock_speed)
    cpc r25, temp1
    brne Not_second
    lds r24, Seconds   		 ; increment seconds every 1/10 of second
    lds r25, Seconds+1


	//grab the revs *4 for this 0.1 second passing
	// to gev rev/sec do speed/4*10 ie speed*5/2

/*	lds YL, Speed
	lds YH, Speed+1
	
	mov temp1, YL
	mov temp2, YH
	
	ldi temp1, 0b00000010
	and temp1, YL
	breq rnd_down
	adiw Y, 1
rnd_down:
	andi YL, 0b11111100
	lsl YL
	rol YH*/



	
/*	cp YL, motor_speed
	brlt accel
	brge decel

accel:
	adiw Y, 63
	
	ldi temp1, 1
	out PORTG, temp1
	rjmp speed_adjusted

decel:
	sbiw Y, 63
	
	ldi temp1, 2
	out PORTG, temp1
	rjmp speed_adjusted*/

speed_adjusted:
	sts Revs, YL
	sts Revs+1, YH
	clear Speed


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
	pop YH
	pop YL
    pop r24
    pop r25
	pop temp2
    pop temp1
    out SREG, temp1
    reti

divisors:
	 .dw 10000, 1000, 100, 10, 1

; Replace with your application code
main:

start_loop:
	
	lds r24, Seconds
	lds r25, Seconds+1

	cpi r24, 4
	brne main
	clear_disp
	//write 'P'	
	clear Seconds
	lds arg1, Revs
	lds arg2, Revs+1
	rcall convert_to_ascii
	clear Revs


    rjmp main




//LCD_FUNCTIONS
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


//CONVERT_TO_ASCII
convert_to_ascii:
	;prologue
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



	//clr arg2 ///THIS IS ONLY FOR WHEN NUMBERS LESS THAN 255

	;change_line 2, 14
start:
	ldi ZL, low(divisors<<1)
	ldi ZH, high(divisors<<1)
	ldi r20, 0x30		// ascii value for zero
	clr r8
	clr r9
convert_loop:
	ldi temp2, 5
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
	breq check_zero
	mov r7, r20	//r7 holds ASCII val for '0'
	add r7, r24 //r7 holds ASCII val for '0' + remainder

	//st X+, r7 //store ASCII val in next part of data memory

	write_reg r7
	inc r8
	rjmp convert_loop
check_zero:
	ldi temp2, 5
	cp r9, temp2
	breq final_number
	cp r8, zero
	breq convert_loop

final_number:	
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


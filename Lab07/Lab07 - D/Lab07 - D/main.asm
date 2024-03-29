;
; Lab05 - A.asm
;
; Created: 20/03/2019 9:02:50 PM
; Author : andre
;

.include "m2560def.inc"
.def temp = r21
.def temp2 = r22
.def zero = r3
.def one = r4
.def input = r6
.def floor = r20
.def direction = r23
.def data = r25
.def req_floor = r24
.equ l_one = 0b10000000
.equ l_two = 0b11000000
.equ PORTLDIR = 0xF0			; 0xF0 = 0b11110000 -> Setting PORTA 7:4 as output and 3:0 as input
.equ INITCOLMASK = 0XEF			; 0xEF = 0b11101111 -> Mask to decide which column is selected
.equ INITROWMASK = 0x01			; 0x01 = 0b00000001 -> Mask to check which row is selected
.equ ROWMASK = 0x0F				; 0x0F = 0b00001111 -> To get keyboard output value using an AND operation

.def row = r16					; current row
.def col = r17					; current column
.def rmask = r18				; mask for row during scan
.def cmask = r19				; mask for column during scan

.equ clock_speed = 781
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

.macro clear_disp							// clear the LCD Display
	do_lcd_command 0b00000001
.endmacro


.macro do_lcd_command_reg					// LCD commands, with registers
	mov r16, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro

.macro change_line							// change line and cursor position on line
	ldi r16, @0
	cpi r16, 2
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
	ldi r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro

.macro write_reg							// write register data to LCD screen
	mov r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro

.macro lcd_set
	sbi PORTA, @0
.endmacro
.macro lcd_clr
	cbi PORTA, @0
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
	.db 3,5,7,9
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
	out DDRC, r16			
	out DDRG, r16
	clr r16
	out DDRD, r16			; Enabling Timer interrupt

	ldi temp, PORTLDIR			; set ports A, C, F, G, L
	sts DDRL, temp
	ser temp
	out DDRF, temp
	out DDRA, temp


	clr temp
	out PORTF, temp
	out PORTA, temp


	

	ldi r16,  0b00001010
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
	pop r24
	pop r25
	pop YL
	pop YH
	pop temp
	out SREG, temp
	reti

//-------------------------------------------------------------

//

main:

	ldi floor, 1 ;this is the FlooR <-------------------------------
	ldi ZL, low(Requests<<1)
	ldi ZH, high(Requests<<1)
	lpm req_floor, Z		; Load requested floor

	ldi direction, 1
	rcall show_floor
	ldi r18, wait_speed
	clr r19
	
	ldi temp, 1
	sts Moving_flag, temp
	ldi YL, 50
	ldi YH, 0
	clear_disp
	mov temp, floor
	subi temp, -48
	do_lcd_data temp



wait_loop:
	rcall scan
	ldi temp, '*'
	cp input, temp				; check if floor is 10, since 10 requires 2 digits to be printed
	brne check_count

	rcall emergency_func
/*	rcall Keypad_input
	ldi temp, star
	cp input, temp
	brne check_count
	write 'A'*/
/*	push r24
	push r25
	lds r24, Seconds
	lds r25, Seconds+1
	adiw r25:r24, 1
	sts Seconds, r24
	sts Seconds+1, r25
	pop r25
	pop r24*/

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
	clear_disp
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
;	push YL
;	push YH
	push XL
	push XH
	push floor
	push r16

	clr r16	
	clr XL	; output
	clr XH
loop:
	cp r16, floor
	breq end_show_floor
	lsl XL
	inc XL
	brcs grtr8
	rjmp end_x
grtr8:
	lsl XH
	inc XH
end_x:	
	inc r16
	rjmp loop


;epilogue
end_show_floor:
	out PORTG, XH
	out PORTC, XL


pop r16
pop floor
pop XH
pop XL
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


//LCD-example functions

.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4



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


scan:
//////////////////////////////////////////////////////////////////////////////
	push r16
	push r17
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	ldi cmask, INITCOLMASK		; load column mask to scan a column
	clr col

colloop:
	cpi col, 4					; check if all columns scanned
	breq end_show			; restart scan if all cols scanned
	sts PORTL, cmask			; scan a column (sts used instead of out since PORTL is in extended I/O space)
	ldi temp, 0xFF				; slow down scan operation (???? WHY ????)

delay: 
	dec temp
	brne delay

	lds r23, PINL				; load current status of PORTL pins (lds must be used instead of in)
	andi r23, ROWMASK			; and the PINL register with row mask
	cpi r23, 0xF				; check if any row low
	breq nextcol				; if temp is all 1s (i.e 0xF), then there are now lows
								; if there is a low, find which row it is
	ldi rmask, INITROWMASK		; load Row mask
	clr row

rowloop:
	cpi row, 4					; if all rows scanned, jump to next column
	breq nextcol
	mov temp2, r23				
	and temp2, rmask			; mask the input with row mask
	breq show					; if the bit is clear, a key has been pressed
								; eg if a key in row 1 is pressed, temp2 = XXXX1101
								; rmask should equal 00000010
								; when AND is used, result is 00000000 -> button pressed
	inc row						; move to next row
	lsl rmask					; left shift mask to check next row
	jmp rowloop

nextcol:						; jump to next column when row scan over
	lsl cmask					; left shift mask to check next col
	inc col
	jmp colloop

show:
	cpi col, 3					; if column = 3, a key in column 3 is pressed, which is a letter key
	breq n_a					; we dont need to deal with this for this lab, so go to n_a

	cpi row, 3					; if row = 3, a key in row 3 has been pressed, which is any special character or 0
	brne n_a					; zero is the only key we are worried about, so go to check_zero

	cpi col, 0
	brne n_a

	ldi r23, '*'
	jmp end_show			

n_a:
	ser r23					; set all bits in temp to 1 for keys we dont care about

end_show:
mov input, r23
/*	cpi r23, '*'				; check if floor is 10, since 10 requires 2 digits to be printed
	brne scan_end
	change_line 2, 0
	write 'E'
	write 'm'
	write 'e'
	write 'r'
	write 'g'
	write 'e'
	write 'n'
	write 'c'
	write 'y'
	rcall emergency_func*/
scan_end:
	
pop r23
pop r22
pop r21
pop r20
pop r19
pop r18
pop r17
pop r16
ret


emergency_func:
	
	push XL
	push XH
	push floor
	push temp
	push temp2

	clear_disp
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
	lcd_set 2

	mov r14, floor
drop_floor_loop:
	cpi floor, 1
	breq drop_floor_end
	lds XL, Seconds
	lds XH, Seconds+1
	rcall Strobe_flash
	cpi XL, 20
	brlt drop_floor_loop	
	dec floor
	rcall show_floor
	clear Seconds
	rjmp drop_floor_loop
drop_floor_end:

halt_loop:
	rcall scan
	ldi temp, '*'
	cp input, temp
	breq return_floor_loop
	lds XL, Seconds
	lds XH, Seconds+1
	cpi XL, 50
	breq Done_Flash
	mov temp, XL
	ror temp
	brcs E_FLASH
	ldi temp, 0
	rjmp out_port
E_FLASH:
	ldi temp, 1

out_port:
	out PORTC, temp

Done_Flash:
	rcall Strobe_flash


	rjmp halt_loop
return_floor_loop:
	cp floor, r14
	breq return_floor_end
	lds XL, Seconds
	lds XH, Seconds+1
	rcall Strobe_flash
	cpi XL, 20
	brlt return_floor_loop
	inc floor
	rcall show_floor
	clear Seconds
	rjmp return_floor_loop

return_floor_end:		
	
	clear_disp
	mov temp, floor
	subi temp, -48
	write_reg temp
	pop temp2
	pop temp
	pop floor
	pop XH
	pop XL
	lcd_clr 1
	ret
//////////
Strobe_flash:
	push temp
	push XL
	mov temp, XL
	ror temp
	brcs strobe_on
	brcc strobe_off

strobe_on:
	lcd_set 1
	pop XL
	pop temp
	ret

strobe_off:
	lcd_clr 1
	pop XL
	pop temp
	ret


/*
.equ COL0 = 0xEF
.equ COL1 = 0b11011111
.equ COL2 = 0b10111111
.equ COL3 = 0b01111111
.equ ROW0 = 0b00000001
.equ ROW1 = 0b00000010
.equ ROW2 = 0b00000100
.equ ROW3 = 0b00001000
.equ A = 20
.equ B = 30
.equ C = 40
.equ D = 50
.equ star = 60
.equ hash = 70
.equ nothing = 80

//interprets Keypad input, returns value in input
Keypad_input:
	push temp
	push temp2
	in temp2, SREG
	push temp2

	

	ldi temp, COL0
	
	sts PORTL, temp

	lds temp, PINL
	mov temp2, temp
	andi temp2, ROW0
	breq number1
	mov temp2, temp
	andi temp2, ROW1
	breq number2
	mov temp2, temp
	andi temp2, ROW2
	breq number3
	mov temp2, temp
	andi temp2, ROW3
	breq letterA
	rjmp section2

number1:
ldi temp, 1
mov input, temp
rjmp Keypad_end
number2:
ldi temp, 2
mov input, temp
rjmp Keypad_end
number3:
ldi temp, 3
mov input, temp
rjmp Keypad_end
letterA:
ldi temp, A
mov input, temp
rjmp Keypad_end
	
section2:
	ldi temp, COL1
	sts PORTL, temp
	lds temp, PINL
	mov temp2, temp
	andi temp2, ROW0
	breq number4
	mov temp2, temp
	andi temp2, ROW1
	breq number5
	mov temp2, temp
	andi temp2, ROW2
	breq number6
	mov temp2, temp
	andi temp2, ROW3
	breq letterB



	ldi temp, COL2
	sts PORTL, temp
	lds temp, PINL
	mov temp2, temp
	andi temp2, ROW0
	breq number7
	mov temp2, temp
	andi temp2, ROW1
	breq number8
	mov temp2, temp
	andi temp2, ROW2
	breq number9
	mov temp2, temp
	andi temp2, ROW3
	breq letterC
	

	
	ldi temp, COL3
	sts PORTL, temp
	lds temp, PINL
	mov temp2, temp
	andi temp2, ROW0
	breq symbol_star
	mov temp2, temp
	andi temp2, ROW1
	breq number0
	mov temp2, temp
	andi temp2, ROW2
	breq symbol_hash
	mov temp2, temp
	andi temp2, ROW3
	breq letterD
	rjmp symbol_none
number0:
ldi temp, 10
mov input, temp
rjmp Keypad_end

number4:
ldi temp, 4
mov input, temp
rjmp Keypad_end
number5:
ldi temp, 5
mov input, temp
rjmp Keypad_end
number6:
ldi temp, 6
mov input, temp
rjmp Keypad_end
number7:
ldi temp, 7
mov input, temp
rjmp Keypad_end
number8:
ldi temp, 8
mov input, temp
rjmp Keypad_end
number9:
ldi temp, 9
mov input, temp
rjmp Keypad_end

letterB:
ldi temp, B
mov input, temp
rjmp Keypad_end
letterC:
ldi temp, C
mov input, temp
rjmp Keypad_end
letterD:
ldi temp, D
mov input, temp
rjmp Keypad_end
symbol_star:
ldi temp, star
mov input, temp
rjmp Keypad_end
symbol_hash:
ldi temp, hash
mov input, temp



rjmp Keypad_end
symbol_none:
ldi temp, nothing
mov input, temp
rjmp Keypad_end
Keypad_end:

	pop temp2
	out SREG, temp2
	pop temp2
	pop temp
	ret*/
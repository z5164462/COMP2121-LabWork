;
; Lab07 - C.asm
;
; Created: 30/03/2019 10:59:04 PM
; Author : Anirudh
;



;ASSUMPTION!!
; No queue handling is being implemented at this stage. To move to floor 1 is just to change the current floor



.include "m2560def.inc"
.def counter = r15				; arbitrary counter
.def row = r16					; current row
.def col = r17					; current column
.def rmask = r18				; mask for row during scan
.def cmask = r19				; mask for column during scan
.def temp = r20
.def temp2 = r21
.def floor = r22

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

.dseg
	floor_array: .BYTE 2

.cseg

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// MACROS
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\


.macro do_lcd_command						// LCD commands
	ldi r16, @0
	rcall lcd_command
	rcall lcd_wait
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

.org 0
	jmp RESET

RESET:
	ldi temp, low(RAMEND)		; initialise stack
	out SPL, temp
	ldi temp, high(RAMEND)
	out SPH, temp

	ldi temp, PORTLDIR			; set ports A, C, F, G, L
	sts DDRL, temp
	ser temp
	out DDRC, temp
	out PORTC, temp
	out DDRG, temp
	out PORTG, temp
	out DDRF, temp
	out DDRA, temp

	clr temp
	out PORTF, temp
	out PORTA, temp

	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off?
	clear_disp				  ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink

	ldi temp, 0b00000000
	out TCCR0A, temp
	ldi temp, 0b00000010
	out TCCR0B, temp
	ldi temp, 1<<TOIE0
	sts TIMSK0, temp
	

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// MAIN
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\

main:
	change_line 2, 0			; set to start of line 2
	write 'F'					; write "Floor Request"
	write 'l'
	write 'o'
	write 'o'
	write 'r'
	write ' '
	write 'R'
	write 'e'
	write 'q'
	write 'u'
	write 'e'
	write 's'
	write 't'

	ldi cmask, INITCOLMASK		; load column mask to scan a column
	clr col

colloop:
	cpi col, 4					; check if all columns scanned
	breq main					; restart scan if all cols scanned
	sts PORTL, cmask			; scan a column (sts used instead of out since PORTL is in extended I/O space)
	ldi temp, 0xFF				; slow down scan operation (???? WHY ????)

delay: 
	dec temp
	brne delay

	lds floor, PINL				; load current status of PORTL pins (lds must be used instead of in)
	andi floor, ROWMASK			; and the PINL register with row mask
	cpi floor, 0xF				; check if any row low
	breq nextcol				; if temp is all 1s (i.e 0xF), then there are now lows
								; if there is a low, find which row it is
	ldi rmask, INITROWMASK		; load Row mask
	clr row

rowloop:
	cpi row, 4					; if all rows scanned, jump to next column
	breq nextcol
	mov temp2, floor				
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
	breq check_zero				; zero is the only key we are worried about, so go to check_zero

								; formula to convert the coordinates into a number is R*3 + C + 1
	mov floor, row				; move row to floor
	lsl floor					; multiply by 2
	add floor, row				; add row again, to multiply row by 3
	add floor, col				; add col
	subi floor, -1				; add 1
	jmp end_show				

check_zero:						; check if they key pressed in the bottom row is a zero
	cpi col, 1					; if the button pressed is in column 1, then it is zero
	brne n_a					; if not, then we dont need to worry about it
	ldi floor, 10				; clear temp to show 0
	jmp end_show			

n_a:
	ser floor					; set all bits in temp to 1 for keys we dont care about

end_show:
	cpi floor, 10				; check if floor is 10, since 10 requires 2 digits to be printed
	brne lt10

	rcall show_floor			; set LED bar up 

	ldi floor, 1				; For floor 10, load 1 into floor
	clear_disp					; clear display initially
	change_line 2, 14			; set cursor to be after "Floor Request "
	rcall convert_to_ascii		; convert digit to ascii value
	write_reg r25				; write the value in r25 onto the LCD
	
	ldi floor, 0				; repeat process for 0
	rcall convert_to_ascii
	write_reg r25
	rjmp end_main

lt10:
	rcall show_floor
	clear_disp
	change_line 2, 14
	rcall convert_to_ascii
	write_reg r25

end_main:
	jmp main

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// LCD COMMANDS
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\

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

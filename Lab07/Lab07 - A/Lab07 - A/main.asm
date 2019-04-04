;
; Lab07 - A.asm
;
; Created: 30/03/2019 6:53:57 PM
; Author : Anirudh
;

.include "m2560def.inc"
.def row = r16					; current row
.def col = r17					; current column
.def rmask = r18				; mask for row during scan
.def cmask = r19				; mask for column during scan
.def temp = r20
.def temp2 = r21

.equ PORTLDIR = 0xF0			; 0xF0 = 0b11110000 -> Setting PORTA 7:4 as output and 3:0 as input
.equ INITCOLMASK = 0XEF			; 0xEF = 0b11101111 -> Mask to decide which column is selected
.equ INITROWMASK = 0x01			; 0x01 = 0b00000001 -> Mask to check which row is selected
.equ ROWMASK = 0x0F				; 0x0F = 0b00001111 -> To get keyboard output value using an AND operation


//-----------------------------
// |  C3   |  C2   |  C1   |  C0   |  R3   |  R2   |  R1   |  R0   |
// |  PL0  |  PL1  |  PL2  |  PL3  |  PL4  |  PL5  |  PL6  |  PL7  |


RESET:
	ldi temp, low(RAMEND)		; initialise stack
	out SPL, temp
	ldi temp, high(RAMEND)
	out SPH, temp

	ldi temp, PORTLDIR
	sts DDRL, temp
	ser temp
	;out PORTL, temp
	out DDRC, temp
	out PORTC, temp

main:
	ldi cmask, INITCOLMASK
	clr col
//	jmp end_show

colloop:
	cpi col, 4					; check if all columns scanned
	breq main					; restart scan if all cols scanned
	sts PORTL, cmask			; scan a column (sts used instead of out since PORTL is in extended I/O space)
	ldi temp, 0xFF				; slow down scan operation (???? WHY ????)

delay: 
	dec temp
	brne delay

	lds temp, PINL				; load current status of PORTL pins (lds must be used instead of in)
	andi temp, ROWMASK			; and the PINL register with row mask
	cpi temp, 0xF				; check if any row low
	breq nextcol				; if temp is all 1s (i.e 0xF), then there are now lows
								; if there is a low, find which row it is
	ldi rmask, INITROWMASK		; load Row mask
	clr row

rowloop:
	cpi row, 4					; if all rows scanned, jump to next column
	breq nextcol
	mov temp2, temp				
	and temp2, rmask			; mask the input with row mask
	breq show					; if the bit is clear, a key has been pressed
								; eg if a key in row 1 is pressed, temp2 = XXXX1101
								; rmask should equal 00000010
								; when AND is used, result is 00000000 -> button pressed
	inc row						; move to next row
	lsl rmask					; left shift mask to check next row
	jmp rowloop

nextcol:						; jump to next column when row scan over
	sec 
	rol cmask					; left shift mask to check next col
	inc col
	jmp colloop

show:
	cpi col, 3					; if column = 3, a key in column 3 is pressed, which is a letter key
	breq n_a					; we dont need to deal with this for this lab, so go to n_a

	cpi row, 3					; if row = 3, a key in row 3 has been pressed, which is any special character or 0
	breq check_zero				; zero is the only key we are worried about, so go to check_zero

								; formula to convert the coordinates into a number is R*3 + C + 1
	mov temp, row				; move row to temp
	lsl temp					; multiply by 2
	add temp, row				; add row again, to multiply row by 3
	add temp, col				; add col
	subi temp, -1				; add 1
	jmp end_show				

check_zero:						; check if they key pressed in the bottom row is a zero
	cpi col, 1					; if the button pressed is in column 1, then it is zero
	brne n_a					; if not, then we dont need to worry about it
	clr temp					; clear temp to show 0
	jmp end_show			

n_a:
	ser temp					; set all bits in temp to 1 for keys we dont care about

end_show:
	out PORTC, temp				; move temp to PORTC to show output
	jmp main


//----------------------------------------------


	
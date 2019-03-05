;
; Lab03 - B.asm
;
; Created: 5/03/2019 12:10:18 PM
; Author : andre
;

.equ i_a_len = 7
.equ e_a_len = 5
.equ o_a_len = 15

.dseg 
	
	output_array:
		.BYTE o_a_len

; set up initial input array, and the enter array
.cseg
	
	rjmp main

	initial_array:
				.dw 1
				.dw 2
				.dw 5
				.dw 7
				.dw 8
				.dw 12
				.dw 20



	enter_array:
				.dw 0
				.dw 1
				.dw 10
				.dw 25
				.dw 6






; Replace with your application code
main:
	ldi YL, low(RAMEND - 2)		;carve out space on the stack for 1  int (2 byte), len
	ldi YH, high(RAMEND - 2)	; Y = SP,  int len
    out SPL, YL					
	out SPH, YH
		
	clr r16					;len = 0
	clr r17
	std Y+1, r16			;store inside stack frame
	std Y+2, r17			

	ldi ZL, low(intial_array<<1)	;load Z with the address from the program memory
	ldi ZH, high(initial_array<<1)

	ldi XL, low(outpuy_array)
	ldi XH, high(output_array)

	





;arguments Z = address of program memory array, r18:19 = len of things being inserted,  r20 (RETURN VALUE) = number being loaded, r24:25 (RETURN VALUE) = new len of list
;local parameters, address, len, counter
read_insert_function:
	;prologue
	push ZL
	push ZH
	push r18
	push r19
	in YL, SPL
	in YH, SPH
	sbiw Y, 6		;carve space for addres, len and counter
	out SPL, YL
	out SPH, YH


	



;arguments X = address of array, r18 = len, r19 = number to be inserted
insert_request_function:
	;prologue
	push r16
	push r17
	push r18
	push r19


	;body

	;function



end_insert:
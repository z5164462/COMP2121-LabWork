;
; Lab03 - B.asm
;
; Created: 5/03/2019 12:10:18 PM
; Author : andre
;

.dseg 
	
	output_array:
		.BYTE 15

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

read_insert_loop:


	



;arguments r16:17 = address of array, r18 = len, r19 = number to be inserted
insert_function:
	;prologue
	push r16
	push r17


	;body

	;function



end_insert:
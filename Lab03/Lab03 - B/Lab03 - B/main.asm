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
	ldi YL, low(RAMEND - 1)		;carve out space on the stack for 1  short int (1 byte), len
	ldi YH, high(RAMEND - 1)	; Y = SP,  int len
    out SPL, YL					
	out SPH, YH
		
	clr r16					;len = 0
	std Y+1, r16			;store inside stack frame		

	ldi ZL, low(intial_array<<1)	;load Z with the address from the program memory
	ldi ZH, high(initial_array<<1)

	ldi XL, low(outpuy_array)
	ldi XH, high(output_array)

	

; ASSUMPTION. List length fits in 1 byte, all inserted numbers fit inside 1 byte



;arguments Z = address of program memory array, X = address of data memory,  r18 = current len 19 = len of things being inserted,  r22 (RETURN VALUE) = new len of list
;local parameters, address, len, counter
read_insert_function:
	;prologue
	push r16
	in YL, SPL		; Y = SP
	in YH, SP
	sbiw Y, 5		;carve space for address(Z), current len (r18), inserting len (r19) and counter (local)
	out SPL, YL
	out SPH, YH		;SP = Y
					;currently moved 10 bytes   (top) 0XHigh-10 |ZH |ZL |XH | XL| r19 |r18	|counter|r16	|(bottom) 0XHigh
	std Y+1, ZL
	std Y+2, ZH
	std Y+3, XH
	std Y+4, XL
	std Y+5, r19
	std Y+6, r18
	
	;end prologue

	;body

	ldd ZH, Y+1		;local Z = Z
	ldd ZL, Y+2
	ldd r19, Y+5	;in_len = arg len of things coming in
	ldd r18, Y+6	;old_len = arg current len
	clr r16	; counter = 0
	

extract_loop:
	cp r16, r19		;while all numbers are not inserted
	breq end_extract
	lpm r20, Z		;load number into r20
	adiw Z, 2		;Z = Z + 2 (word)
	
	;insert
	inc r16			;count++
	rjmp exctract_loop

	mov r20, r18
	add r20, r19	; return old_len + in_len

	;end body

	;epilogue
	adiw Y, 8
	out SPH, YH
	out SPL, YL
	pop


	ret

end_extract:

;arguments X = address of array, r18:19 = len
insert_request_function:
	;prologue
	push r16
	push r17
	push r18
	push r19


	;body

	;function



end_insert:
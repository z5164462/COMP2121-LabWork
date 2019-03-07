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
.org 0x300
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
	clr r17					;
	std Y+1, r16			;store inside stack frame		

	ldi ZL, low(initial_array<<1)	;load Z with the address from the program memory
	ldi ZH, high(initial_array<<1)



	
load_loop:
	cpi r16, i_a_len
	breq end_load_loop
	lpm r17, Z
	adiw Z, 2
	ldi XL, low(output_array)
	ldi XH, high(output_array)
	rcall insert_request
	rjmp load_loop

end_load_loop:
	
	ldi ZL, low(enter_array<<1)	;load Z with the address from the program memory
	ldi ZH, high(enter_array<<1)

	clr r20
enter_loop:

	cpi r20, e_a_len
	breq end_enter_loop
	lpm r17, Z+
	adiw Z, 1
	ldi XL, low(output_array)
	ldi XH, high(output_array)
	rcall insert_request
	inc r20
	rjmp enter_loop

	end_enter_loop:
		rjmp end_enter_loop

;ASSUMPTION. List length fits in 1 byte, all inserted numbers fit inside 1 byte

;parameters(address, len, num), returns new_len in r16

insert_request:
	;prologue
	push YL
	push YH
	push r17
	in YL, SPL
	in YH, SPH
	
	sbiw Y, 1	;local counter
	sbiw Y, 4	;args Address(2) len input_val
	out SPL, YL
	out SPH, YH


	std Y+1, ZH
	std Y+2, ZL
	std Y+3, r16
	std Y+4, r17

	;end prologue
		

	;body
	clr r18		;counter = 0


search_loop:
	cp r18, r16				; check if last element of output array has been reached by comparing index and list length
	breq end_search_loop	; if they are equal, that means the end of the output list has been reached without a match or insert, therefore the element needs to be inserted at the end of the list
	ld r19, X				; load the output_array value for comparison r19 = output_array[r18] and Z++ 
	cp r17, r19				; compare inserting number with i-th number
	breq end_insert_loop	; if the numbers match, it already exists in list, therefore dont insert
	brlo end_search_loop	; if the insert number is smaller than the existing comparison number, insert it here
	adiw X, 1
	inc r18					; counter++
	rjmp search_loop		
end_search_loop:
	inc r16	;len++
	

insert_loop:
	cp r18, r16				; comparison of index and list length to check if the end of the list has been reached
	breq end_insert_loop	
	st X+, r17
	mov r17, r19
	ld r19, X
	inc r18
	rjmp insert_loop

end_insert_loop:

	;epilogue
	adiw Y, 5
	out SPH, YH
	out SPL, YL
	pop r17
	pop YH
	pop YL
	ret
	





























;arguments Z = address of program memory array, X = address of data memory,  r18 = current len 19 = len of things being inserted,  r22 (RETURN VALUE) = new len of list
;local parameters, address, len, counter
/*read_insert_function:
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

end_extract:
	mov r20, r18
	add r20, r19	; return old_len + in_len

	;end body

	;epilogue
	adiw Y, 8
	out SPH, YH
	out SPL, YL
	pop r16
	ret*/



;arguments X = address of array, r18:19 = len

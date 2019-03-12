;
; Lab03 - C.asm
;
; Created: 12/03/2019 12:07:13 PM
; Author : andre
;


; Replace with your application code
;
; Lab03 - B.asm
;
; Created: 5/03/2019 12:10:18 PM
; Author : andre
;

.equ i_a_len = 4
.equ e_a_len = 3
.equ o_a_len = 15

.dseg 
.org 0x300
	output_array:
		.BYTE o_a_len

; set up initial input array, and the enter array
.cseg
	
	rjmp main

	initial_array:
				.dw 5
				.dw 7
				.dw 8
				.dw 1



	enter_array:
				.dw 10
				.dw 6
				.dw 7







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


	ldi XL, low(output_array)
	ldi XH, high(output_array)
	
	ldi r20, 3
	ldi r21, 1

load_loop:
	cpi r16, i_a_len
	breq end_load_loop
	lpm r17, Z
	adiw Z, 2

	rcall insert_request
	rjmp load_loop

end_load_loop:
	
	ldi ZL, low(enter_array<<1)	;load Z with the address from the program memory
	ldi ZH, high(enter_array<<1)

	clr r22	;clear enter_array_counter
enter_loop:

	cpi r22, e_a_len
	breq end_enter_loop
	lpm r17, Z+
	adiw Z, 1
	ldi XL, low(output_array)
	ldi XH, high(output_array)
	rcall insert_request
	inc r22
	rjmp enter_loop

	end_enter_loop:
		rjmp end_enter_loop

;ASSUMPTION. List length fits in 1 byte, all inserted numbers fit inside 1 byte
;ASSUMPTION. If current floor is requested, request is ignored


;EXTENDED INSERT REQUEST

;parameters(address, len, input_floor, current_floor, direction), returns new_len in r16
;			X		r16		r17				r20				r21


insert_request:
	;prologue
	push YL
	push YH
	push r17
	push r18
	push r19
	push r20
	push r21
	push XL
	push XH
	in YL, SPL
	in YH, SPH
	
	sbiw Y, 1	;local counter
	sbiw Y, 6	;args Address(2), len, input_floor, current_floor, direction
	out SPL, YL
	out SPH, YH


	std Y+1, XH		;address
	std Y+2, XL
	std Y+3, r16	;len
	std Y+4, r17	;input_floor
	std Y+5, r20	;current_floor
	std Y+6, r21	;direction

	;end prologue
		

	;body
	clr r18		;counter = 0

	cp r17, r20
	breq end_insert_request

	cpi r21, 1
	breq up_search
	rjmp down_search


up_search:
	cp r17, r20
	brlt up_descending_loop
up_ascending_loop:
	cp r18, r16
	breq end_search
	ld r19, X
	cp r17, r19
	breq end_insert_request
	brlo insert_start
	cp r19, r20
	brlo insert_start
	adiw X, 1
	inc r18
	rjmp up_ascending_loop

up_descending_loop:
	cp r18, r16
	breq end_search
	ld r19, X
	cp r17, r19
	breq end_insert_request
	cp r19, r20
	brlo down_search
	adiw X, 1
	inc r18
	rjmp up_descending_loop



down_search:
	cp r20, r17
	brlt down_ascending_loop
down_descending_loop:
	cp r18, r16
	breq end_search
	ld r19, X
	cp r19, r17
	breq end_insert_request
	brlo insert_start
	cp r20, r19
	brlo insert_start
	adiw X, 1
	inc r18
	rjmp down_descending_loop

down_ascending_loop:
	cp r18, r16
	breq end_search
	ld r19, X
	cp r19, r17
	breq end_insert_request
	cp r20, r19
	brlo up_search
	adiw X, 1
	inc r18
	rjmp down_ascending_loop

end_search:
	rjmp insert_start



/*
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
end_search_loop:*/
	
insert_start:	
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
	nop
end_insert_request:



	;epilogue
	adiw Y, 7
	out SPH, YH
	out SPL, YL
	pop XH
	pop XL
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop YH
	pop YL
	ret
	

/*

(5,6,7,8,9,10,11, 3) inserting 1 , on floor 4 going up

if(going up)
jmp upsearch
if(going down)
jmp downsearch

up search
	if (inserting_floor > current_floor)
		while(i < len)
			if (ith > inserting)
				shift insert at i
			if (ith < current_floor)
				shift insert at i
	if(inserting_floor < current_floor)
		while(i < len)
			if(ith_floor < current_floor)
				down search			
	shift insert at i  

down search
	if (inserting_floor < current_floor)
		while(i < len)
			if (ith < inserting)
				shift insert at i
			if (ith > current_floor)
				shift insert at i
	if(inserting_floor > current_floor)
		while(i < len)
			if(ith_floor > current_floor)
				up search		
	shift insert at i


r16 = len
r17 = inserting
r18 = i
r19 = ith
r20 = current floor
r21 = direction

*/
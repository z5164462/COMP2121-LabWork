


; Replace with your application code
;
; Lab03 - C.asm
;
; Created: 12/03/2019 12:10:18 PM
; Author : andrudh
;

.equ i_a_len = 0
.equ e_a_len = 4
.equ o_a_len = 15
.equ floor = 3
; 0 - DOWN, 1 - UP
.equ direction = 1

.dseg 
.org 0x300
	output_array:
		.BYTE o_a_len

; set up initial input array, and the enter array
.cseg
	
	rjmp main

	initial_array:
				;.dw 5
				;.dw 7
				;.dw 8
				;.dw 1
				;.dw 11
				;.dw 15



	enter_array:
				.dw 2
				.dw 5
				.dw 4
				.dw 1





; Replace with your application code
main:
	ldi YL, low(RAMEND - 1)		;carve out space on the stack for 1 short int (1 byte), len
	ldi YH, high(RAMEND - 1)	; Y = SP,  int len
    out SPL, YL					
	out SPH, YH
		
	clr r16					;len = 0
	clr r17					;
	std Y+1, r16			;store inside stack frame		

	ldi ZL, low(initial_array<<1)	;load Z with the address from the program memory
	ldi ZH, high(initial_array<<1)


	ldi XL, low(output_array)	; load X with address for output
	ldi XH, high(output_array)
	
	ldi r20, floor				; register for input floor
	;ldi r21, direction			; register for direction of travel (UP/DOWN)

load_loop:
	cpi r16, i_a_len			; check if all inital array elements have been loaded by comparing with initial len
	breq end_load_loop
	lpm r17, Z					; load next inital array element
	adiw Z, 2					; increment to next array element (Add 2 because word)

	rcall insert_request

	rjmp load_loop

end_load_loop:
	
	ldi ZL, low(enter_array<<1)	  ; load Z with the address from the program memory (reset pointer location to beginning)
	ldi ZH, high(enter_array<<1)

	clr r22	; clear enter_array_counter
enter_loop:

	cpi r22, e_a_len			; check if all elements of enter array have been insterted
	breq end_enter_loop
	lpm r17, Z+					; insert next element from enter array
	adiw Z, 1
	ldi XL, low(output_array)	; load X with address of output array from program memory (reset pointer to beginning)
	ldi XH, high(output_array)
	rcall insert_request
	inc r22						; increment enter_array_counter
	rjmp enter_loop

	end_enter_loop:
		rjmp end_enter_loop

;ASSUMPTION. List length fits in 1 byte, all inserted numbers fit inside 1 byte
;ASSUMPTION. If current floor is requested, request is ignored


;EXTENDED INSERT REQUEST

;parameters(address, len, input_floor, current_floor, direction), returns new_len in r16
;			X		r16		r17				r20				r21
; r18 holds counter
; r19 holds ith floor

insert_request:
	;prologue
	push YL
	push YH
	push r17
	push r18
	push r19
	push r20
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



	cp r17, r20			; if input floor is current floor, quit request
	breq skip_to_end
	cp r16, r18			; check if initial len is 0
	breq decide_direction 
	rjmp skip

decide_direction:
	clr r21
	cp r17, r20
	brlt down_search
	ldi r21, 1
	rjmp up_search

skip_to_end:
	rjmp end_insert_request

skip:
	cpi r21, 1			; direction check: if direction register holds 1, do  up_search, else down_search
	breq up_search
	rjmp down_search


up_search:
	cp r17, r20			; if input floor < current floor, jump to up_descending_loop, else up_ascending_loop
	brlt up_descending_loop
up_ascending_loop:
	cp r18, r16			; compare counter to len (check if end of list reached)
	breq end_search
	ld r19, X			; load floor from output array
	cp r17, r19			; check if input floor already exists
	breq end_insert_request	; quit if it does
	brlo insert_start	; if input floor lower than ith floor, insert 
	cp r19, r20			; compare ith floor to current floor
	brlo insert_start	; if ith < current, insert
	adiw X, 1			; increment output array
	inc r18				; increment counter
	rjmp up_ascending_loop

up_descending_loop:
	cp r18, r16			; compare counter to len (check if end of list reached)
	breq end_search		
	ld r19, X			; load floor from output array
	cp r17, r19			; check if input floor already exists
	breq end_insert_request	; quit if it does
	cp r19, r20			; compare ith floor to current floor
	brlo down_search	; if input floor < current floor and ith floor < current floor jmp to down search
	adiw X, 1			; increment output array
	inc r18				; increment counter
	rjmp up_descending_loop



down_search:
	cp r20, r17			; compare current floor < input floor
	brlt down_ascending_loop
down_descending_loop:
	cp r18, r16			; compare counter to len (check if end of list reached)
	breq end_search		
	ld r19, X			; load floor from output array
	cp r19, r17			; check if input floor already exists
	breq end_insert_request	; quit if it does
	brlo insert_start	; if current floor < input floor insert here
	cp r20, r19			; compare current floor to ith floor
	brlo insert_start	
	adiw X, 1			; increment output array
	inc r18				; increment counter
	rjmp down_descending_loop

down_ascending_loop:
	cp r18, r16			; compare counter to len (check if end of list reached)
	breq end_search		
	ld r19, X			; load floor from output array
	cp r19, r17			; check if input floor already exists
	breq end_insert_request	; quit if it does
	cp r20, r19			; compare ith floor to current floor
	brlo up_search		; if ith floor > current and input floor > current floor, jmp to upsearch
	adiw X, 1			; increment output array
	inc r18				; increment counter
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
			i++
	if(inserting_floor < current_floor)
		while(i < len)
			if(ith_floor < current_floor)
				down search		
			i++	
	shift insert at i  

down search
	if (inserting_floor < current_floor)
		while(i < len)
			if (ith < inserting)
				shift insert at i
			if (ith > current_floor)
				shift insert at i
			i++
	if(inserting_floor > current_floor)
		while(i < len)
			if(ith_floor > current_floor)
				up search
			i++		
	shift insert at i


r16 = len
r17 = inserting
r18 = i
r19 = ith
r20 = current floor
r21 = direction

*/
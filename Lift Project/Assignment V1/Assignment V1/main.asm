;
; Assignment V1.asm
;
; Created: 31/03/2019 9:58:14 AM
; Author : andrew fleming z5164462 anirudh rami z5164466
;


; Replace with your application code

;initial definitions and assignments

.dseg
Queue_len:
	.byte 1

.cseg

.def temp = r16
.def current_floor = r17
.def next_floor = r18
.def input_value = r19
.def lift_status = r20
/*
lift_status is a status register with the bits as flags
b0 = Stopped? 
b1 = Going up?
b2 = Doors Open?
b3 = Doors Opening?
b4 = Floors to come?
b5 = Flash on?
b6 = Emergency?
b7 = Halted?
*/
.equ stopped =		0b00000001
.equ goingUp =		0b00000010
.equ doorsOpen =	0b00000100
.equ opening =		0b00001000
.equ floorsToCome = 0b00010000
.equ flashOn =		0b00100000
.equ emergency =	0b01000000
.equ halted =		0b10000000


.def TimerL = r28
.def TimerH = r29


// Function to insert input floor into list 
// parameters Address of queue (X), input_floor (r21), current_floor (global), direction (b1 of r20, global)
// will set next_floor (global), direction (b1 of r20, global)

insert_request:

//prologue

push SREG
push temp
push r7
push r21 //parameter input
push r22 //local length

clr r7 // counter = 0

cp current_floor, r21 //if the current floor is the input floor, break to end
breq end_insert_request
lds r22, Queue_len

andi temp, goingUp	// check the goingUp bit
breq down_search	// if zero, sort down
rjmp up_search		//else sort up


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



end_insert_request:
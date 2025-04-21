;                                    CS12      CS11      CS10    
.equ CLK_NO = (1<<CS10)                ;0        0         1
.equ CLK_8 = (1<<CS11)                 ;0        1         0
.equ CLK_64 = ((1<<CS11)|(1<<CS10))    ;0        1         1
.equ CLK_256 =(1<<CS12)                ;1        0         0
.equ CLK_1024 = ((1<<CS12)|(1<<CS10))  ;1        0         1


.equ TM_QTR    = 15624               ; Timer1 ctc val for 0.25 s
.equ RED_QTR   = 20                  ; Red phase, 20×0.25 s = 5 s delay
.equ GREEN_QTR = 20                  ; Green phase, same as red
.equ YEL_QTR   = 8                   ; Yellow phase = 2 s
.equ WALK_QTR  = 20                  ; Walk phase = 5 s

;assign values using bitwise shit for the light states
.equ ST_RED    = 1                   ; val for red
.equ ST_GREEN  = 2                   ; val for green
.equ ST_YELLOW = 3                   ; val for yellow
.equ ST_WALK   = 4                   ; val for white/walk

;assign gpio ports 
; -----------------------------------------------------------------------------
.equ LED_RED   = PORTB0              ;  pb0, 8, red led 
.equ LED_YEL   = PORTB1              ; PB1, 9 on arduino   yellow  LED
.equ LED_GRN   = PORTB2              ;  pb2, 10 on arduino green LED
.equ LED_WALK  = PORTD7              ;  d7 on arduino traffic LED, not portb
.equ BUTTON_P  = PIND2               ;  pind2, button input

;global vars
; -----------------------------------------------------------------------------
.def stateReg = r18    ; current state of light val
.def  tickReg = r19    ;how many ticks left in curr state
.def walkReg= r20    ;register for button press
.def tickFlag = r21      ;.25s using ctc mode
.def temp = r16     ;temp register
;---------------------------------------------------

;:Vector table
;---------------------------------------------------

.org      0x0000
          rjmp       


          
.org Oc01Aaddr            
          rjmp tm0_ISR  
.org INT_VECTORS_SIZE

;---------------------------------------------------
;gpio setup
;---------------------------------------------------
;config the output for pb0-3 into temp using 
;bitshift wise operator
ldi  tmp, (1<<LED_RED) | (1<<LED_YEL) | (1<<LED_GRN)
out  DDRB, tmp    ;set bits ready for outputs

sbi  DDRD, LED_WALK      ;led on
cbi  PORTD, LED_WALK    ;led off

;---------------------------------------------------
;config button for pull up
;---------------------------------------------------

cbi DDRD, BUTTON_P      ;button in, if 0
sbi  PORTD, BUTTON_P      ;enable pull up if set, ie 1

;---------------------------------------------------
;ctc mode 
;
;Load TCCR1A & TCCR1B
          clr       temp
          sts       TCCR1A, temp   
          ldi       temp, (1<<WGM12) | CLK_256     ;actual ctc model, set when equal to 1, waveform gen
          sts       TCCR1B, temp
 
;Load OCR1AH:OCR1AL with stop count
          ldi       temp, HIGH(TM_QTR)              ;load OCR1AH value in
          sts         OCR1AH, temp                   ;CTC mode
          ldi         temp, LOW(TM_QTR)
          sts       OCR1AL , temp       ;Load TCNT1H:TCNT1L with initial count
       ;enable ovi interupt
          ldi       r20, (1<<OCIE0A)
          sts       TIMSK0, temp


;load current registers

ldi  stateReg, ST_RED    ;start sequence in red
ldi  tickReg, RED_QTR      ;set the timer for 5s
clr walkReg    ;no walk can be called yet
clr  tickFlag    ;clr all timer/ tick

sei

main_loop

; so we need to poll button?
;check timer if not red valid anymore
go to next state
; if not in either red green yellow its in walk








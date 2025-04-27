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
.def currentstateReg  = r18    ; current state of light val
.def  phaseReg = r19    ;how many ticks left in curr state
.def walkFlagReg= r20    ;register for button press
.def tickFlagReg = r21      ;.25s using ctc mode
.def temp = r16     ;temp register
;---------------------------------------------------

;:Vector table
;---------------------------------------------------

.org      0x0000
          rjmp    gpio_setup

          
.org OC1Aaddr
          rjmp tm1_ISR  
.org INT_VECTORS_SIZE

;---------------------------------------------------
gpio_setup:
;---------------------------------------------------
;config the output for pb0-3 into temp using 
;bitshift wise operator
ldi  temp, (1<<LED_RED) | (1<<LED_YEL) | (1<<LED_GRN)
out  DDRB, temp    ;set bits ready for outputs

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
          ldi       temp, (1<<OCIE1A)
          sts       TIMSK1, temp


;load current registers

ldi  currentstateReg , ST_RED    ;start sequence in red
ldi  phaseReg, RED_QTR      ;set the timer for 5s
clr walkFlagReg    ;no walk can be called yet
clr  tickFlagReg    ;clr all timer/ tick

sei
rjmp main_loop

main_loop:


wait_for_isr:
tst tickFlagReg
breq  wait_for_isr
clr tickFlagReg



;ceck if putton is low, pressed, set the ped flag
sbis          PIND, BUTTON_P                    ;if its set, skip the next instruction
ldi          walkFlagReg ,1                     ; executes when pind is pulled low, 0

dec           phaseReg                            ;decrease countdowtjmer
brne          main_loop
;Phase timer, timer left in this light, reached zero, branch based on current state, 
  cpi currentstateReg, ST_RED
  breq red_done
  cpi currentstateReg, ST_GREEN
  breq green_done
  cpi currentstateReg, ST_YELLOW
  breq yellow_done
  rjmp walk_done                     ; otherwise we must be in walk

red_done:
          tst          walkFlagReg                    ; was the ped requested
          breq to_green                    ;if not go to green
; if so enter walk phase
  clr walkFlagReg                    ; clear request
  ldi currentstateReg, ST_WALK              ; update state
  ldi phaseReg, WALK_QTR             ; set walk time
  sbi PORTB, LED_RED                 ; keep red on 
  sbi PORTD, LED_WALK                ; turn on pedestrian led
  rjmp main_loop                     ; back to main 


;switch light to green after red
to_green:
ldi currentstateReg, ST_GREEN                              ;set state to green 
ldi phaseReg, GREEN_QTR         ;time for green
cbi PORTB, LED_RED                     ;turn off red led
sbi PORTB, LED_GRN                    ;turn on green led
rjmp main_loop                              
 
green_done:
ldi currentstateReg, ST_YELLOW                              ;set state to yellow
ldi phaseReg, YEL_QTR
cbi       PORTB, LED_GRN
sbi       PORTB, LED_YEL
rjmp main_loop

yellow_done:
ldi currentstateReg, ST_RED                    ; go back to red
ldi phaseReg, RED_QTR                    ;set duration for red 5s
cbi PORTB, LED_YEL
sbi PORTB, LED_RED
rjmp main_loop


walk_done:
cbi          PORTD, LED_WALK                    ;turn off led for ped
ldi currentstateReg, ST_GREEN                              ;go to green state
ldi phaseReg, GREEN_QTR                    ;set green duration
cbi PORTB, LED_RED                    ;turn off red led
sbi PORTB,LED_GRN             ;turn on green           
rjmp main_loop


;so for the isr we will just set the timer flag 

tm1_ISR:
ldi       temp, 1
mov tickFlagReg , temp                          ;
reti








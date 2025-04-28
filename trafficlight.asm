;=========================================================================
; Project: AVR Traffic Light System with Crossing Lane & Blinking Walk
; Author:  Group 8: Adil Zaben
;                   Jeremias Serrat
;                   Liang Villarrubia Rio              
; Date:    2025-04-27 ;
; Descr:   Controls two perpendicular traffic lights (Main & Crossing)
;          with independent crosswalk buttons on INT0/INT1.
;          Main LEDs on PORTB, Crossing LEDs on PORTD (PD4-PD7).
;          Implements R-Y-G sequence with All-Red phases.
;          Pedestrian walk lights blink for the second half.
;          Uses Timer1 CTC interrupt with Prescaler 256.
;=========================================================================


; --- Constants: Timing (in seconds) ---
.equ GREEN_TIME_S  = 5                         ; Duration for Green light (seconds)
.equ YELLOW_TIME_S = 2                         ; Duration for Yellow light (seconds)
.equ ALL_RED_TIME_S= 1                         ; Duration for All-Red transition phase
.equ WALK_TIME_S   = 6                         ; Total duration for Walk light (seconds)
.equ POST_WALK_S   = 1                         ; Short Red duration after walk light off before next phase

; --- Constants: Blinking Walk Timing ---
.equ BLINK_TIME_S  = 3                         ; Duration for blinking part (Must be <= WALK_TIME_S / 2)
.equ WALK_SOLID_S  = WALK_TIME_S - BLINK_TIME_S; Duration for solid part

; --- Constants: Timer Configuration (Timer1 CTC Mode, Prescaler 256) ---
.equ T1_PRESCALER = (1<<CS12)|(0<<CS11)|(0<<CS10) ; clk/256
.equ T1_OCR1A_VAL      = 15624                 ; Target count for ~0.25 sec interrupt
.equ TICKS_PER_SECOND  = 4                     ; Number of timer interrupts per second

; --- Constants: Calculate Ticks from Seconds ---
.equ GREEN_TICKS     = GREEN_TIME_S * TICKS_PER_SECOND
.equ YELLOW_TICKS    = YELLOW_TIME_S * TICKS_PER_SECOND
.equ ALL_RED_TICKS   = ALL_RED_TIME_S * TICKS_PER_SECOND
.equ WALK_SOLID_TICKS= WALK_SOLID_S * TICKS_PER_SECOND
.equ BLINK_TICKS     = BLINK_TIME_S * TICKS_PER_SECOND
.equ POST_WALK_TICKS = POST_WALK_S * TICKS_PER_SECOND

; --- Constants: State Definitions ---
.equ STATE_MAIN_GREEN      = 0                 ; State: Main lane has Green, Crossing lane has Red
.equ STATE_MAIN_YELLOW     = 1                 ; State: Main lane has Yellow, Crossing lane has Red
.equ STATE_ALL_RED_1       = 2                 ; State: Both lanes have Red (after Main Yellow)
.equ STATE_CROSS_GREEN     = 3                 ; State: Main lane has Red, Crossing lane has Green
.equ STATE_CROSS_YELLOW    = 4                 ; State: Main lane has Red, Crossing lane has Yellow
.equ STATE_ALL_RED_2       = 5                 ; State: Both lanes have Red (after Crossing Yellow)
; Main Crosswalk States
.equ STATE_MAIN_XW_SOLID   = 6                 ; State: Both Red, Main Walk light is solid ON
.equ STATE_MAIN_XW_BLINK   = 7                 ; State: Both Red, Main Walk light is blinking
.equ STATE_MAIN_XW_END     = 8                 ; State: Both Red, Main Walk light is OFF (post-walk delay)
; Crossing Crosswalk States
.equ STATE_CROSS_XW_SOLID  = 9                 ; State: Both Red, Crossing Walk light is solid ON
.equ STATE_CROSS_XW_BLINK  = 10                ; State: Both Red, Crossing Walk light is blinking
.equ STATE_CROSS_XW_END    = 11                ; State: Both Red, Crossing Walk light is OFF (post-walk delay)

; --- Constants: Pin Definitions ---
; Port B: Main Traffic Light LEDs (Lane 1)
.equ LED1_PORT     = PORTB                     ; I/O Register: Port B Data Register (for Main LEDs)
.equ LED1_DDR      = DDRB                      ; I/O Register: Port B Data Direction Register
.equ RED1_PIN      = PB3                       ; Pin Assignment: Main Red LED connected to Port B, bit 3
.equ YELLOW1_PIN   = PB2                       ; Pin Assignment: Main Yellow LED connected to Port B, bit 2
.equ GREEN1_PIN    = PB1                       ; Pin Assignment: Main Green LED connected to Port B, bit 1
.equ WALK1_PIN     = PB0                       ; Pin Assignment: Main Walk LED connected to Port B, bit 0
; Port D: Crossing Traffic Light LEDs (Lane 2) & Buttons
; *** LEDs moved to PD4-PD7 to avoid conflict with Buttons on PD2/PD3 ***
.equ LED2_PORT     = PORTD                     ; I/O Register: Port D Data Register (for Crossing LEDs & Buttons)
.equ LED2_DDR      = DDRD                      ; I/O Register: Port D Data Direction Register
.equ RED2_PIN      = PD7                       ; Pin Assignment: Crossing Red LED connected to Port D, bit 7
.equ YELLOW2_PIN   = PD6                       ; Pin Assignment: Crossing Yellow LED connected to Port D, bit 6
.equ GREEN2_PIN    = PD5                       ; Pin Assignment: Crossing Green LED connected to Port D, bit 5
.equ WALK2_PIN     = PD4                       ; Pin Assignment: Crossing Walk LED connected to Port D, bit 4
; Buttons on PORTD
.equ BUTTON_PORT   = PORTD                     ; I/O Register: Port D Data Register (used for Button pull-ups)
.equ BUTTON_PINREG = PIND                      ; I/O Register: Port D Input Pins Address (for reading button state)
.equ BUTTON_DDR    = DDRD                      ; I/O Register: Port D Data Direction Register
.equ BUTTON1_PIN   = PD2                       ; Pin Assignment: Main Button (Crosswalk 1) connected to Port D, bit 2 (INT0)
.equ BUTTON2_PIN   = PD3                       ; Pin Assignment: Crossing Button (Crosswalk 2) connected to Port D, bit 3 (INT1)

; --- Register Definitions (.def) ---
.def temp          = r16                       ; Register Alias: r16 used as a general-purpose temporary register
.def tick_counter  = r17                       ; Register Alias: r17 used to count down timer ticks for state duration
.def flags         = r18                       ; Register Alias: r18 used to store various status flags (bit-field)
.def state_reg     = r19                       ; Register Alias: r19 used to store the current state number

; --- Bit definitions for 'flags' register (r18) ---
.equ FLAG_CROSSWALK_REQ1  = 0                  ; Bit 0: Set when Main crosswalk button is pressed
.equ FLAG_CROSSWALK_REQ2  = 1                  ; Bit 1: Set when Crossing crosswalk button is pressed
.equ FLAG_BLINK_STATE     = 2                  ; Bit 2: Toggles ON/OFF during walk light blinking phase
.equ FLAG_DEBOUNCE1_BUSY  = 3                  ; Bit 3: Set while Button 1 is being debounced
.equ FLAG_DEBOUNCE2_BUSY  = 4                  ; Bit 4: Set while Button 2 is being debounced

;=========================================================================
; Vector Table
;=========================================================================

.org 0x0000
    rjmp  RESET
.org INT0addr
    rjmp  EXT_INT0_ISR
.org INT1addr
    rjmp  EXT_INT1_ISR
.org OC1Aaddr
    rjmp  TIMER1_COMPA_ISR

.org INT_VECTORS_SIZE
;-------------------------------------------------------------------------

;=========================================================================
; Main Program
;=========================================================================
RESET:
; Clear state variables/flags registers
    clr   state_reg
    clr   tick_counter
    clr   flags

; Call setup routines
    rcall io_init
    rcall timer1_init
    rcall interrupts_init

; Set initial state
    ldi   state_reg, STATE_MAIN_GREEN
    ldi   tick_counter, GREEN_TICKS
    rcall update_lights

; Enable Global Interrupts
    sei

main_loop:
    rjmp  main_loop
;-------------------------------------------------------------------------

;=========================================================================
; Initialization Subroutines
;=========================================================================

io_init:
; Desc: Configures Data Direction Registers (DDR) and Pull-ups for I/O.
;       PORTB = Main LEDs Output
;       PORTD = Cross LEDs (PD4-7 Output) & Buttons (PD2-3 Input Pullup)
;-------------------------------------------------------------------------

; --- Configure Main Lane LEDs (PORTB - Output) ---
; Set PB0, PB1, PB2, PB3 as outputs in DDRB
    ldi   temp, (1<<RED1_PIN)|(1<<YELLOW1_PIN)|(1<<GREEN1_PIN)|(1<<WALK1_PIN)
    out   LED1_DDR, temp                  ; Write pattern to DDRB

; Set initial state for Main LEDs (All Off)
    clr   temp                            ; Load 0 into temp
    out   LED1_PORT, temp                 ; Write 0 to PORTB

; --- Configure Crossing Lane LEDs (PORTD PD4-PD7 - Output) ---
; Set PD4, PD5, PD6, PD7 as outputs in DDRD
; (Leave PD0-PD3 as inputs for now, will be set below)
    ldi   temp, (1<<RED2_PIN)|(1<<YELLOW2_PIN)|(1<<GREEN2_PIN)|(1<<WALK2_PIN)
    out   LED2_DDR, temp                  ; Write LED output bits to DDRD (overwrites previous DDRD state)
                                          

; Enable Pull-ups for BUTTON1_PIN(PD2), BUTTON2_PIN(PD3)
; Set initial state for Crossing LEDs (All Off)
; Combine setting pull-ups (bit=1) and turning LEDs off (bit=0)
    ldi   temp, (0<<WALK2_PIN)|(0<<GREEN2_PIN)|(0<<YELLOW2_PIN)|(0<<RED2_PIN) | (1<<BUTTON2_PIN)|(1<<BUTTON1_PIN)
    out   LED2_PORT, temp                 ; Write combined pattern to PORTD

    ret
;-------------------------------------------------------------------------

timer1_init:
; Desc: Configures Timer1 for CTC mode interrupt at ~0.25s.
;       Uses Prescaler 256.
;-------------------------------------------------------------------------
    clr   temp                            ; COM1A/B = 0
    sts   TCCR1A, temp
    ldi   temp, (1<<WGM12) | T1_PRESCALER ; Mode 4 (CTC), Prescaler 256
    sts   TCCR1B, temp
    ldi   temp, high(T1_OCR1A_VAL)        ; Load OCR1A = 15624
    sts   OCR1AH, temp
    ldi   temp, low(T1_OCR1A_VAL)
    sts   OCR1AL, temp
    ldi   temp, (1<<OCIE1A)               ; Enable Timer1 OCIE1A
    sts   TIMSK1, temp
    ret
;-------------------------------------------------------------------------

interrupts_init:
; Desc: Configures and enables Timer1 Compare, INT0, and INT1 interrupts.
;-------------------------------------------------------------------------
    ldi   temp, (1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00) ; INT0/INT1 Falling edge
    sts   EICRA, temp
    ldi   temp, (1<<INT1)|(1<<INT0)        ; Enable INT0, INT1
    out   EIMSK, temp
    ret
;-------------------------------------------------------------------------

;=========================================================================
; Interrupt Service Routines (ISRs)
;=========================================================================

TIMER1_COMPA_ISR:
; Desc: Timer1 Compare Match ISR (~0.25s interval). Handles state timing,
;       blinking logic, and button debounce release.
; Clobbers: temp (r16), r24, r25, flags (r18), tick_counter (r17), state_reg (r19)
;-------------------------------------------------------------------------
    push  temp                           ; Save r16
    in    temp, SREG                     ; Save SREG
    push  temp
    push  r24                            ; Save r24
    push  r25                            ; Save r25

; --- Debounce Logic ---
    sbrc  flags, FLAG_DEBOUNCE1_BUSY
    rcall handle_debounce1
    
    sbrc  flags, FLAG_DEBOUNCE2_BUSY
    rcall handle_debounce2

; --- Blinking Logic ---
    mov   r24, state_reg ; Copy state (r19) to r24 for compare
   
    cpi   r24, STATE_MAIN_XW_BLINK
    breq  toggle_blink
    
    cpi   r24, STATE_CROSS_XW_BLINK
    breq  toggle_blink
    
    rjmp  timer_decr_state              ; Not a blink state

toggle_blink:
    ldi   r24, (1<<FLAG_BLINK_STATE)    ; Mask in r24
    eor   flags, r24                    ; eor r18, r24
    
    rcall update_lights                 ; Update LEDs immediately
; Fall through to timer_decr_state

timer_decr_state:
; --- State Timing Logic ---
    dec   tick_counter                  ; Decrement r17
    brne  timer_isr_exit                ; If counter > 0, exit

; --- State Transition Logic (Timer Expired) ---
    rcall update_state                  ; Updates r19, r17, r18
    rcall update_lights                 ; Uses r19, r18

timer_isr_exit:
    pop   r25                           ; Restore registers
    pop   r24
    pop   temp                          ; Restore SREG
    out   SREG, temp
    pop   temp                          ; Restore r16
    reti                                ; Return from interrupt

handle_debounce1:
; Desc: Clears debounce busy flag 1 and robustly re-enables INT0/INT1
;       based on whether INT1 is currently debouncing.
; Uses: r24, r25 (clobbered), flags (r18) Modifies: flags (r18), EIMSK

; 1. Clear busy flag 1 in flags (r18)
    ldi   r25, (1<<FLAG_DEBOUNCE1_BUSY)
    com   r25                           ; Create mask to clear only bit FLAG_DEBOUNCE1_BUSY
    and   flags, r25

; 2. Determine desired EIMSK state
;    Default: Enable both INT0 and INT1
    ldi   r24, (1<<INT1)|(1<<INT0)

; 3. Check if Button 2 (INT1) is STILL being debounced
    sbrc  flags, FLAG_DEBOUNCE2_BUSY    ; Skip next instruction if FLAG_DEBOUNCE2_BUSY is CLEAR
    ldi   r24, (1<<INT0)                ; If Button 2 IS busy, only enable INT0 for now

; 4. Write the determined state to EIMSK
    sts   EIMSK, r24
    ret

handle_debounce2:
; Desc: Clears debounce busy flag 2 and robustly re-enables INT0/INT1
;       based on whether INT0 is currently debouncing.
; Uses: r24, r25 (clobbered), flags (r18) Modifies: flags (r18), EIMSK

; 1. Clear busy flag 2 in flags (r18)
    ldi   r25, (1<<FLAG_DEBOUNCE2_BUSY)
    com   r25                           ; Create mask to clear only bit FLAG_DEBOUNCE2_BUSY
    and   flags, r25

; 2. Determine desired EIMSK state
;    Default: Enable both INT0 and INT1
    ldi   r24, (1<<INT1)|(1<<INT0)

; 3. Check if Button 1 (INT0) is STILL being debounced
    sbrc  flags, FLAG_DEBOUNCE1_BUSY    ; Skip next instruction if FLAG_DEBOUNCE1_BUSY is CLEAR
    ldi   r24, (1<<INT1)                ; If Button 1 IS busy, only enable INT1 for now

; 4. Write the determined state to EIMSK
    sts   EIMSK, r24
    ret
;-------------------------------------------------------------------------

EXT_INT0_ISR: ; Button 1 on PD2 (Main Lane Crosswalk)
; Desc: Sets request flag 1, handles debouncing start.
; Clobbers: temp (r16), r24 Modifies: flags (r18), EIMSK
;-------------------------------------------------------------------------
    push  temp                          ; r16
    in    temp, SREG
    push  temp
    push  r24                           ; Save r24

    mov   r24, flags                    ; Check if already busy
    andi  r24, (1<<FLAG_DEBOUNCE1_BUSY)
    brne  int0_exit                     ; If busy, exit

    lds   r24, EIMSK                    ; Disable INT0 temporarily
    andi  r24, ~(1<<INT0)
    sts   EIMSK, r24

    ldi   r24, (1<<FLAG_DEBOUNCE1_BUSY)|(1<<FLAG_CROSSWALK_REQ1) ; Mask for bits to set
    or    flags, r24                    ; OR flags (r18) with mask

int0_exit:
    pop   r24                           ; Restore r24
    pop   temp                          ; Restore SREG
    out   SREG, temp
    pop   temp                          ; Restore r16
    reti
;-------------------------------------------------------------------------

EXT_INT1_ISR: ; Button 2 on PD3 (Crossing Lane Crosswalk)
; Desc: Sets request flag 2, handles debouncing start.
; Clobbers: temp (r16), r24 Modifies: flags (r18), EIMSK
;-------------------------------------------------------------------------
    push  temp                          ; r16
    in    temp, SREG
    push  temp
    push  r24                           ; Save r24

    mov   r24, flags                    ; Check if already busy
    andi  r24, (1<<FLAG_DEBOUNCE2_BUSY)
    brne  int1_exit                     ; If busy, exit

    lds   r24, EIMSK                    ; Disable INT1 temporarily
    andi  r24, ~(1<<INT1)
    sts   EIMSK, r24

    ldi   r24, (1<<FLAG_DEBOUNCE2_BUSY)|(1<<FLAG_CROSSWALK_REQ2) ; Mask for bits to set
    or    flags, r24                    ; OR flags (r18) with mask

int1_exit:
    pop   r24                           ; Restore r24
    pop   temp                          ; Restore SREG
    out   SREG, temp
    pop   temp                          ; Restore r16
    reti
;-------------------------------------------------------------------------


;=========================================================================
; State Logic and Helper Subroutines
;=========================================================================

update_state:
; Desc: Determines next state based on current state (r19) and flags (r18).
;       Loads ticks (r17) for the new state.
; Uses: state_reg (r19), flags (r18)
; Modifies: state_reg (r19), tick_counter (r17), flags (r18)
; Clobbers: temp (r16)
;-------------------------------------------------------------------------
    ; Equivalent to: switch (state_reg) {

    cpi   state_reg, STATE_MAIN_GREEN      ; case STATE_MAIN_GREEN:
    breq  next_state_MG                    ;   goto next_state_MG; break;

    cpi   state_reg, STATE_MAIN_YELLOW     ; case STATE_MAIN_YELLOW:
    breq  next_state_MY                    ;   goto next_state_MY; break;

    cpi   state_reg, STATE_ALL_RED_1       ; case STATE_ALL_RED_1:
    breq  next_state_AR1                   ;   goto next_state_AR1; break;

    cpi   state_reg, STATE_CROSS_GREEN     ; case STATE_CROSS_GREEN:
    breq  next_state_CG                    ;   goto next_state_CG; break;

    cpi   state_reg, STATE_CROSS_YELLOW    ; case STATE_CROSS_YELLOW:
    breq  next_state_CY                    ;   goto next_state_CY; break;

    cpi   state_reg, STATE_ALL_RED_2       ; case STATE_ALL_RED_2:
    breq  next_state_AR2                   ;   goto next_state_AR2; break;

    ; Main Crosswalk States
    cpi   state_reg, STATE_MAIN_XW_SOLID   ; case STATE_MAIN_XW_SOLID:
    breq  next_state_MXS                   ;   goto next_state_MXS; break;

    cpi   state_reg, STATE_MAIN_XW_BLINK   ; case STATE_MAIN_XW_BLINK:
    breq  next_state_MXB                   ;   goto next_state_MXB; break;

    cpi   state_reg, STATE_MAIN_XW_END     ; case STATE_MAIN_XW_END:
    breq  next_state_MXE                   ;   goto next_state_MXE; break;

    ; Crossing Crosswalk States
    cpi   state_reg, STATE_CROSS_XW_SOLID  ; case STATE_CROSS_XW_SOLID:
    breq  next_state_CXS                   ;   goto next_state_CXS; break;

    cpi   state_reg, STATE_CROSS_XW_BLINK  ; case STATE_CROSS_XW_BLINK:
    breq  next_state_CXB                   ;   goto next_state_CXB; break;

    cpi   state_reg, STATE_CROSS_XW_END    ; case STATE_CROSS_XW_END:
    breq  next_state_CXE                   ;   goto next_state_CXE; break;

    rjmp  next_state_AR2                   ; default: // Or error case
                                           ;   goto next_state_AR2;
                                           ; } // End of switch

next_state_MG: ; Transition FROM Main Green
    ldi   state_reg, STATE_MAIN_YELLOW     ; Set next state to Main Yellow
    ldi   tick_counter, YELLOW_TICKS       ; Load duration for Yellow light
    ret                                    ; Return (back to Timer ISR)

next_state_MY: ; Transition FROM Main Yellow
    ldi   state_reg, STATE_ALL_RED_1       ; Set next state to All Red (phase 1, after Main Yellow)
    ldi   tick_counter, ALL_RED_TICKS      ; Load duration for All Red
    ret

next_state_AR1: ; Transition FROM All Red (phase 1 - after Main Yellow)
    ; Check if the Crossing Lane needs service before giving it Green
    sbrc  flags, FLAG_CROSSWALK_REQ2       ; Check if Crossing Lane button was pressed (Skip next if clear)
    rjmp  start_cross_xw                   ; If pressed, jump to start Crossing crosswalk sequence
    ; If Crossing button was NOT pressed:
    ldi   state_reg, STATE_CROSS_GREEN     ; Set next state to Crossing Green
    ldi   tick_counter, GREEN_TICKS        ; Load duration for Green light
    ret

next_state_CG: ; Transition FROM Crossing Green
    ldi   state_reg, STATE_CROSS_YELLOW    ; Set next state to Crossing Yellow
    ldi   tick_counter, YELLOW_TICKS       ; Load duration for Yellow light
    ret

next_state_CY: ; Transition FROM Crossing Yellow
    ldi   state_reg, STATE_ALL_RED_2       ; Set next state to All Red (phase 2, after Crossing Yellow)
    ldi   tick_counter, ALL_RED_TICKS      ; Load duration for All Red
    ret

next_state_AR2: ; Transition FROM All Red (phase 2 - after Crossing Yellow)
    ; Check if the Main Lane needs service before giving it Green
    sbrc  flags, FLAG_CROSSWALK_REQ1       ; Check if Main Lane button was pressed (Skip next if clear)
    rjmp  start_main_xw                    ; If pressed, jump to start Main crosswalk sequence
    ; If Main button was NOT pressed:
    ldi   state_reg, STATE_MAIN_GREEN      ; Set next state to Main Green
    ldi   tick_counter, GREEN_TICKS        ; Load duration for Green light
    ret

start_main_xw: ; Entered FROM All Red phase 2 (AR2) if Main button was pressed
    ; Service the Main crosswalk request
    ldi   temp, (1<<FLAG_CROSSWALK_REQ1)   ; Prepare mask for Req1 bit
    com   temp                             ; Invert mask (to 1111_1110)
    and   flags, temp                      ; Clear Req1 flag in flags register (r18), acknowledging service
    ldi   state_reg, STATE_MAIN_XW_SOLID   ; Set next state to Main Walk Solid
    ldi   tick_counter, WALK_SOLID_TICKS   ; Load duration for solid walk phase
    ret

start_cross_xw: ; Entered FROM All Red phase 1 (AR1) if Crossing button was pressed
    ; Service the Crossing crosswalk request
    ldi   temp, (1<<FLAG_CROSSWALK_REQ2)   ; Prepare mask for Req2 bit
    com   temp                             ; Invert mask (to 1111_1101)
    and   flags, temp                      ; Clear Req2 flag in flags register (r18), acknowledging service
    ldi   state_reg, STATE_CROSS_XW_SOLID  ; Set next state to Crossing Walk Solid
    ldi   tick_counter, WALK_SOLID_TICKS   ; Load duration for solid walk phase
    ret

next_state_MXS: ; Transition FROM Main Walk Solid
    ; First part of walk cycle done, move to blinking part
    ldi   state_reg, STATE_MAIN_XW_BLINK   ; Set next state to Main Walk Blinking
    ldi   tick_counter, BLINK_TICKS        ; Load duration for blinking phase
    ret

next_state_MXB: ; Transition FROM Main Walk Blinking
    ; Blinking part done, move to short "all red" delay after walk
    ldi   state_reg, STATE_MAIN_XW_END     ; Set next state to Main Walk End (post-walk delay)
    ldi   tick_counter, POST_WALK_TICKS    ; Load duration for post-walk delay
    ret

next_state_MXE: ; Transition FROM Main Walk End (post-walk delay)
    ; Finished Main crosswalk sequence, go to All Red before potentially giving Crossing Green
    ldi   state_reg, STATE_ALL_RED_1       ; Set next state back to All Red (phase 1)
    ldi   tick_counter, ALL_RED_TICKS      ; Load duration for All Red
    ret

next_state_CXS: ; Transition FROM Crossing Walk Solid
    ; First part of walk cycle done, move to blinking part
    ldi   state_reg, STATE_CROSS_XW_BLINK  ; Set next state to Crossing Walk Blinking
    ldi   tick_counter, BLINK_TICKS        ; Load duration for blinking phase
    ret

next_state_CXB: ; Transition FROM Crossing Walk Blinking
    ; Blinking part done, move to short "all red" delay after walk
    ldi   state_reg, STATE_CROSS_XW_END    ; Set next state to Crossing Walk End (post-walk delay)
    ldi   tick_counter, POST_WALK_TICKS    ; Load duration for post-walk delay
    ret

next_state_CXE: ; Transition FROM Crossing Walk End (post-walk delay)
    ; Finished Crossing crosswalk sequence, go to All Red before potentially giving Main Green
    ldi   state_reg, STATE_ALL_RED_2       ; Set next state back to All Red (phase 2)
    ldi   tick_counter, ALL_RED_TICKS      ; Load duration for All Red
    ret
;-------------------------------------------------------------------------

update_lights:
; Desc: Sets LED outputs on LED1_PORT (PORTB) and LED2_PORT (PORTD)
;       based on current state (r19) and blink flag (in r18).
; Uses: state_reg (r19), flags (r18)
; Modifies: LED1_PORT (PORTB), LED2_PORT (PORTD)
; Clobbers: r20, r21
;-------------------------------------------------------------------------
    push  r20                               ; Save registers used locally
    push  r21                               
    clr   r20                               ; r20 = pattern for PORTB (Main)
    clr   r21                               ; r21 = pattern for PORTD (Crossing)

    ; Equivalent to: switch (state_reg) {

    cpi   state_reg, STATE_MAIN_GREEN       ; case STATE_MAIN_GREEN:
    breq  set_MG_lights                     ;   goto set_MG_lights; break;

    cpi   state_reg, STATE_MAIN_YELLOW      ; case STATE_MAIN_YELLOW:
    breq  set_MY_lights                     ;   goto set_MY_lights; break;

    cpi   state_reg, STATE_ALL_RED_1        ; case STATE_ALL_RED_1:
    breq  set_AR_lights                     ;   goto set_AR_lights; break; // Fallthrough handled by multiple cases jumping here

    cpi   state_reg, STATE_CROSS_GREEN      ; case STATE_CROSS_GREEN:
    breq  set_CG_lights                     ;   goto set_CG_lights; break;

    cpi   state_reg, STATE_CROSS_YELLOW     ; case STATE_CROSS_YELLOW:
    breq  set_CY_lights                     ;   goto set_CY_lights; break;

    cpi   state_reg, STATE_ALL_RED_2        ; case STATE_ALL_RED_2:
    breq  set_AR_lights                     ;   goto set_AR_lights; break; // Fallthrough

    ; Main Crosswalk States
    cpi   state_reg, STATE_MAIN_XW_SOLID    ; case STATE_MAIN_XW_SOLID:
    breq  set_MXS_lights                    ;   goto set_MXS_lights; break;

    cpi   state_reg, STATE_MAIN_XW_BLINK    ; case STATE_MAIN_XW_BLINK:
    breq  set_MXB_lights                    ;   goto set_MXB_lights; break;

    cpi   state_reg, STATE_MAIN_XW_END      ; case STATE_MAIN_XW_END:
    breq  set_AR_lights                     ;   goto set_AR_lights; break; // Fallthrough

    ; Crossing Crosswalk States
    cpi   state_reg, STATE_CROSS_XW_SOLID   ; case STATE_CROSS_XW_SOLID:
    breq  set_CXS_lights                    ;   goto set_CXS_lights; break;

    cpi   state_reg, STATE_CROSS_XW_BLINK   ; case STATE_CROSS_XW_BLINK:
    breq  set_CXB_lights                    ;   goto set_CXB_lights; break;

    cpi   state_reg, STATE_CROSS_XW_END     ; case STATE_CROSS_XW_END:
    breq  set_AR_lights                     ;   goto set_AR_lights; break; // Fallthrough

    rjmp  write_patterns                    ; default: // Or error case
                                            ;   goto write_patterns;
                                            ; } // End of switch


set_MG_lights: ; Main Green (PORTB), Cross Red (PORTD)
    sbr   r20, (1<<GREEN1_PIN)                  ; Set GREEN1 bit in r20
    sbr   r21, (1<<RED2_PIN)                    ; Set RED2 bit (PD4) in r21
    
    rjmp  write_patterns

set_MY_lights: ; Main Yellow (PORTB), Cross Red (PORTD)
    sbr   r20, (1<<YELLOW1_PIN)                 ; Set YELLOW1 bit in r20
    sbr   r21, (1<<RED2_PIN)                    ; Set RED2 bit (PD4) in r21
    
    rjmp  write_patterns

set_AR_lights: ; All Red (PORTB & PORTD)
    sbr   r20, (1<<RED1_PIN)                    ; Set RED1 bit in r20
    sbr   r21, (1<<RED2_PIN)                    ; Set RED2 bit (PD4) in r21

    rjmp  write_patterns

set_CG_lights: ; Main Red (PORTB), Cross Green (PORTD)
    sbr   r20, (1<<RED1_PIN)                    ; Set RED1 bit in r20
    sbr   r21, (1<<GREEN2_PIN)                  ; Set GREEN2 bit (PD6) in r21

    rjmp  write_patterns

set_CY_lights: ; Main Red (PORTB), Cross Yellow (PORTD)
    sbr   r20, (1<<RED1_PIN)                    ; Set RED1 bit in r20
    sbr   r21, (1<<YELLOW2_PIN)                 ; Set YELLOW2 bit (PD5) in r21

    rjmp  write_patterns

set_MXS_lights: ; All Red, Main Walk Solid ON (PORTB)
    sbr   r20, (1<<RED1_PIN)|(1<<WALK1_PIN)     ; Set RED1, WALK1 bits in r20
    sbr   r21, (1<<RED2_PIN)                    ; Set RED2 bit (PD4) in r21

    rjmp  write_patterns

set_MXB_lights: ; All Red, Main Walk Blinking (PORTB)
    sbr   r20, (1<<RED1_PIN)                    ; Set RED1 bit in r20
    sbr   r21, (1<<RED2_PIN)                    ; Set RED2 bit (PD4) in r21

    sbrc  flags, FLAG_BLINK_STATE               ; Check blink flag in r18
    sbr   r20, (1<<WALK1_PIN)                   ; Set WALK1 bit in r20 if flag is 1

    rjmp  write_patterns

set_CXS_lights: ; All Red, Cross Walk Solid ON (PORTD)
    sbr   r20, (1<<RED1_PIN)                    ; Set RED1 bit in r20
    sbr   r21, (1<<RED2_PIN)|(1<<WALK2_PIN)     ; Set RED2(PD4), WALK2(PD7) bits in r21

    rjmp  write_patterns

set_CXB_lights: ; All Red, Cross Walk Blinking (PORTD)
    sbr   r20, (1<<RED1_PIN)                    ; Set RED1 bit in r20
    sbr   r21, (1<<RED2_PIN)                    ; Set RED2 bit (PD4) in r21

    sbrc  flags, FLAG_BLINK_STATE               ; Check blink flag in r18
    sbr   r21, (1<<WALK2_PIN)                   ; Set WALK2 bit (PD7) in r21 if flag is 1

    rjmp  write_patterns

write_patterns:
; r20 contains the pattern for PORTB (Main LEDs)
; r21 contains the pattern for PORTD LEDs (PD4-PD7), with PD0-PD3 currently 0

; --- PRESERVE PULL-UPS ON PORTD ---
; Before writing r21 to PORTD, ensure bits for button pull-ups (PD2, PD3) are set to 1.
; We can do this by ORing r21 with a mask for the button pins.
    ori   r21, (1<<BUTTON2_PIN)|(1<<BUTTON1_PIN)
; Now r21 contains the correct state for LEDs (PD4-7) AND ensures PD2/PD3 are HIGH for pull-ups.

; --- Write patterns to physical ports ---
    out   LED1_PORT, r20                       ; Write r20 to PORTB
    out   LED2_PORT, r21                       ; Write modified r21 (LEDs + Pull-ups) to PORTD

    pop   r21                                  ; Restore registers
    pop   r20
    ret
;-------------------------------------------------------------------------
;-------------------------------------------------------------------------

;=========================================================================
; End of Program
;=========================================================================
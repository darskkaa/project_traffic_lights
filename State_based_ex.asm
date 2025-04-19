;
; timer_blink_intterr_traffic.asm
;
; Created: 4/19/2025 3:29:01 PM
; Author  Adil
;


.equ      TM_COUNT = 155
.equ      LED_COUNT = 25
.equ      BTN_BOUNCE = 40

.equ      START_BLINK = 3
.equ      MIN_BLINK = 1
.equ      MAX_BLINK = 10

.equ      LED_PIN = PORTB2


.equ      BTN_UP = PIND2
.equ      BTN_DOWN = PIND3
 ;PROGRAM STATE ENUM   
.equ       ST_WAIT_INP = 1
.equ      ST_START_BLINK -2
.equ      ST_BLINK_ON = 2
.equ      ST_BLINK_OFF = 3
          ;GLOBAL VARS
.equ      state = 0x0100
.equ      blinkCount = 0x0101
.equ      blinkCounter = 0x0102
.equ      ledTimer = 0x0103
.equ      buttonBounce = 0x0105

..def     temp = r16
.def      retState = r17

;:Vector table
;---------------------------------------------------

.org      0x0000
          jmp       main

.org INT0addr
          jmp   btn_up_ISR
          
 .org INT1addr
          jmp       btn_down_ISR
          
.org Oc0addr
          jmp tm0_ISR  
.org INT_VECTORS_SIZE


main:
          ldi       r16, ST_WAIT_INP
          sts       state, r16

          ldi       r16 ,3
          sts       blinkCount, r16

          call gpio_setup

          call tm0_setip
          call f_start_blink
          sei

 main_loop:
          lds       temp, timerFlag
          tst       temp
          breq      end_main

          lds       temp, state

          cpi       temp, ST_START_BLINK
          breq      case_start_blink

          cpi       temp, ST_BLINK_ON
          breq      case_blink_on

          cpi       temp, ST_BLINK_OFF
                    breq      case_blink_off

                    rjmp end_case

case_start_blink:

          call      f_start_blink
          rjmp      end_case

case_blink_on:
          call      f_blink_on
          rjmp      end_case
case_blink_off:
          call      f_blink_off

end_case:
          clr       temp
          sts       timerFlag, temp
end_main:
          rjmp      main_loop

gpio_setup:

.equ      INT0_FALL = (1<<ISC01) |(0<<ISC00)
.equ      INT0_RISE = (1<<ISC01) |(1<<ISC00)

.equ      INT1_FALL = (1<<ISC11) |(0<<ISC10)

.equ      IN1T_RISE = (1<<ISC11) |(1<<ISC10)

;config output

          sbi       DDRB, DDB2
          ;config input
          cbi       DDRB, BTN_UP
          cbi       PORTD, BTN_UP
          sbi       EIMSK, INT0
          ldi       r20, INT0_RISE

          ;config input

          cbi       DDRB, BTN_DOWN
          sbi       PORTD, BTN_DOWN
          sbi       EIMSK, INT1
          ori       r20, INT1_FALL

          sts       EIRCA, r20
          ret
tm0_setup:
.equ CLK_NO = (1<<CS10)                ;0        0         1
.equ CLK_8 = (1<<CS11)                 ;0        1         0
.equ CLK_64 = ((1<<CS11)|(1<<CS10))    ;0        1         1
.equ CLK_256 =(1<<CS12)                ;1        0         0
.equ CLK_1024 = ((1<<CS12)|(1<<CS10))  ;1        0         1

          ;set counter
          clr       r20
          out       TCNT0, r20

          ldi       r20, TM_COUNT
          out       OCR0A, r20

          ;set reg mode A 

          ldi       r20, (1<<WGM01)
          out       TCCR0A, r20

          ;set mode and clk b reg

          ldi       r20, CLK_1024
          out       TCCR0B, r20

          ;enable ovi interupt
          ldi       r20, (1<<OCIE0A)
          sts       TIMSK0, r20

          ret


f_reset_timer:

          push      temp
           ldi      temp, LED_COUNT
           sts      ledTimer, temp
           pop      temp
           ret


f_start_blink:

          push      temp
          lds       temp, blinkCount
          sts       blinkCounter, temp

          call      f_reset_timer

          ldi       temp, ST_BLINK_ON
          sts       state, temp
          pop       temp
          ret

f_blink_on:
          push      retSTate
          push      temp

          ldi       retSTate, ST_BLINk_ON

          sbi       PORTB, LED_PIN

          lds       temp, ledTimer
          tst       ledTimer
          brne      blink_on_ret

          call      f_reset_timer

          ldi       retState, ST_BLINK_OFF

blink_on_ret:

          sts       state, retSTate


          pop       temp
          pop       retState

          ret


f_blink_off:

          push      retSTate
          push      temp
          ldi       retSTate, ST_BLINK_OFF

          cbi       PORTB, LED_PIN

          lds       temp, ledTimer
          tst       ledTimer
          brne      blink_off_ret

          lds       temp, blinkCounter
          dec       temp
          sts       blinkCounter, temp

          breq      blink_off_reset

          call      f_reset_timer

          ldi       retState, ST_BLINK_ON

          rjmp      blink_off_ret

blink_off_reset:

          ldi       retState, ST_WAIT_INP

blink_off_ret:
          sts       state, retSTate

          pop       temp
          pop       retSTate

          ret


tm0_ISR:
          push      temp

          ldi       temp, 1
          sts       timerFlag, temp

          lds       temp, ledTimer
          tst       temp
          breq      chk_bounce

          dec       temp
          sts       ledTimer, temp

chk_bounce:

          lds       temp, buttonBounce
          tst       temp
          breq      tm0_ret

          dec       temp
          sts       buttonBounce, temp

tm_0_ret:

          clr       temp
          out       TCNT0, temp

          pop       temp

          reti
btn_up_ISR:

          push      retState
          push      temp

          ldi       retState, ST_WAIT_INP

          lds       temp, buttonBounce

          tst       temp
          brne      btn_up_ret

          ldi       temp, BTN_BOUNCE
          sts       buttonBounce, temp

          lds       temp, blinkCOunt
          cpi       temp, MAX_BLINK
          brsh      btn_up_blink

          inc       temp
          sts       blinkCount, temo

btn_up_blink:
          ldi    retSTate, ST_START_BLINK
          
btn_up_ret:
          sts       state, retSTate

          pop       temp
          pop       retSTate

          reti

btn_down_ISR:
          push      retState
          push      temp

          ldi       retSTate, ST_WAIT_INP

          lds       temp, buttonBounce
          tst       temp
          brne      btn_down_ret

          ldi       temp, BTN_BOUNCE
          sts       buttonBounce, temp

          lds       temp, blinkCOunt
          cpi       temp, MIN_BLINK
          brlo      btn_down_blink
          breq      btn_down_blink

          dec       temp
          sts       blinkCount, temp

btn_down_blink:
          ldi       retSTAte, ST_START_BLINK

btn_down_ret:
          sts       state, retSTate
          pop       temp
          pop       retSTate
          reti



          
             













            

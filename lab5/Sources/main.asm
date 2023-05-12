

; export symbols
            XDEF Entry, _Startup                ; export 'Entry' symbol
            ABSENTRY Entry                      ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 



********************************************************************************************
* Lab 5: Robot Roaming Program (9S32C)                                                     *
********************************************************************************************

; equates section
********************************************************************************************
LCD_DAT         EQU   PORTB                     ; LCD data port, bits - PB7,...,PB0
LCD_CNTR        EQU   PTJ                       ; LCD control port, bits - PJ6(RS),PJ7(E)
LCD_E           EQU   $80                       ; LCD E-signal pin
LCD_RS          EQU   $40                       ; LCD RS-signal pin
FWD_INT         EQU   69                        ; 3 second delay (at 23Hz)
REV_INT         EQU   69                        ; 3 second delay (at 23Hz)
FWD_TRN_INT     EQU   46                        ; 2 second delay (at 23Hz)
REV_TRN_INT     EQU   46                        ; 2 second delay (at 23Hz)
START           EQU   0
FWD             EQU   1
REV             EQU   2
ALL_STP         EQU   3
FWD_TRN         EQU   4
REV_TRN         EQU   5

; variable section
********************************************************************************************
                ORG   $3850                     ; Where our TOF counter register lives
TOF_COUNTER     dc.b  0                         ; The timer, incremented at 23Hz
CRNT_STATE      dc.b  3                         ; Current state register
T_FWD           ds.b  1                         ; FWD time
T_REV           ds.b  1                         ; REV time
T_FWD_TRN       ds.b  1                         ; FWD_TURN time
T_REV_TRN       ds.b  1                         ; REV_TURN time
TEN_THOUS       ds.b  1                         ; 10,000 digit
THOUSANDS       ds.b  1                         ; 1,000 digit
HUNDREDS        ds.b  1                         ; 100 digit
TENS            ds.b  1                         ; 10 digit
UNITS           ds.b  1                         ; 1 digit
BCD_SPARE       ds.b  10
NO_BLANK        ds.b  1                         ; Used in 'leading zero' blanking by BCD2ASC

; code section
*********************************************************************************************
                ORG   $4000                     ; Where the code starts --------------------+
Entry:                                          ;                                           |
_Startup:                                       ;                                           |
                CLI                             ; Enable interrupts                         |                          
                LDS   #$4000                    ; Initialize the stack pointer              I
                                                ;                                           N
                BSET  DDRA,%00000011            ; STAR_DIR, PORT_DIR                        I
                BSET  DDRT,%00110000            ; STAR_SPEED, PORT_SPEED                    T
                                                ;                                           I
                JSR   initAD                    ; Initialize ATD converter                  A
                                                ;                                           L
                JSR   initLCD                   ; Initialize the LCD                        I
                JSR   clrLCD                    ; Clear LCD & home cursor                   Z
                                                ;                                           A
                LDX   #msg1                     ; Display msg1                              T
                JSR   putsLCD                   ; ""                                        I
                                                ;                                           O
                LDAA  #$C0                      ; Move LCD cursor to the 2nd row            N
                JSR   cmd2LCD                   ;                                           |
                LDX   #msg2                     ; Display msg2                              |
                JSR   putsLCD                   ; ""                                        |
                                                ;                                           |
                JSR   ENABLE_TOF                ; Jump to TOF initialization ---------------+
                
          MAIN: JSR   UPDT_DISPL                ; ----------------------------------------- M
                LDAA  CRNT_STATE                ;                                           A
                JSR   DISPATCHER                ;                                           I
                BRA   MAIN                      ; ----------------------------------------- N

; data section
*********************************************************************************************
          msg1: dc.b  "Battery volt ",0
          msg2: dc.b  "State ",0
           tab: dc.b  "START  ",0
                dc.b  "FWD    ",0
                dc.b  "REV    ",0
                dc.b  "ALL_STP",0
                dc.b  "FWD_TRN",0
                dc.b  "REV_TRN",0

; subroutine section
********************************************************************************************
DISPATCHER      CMPA  #START                    ; If it's the START state ------------------+
                BNE   NOT_START                 ;                                           |
                JSR   START_ST                  ; then call START_ST routine                D
                BRA   DISP_EXIT                 ; and exit                                  I
                                                ;                                           S
NOT_START       CMPA  #FWD                      ; Else if it's the FORWARD state            P
                BNE   NOT_FORWARD               ;                                           A
                JSR   FWD_ST                    ; then call the FORWARD routine             T
                JMP   DISP_EXIT                 ; and exit                                  C
                                                ;                                           H
NOT_FORWARD     CMPA  #REV                      ; Else if it's the REVERSE state            E
                BNE   NOT_REVERSE               ;                                           R
                JSR   REV_ST                    ; then call the REVERSE routine             |
                JMP   DISP_EXIT                 ; and exit                                  |
                                                ;                                           |
NOT_REVERSE     CMPA  #ALL_STP                  ; Else if it's the ALL_STOP state           |
                BNE   NOT_ALL_STOP              ;                                           |
                JSR   ALL_STP_ST                ; then call the ALL_STOP routine            |
                JMP   DISP_EXIT                 ; and exit                                  |
                                                ;                                           |
NOT_ALL_STOP    CMPA  #FWD_TRN                  ; Else if it's the FORWARD_TURN state       |
                BNE   NOT_FWD_TRN               ;                                           |
                JSR   FWD_TRN_ST                ; then call the FORWARD_TURN routine        |
                JMP   DISP_EXIT                 ; and exit                                  |
                                                ;                                           |
NOT_FWD_TRN     CMPA  #REV_TRN                  ; Else if it's the REV_TRN state            |
                BNE   NOT_REV_TRN               ;                                           |
                JSR   REV_TRN_ST                ; then call REV_TRN_ST routine              |
                BRA   DISP_EXIT                 ; and exit                                  |
                                                ;                                           |
NOT_REV_TRN     SWI                             ; Else the CRNT_ST is not defined, so stop  |
DISP_EXIT       RTS                             ; Exit from the state dispatcher -----------+ 

********************************************************************************************
START_ST        BRCLR PORTAD0,$04,NO_FWD        ; If /FWD_BUMP
                JSR   INIT_FWD                  ; Initialize the FORWARD state
                MOVB  #FWD,CRNT_STATE           ; Go into the FORWARD state
                BRA   START_EXIT

NO_FWD          NOP                             ; Else
START_EXIT      RTS                             ; return to the MAIN routine

********************************************************************************************
FWD_ST          BRSET PORTAD0,$04,NO_FWD_BUMP   ; If FWD_BUMP then
                JSR   INIT_REV                  ; initialize the REVERSE routine
                MOVB  #REV,CRNT_STATE           ; set the state to REVERSE
                JMP   FWD_EXIT                  ; and return

NO_FWD_BUMP     BRSET PORTAD0,$08,NO_REAR_BUMP  ; If REAR_BUMP, then we should stop
                JSR   INIT_ALL_STP              ; so initialize the ALL_STOP state
                MOVB  #ALL_STP,CRNT_STATE       ; and change state to ALL_STOP
                JMP   FWD_EXIT                  ; and return

NO_REAR_BUMP    LDAA  TOF_COUNTER               ; If Tc>Tfwd then
                CMPA  T_FWD                     ; the robot should make a turn
                BNE   NO_FWD_TRN                ; so
                JSR   INIT_FWD_TRN              ; initialize the FORWARD_TURN state
                MOVB  #FWD_TRN,CRNT_STATE       ; and go to that state
                JMP   FWD_EXIT
NO_FWD_TRN      NOP                             ; Else
FWD_EXIT        RTS                             ; return to the MAIN routine

********************************************************************************************
REV_ST          LDAA  TOF_COUNTER               ; If Tc>Trev then
                CMPA  T_REV                     ; the robot should make a FWD turn
                BNE   NO_REV_TRN                ; so
                JSR   INIT_REV_TRN              ; initialize the REV_TRN state
                MOVB  #REV_TRN,CRNT_STATE       ; set state to REV_TRN
                BRA   REV_EXIT                  ; and return
NO_REV_TRN      NOP                             ; Else
REV_EXIT        RTS                             ; return to the MAIN routine

********************************************************************************************
ALL_STP_ST      BRSET PORTAD0,$04,NO_START      ; If FWD_BUMP
                BCLR  PTT,%00110000             ; initialize the START state (both motors off)
                MOVB  #START,CRNT_STATE         ; set the state to START
                BRA   ALL_STP_EXIT              ; and return
NO_START        NOP                             ; Else
ALL_STP_EXIT    RTS                             ; return to the MAIN routine

********************************************************************************************
FWD_TRN_ST      LDAA  TOF_COUNTER               ; If Tc>Tfwdturn then
                CMPA  T_FWD_TRN                 ; the robot should go FWD
                BNE   NO_FWD_FT                 ; so
                JSR   INIT_FWD                  ; initialize the FWD state
                MOVB  #FWD,CRNT_STATE           ; set state to FWD
                BRA   FWD_TRN_EXIT              ; and return
NO_FWD_FT       NOP                             ; Else
FWD_TRN_EXIT    RTS                             ; return to the MAIN routine

********************************************************************************************
REV_TRN_ST      LDAA  TOF_COUNTER               ; If Tc>Trevturn then
                CMPA  T_REV_TRN                 ; the robot should go FWD
                BNE   NO_FWD_RT                 ; so
                JSR   INIT_FWD                  ; initialize the FWD state
                MOVB  #FWD,CRNT_STATE           ; set state to FWD
                BRA   REV_TRN_EXIT              ; and return
NO_FWD_RT       NOP                             ; Else
REV_TRN_EXIT    RTS                             ; return to the MAIN routine

********************************************************************************************
INIT_FWD        BCLR  PORTA,%00000011           ; Set FWD direction for both motors
                BSET  PTT,%00110000             ; Turn on the drive motors
                LDAA  TOF_COUNTER               ; Mark the fwd time Tfwd
                ADDA  #FWD_INT
                STAA  T_FWD
                RTS

********************************************************************************************
INIT_REV        BSET  PORTA,%00000011           ; Set REV direction for both motors
                BSET  PTT,%00110000             ; Turn on the drive motors
                LDAA  TOF_COUNTER               ; Mark the fwd time Tfwd
                ADDA  #REV_INT
                STAA  T_REV
                RTS

********************************************************************************************
INIT_ALL_STP    BCLR  PTT,%00110000             ; Turn off the drive motors
                RTS

********************************************************************************************
INIT_FWD_TRN    BSET  PORTA,%00000010           ; Set REV dir. for STARBOARD (right) motor
                LDAA  TOF_COUNTER               ; Mark the fwd_turn time Tfwdturn
                ADDA  #FWD_TRN_INT
                STAA  T_FWD_TRN
                RTS

********************************************************************************************
INIT_REV_TRN    BCLR  PORTA,%00000010           ; Set FWD dir. for STARBOARD (right) motor
                LDAA  TOF_COUNTER               ; Mark the fwd time Tfwd
                ADDA  #REV_TRN_INT
                STAA  T_REV_TRN
                RTS



; utility subroutines
********************************************************************************************
initLCD         BSET  DDRB,%11111111            ; configure pins PB7,...,PB0 for output
                BSET  DDRJ,%11000000            ; configure pins PJ7(E), PJ6(RS) for output
                LDY   #2000                     ; wait for LCD to be ready
                JSR   del_50us                  ; -"-
                LDAA  #$28                      ; set 4-bit data, 2-line display
                JSR   cmd2LCD                   ; -"-
                LDAA  #$0C                      ; display on, cursor off, blinking off
                JSR   cmd2LCD                   ; -"-
                LDAA  #$06                      ; move cursor right after entering a character
                JSR   cmd2LCD                   ; -"-
                RTS

********************************************************************************************
clrLCD          LDAA  #$01                      ; clear cursor and return to home position
                JSR   cmd2LCD                   ; -"-
                LDY   #40                       ; wait until "clear cursor" command is complete
                JSR   del_50us                  ; -"-
                RTS

********************************************************************************************
del_50us:       PSHX                            ;2 E-clk Protect the X register
eloop:          LDX   #300                      ;2 E-clk Initialize the inner loop counter
iloop:          NOP                             ;1 E-clk No operation
                DBNE  X,iloop                   ;3 E-clk If the inner cntr not 0, loop again
                DBNE  Y,eloop                   ;3 E-clk If the outer cntr not 0, loop again
                PULX                            ;3 E-clk Restore the X register
                RTS                             ;5 E-clk Else return

********************************************************************************************
cmd2LCD:        BCLR  LCD_CNTR,LCD_RS           ; select the LCD Instruction Register (IR)
                JSR   dataMov                   ; send data to IR
      	        RTS


********************************************************************************************
putsLCD         LDAA  1,X+                      ; get one character from the string
                BEQ   donePS                    ; reach NULL character?
                JSR   putcLCD
                BRA   putsLCD
donePS 	        RTS



********************************************************************************************
putcLCD         BSET  LCD_CNTR,LCD_RS           ; select the LCD Data register (DR)
                JSR   dataMov                   ; send data to DR
                RTS


********************************************************************************************
dataMov         BSET  LCD_CNTR,LCD_E            ; pull the LCD E-sigal high
                STAA  LCD_DAT                   ; send the upper 4 bits of data to LCD
                BCLR  LCD_CNTR,LCD_E            ; pull the LCD E-signal low to complete the write oper.
                LSLA                            ; match the lower 4 bits with the LCD data pins
                LSLA                            ; -"-
                LSLA                            ; -"-
                LSLA                            ; -"-
                BSET  LCD_CNTR,LCD_E            ; pull the LCD E signal high
                STAA  LCD_DAT                   ; send the lower 4 bits of data to LCD
                BCLR  LCD_CNTR,LCD_E            ; pull the LCD E-signal low to complete the write oper.
                LDY   #1                        ; adding this delay will complete the internal
                JSR   del_50us                  ; operation for most instructions
                RTS


********************************************************************************************
initAD          MOVB  #$C0,ATDCTL2              ; power up AD, select fast flag clear
                JSR   del_50us                  ; wait for 50 us
                MOVB  #$00,ATDCTL3              ; 8 conversions in a sequence
                MOVB  #$85,ATDCTL4              ; res=8, conv-clks=2, prescal=12
                BSET  ATDDIEN,$0C               ; configure pins AN03,AN02 as digital inputs
                RTS  


********************************************************************************************
int2BCD         XGDX                            ; Save the binary number into .X
                LDAA  #0                        ; Clear the BCD_BUFFER
                STAA  TEN_THOUS
                STAA  THOUSANDS
                STAA  HUNDREDS
                STAA  TENS
                STAA  UNITS
                STAA  BCD_SPARE
                STAA  BCD_SPARE+1

                CPX   #0                        ; Check for a zero input
                BEQ   CON_EXIT                  ; and if so, exit

                XGDX                            ; Not zero, get the binary number back to .D as dividend
                LDX   #10                       ; Setup 10 (Decimal!) as the divisor
                IDIV                            ; Divide: Quotient is now in .X, remainder in .D
                STAB  UNITS                     ; Store remainder
                CPX   #0                        ; If quotient is zero,
                BEQ   CON_EXIT                  ; then exit

                XGDX                            ; else swap first quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  TENS
                CPX   #0
                BEQ   CON_EXIT

                XGDX                            ; Swap quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  HUNDREDS
                CPX   #0
                BEQ   CON_EXIT

                XGDX                            ; Swap quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  THOUSANDS
                CPX   #0
                BEQ   CON_EXIT

                XGDX                            ; Swap quotient back into .D
                LDX   #10                       ; and setup for another divide by 10
                IDIV
                STAB  TEN_THOUS

      CON_EXIT: RTS                             ; We?re done the conversion


********************************************************************************************
BCD2ASC         LDAA  #$0                       ; Initialize the blanking flag
                STAA  NO_BLANK

       C_TTHOU: LDAA  TEN_THOUS                 ; Check the ?ten_thousands? digit
                ORAA  NO_BLANK
                BNE   NOT_BLANK1

      ISBLANK1: LDAA  #$20                      ; It?s blank
                STAA  TEN_THOUS                 ; so store a space
                BRA   C_THOU                    ; and check the ?thousands? digit

    NOT_BLANK1: LDAA  TEN_THOUS                 ; Get the ?ten_thousands? digit
                ORAA  #$30                      ; Convert to ascii
                STAA  TEN_THOUS
                LDAA  #$1                       ; Signal that we have seen a ?non-blank? digit
                STAA  NO_BLANK

        C_THOU: LDAA  THOUSANDS                 ; Check the thousands digit for blankness
                ORAA  NO_BLANK                  ; If it?s blank and ?no-blank? is still zero
                BNE   NOT_BLANK2
                     
      ISBLANK2: LDAA  #$30                      ; Thousands digit is blank
                STAA  THOUSANDS                 ; so store a space
                BRA   C_HUNS                    ; and check the hundreds digit

    NOT_BLANK2: LDAA  THOUSANDS                 ; (similar to ?ten_thousands? case)
                ORAA  #$30
                STAA  THOUSANDS
                LDAA  #$1
                STAA  NO_BLANK

        C_HUNS: LDAA  HUNDREDS                  ; Check the hundreds digit for blankness
                ORAA  NO_BLANK                  ; If it?s blank and ?no-blank? is still zero
                BNE   NOT_BLANK3

      ISBLANK3: LDAA  #$20                      ; Hundreds digit is blank
                STAA  HUNDREDS                  ; so store a space
                BRA   C_TENS                    ; and check the tens digit
                     
    NOT_BLANK3: LDAA  HUNDREDS                  ; (similar to ?ten_thousands? case)
                ORAA  #$30
                STAA  HUNDREDS
                LDAA  #$1
                STAA  NO_BLANK

        C_TENS: LDAA  TENS                      ; Check the tens digit for blankness
                ORAA  NO_BLANK                  ; If it?s blank and ?no-blank? is still zero
                BNE   NOT_BLANK4
                     
      ISBLANK4: LDAA  #$20                      ; Tens digit is blank
                STAA  TENS                      ; so store a space
                BRA   C_UNITS                   ; and check the units digit

    NOT_BLANK4: LDAA  TENS                      ; (similar to ?ten_thousands? case)
                ORAA  #$30
                STAA  TENS

       C_UNITS: LDAA  UNITS                     ; No blank check necessary, convert to ascii.
                ORAA  #$30
                STAA  UNITS

                RTS                             ; We?re done

********************************************************************************************
ENABLE_TOF      LDAA  #%10000000
                STAA  TSCR1                     ; Enable TCNT
                STAA  TFLG2                     ; Clear TOF
                LDAA  #%10000100                ; Enable TOI and select prescale factor equal to 16
                STAA  TSCR2
                RTS

********************************************************************************************
TOF_ISR         INC   TOF_COUNTER
                LDAA  #%10000000                ; Clear
                STAA  TFLG2                     ; TOF
                RTI


********************************************************************************************
* Update Display (Battery Voltage + Current State)                                         *
********************************************************************************************
UPDT_DISPL      MOVB  #$90,ATDCTL5              ; R-just., uns., sing. conv., mult., ch=0, start
                BRCLR ATDSTAT0,$80,*            ; Wait until the conver. seq. is complete
                LDAA  ATDDR0L                   ; Load the ch0 result - battery volt - into A
                LDAB  #39                       ; AccB = 39
                MUL                             ; AccD = 1st result x 39
                ADDD  #600                      ; AccD = 1st result x 39 + 600
                JSR   int2BCD
                JSR   BCD2ASC
                LDAA  #$8D                      ; move LCD cursor to the 1st row, end of msg1
                JSR   cmd2LCD                   ; "                
                LDAA  TEN_THOUS                 ; output the TEN_THOUS ASCII character
                JSR   putcLCD                   ; "
                LDAA  THOUSANDS                 ; output the THOUSANDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  #$2E                      ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  HUNDREDS                  ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "                
;------------------------------------------------
                LDAA  #$C6                      ; Move LCD cursor to the 2nd row, end of msg2
                JSR   cmd2LCD                   ;
                
                LDAB  CRNT_STATE                ; Display current state
                LSLB                            ; "
                LSLB                            ; "
                LSLB                            ; "
                LDX   #tab                      ; "
                ABX                             ; "
                JSR   putsLCD                   ; "
                RTS
********************************************************************************************
* Interrupt Vectors                                                                        *
********************************************************************************************
                ORG   $FFFE
                DC.W  Entry                     ; Reset Vector
            
                ORG   $FFDE
                DC.W  TOF_ISR                   ; Timer Overflow Interrupt Vector
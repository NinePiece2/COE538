********************************************************************
* Demonstration Program                                            *
*                                                                  *
* This program illustrates how to use the assembler.               *
* It adds together two 8 bit numbers and leaves the result         *
* in the 'SUM' location.                                           *
* Author: Peter Hiscocks                                           *
********************************************************************
; export symbols
            XDEF Entry, _Startup ; export ‘Entry’ symbol
            ABSENTRY Entry       ; for absolute assembly: mark
                                 ; this as applicat. entry point

; Include derivative-specific definitions
            INCLUDE 'derivative.inc'
********************************************************************
* Code section                                                     *
********************************************************************
            ORG $3000
            
********************************************************************
* The actual program starts here                                   *
********************************************************************
            ORG $4000
Entry:
_Startup:

TESTMESSAGE FCC 'Hi There!'     ; The message
            FCB 00              ; The message terminator
            LDX #TESTMESSAGE    ; Initializing the pointer into the message
            JSR DISPLAYSTRING   ; Write the string
            SWI                 ; Break to the monitor

********************************************************************
* Interrupt Vectors                                                *
********************************************************************
            ORG $FFFE
            FDB Entry            ; Reset Vector
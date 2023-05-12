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
            
FIRSTNUM FCB 01                  ; First Number
SECNUM   FCB 02                  ; Second Number
SUM      RMB 1                   ; Result of addition
********************************************************************
* The actual program starts here                                   *
********************************************************************
            ORG $4000
Entry:
_Startup:
            LDAA FIRSTNUM        ; Get the first number into ACCA
            ADDA SECNUM          ; Add to it the second number
            STAA SUM             ; and store the sum
            SWI                  ; break to the monitor
********************************************************************
* Interrupt Vectors                                                *
********************************************************************
            ORG $FFFE
            FDB Entry            ; Reset Vector
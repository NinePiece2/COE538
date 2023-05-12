********************************************************************
* Demonstration Program                                            *
*                                                                  *
* This progtam multiplies together unsigned two 8 bit numbers and  *
* leaves the result in the 'PRODUCT' location.                     *
* Author: Romit Sagu                                               *
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
            
MULTIPLICAND FCB 04                  ; First Number
MULTIPLIER   FCB 05                  ; Second Number
PRODUCT      RMB 2                   ; Result of multiplication
********************************************************************
* The actual program starts here                                   *
********************************************************************
            ORG $4000
Entry:
_Startup:
            LDAA MULTIPLICAND        ; Get the first number into LDAA
            LDAB MULTIPLIER          ; Get the second number into LDAB
            MUL                      ; multiplies the two numbers
            STD PRODUCT              ; stores the result
            SWI                      ; break to the monitor
********************************************************************
* Interrupt Vectors                                                *
********************************************************************
            ORG $FFFE
            FDB Entry                ; Reset Vector
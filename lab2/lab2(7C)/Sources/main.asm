********************************************************************
* Demonstration Program                                            *
*                                                                  *
* This program generates a tone (a waveform of alternating 0s      *
* and 1s) on the bizzar LS1 using software delay.                  *
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
            
********************************************************************
* The actual program starts here                                   *
********************************************************************
            ORG $4000
Entry:
_Startup:
            BSET DDRP,%11111111 ; Config. Port P for output
            LDAA #%10000000     ; Prepare to drive PP7 high
MainLoop    STAA PTP            ; Drive PP7 (Store Value of A to PTP(Port P))
            LDX #$1FFF          ; Initialize the loop counter
Delay       DEX                 ; Decrement the loop counter
            BNE Delay           ; If not done, continue to loop
            EORA #%10000000     ; Toggle the MSB of AccA
            BRA MainLoop        ; Go to MainLoop
            
********************************************************************
* Interrupt Vectors                                                *
********************************************************************
            ORG $FFFE
            FDB Entry            ; Reset Vector

********************************************************************
* Demonstration Program                                            *
*                                                                  *
* This program reads SW1 and outputs its state on LED1             *
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
            LDAA #$FF ; ACCA = $FF
            STAA DDRH ; Config. Port H for output
            STAA PERT ; Enab. pull-up res. of Port T
            Loop: LDAA PTT ; Read Port T
            STAA PTH ; Display SW1 on LED1 connected to Port H
            BRA Loop ; Loop
********************************************************************
* Interrupt Vectors                                                *
********************************************************************
            ORG $FFFE
            FDB Entry            ; Reset Vector

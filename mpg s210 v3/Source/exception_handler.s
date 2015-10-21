; This file has been updated from the Keil version to IAR version based on our current
; (limited)understanding of how to use assembler Section Control Directives

;	  PRESERVE8
    
    ;AREA SVC_Area, CODE, READONLY
    ;SECTION .text:CODE:REORDER(3)
    SECTION SVC_AREA:CODE:ROOT
    THUMB
    ;EXPORT SVC_Handler         ; goes to interrupt vector [11]
    PUBLIC SVC_Handler         ; goes to interrupt vector [11]
    EXTERN C_SVC_Handler       ; C code SVC demultiplexer

SVC_Handler
    MRS   R1, MSP              ; Address Main Stack.
    LDR   R0,[R1,#24]          ; lr stacked by SVC interrupt = address of instruction following SVC 
    SUBS  R0,#2                ; Address of SVC instruction (Thumb format)
    LDRB  R0,[R0]              ; SVC instruction low octet: SVC number
    LDR   R2, =C_SVC_Handler
    BX    R2                   ; C code: C_SVC_Handler(unsigned SvcNumber, unsigned *regs)

    ;ALIGN
     ALIGNROM 2
   END

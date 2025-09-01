.syntax unified
.thumb

.section .text
.global _start
.type _start, %function

_start:
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    
    MOV r0, #1
    MOV r1, #1
    MOV r2, #1
    POP {lr}
    LDR lr, =0x0001b221
    LDR pc, =0x0001c069




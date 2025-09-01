.syntax unified
.thumb

.global _start

.equ SRAM_START, 0x20002000 @ preserve the vector table & mcuboot globals
.equ SRAM_END, 0x2003FF80
.equ MMARGIN, 128
.equ BR_PROV_INIT, 0x00028520
.equ BR_BROADCAST, 0x00027C84

.equ AIRCR, 0xE000ED0C
.equ VECTKEY, 0x05FA
.equ SYSRESETREQ, (1 << 2)

_start:
start_point:
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

    CPSID i                         @ disable interrupts

    MOVS r7, #0 
    MOVW r4, #(SRAM_START & 0xFFFF) @ thumb instructions are too small to encode full addr directly
    MOVT r4, #(SRAM_START >> 16)    @ MOVW writes the low bits, MOVT writes the high bits - avoids using a literal pool to load the adddr, which is then flooded with 0s 
    ADR r6, start_point

flood_pre:
    CMP r4, r6
    BHS end_flood_pre               @ loop until r0 reaches r1
    STR r7, [r4], #4                @ laid out like this because otherwise it'll write one word too many 
    B flood_pre
end_flood_pre:

    MRS r1, CONTROL                 @ switch back to using msp as the active stack
    BIC r1, r1, #2
    MSR CONTROL, r1
    ISB

    MRS r0, msp
    AND r0, r0, #0xFFFFFFF8 
    MOV sp, r0
    MOV r6, r0
    SUB r6, r6, #256
    ADR r4, end_point


flood_post:
    CMP r4, r6
    BHS end_flood_post
    STR r7, [r4], #4
    B flood_post

    CPSIE i @ reenable interrupts 

reset:
    LDR r0, =AIRCR                  @ 0xE000ED0C
    LDR r1, [r0]                    @ reset the nRF device (typical for ARM cortex-m4s)

    MOVW r2, #0xFFFF
    AND r1, r1, r2    

    MOVW r2, #0x5FA
    LSL r2, r2, #16       
    ORR r1, r1, r2 
    ORR r1, r1, #SYSRESETREQ

    STR r1, [r0]                    @ write -> reset
    B reset

end_point:
    NOP


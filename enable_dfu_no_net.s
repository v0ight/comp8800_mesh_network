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

    LDR r5, =0x0003cbd0   @ addr of enable_dfu
    ORR r5, r5, #1
    BLX r5

    LDR r5, =0x0003c60c   @ addr of broadcast_sensor_data addr, to spin forever
    ORR r5, r5, #1
    BX r5





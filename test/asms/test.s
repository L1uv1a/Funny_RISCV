    .text
main:
    nop
    nop
    nop
    la    s0, data  # Load address data (0x12345678)
    addi  s0, s0, 8
    lh    a0, 1(s0) # 3355 

    nop
    nop
    nop

    .data
data:
    .word 0x12345678 # data
    .word 0          # data + 4
    .word 0x11335577 # data + 8
    .word 0          # data + 12
    .word 0xABCDEF19 # data + 16


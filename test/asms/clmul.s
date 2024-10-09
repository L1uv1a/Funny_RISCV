#
# TEST CODE FOR CLMUL
#
        # -----------------------------------------
        # Program section (known as text)
        # -----------------------------------------
        .text

# Start symbol (must be present), exported as a global symbol.
_start: .global _start

# Export main as a global symbol
        .global main

# Label for entry point of test code
main:
        ### TEST CODE STARTS HERE ###
        
        #Case 1: CLMUL(0xAAAAAAAB, 0x0002FE7D) = 0xFFFC324F
        li      x1, 0xAAAAAAAB         # set x1 to 100 (0x00000064)
        li      x2, 0x0002FE7D        # set x2 to 150 (0x00000096)
        clmul  x3, x1, x2             # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0xFFFC324F
        bne 	x3, x4, fail

        #Case 2: CLMUL(13, 11) = 0x7F
        li      x1, 13         # set x1 to 100 (0x00000064)
        li      x2, 11         # set x2 to 150 (0x00000096)
        clmul  x1, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0x7F
        bne 	x1, x4, fail

        #Case 3: CLMUL(13,13) = 0x51
        li      x1, 13         # set x1 to 100 (0x00000064)
        clmul  x1, x1, x1      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0x51
        bne 	x1, x4, fail

        #Case 5: (0 << 2) + 15 = 15
        li      x1,  31        # set x1 to 100 (0x00000064)
        clmul  x1, x0, x1      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0
        bne 	x1, x4, fail 

        ###    END OF TEST CODE   ###

        # Exit test using RISC-V International's riscv-tests pass/fail criteria
        pass:
        li      a0, 0           # set a0 (x10) to 0 to indicate a pass code
        li      a7, 93          # set a7 (x17) to 93 (5dh) to indicate reached the end of the test
        ebreak
        
        fail:
        li      a0, 1           # fail code
        li      a7, 93          # reached end of code
        ebreak
        
        # -----------------------------------------
        # Data section. Note starts at 0x1000, as 
        # set by DATAADDR variable in rv_asm.bat.
        # -----------------------------------------
        .data

        # Data section
data:
        

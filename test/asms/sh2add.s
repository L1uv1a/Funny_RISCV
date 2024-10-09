#
# TEST CODE FOR SH2ADD
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
        
        #Case 1: (0x00000001 << 2) + 0x7FFFFFFF = 0x80000003
        li      x1, 0x00000001         # set x1 to 100 (0x00000064)
        li      x2, 0x7FFFFFFF         # set x2 to 150 (0x00000096)
        sh2add  x3, x1, x2             # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0x80000003
        bne 	x3, x4, fail

        #Case 2: (13 << 2) + 11 = 63
        li      x1, 13         # set x1 to 100 (0x00000064)
        li      x2, 11         # set x2 to 150 (0x00000096)
        sh2add  x1, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 63
        bne 	x1, x4, fail

        #Case 3: (14 << 2) + 11 = 67 
        li      x1, 14         # set x1 to 100 (0x00000064)
        li      x2, 11         # set x2 to 150 (0x00000096)
        sh2add  x2, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 67
        bne 	x2, x4, fail

        #Case 4: (13 << 2) + 13 = 65
        li      x1, 13         # set x1 to 100 (0x00000064)
        sh2add  x1, x1, x1      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 65
        bne 	x1, x4, fail

        #Case 5: (0 << 2) + 15 = 15
        li      x1,  15        # set x1 to 100 (0x00000064)
        sh2add  x1, x0, x1      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 15
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
        

#
# TEST CODE FOR SH1ADD
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
        
        #Case 1: (1<<1) + 1 = 3
        li      x1, 1         # set x1 to 100 (0x00000064)
        li      x2, 1         # set x2 to 150 (0x00000096)
        sh1add  x3, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 3
        bne 	x3, x4, fail

        #Case 2: (3 << 1) + 7 = 13
        li      x1, 3         # set x1 to 100 (0x00000064)
        li      x2, 7         # set x2 to 150 (0x00000096)
        sh1add  x3, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 13
        bne 	x3, x4, fail

        #Case 3: (0 << 1) + 0 = 0 
        li      x1, 0         # set x1 to 100 (0x00000064)
        li      x2, 0         # set x2 to 150 (0x00000096)
        sh1add  x3, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0
        bne 	x3, x4, fail

        #Case 4: (0 << 1) + 0xFFFF8000 = 0xFFFF8000
        li      x1, 0         # set x1 to 100 (0x00000064)
        li      x2, -32768         # set x2 to 150 (0x00000096)
        sh1add  x3, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, -32768
        bne 	x3, x4, fail

        #Case 5: (0x80000000 << 1) + 0x00000000 = 0
        li      x1,  0x80000000        # set x1 to 100 (0x00000064)
        li      x2,  0        # set x2 to 150 (0x00000096)
        sh1add  x3, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0
        bne 	x3, x4, fail 

        #Case 6: (0x80000000 << 1) + 0xFFFF8000 = 0xFFFF8000
        li      x1,  0x80000000        # set x1 to 100 (0x00000064)
        li      x2,  0xFFFF8000        # set x2 to 150 (0x00000096)
        sh1add  x3, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0xFFFF8000
        bne 	x3, x4, fail 

        #Case 7: (0x7FFFFFFF << 1) + 0x00007FFF = 0x00007FFD
        li      x1,  0x7FFFFFFF        # set x1 to 100 (0x00000064)
        li      x2,  0x00007FFF        # set x2 to 150 (0x00000096)
        sh1add  x3, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0x00007FFD
        bne 	x3, x4, fail 

        #Case 8: (0xFFFFFFFF << 1) + 0xFFFFFFFF = 0xFFFFFFFD
        li      x1,  0xFFFFFFFF        # set x1 to 100 (0x00000064)
        li      x2,  0xFFFFFFFF        # set x2 to 150 (0x00000096)
        sh1add  x3, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0xFFFFFFFD
        bne 	x3, x4, fail 

        #Case 9: (0x00000001 << 1) + 0x7FFFFFFF = 0x80000001
        li      x1,  0x00000001       # set x1 to 100 (0x00000064)
        li      x2,  0x7FFFFFFF                  # set x2 to 150 (0x00000096)
        sh1add  x3, x1, x2      # add x1(100) to x2(150), x3=250 (0x000000FA) 
        li 	 x4, 0x80000001
        bne 	x3, x4, fail 
        
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
        

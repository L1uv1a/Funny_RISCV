`define ZERO_REG_ADDR 0
`define ZERO_REG_DATA 0

// region decode
`define ALU_WIDTH 14
`define ADD 0
`define SUB 1
`define SLT 2
`define SLTU 3
`define XOR 4
`define OR 5
`define AND 6
`define SLL 7
`define SRL 8
`define SRA 9
`define EQ 10
`define NEQ 11
`define GE 12
`define GEU 13

`define OPCODE_WIDTH 11
`define RTYPE 0
`define ITYPE 1
`define LOAD 2
`define STORE 3
`define BRANCH 4
`define JAL 5
`define JALR 6
`define LUI 7
`define AUIPC 8
`define SYSTEM 9
`define FENCE 10

`define EXCEPTION_WIDTH 4
`define ILLEGAL 0
`define ECALL 1
`define EBREAK 2
`define MRET 3
// endregion decode

`define FUNCT3_LOAD_STORE_BYTE 3'b000
`define FUNCT3_LOAD_STORE_HALF 3'b001
`define FUNCT3_LOAD_STORE_WORD 3'b010
`define FUNCT3_LOAD_BYTE_U 3'b100
`define FUNCT3_LOAD_HALF_U 3'b101

`define BYTE_SIGN_EXPAND_BITS 24
`define BYTE_SIGN_BIT 7
`define HALF_SIGN_EXPAND_BITS 16
`define HALF_SIGN_BIT 15

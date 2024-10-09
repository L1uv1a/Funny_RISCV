`define ZERO_REG_ADDR 0
`define TRUE 1
`define FALSE 0

`define FUNCT7_BIT6 30

`define RS2_RANGE 24:20
`define RS1_RANGE 19:15
`define RD_RANGE 11:7
`define FUNCT3_RANGE 14:12
`define OPCODE_RANGE 6:0
`define FUNCT7_RANGE 31:25

`define ALU_WIDTH 41
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
`define SH1ADD 14
`define SH2ADD 15
`define SH3ADD 16
`define XNOR 17
`define ZEXT_H 18
`define ANDN 19
`define CLZ 20
`define CPOP 21
`define CTZ 22
`define MIN 25
`define MINU 26
`define MAX 23
`define MAXU 24
`define ORC_B 27
`define ORN 28
`define REV8 29
`define ROR 30
`define ROL 31
`define SEXT_B 32
`define SEXT_H 33
`define CLMUL 34
`define CLMULH 35
`define CLMULR 36
`define BCLR 37
`define BEXT 38
`define BINV 39
`define BSET 40



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

`define OPCODE_RTYPE 7'b0110011 
`define OPCODE_ITYPE 7'b0010011
`define OPCODE_LOAD 7'b0000011
`define OPCODE_STORE 7'b0100011
`define OPCODE_BRANCH 7'b1100011
`define OPCODE_JAL 7'b1101111
`define OPCODE_JALR 7'b1100111
`define OPCODE_LUI 7'b0110111
`define OPCODE_AUIPC 7'b0010111
`define OPCODE_SYSTEM 7'b1110011
`define OPCODE_FENCE 7'b0001111

`define FUNCT3_ADD 3'b000
`define FUNCT3_SLT 3'b010 
`define FUNCT3_SLTU 3'b011
`define FUNCT3_XOR 3'b100
`define FUNCT3_OR 3'b110
`define FUNCT3_AND 3'b111
`define FUNCT3_SLL 3'b001
`define FUNCT3_SRA 3'b101
`define FUNCT3_EQ 3'b000
`define FUNCT3_NEQ 3'b001
`define FUNCT3_LT 3'b100
`define FUNCT3_GE 3'b101
`define FUNCT3_LTU 3'b110
`define FUNCT3_GEU 3'b111
`define FUNCT3_SH1ADD 3'b010
`define FUNCT3_SH2ADD 3'b100
`define FUNCT3_SH3ADD 3'b110
`define FUNCT3_XNOR 3'b100
`define FUNCT3_ZEXT_H 3'b100
`define FUNCT3_ANDN 3'b111
`define FUNCT3_CLZ 3'b001
`define FUNCT3_CPOP 3'b001
`define FUNCT3_CTZ 3'b001
`define FUNCT3_MAX 3'b110
`define FUNCT3_MAXU 3'b111
`define FUNCT3_MIN 3'b100
`define FUNCT3_MINU 3'b101
`define FUNCT3_ORC_B 3'b101
`define FUNCT3_ORN 3'b110
`define FUNCT3_REV8 3'b101
`define FUNCT3_ROR 3'b101
`define FUNCT3_ROL 3'b001
`define FUNCT3_SEXT 3'b001
`define FUNCT3_CLMUL 3'b001
`define FUNCT3_CLMULH 3'b011
`define FUNCT3_CLMULR 3'b010
`define FUNCT3_BCRL 3'b001
`define FUNCT3_BEXT 3'b101
`define FUNCT3_BINV 3'b001
`define FUNCT3_BSET 3'b001


`define FUNCT3_LOAD_STORE_BYTE 3'b000
`define FUNCT3_LOAD_STORE_HALF 3'b001
`define FUNCT3_LOAD_STORE_WORD 3'b010
`define FUNCT3_LOAD_BYTE_U 3'b100
`define FUNCT3_LOAD_HALF_U 3'b101

`define BYTE_SIGN_EXPAND_BITS 24
`define BYTE_SIGN_BIT 7
`define HALF_SIGN_EXPAND_BITS 16
`define HALF_SIGN_BIT 15
// `define WORD_SIGN_EXPAND_BITS 0
// `define WORD_SIGN_BIT 31

// Imm extraction
`define IMM_SIGN_BIT 31

`define ITYPE_IMM_SIGN_EXPAND_BITS 20
`define ITYPE_IMM_RANGE 31:20

`define STYPE_IMM_SIGN_EXPAND_BITS 20
`define STYPE_IMM_RANGE_11_5 31:25
`define STYPE_IMM_RANGE_4_0 11:7

`define BTYPE_IMM_SIGN_EXPAND_BITS 19
`define BTYPE_IMM_RANGE_12 31
`define BTYPE_IMM_RANGE_11 7
`define BTYPE_IMM_RANGE_10_5 30:25
`define BTYPE_IMM_RANGE_4_1 11:8
`define BTYPE_IMM_ZERO_FILL_BITS 1

`define JTYPE_IMM_SIGN_EXPAND_BITS 11
`define JTYPE_IMM_RANGE_20 31
`define JTYPE_IMM_RANGE_19_12 19:12
`define JTYPE_IMM_RANGE_11 20
`define JTYPE_IMM_RANGE_10_1 30:21
`define JTYPE_IMM_ZERO_FILL_BITS 1

`define UTYPE_IMM_RANGE 31:12
`define UTYPE_IMM_ZERO_FILL_BITS 12

`define XTYPE_IMM_RANGE 31:20
`define XTYPE_IMM_ZERO_FILL_BITS 20

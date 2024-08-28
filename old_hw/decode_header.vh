// region instruction extraction bit range
`define FUNCT7_BIT6 30
`define RS2_RANGE 24:20
`define RS1_RANGE 19:15
`define RD_RANGE 11:7
`define FUNCT3_RANGE 14:12
`define OPCODE_RANGE 6:0
// endregion instruction extraction bit range

// region Opcode type decode
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
// region Opcode type decode

// region ALU operation type decode
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

`define FUNCT3_LB 3'b000
`define FUNCT3_LBU 3'b100
`define FUNCT3_LH 3'b001
`define FUNCT3_LHU 3'b101
`define FUNCT3_LW 3'b010

`define FUNCT3_SB 3'b000
`define FUNCT3_SH 3'b001
`define FUNCT3_SW 3'b010
// region ALU operation type decode

// region Imm extraction
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
// region Imm extraction

`include "rv32i_header.vh"
`include "decode_header.vh"

module decode (
    input clk,
    input rst,

    // region control from [STAGE 1 FETCH]
    input [31:0] fetch_instr,  // fetched instruction from [STAGE 1 FETCH]
    input [31:0] fetch_pc,     // pc from [STAGE 1 FETCH] (previous stage)
    // endregion control from [STAGE 1 FETCH]

    // region decoded signals
    output reg [31:0] decode_pc,  // pc from [STAGE 2 DECODE] (current stage)

    output     [ 4:0] decode_rs1,    // source register 1 address
    output reg [ 4:0] decode_r_rs1,  // registed source register 1 address
    output     [ 4:0] decode_rs2,    // source register 2 address
    output reg [ 4:0] decode_r_rs2,  // registed source register 2 address
    output reg [ 4:0] decode_r_rd,   // registed destination register address
    output reg [31:0] decode_imm,    // extended value for immediate
    output reg [ 2:0] decode_funct3, // function type

    output reg [      `ALU_WIDTH-1:0] decode_alu_type,     // alu operation type
    output reg [   `OPCODE_WIDTH-1:0] decode_opcode_type,  // opcode type
    output reg [`EXCEPTION_WIDTH-1:0] decode_exception,    // illegal instr, ecall, ebreak, mret
    // endregion decoded signals

    // region Pipeline control
    input      clk_en,       // control by previous stage ([STAGE 1 FETCH])
    output reg next_clk_en,  // clk enable for pipeline stalling of next state ([STAGE 3 EXECUTE])
    input      stall,        // stall this stage
    output reg next_stall,   // stalls the pipeline
    input      flush,        // flush this stage
    output reg next_flush    // flushes previous stages
    // endregion Pipeline control
);

  // region extract from fetched instruction
  wire [4:0] rs2 = fetch_instr[`RS2_RANGE];
  wire [4:0] rs1 = fetch_instr[`RS1_RANGE];

  wire [4:0] rd = fetch_instr[`RD_RANGE];
  wire [2:0] funct3 = fetch_instr[`FUNCT3_RANGE];
  wire [6:0] opcode = fetch_instr[`OPCODE_RANGE];
  wire funct7_bit6 = fetch_instr[`FUNCT7_BIT6];
  // endregion extract from fetched instruction

  // region handle output ports and registers
  assign decode_rs1 = rs1;
  assign decode_rs2 = rs2;

  // update registers of this stage
  always @(posedge clk, posedge rst) begin : update_stage_registers
    if (rst) begin
      decode_pc          <= 0;
      decode_r_rs1       <= 0;
      decode_r_rs2       <= 0;
      decode_r_rd        <= 0;
      decode_funct3      <= 0;
      decode_imm         <= 0;
      decode_alu_type    <= 0;
      decode_opcode_type <= 0;
      decode_exception   <= 0;
    end  // update registers if this stage is enabled and pipeline is not stalled
    else if ((!stall_bit) && clk_en) begin
      decode_pc                   <= fetch_pc;
      decode_r_rs1                <= rs1;
      decode_r_rs2                <= rs2;
      decode_r_rd                 <= rd;
      decode_funct3               <= funct3;
      decode_imm                  <= imm;

      // ALU Operation type
      decode_alu_type[`ADD]       <= alu_add;
      decode_alu_type[`SUB]       <= alu_sub;
      decode_alu_type[`SLT]       <= alu_slt;
      decode_alu_type[`SLTU]      <= alu_sltu;
      decode_alu_type[`XOR]       <= alu_xor;
      decode_alu_type[`OR]        <= alu_or;
      decode_alu_type[`AND]       <= alu_and;
      decode_alu_type[`SLL]       <= alu_sll;
      decode_alu_type[`SRL]       <= alu_srl;
      decode_alu_type[`SRA]       <= alu_sra;
      decode_alu_type[`EQ]        <= alu_eq;
      decode_alu_type[`NEQ]       <= alu_neq;
      decode_alu_type[`GE]        <= alu_ge;
      decode_alu_type[`GEU]       <= alu_geu;

      // Opcode type
      decode_opcode_type[`RTYPE]  <= opcode_rtype;
      decode_opcode_type[`ITYPE]  <= opcode_itype;
      decode_opcode_type[`LOAD]   <= opcode_load;
      decode_opcode_type[`STORE]  <= opcode_store;
      decode_opcode_type[`BRANCH] <= opcode_branch;
      decode_opcode_type[`JAL]    <= opcode_jal;
      decode_opcode_type[`JALR]   <= opcode_jalr;
      decode_opcode_type[`LUI]    <= opcode_lui;
      decode_opcode_type[`AUIPC]  <= opcode_auipc;
      decode_opcode_type[`SYSTEM] <= opcode_system;
      decode_opcode_type[`FENCE]  <= opcode_fence;

      // Exceptions
      decode_exception[`ILLEGAL]  <= ((!valid_opcode) || illegal_shift);
      decode_exception[`ECALL]    <= (system_noncsr && (fetch_instr[21:20] == 2'b00));
      decode_exception[`EBREAK]   <= (system_noncsr && (fetch_instr[21:20] == 2'b01));
      decode_exception[`MRET]     <= (system_noncsr && (fetch_instr[21:20] == 2'b10));
    end
  end
  // endregion handle output ports and registers

  // region update stages control
  always @(posedge clk or posedge rst) begin : update_stages_cotrol
    if (rst) next_clk_en <= 0;
    else begin
      // flush this stage so clock-enable of next stage is disabled at next clock cycle
      if ((!stall_bit) && flush) next_clk_en <= 0;
      // clock-enable will change only when not stalled
      else if (!stall_bit) next_clk_en <= clk_en;
      // if this stage is stalled but next stage is not, disable clock enable of next stage at next clock cycle (pipeline bubble)
      else if (stall_bit && (!stall)) next_clk_en <= 0;
    end
  end
  // endregion update stages control

  // region opcode type decode
  wire opcode_rtype = (opcode == `OPCODE_RTYPE);
  wire opcode_itype = (opcode == `OPCODE_ITYPE);
  wire opcode_load = (opcode == `OPCODE_LOAD);
  wire opcode_store = (opcode == `OPCODE_STORE);
  wire opcode_branch = (opcode == `OPCODE_BRANCH);
  wire opcode_jal = (opcode == `OPCODE_JAL);
  wire opcode_jalr = (opcode == `OPCODE_JALR);
  wire opcode_lui = (opcode == `OPCODE_LUI);
  wire opcode_auipc = (opcode == `OPCODE_AUIPC);
  wire opcode_system = (opcode == `OPCODE_SYSTEM);
  wire opcode_fence = (opcode == `OPCODE_FENCE);

  wire system_noncsr = (opcode == `OPCODE_SYSTEM) && (funct3 == 0) ; //system instruction but not CSR operation

  wire valid_opcode = (opcode_rtype  ||
                       opcode_itype  ||
                       opcode_load   ||
                       opcode_store  ||
                       opcode_branch ||
                       opcode_jal    ||
                       opcode_jalr   ||
                       opcode_lui    ||
                       opcode_auipc  ||
                       opcode_system ||
                       opcode_fence);
  // endregion opcode type decode

  wire illegal_shift = (opcode_itype && (alu_sll || alu_srl || alu_sra) && fetch_instr[25]);

  /* Stall conditions:
  - stall this stage
  - stall next stage
  */
  wire stall_bit = (stall || next_stall);

  // region update pipeline control
  always_comb begin : update_pipeline_control
    next_stall = stall;  // stall previous stage when decoder needs wait time
    next_flush = flush;  // flush this stage along with the previous stages
  end
  // endregion update pipeline control

  // region ALU operation decode
  reg alu_add;
  reg alu_sub;
  reg alu_slt;
  reg alu_sltu;
  reg alu_xor;
  reg alu_or;
  reg alu_and;
  reg alu_sll;
  reg alu_srl;
  reg alu_sra;
  reg alu_eq;
  reg alu_neq;
  reg alu_ge;
  reg alu_geu;

  always_comb begin : alu_operation_decode
    // default values
    alu_add  = 0;
    alu_sub  = 0;
    alu_slt  = 0;
    alu_sltu = 0;
    alu_xor  = 0;
    alu_or   = 0;
    alu_and  = 0;
    alu_sll  = 0;
    alu_srl  = 0;
    alu_sra  = 0;
    alu_eq   = 0;
    alu_neq  = 0;
    alu_ge   = 0;
    alu_geu  = 0;

    // decode
    if ((opcode == `OPCODE_RTYPE) || (opcode == `OPCODE_ITYPE)) begin
      // add/sub in R Type
      if (opcode == `OPCODE_RTYPE) begin
        // add and sub has same funct3 code differs on funct7_bit6
        if ((funct3 == `FUNCT3_ADD) && funct7_bit6) alu_sub = 1;
        else alu_add = 1;
      end else begin
        alu_add = (funct3 == `FUNCT3_ADD);
      end

      alu_slt  = (funct3 == `FUNCT3_SLT);
      alu_sltu = (funct3 == `FUNCT3_SLTU);
      alu_xor  = (funct3 == `FUNCT3_XOR);
      alu_or   = (funct3 == `FUNCT3_OR);
      alu_and  = (funct3 == `FUNCT3_AND);
      alu_sll  = (funct3 == `FUNCT3_SLL);

      // srl and sra has same funct3 code differs on funct7_bit6
      alu_srl  = (funct3 == `FUNCT3_SRA) ? (!funct7_bit6) : 0;
      alu_sra  = (funct3 == `FUNCT3_SRA) ? funct7_bit6 : 0;
    end else if (opcode == `OPCODE_BRANCH) begin
      alu_eq   = (funct3 == `FUNCT3_EQ);
      alu_neq  = (funct3 == `FUNCT3_NEQ);
      alu_slt  = (funct3 == `FUNCT3_LT);
      alu_ge   = (funct3 == `FUNCT3_GE);
      alu_sltu = (funct3 == `FUNCT3_LTU);
      alu_geu  = (funct3 == `FUNCT3_GEU);
    end
  end
  // endregion ALU operation decode

  // region Imm extraction
  reg [31:0] imm;

  always_comb begin : imm_decode
    case (opcode)
      `OPCODE_ITYPE, `OPCODE_LOAD, `OPCODE_JALR: imm = imm_i;
      `OPCODE_STORE: imm = imm_s;
      `OPCODE_BRANCH: imm = imm_b;
      `OPCODE_JAL: imm = imm_j;
      `OPCODE_LUI, `OPCODE_AUIPC: imm = imm_u;
      `OPCODE_SYSTEM, `OPCODE_FENCE: imm = imm_x;
      default: imm = 0;
    endcase
  end

  // sign bit of imm
  wire imm_sign_bit = fetch_instr[`IMM_SIGN_BIT];

  wire [31:0] imm_i = {
    {`ITYPE_IMM_SIGN_EXPAND_BITS{imm_sign_bit}},  // Sign expand
    fetch_instr[`ITYPE_IMM_RANGE]  // Imm extraction
  };

  wire [31:0] imm_s = {
    {`STYPE_IMM_SIGN_EXPAND_BITS{imm_sign_bit}},  // Sign expand
    fetch_instr[`STYPE_IMM_RANGE_11_5],  // Imm extraction
    fetch_instr[`STYPE_IMM_RANGE_4_0]
  };

  wire [31:0] imm_b = {
    {`BTYPE_IMM_SIGN_EXPAND_BITS{imm_sign_bit}},  // Sign expand
    fetch_instr[`BTYPE_IMM_RANGE_12],  // Imm extraction
    fetch_instr[`BTYPE_IMM_RANGE_11],
    fetch_instr[`BTYPE_IMM_RANGE_10_5],
    fetch_instr[`BTYPE_IMM_RANGE_4_1],
    {`BTYPE_IMM_ZERO_FILL_BITS{1'b0}}  // zero fill
  };

  wire [31:0] imm_j = {
    {`JTYPE_IMM_SIGN_EXPAND_BITS{imm_sign_bit}},  // Sign expand
    fetch_instr[`JTYPE_IMM_RANGE_20],  // Imm extraction
    fetch_instr[`JTYPE_IMM_RANGE_19_12],
    fetch_instr[`JTYPE_IMM_RANGE_11],
    fetch_instr[`JTYPE_IMM_RANGE_10_1],
    {`JTYPE_IMM_ZERO_FILL_BITS{1'b0}}  // zero fill
  };

  wire [31:0] imm_u = {
    fetch_instr[`UTYPE_IMM_RANGE],  // Imm extraction
    {`UTYPE_IMM_ZERO_FILL_BITS{1'b0}}  // zero fill
  };

  wire [31:0] imm_x = {{`XTYPE_IMM_ZERO_FILL_BITS{1'b0}}, fetch_instr[`XTYPE_IMM_RANGE]};
  // endregion Imm extraction

endmodule

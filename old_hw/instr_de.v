/* decodes the 32 bit instruction [DECODE STAGE]
*/

`include "rv32i_header.vh"
`include "decode_header.vh"

module instr_de (
    input wire clk,
    input wire rstn,

    input  wire [31:0] instr,    // 32 bit instruction
    input  wire [31:0] prev_pc,  // PC value from previous stage
    output reg  [31:0] pc,       // PC value
    output wire [ 4:0] rs1,      // address for register source 1
    output reg  [ 4:0] r_rs1,    // registered address for register source 1
    output wire [ 4:0] rs2,      // address for register source 2
    output reg  [ 4:0] r_rs2,    // registered address for register source 2
    output reg  [ 4:0] rd,       // address for destination address
    output reg  [31:0] imm,      // extended value for immediate
    output reg  [ 2:0] funct3,   // function type

    // ALU control
    output reg [      `ALU_WIDTH-1:0] alu_operation,  // alu operation type
    output reg [   `OPCODE_WIDTH-1:0] opcode_type,    // opcode type
    output reg [`EXCEPTION_WIDTH-1:0] exception,      // illegal instr, ecall, ebreak, mret

    // Pipeline control
    input  wire prev_clk_en,  // input clk enable for pipeline stalling of this stage
    output reg  clk_en,       // output clk enable for pipeline stalling of next stage
    input  wire prev_stall,   //informs this stage to stall
    output reg  stall,        //informs pipeline to stall
    input  wire prev_flush,   //flush this stage
    output reg  flush         //flush previous stages
);

  //rs1 and rs2 are not registered since regs module do the registering itself
  assign rs2 = instr[`RS2_RANGE];
  assign rs1 = instr[`RS1_RANGE];

  wire [4:0] extracted_rd = instr[`RD_RANGE];


  wire [2:0] funct3_d = instr[`FUNCT3_RANGE];
  wire [6:0] opcode = instr[`OPCODE_RANGE];
  wire funct7_bit6 = instr[`FUNCT7_BIT6];

  reg [31:0] imm_d;

  reg alu_add_d;
  reg alu_sub_d;
  reg alu_slt_d;
  reg alu_sltu_d;
  reg alu_xor_d;
  reg alu_or_d;
  reg alu_and_d;
  reg alu_sll_d;
  reg alu_srl_d;
  reg alu_sra_d;
  reg alu_eq_d;
  reg alu_neq_d;
  reg alu_ge_d;
  reg alu_geu_d;

  // region opcode type decode
  wire opcode_rtype_d = (opcode == `OPCODE_RTYPE);
  wire opcode_itype_d = (opcode == `OPCODE_ITYPE);
  wire opcode_load_d = (opcode == `OPCODE_LOAD);
  wire opcode_store_d = (opcode == `OPCODE_STORE);
  wire opcode_branch_d = (opcode == `OPCODE_BRANCH);
  wire opcode_jal_d = (opcode == `OPCODE_JAL);
  wire opcode_jalr_d = (opcode == `OPCODE_JALR);
  wire opcode_lui_d = (opcode == `OPCODE_LUI);
  wire opcode_auipc_d = (opcode == `OPCODE_AUIPC);
  wire opcode_system_d = (opcode == `OPCODE_SYSTEM);
  wire opcode_fence_d = (opcode == `OPCODE_FENCE);

  wire system_noncsr = (opcode == `OPCODE_SYSTEM) && (funct3_d == 0) ; //system instruction but not CSR operation
  wire valid_opcode = (opcode_rtype_d  ||
                       opcode_itype_d  ||
                       opcode_load_d   ||
                       opcode_store_d  ||
                       opcode_branch_d ||
                       opcode_jal_d    ||
                       opcode_jalr_d   ||
                       opcode_lui_d    ||
                       opcode_auipc_d  ||
                       opcode_system_d ||
                       opcode_fence_d);
  // endregion opcode type decode

  wire illegal_shift = (opcode_itype_d && (alu_sll_d || alu_srl_d || alu_sra_d)) && instr[25];

  wire stall_bit = (prev_stall || stall);  // stall this stage when next stages are stalled

      // region Imm extraction
  wire [31:0] imm_i = {
    {`ITYPE_IMM_SIGN_EXPAND_BITS{instr[`IMM_SIGN_BIT]}},  // Sign expand
    instr[`ITYPE_IMM_RANGE]  // Imm extraction
  };


  wire [31:0] imm_s = {
    {`STYPE_IMM_SIGN_EXPAND_BITS{instr[`IMM_SIGN_BIT]}},  // Sign expand
    instr[`STYPE_IMM_RANGE_11_5],  // Imm extraction
    instr[`STYPE_IMM_RANGE_4_0]
  };

  wire [31:0] imm_b = {
    {`BTYPE_IMM_SIGN_EXPAND_BITS{instr[`IMM_SIGN_BIT]}},  // Sign expand
    instr[`BTYPE_IMM_RANGE_12],  // Imm extraction
    instr[`BTYPE_IMM_RANGE_11],
    instr[`BTYPE_IMM_RANGE_10_5],
    instr[`BTYPE_IMM_RANGE_4_1],
    {`BTYPE_IMM_ZERO_FILL_BITS{1'b0}}  // zero fill
  };

  wire [31:0] imm_j = {
    {`JTYPE_IMM_SIGN_EXPAND_BITS{instr[`IMM_SIGN_BIT]}},  // Sign expand
    instr[`JTYPE_IMM_RANGE_20],  // Imm extraction
    instr[`JTYPE_IMM_RANGE_19_12],
    instr[`JTYPE_IMM_RANGE_11],
    instr[`JTYPE_IMM_RANGE_10_1],
    {`JTYPE_IMM_ZERO_FILL_BITS{1'b0}}  // zero fill
  };

  wire [31:0] imm_u = {
    instr[`UTYPE_IMM_RANGE],  // Imm extraction
    {`UTYPE_IMM_ZERO_FILL_BITS{1'b0}}  // zero fill
  };

  wire [31:0] imm_x = {{`XTYPE_IMM_ZERO_FILL_BITS{1'b0}}, instr[`XTYPE_IMM_RANGE]};

  // region update stage registers
  //register the outputs of this decoder module for shorter combinational timing paths
  always @(posedge clk, negedge rstn) begin : update_stage_registers
    if (!rstn) begin
      // Do nothing here
      // update registers only if this stage is enabled and pipeline is not stalled
    end else if (prev_clk_en && (!stall_bit)) begin
      // capture any necessary registers to hold value between stages of the pipeline process
      pc                   <= prev_pc;
      r_rs1                <= rs1;
      r_rs2                <= rs2;
      rd                   <= extracted_rd;
      funct3               <= funct3_d;
      imm                  <= imm_d;

      /// ALU Operations ////
      alu_operation[`ADD]  <= alu_add_d;
      alu_operation[`SUB]  <= alu_sub_d;
      alu_operation[`SLT]  <= alu_slt_d;
      alu_operation[`SLTU] <= alu_sltu_d;
      alu_operation[`XOR]  <= alu_xor_d;
      alu_operation[`OR]   <= alu_or_d;
      alu_operation[`AND]  <= alu_and_d;
      alu_operation[`SLL]  <= alu_sll_d;
      alu_operation[`SRL]  <= alu_srl_d;
      alu_operation[`SRA]  <= alu_sra_d;
      alu_operation[`EQ]   <= alu_eq_d;
      alu_operation[`NEQ]  <= alu_neq_d;
      alu_operation[`GE]   <= alu_ge_d;
      alu_operation[`GEU]  <= alu_geu_d;

      opcode_type[`RTYPE]  <= opcode_rtype_d;
      opcode_type[`ITYPE]  <= opcode_itype_d;
      opcode_type[`LOAD]   <= opcode_load_d;
      opcode_type[`STORE]  <= opcode_store_d;
      opcode_type[`BRANCH] <= opcode_branch_d;
      opcode_type[`JAL]    <= opcode_jal_d;
      opcode_type[`JALR]   <= opcode_jalr_d;
      opcode_type[`LUI]    <= opcode_lui_d;
      opcode_type[`AUIPC]  <= opcode_auipc_d;
      opcode_type[`SYSTEM] <= opcode_system_d;
      opcode_type[`FENCE]  <= opcode_fence_d;

      // Exceptions
      exception[`ILLEGAL]  <= (!valid_opcode || illegal_shift);
      exception[`ECALL]    <= (system_noncsr && instr[21:20] == 2'b00);
      exception[`EBREAK]   <= (system_noncsr && instr[21:20] == 2'b01);
      exception[`MRET]     <= (system_noncsr && instr[21:20] == 2'b10);
    end
  end
  // endregion update stage registers

  // region update stages control
  always @(posedge clk or negedge rstn) begin : update_stages_cotrol
    if (!rstn) clk_en <= 0;
    else begin
      // flush this stage so clock-enable of next stage is disabled at next clock cycle
      if ((!stall_bit) && prev_flush) clk_en <= 0;
      // clock-enable will change only when not stalled
      else if (!stall_bit) clk_en <= prev_clk_en;
      // if this stage is stalled but next stage is not, disable clock enable of next stage at next clock cycle (pipeline bubble)
      else if (stall_bit && (!prev_stall)) clk_en <= 0;
    end
  end
  // endregion update stages control

  // region update pipeline
  always @* begin : update_pipeline
    stall = prev_stall;  // stall previous stage when decoder needs wait time
    flush = prev_flush;  // flush this stage along with the previous stages 
  end
  // endregion update pipeline

  // region ALU operation decode
  always @* begin : alu_operation_decode
    alu_add_d  = 0;
    alu_sub_d  = 0;
    alu_slt_d  = 0;
    alu_sltu_d = 0;
    alu_xor_d  = 0;
    alu_or_d   = 0;
    alu_and_d  = 0;
    alu_sll_d  = 0;
    alu_srl_d  = 0;
    alu_sra_d  = 0;
    alu_eq_d   = 0;
    alu_neq_d  = 0;
    alu_ge_d   = 0;
    alu_geu_d  = 0;

    // ALU operation decode
    if ((opcode == `OPCODE_RTYPE) || (opcode == `OPCODE_ITYPE)) begin
      if (opcode == `OPCODE_RTYPE) begin
        alu_add_d = (funct3_d == `FUNCT3_ADD) ? (!funct7_bit6) : 0;  //add and sub has same funct3 code differs on funct7_bit6
        alu_sub_d = (funct3_d == `FUNCT3_ADD) ? funct7_bit6 : 0;
      end else begin
        alu_add_d = (funct3_d == `FUNCT3_ADD);
      end
      alu_slt_d = (funct3_d == `FUNCT3_SLT);
      alu_sltu_d = (funct3_d == `FUNCT3_SLTU);
      alu_xor_d = (funct3_d == `FUNCT3_XOR);
      alu_or_d = (funct3_d == `FUNCT3_OR);
      alu_and_d = (funct3_d == `FUNCT3_AND);
      alu_sll_d = (funct3_d == `FUNCT3_SLL);
      alu_srl_d  = (funct3_d == `FUNCT3_SRA) ? (!funct7_bit6) : 0;  //srl and sra has same funct3 code differs on funct7_bit6
      alu_sra_d = (funct3_d == `FUNCT3_SRA) ? funct7_bit6 : 0;
    end else if (opcode == `OPCODE_BRANCH) begin
      alu_eq_d   = (funct3_d == `FUNCT3_EQ);
      alu_neq_d  = (funct3_d == `FUNCT3_NEQ);
      alu_slt_d  = (funct3_d == `FUNCT3_LT);
      alu_ge_d   = (funct3_d == `FUNCT3_GE);
      alu_sltu_d = (funct3_d == `FUNCT3_LTU);
      alu_geu_d  = (funct3_d == `FUNCT3_GEU);
    end else alu_add_d = 1'b1;  // add operation for all remaining instructions
  end
  // endregion ALU operation decode

  // region Imm decode
  always @* begin : imm_decode
    // Imm decode
    case (opcode)
      `OPCODE_ITYPE, `OPCODE_LOAD, `OPCODE_JALR: imm_d = imm_i;
      `OPCODE_STORE: imm_d = imm_s;
      `OPCODE_BRANCH: imm_d = imm_b;
      `OPCODE_JAL: imm_d = imm_j;
      `OPCODE_LUI, `OPCODE_AUIPC: imm_d = imm_u;
      `OPCODE_SYSTEM, `OPCODE_FENCE: imm_d = imm_x;
      default: imm_d = 0;
    endcase
  end
  // region Imm decode


  // endregion Imm extraction

endmodule

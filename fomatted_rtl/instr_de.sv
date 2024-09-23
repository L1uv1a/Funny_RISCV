/* decodes the 32 bit instruction [DECODE STAGE]
*/

`include "rv32i_header.svh"

module instr_de (
    input wire clk,
    input wire rstn,

    input  wire [31:0] fetch_instr_i,    // 32 bit instruction
    input  wire [31:0] fetch_pc_i,  // PC value from previous stage
    output reg  [31:0] decode_pc_o,       // PC value
    output wire [ 4:0] rf_rs1_addr_o,      // Address of RS2 for Register File
    output reg  [ 4:0] fw_rs1_addr_o,    // Address of RS1 for Forward Module be used in Execute Stage
    output wire [ 4:0] rf_rs2_addr_o,      // Address of RS2 for Register File
    output reg  [ 4:0] fw_rs2_addr_o,    // Address of RS2 for Forward Module be used in Execute Stage
    output reg  [ 4:0] decode_rd_addr_o,       // address for destination address
    output reg  [31:0] decode_imm_o,      // extended value for immediate
    output reg  [ 2:0] decode_funct3_o,   // function type

    // ALU control
    output reg [      `ALU_WIDTH-1:0] decode_alu_operation_o,  // alu operation type
    output reg [   `OPCODE_WIDTH-1:0] decode_opcode_type_o,    // opcode type
    output reg [`EXCEPTION_WIDTH-1:0] decode_exception_o,      // illegal instr, ecall, ebreak, mret

    // Pipeline control
    input  wire        decode_en_i,  // input clk enable for pipeline stalling of this stage
    output reg         execute_en_o,       // output clk enable for pipeline stalling of next stage
    input  wire        decode_stall_i,   //informs this stage to stall
    output reg         decode_pipeline_stall_o,        //informs pipeline to stall
    input  wire        decode_flush_i,   //flush this stage
    output reg         decode_pipeline_flush_o         //flush previous stages
);

  //rf_rs1_addr_o and rf_rs2_addr_o are not registered since regs module do the registering itself
  assign rf_rs2_addr_o = fetch_instr_i[`RS2_RANGE];
  assign rf_rs1_addr_o = fetch_instr_i[`RS1_RANGE];

  wire [4:0] extracted_rd = fetch_instr_i[`RD_RANGE];


  wire [2:0] funct3_d = fetch_instr_i[`FUNCT3_RANGE];
  wire [6:0] opcode = fetch_instr_i[`OPCODE_RANGE];
  wire funct7_bit6 = fetch_instr_i[`FUNCT7_BIT6];

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

  



  
            ///////////////////
            // Opcode Decode //
            ///////////////////


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
  

  wire illegal_shift = (opcode_itype_d && (alu_sll_d || alu_srl_d || alu_sra_d)) && fetch_instr_i[25];
              

            /////////////////////////////////
            // Pipelined Register Updating //
            /////////////////////////////////
  wire stall_bit = (decode_stall_i || decode_pipeline_stall_o);  // stall this stage when next stages are stalled
  //register the outputs of this decoder module for shorter combinational timing paths
  always @(posedge clk, negedge rstn) begin : update_stage_registers
    if (!rstn) begin
      // Do nothing here
      // update registers only if this stage is enabled and pipeline is not stalled
    end else if (decode_en_i && (!stall_bit)) begin
      // capture any necessary registers to hold value between stages of the pipeline process
      pc                   <= fetch_pc_i;
      fw_rs1_addr_o        <= rf_rs1_addr_o;
      fw_rs2_addr_o        <= rf_rs2_addr_o;
      decode_rd_o          <= extracted_rd;
      decode_funct3_o      <= funct3_d;
      decode_imm_o         <= imm_d;

      /// ALU Operations ////
      decode_alu_operation_o[`ADD]  <= alu_add_d;
      decode_alu_operation_o[`SUB]  <= alu_sub_d;
      decode_alu_operation_o[`SLT]  <= alu_slt_d;
      decode_alu_operation_o[`SLTU] <= alu_sltu_d;
      decode_alu_operation_o[`XOR]  <= alu_xor_d;
      decode_alu_operation_o[`OR]   <= alu_or_d;
      decode_alu_operation_o[`AND]  <= alu_and_d;
      decode_alu_operation_o[`SLL]  <= alu_sll_d;
      decode_alu_operation_o[`SRL]  <= alu_srl_d;
      decode_alu_operation_o[`SRA]  <= alu_sra_d;
      decode_alu_operation_o[`EQ]   <= alu_eq_d;
      decode_alu_operation_o[`NEQ]  <= alu_neq_d;
      decode_alu_operation_o[`GE]   <= alu_ge_d;
      decode_alu_operation_o[`GEU]  <= alu_geu_d;

      decode_opcode_type_o[`RTYPE]  <= opcode_rtype_d;
      decode_opcode_type_o[`ITYPE]  <= opcode_itype_d;
      decode_opcode_type_o[`LOAD]   <= opcode_load_d;
      decode_opcode_type_o[`STORE]  <= opcode_store_d;
      decode_opcode_type_o[`BRANCH] <= opcode_branch_d;
      decode_opcode_type_o[`JAL]    <= opcode_jal_d;
      decode_opcode_type_o[`JALR]   <= opcode_jalr_d;
      decode_opcode_type_o[`LUI]    <= opcode_lui_d;
      decode_opcode_type_o[`AUIPC]  <= opcode_auipc_d;
      decode_opcode_type_o[`SYSTEM] <= opcode_system_d;
      decode_opcode_type_o[`FENCE]  <= opcode_fence_d;

      // Exceptions
      decode_exception_o[`ILLEGAL]  <= (!valid_opcode || illegal_shift);
      decode_exception_o[`ECALL]    <= (system_noncsr && fetch_instr_i[21:20] == 2'b00);
      decode_exception_o[`EBREAK]   <= (system_noncsr && fetch_instr_i[21:20] == 2'b01);
      decode_exception_o[`MRET]     <= (system_noncsr && fetch_instr_i[21:20] == 2'b10);
    end
  end
  

  

          //////////////////////
          // Pipeline Control //
          //////////////////////


  always @(posedge clk or negedge rstn) begin : update_stages_control
    if (!rstn) execute_en_o <= 0;
    else begin
      // flush this stage so clock-enable of next stage is disabled at next clock cycle
      if ((!stall_bit) && decode_flush_i) execute_en_o <= 0;
      // clock-enable will change only when not stalled
      else if (!stall_bit) execute_en_o <= decode_en_i;
      // if this stage is stalled but next stage is not, disable clock enable of next stage at next clock cycle (pipeline bubble)
      else if (stall_bit && (!decode_stall_i)) execute_en_o <= 0;
    end
  end
  


  always @* begin : update_pipeline
    decode_pipeline_stall_o = decode_stall_i;  // stall previous stage when decoder needs wait time
    decode_pipeline_flush_o = decode_flush_i;  // flush this stage along with the previous stages 
  end
 



            /////////////////
            // ALU Decoder //
            /////////////////


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

    // ALU Decode logic handle
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
 


          //////////////////////////
          // Immediate Extraction //
          //////////////////////////

  wire [31:0] imm_i = {
    {`ITYPE_IMM_SIGN_EXPAND_BITS{fetch_instr_i[`IMM_SIGN_BIT]}},  // Sign expand
    fetch_instr_i[`ITYPE_IMM_RANGE]  // Imm extraction
  };


  wire [31:0] imm_s = {
    {`STYPE_IMM_SIGN_EXPAND_BITS{fetch_instr_i[`IMM_SIGN_BIT]}},  // Sign expand
    fetch_instr_i[`STYPE_IMM_RANGE_11_5],  // Imm extraction
    fetch_instr_i[`STYPE_IMM_RANGE_4_0]
  };

  wire [31:0] imm_b = {
    {`BTYPE_IMM_SIGN_EXPAND_BITS{fetch_instr_i[`IMM_SIGN_BIT]}},  // Sign expand
    fetch_instr_i[`BTYPE_IMM_RANGE_12],  // Imm extraction
    fetch_instr_i[`BTYPE_IMM_RANGE_11],
    fetch_instr_i[`BTYPE_IMM_RANGE_10_5],
    fetch_instr_i[`BTYPE_IMM_RANGE_4_1],
    {`BTYPE_IMM_ZERO_FILL_BITS{1'b0}}  // zero fill
  };

  wire [31:0] imm_j = {
    {`JTYPE_IMM_SIGN_EXPAND_BITS{fetch_instr_i[`IMM_SIGN_BIT]}},  // Sign expand
    fetch_instr_i[`JTYPE_IMM_RANGE_20],  // Imm extraction
    fetch_instr_i[`JTYPE_IMM_RANGE_19_12],
    fetch_instr_i[`JTYPE_IMM_RANGE_11],
    fetch_instr_i[`JTYPE_IMM_RANGE_10_1],
    {`JTYPE_IMM_ZERO_FILL_BITS{1'b0}}  // zero fill
  };

  wire [31:0] imm_u = {
    fetch_instr_i[`UTYPE_IMM_RANGE],  // Imm extraction
    {`UTYPE_IMM_ZERO_FILL_BITS{1'b0}}  // zero fill
  };

  wire [31:0] imm_x = {{`XTYPE_IMM_ZERO_FILL_BITS{1'b0}}, fetch_instr_i[`XTYPE_IMM_RANGE]};



          ////////////////////////
          // Immediate Selector //
          ////////////////////////

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
  // region decode_imm_o decode

  
endmodule

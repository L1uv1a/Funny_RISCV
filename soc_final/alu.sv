`include "rv32i_header.svh"

module alu (
  input wire clk,
  input wire rstn,

  // instruction code from previous stage 
  input  wire [`ALU_WIDTH-1:0]        alu_operation,

  // rs1 address
  input  wire [4:0]                   prev_rs1,           // rs1 addr from previous stage
  output reg  [4:0]                   rs1,                // rs1 addr

  // rs1, rs2 datas
  input  wire [31:0]                  prev_rs1_data,      // rs1 data from previous stage
  output reg  [31:0]                  rs1_data,           // rs1 data
  input  wire [31:0]                  prev_rs2_data,      // rs2 data from previous stage
  output reg  [31:0]                  rs2_data,           // rs2 data

  // Immediates
  input  wire [31:0]                  prev_imm,           // Imm value from previous stage
  output reg  [11:0]                  imm,                // Imm value

  // Funct3
  input  wire [2:0]                   prev_funct3,        // funct3 from previous stage
  output reg  [2:0]                   funct3,             // funct3

  // Opcode type
  input  wire [`OPCODE_WIDTH-1:0]     prev_opcode_type,   // opcode type from previous stage
  output reg  [`OPCODE_WIDTH-1:0]     opcode_type,        // opcode type

  // Exception
  input  wire [`EXCEPTION_WIDTH-1:0]  prev_exception,     // exception from previous stage
  output reg  [`EXCEPTION_WIDTH-1:0]  exception,          // exception

  // Result of arithmetic operation
  output reg  [31:0]                  alu_result,

  // PC control
  input  wire [31:0]                  prev_pc,            // pc
  output reg  [31:0]                  pc,                 // pc in pipeline
  output reg  [31:0]                  next_pc,            // new pc
  output reg                          change_pc,          // pc need to jump

  // Base registers (x) control
  output reg                          rd_w_en,            // enable write rd to regs (from alu)
  input  wire [ 4:0]                  prev_rd,            // rd addr from previous stage
  output reg  [ 4:0]                  rd,                 // rd addr
  output reg  [31:0]                  rd_wdata,           // rd data to be written back to rd addr
  output reg                          rd_valid,           // rd_wdata is valid (not Load or CSR instructions)

  // Pipeline control
  output reg                          stall_from_alu,     // stall next stage [Mem] for load/store instructions
  input  wire                         prev_clk_en,        // input clk enable for pipeline stalling of this stage
  output reg                          clk_en,             // output clk enable for pipeline stalling of this stage
  input  wire                         prev_stall,         // informs this stage to stall
  input  wire                         force_stall,        // force this stage to stall
  output reg                          stall,              // informs pipeline to stall
  input  wire                         prev_flush,         // flush this stage
  output reg                          flush               // flush previous stages
);
  ///////////////////
  // ALU operation //     (signals that determine result is given by which arithmetic operation)
  ///////////////////
  wire        alu_add        =  alu_operation[`ADD];
  wire        alu_sub        =  alu_operation[`SUB];
  wire        alu_slt        =  alu_operation[`SLT];
  wire        alu_sltu       =  alu_operation[`SLTU];
  wire        alu_xor        =  alu_operation[`XOR];
  wire        alu_or         =  alu_operation[`OR];
  wire        alu_and        =  alu_operation[`AND];
  wire        alu_sll        =  alu_operation[`SLL];
  wire        alu_srl        =  alu_operation[`SRL];
  wire        alu_sra        =  alu_operation[`SRA];
  wire        alu_eq         =  alu_operation[`EQ];
  wire        alu_neq        =  alu_operation[`NEQ];
  wire        alu_ge         =  alu_operation[`GE];
  wire        alu_geu        =  alu_operation[`GEU];
  wire         alu_sh1add     = alu_operation[`SH1ADD];
  wire         alu_sh2add     = alu_operation[`SH2ADD];
  wire         alu_sh3add     = alu_operation[`SH3ADD];
  wire         alu_xnor       = alu_operation[`XNOR];
  wire         alu_zext_h     = alu_operation[`ZEXT_H];
  wire         alu_andn       = alu_operation[`ANDN];
  wire         alu_clz        = alu_operation[`CLZ];
  wire         alu_cpop       = alu_operation[`CPOP];
  wire         alu_ctz        = alu_operation[`CTZ];
  wire         alu_orc_b      = alu_operation[`ORC_B];
  wire         alu_orn        = alu_operation[`ORN];
  wire         alu_rev8       = alu_operation[`REV8];
  wire         alu_ror        = alu_operation[`ROR];
  wire         alu_rol        = alu_operation[`ROL];
  wire         alu_sext_b     = alu_operation[`SEXT_B];
  wire         alu_sext_h     = alu_operation[`SEXT_H];
  wire         alu_clmul      = alu_operation[`CLMUL];
  wire         alu_clmulh     = alu_operation[`CLMULH];
  wire         alu_clmulr     = alu_operation[`CLMULR];
  wire         alu_bclr       = alu_operation[`BCLR];
  wire         alu_bext       = alu_operation[`BEXT];
  wire         alu_binv       = alu_operation[`BINV];
  wire         alu_bset       = alu_operation[`BSET];
  wire         alu_min        = alu_operation[`MIN];
  wire         alu_max        = alu_operation[`MAX];
  wire         alu_minu       = alu_operation[`MINU];
  wire         alu_maxu       = alu_operation[`MAXU];

  /////////////////
  // opcode type //
  /////////////////
  wire        opcode_rtype   =  prev_opcode_type[`RTYPE];
  wire        opcode_itype   =  prev_opcode_type[`ITYPE];
  wire        opcode_load    =  prev_opcode_type[`LOAD];
  wire        opcode_store   =  prev_opcode_type[`STORE];
  wire        opcode_branch  =  prev_opcode_type[`BRANCH];
  wire        opcode_jal     =  prev_opcode_type[`JAL];
  wire        opcode_jalr    =  prev_opcode_type[`JALR];
  wire        opcode_lui     =  prev_opcode_type[`LUI];
  wire        opcode_auipc   =  prev_opcode_type[`AUIPC];
  wire        opcode_system  =  prev_opcode_type[`SYSTEM];
  wire        opcode_fence   =  prev_opcode_type[`FENCE];

  ///////////////////////////
  // alu registered values //
  ///////////////////////////
  reg  [31:0] operand_a;                                       // operand a
  reg  [31:0] operand_b;                                       // operand b
  reg  [31:0] result;                                     // result
  reg  [31:0] rd_wdata_d;                                 // next value to be written back to rd address
  reg         rd_w_en_d;                                  // enable writeback rd_wdata to base registers (x) at rd addr
  reg         rd_valid_d;                                 // rd_wdata is valid (not Load or CSR instructions)
  reg  [31:0] a_pc;
  wire [31:0] pc_plus_imm    =  a_pc + prev_imm;
  wire [31:0] pc_plus_4      =  prev_pc + 4;

  wire        stall_bit      =  (prev_stall || stall);    // stall this stage when next stages are stalled

  ////////////////////////////////
  // update alu stage registers //
  ////////////////////////////////
  always @(posedge clk, negedge rstn) begin : update_stage_registers
    // update registers only if this stage is enabled
    if (!rstn) begin                                      
      exception             <=  0;
      stall_from_alu        <=  0;
    end
    else if ((!stall_bit) && prev_clk_en) begin
      opcode_type           <=  prev_opcode_type;
      exception             <=  prev_exception;
      alu_result            <=  result;
      rs1                   <=  prev_rs1;
      rs1_data              <=  prev_rs1_data;
      rs2_data              <=  prev_rs2_data;
      rd                    <=  prev_rd;
      imm                   <=  prev_imm[11:0];
      funct3                <=  prev_funct3;
      rd_wdata              <=  rd_wdata_d;
      rd_valid              <=  rd_valid_d;
      rd_w_en               <=  rd_w_en_d;
      // stall next stage [Mem] when need to store/load (accessing data memory always takes more than 1 cycle)
      stall_from_alu        <=  (prev_opcode_type[`STORE] || prev_opcode_type[`LOAD]);
      pc                    <=  prev_pc;
    end
  end

  ///////////////////////////////////
  // update stages control signals //
  ///////////////////////////////////
  always @(posedge clk, negedge rstn) begin : update_stages_cotrol
    if (!rstn)                             clk_en <= 0;
    else begin
      // flush this stage so clock-enable of next stage is disabled at next clock cycle
      if ((!stall_bit) && prev_flush)      clk_en <= 0;
      // clock-enable will change only when not stalled
      else if (!stall_bit)                 clk_en <= prev_clk_en;
      // if this stage is stalled but next stage is not, disable clock enable of next stage at next clock cycle (pipeline bubble)
      else if (stall_bit && (!prev_stall)) clk_en <= 0;
    end
  end

  assign operand_a = (opcode_jal || opcode_auipc) ? prev_pc : prev_rs1_data;// Operand A can either be PC or RS1
  assign operand_b = (opcode_rtype || opcode_branch) ? prev_rs2_data : prev_imm; // Operand B can either be RS2 or Imm



  ///////////////////////////
  // Bit Counting Region   //
  ///////////////////////////
    
  logic [31:0] bitcnt_bits;
  logic [31:0] bitcnt_bit_mask;
  logic [31:0] bitcnt_mask_op;
  logic [31:0] operand_a_rev;
  logic [5:0] bitcnt_partial [32];

  for (genvar k = 0; k < 32; k++) begin : gen_rev_operand_a
    assign operand_a_rev[k] = operand_a[31-k];
  end
  // Bit-mask generation for clz and ctz:
  // The bit mask is generated by spreading the lowest-order set bit in the operand to all
  // higher order bits. The resulting mask is inverted to cover the lowest order zeros. In order
  // to create the bit mask for leading zeros, the input operand needs to be reversed.
  assign bitcnt_mask_op = alu_clz ? operand_a_rev : operand_a;

  always_comb begin : Bitcounting
      bitcnt_bit_mask = bitcnt_mask_op;
      bitcnt_bit_mask |= bitcnt_bit_mask << 1;
      bitcnt_bit_mask |= bitcnt_bit_mask << 2;
      bitcnt_bit_mask |= bitcnt_bit_mask << 4;
      bitcnt_bit_mask |= bitcnt_bit_mask << 8;
      bitcnt_bit_mask |= bitcnt_bit_mask << 16;
      bitcnt_bit_mask = ~bitcnt_bit_mask;

      if(alu_clz || alu_ctz) bitcnt_bits = bitcnt_bit_mask & ~bitcnt_mask_op;
      else bitcnt_bits = operand_a;
  end


  always_comb begin : BitCountingCalculate
    bitcnt_partial = '{default: '0};
      // stage 1
      for (int unsigned i = 1; i < 32; i += 2) begin
        bitcnt_partial[i] = {5'h0, bitcnt_bits[i]} + {5'h0, bitcnt_bits[i-1]};
      end
      // stage 2
      for (int unsigned i = 3; i < 32; i += 4) begin
        bitcnt_partial[i] = bitcnt_partial[i-2] + bitcnt_partial[i];
      end
      // stage 3
      for (int unsigned i = 7; i < 32; i += 8) begin
        bitcnt_partial[i] = bitcnt_partial[i-4] + bitcnt_partial[i];
      end
      // stage 4
      for (int unsigned i = 15; i < 32; i += 16) begin
        bitcnt_partial[i] = bitcnt_partial[i-8] + bitcnt_partial[i];
      end
      // stage 5
      bitcnt_partial[31] = bitcnt_partial[15] + bitcnt_partial[31];
  end


  ///////////////////////////////////
  // Carry-less Multiply Region    //
  ///////////////////////////////////
  logic [31:0] clmul_op_a;
  logic [31:0] clmul_op_b;
  logic [31:0] operand_b_rev;
  logic [31:0] clmul_and_stage[32];
  logic [31:0] clmul_xor_stage1[16];
  logic [31:0] clmul_xor_stage2[8];
  logic [31:0] clmul_xor_stage3[4];
  logic [31:0] clmul_xor_stage4[2];
  logic [31:0] clmul_result_rev;
  logic [31:0] clmul_result_raw;

  for (genvar i = 0; i < 32; i++) begin : gen_rev_operand_b
    assign operand_b_rev[i] = operand_b[31-i];
  end

  always_comb begin : Carry_Less_Multiply_Calculate
    clmul_op_a = alu_clmulr | alu_clmulh ? operand_a_rev : operand_a;
    clmul_op_b = alu_clmulr | alu_clmulh ? operand_b_rev : operand_b;

    for (int unsigned i = 0; i < 32; i++) begin : gen_clmul_and_op
      clmul_and_stage[i] = clmul_op_b[i] ? clmul_op_a << i : '0;
    end

    for (int unsigned i = 0; i < 16; i++) begin : gen_clmul_xor_op_l1
      clmul_xor_stage1[i] = clmul_and_stage[2*i] ^ clmul_and_stage[2*i+1];
    end

    for (int unsigned i = 0; i < 8; i++) begin : gen_clmul_xor_op_l2
      clmul_xor_stage2[i] = clmul_xor_stage1[2*i] ^ clmul_xor_stage1[2*i+1];
    end

    for (int unsigned i = 0; i < 4; i++) begin : gen_clmul_xor_op_l3
      clmul_xor_stage3[i] = clmul_xor_stage2[2*i] ^ clmul_xor_stage2[2*i+1];
    end

    for (int unsigned i = 0; i < 2; i++) begin : gen_clmul_xor_op_l4
      clmul_xor_stage4[i] = clmul_xor_stage3[2*i] ^ clmul_xor_stage3[2*i+1];
    end

    clmul_result_raw = clmul_xor_stage4[0] ^ clmul_xor_stage4[1];

    for (int unsigned i = 0; i < 32; i++) begin : gen_rev_clmul_result
      clmul_result_rev[i] = clmul_result_raw[31-i];
    end
  end


  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Shift Region (Using for ROR, ROL, may be use for SRA, SRL, SLL, ..etc when we optimize our core)    //
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  logic shift_left;
  logic [4:0] shift_amt, shift_amt_compl;
  logic [31:0] shift_operand;
  logic [31:0] shift_result, shift_result_rev;
  logic [31:0] temp_shift_value;
  logic alu_rotate;
  logic valid;
  logic rotate_stall;
  
  assign alu_rotate = alu_rol || alu_ror;
  assign shift_amt_compl = 32 - operand_b[4:0];
  assign shift_amt = (!valid) ? operand_b[4:0] : shift_amt_compl;
  assign rotate_stall = alu_rotate && (!valid);
  
  always_comb begin : Shift_order
    if(alu_rol) shift_left = !valid; //Ex: ROL 5 need shift left 5 | shift right 27
    else if(alu_ror) shift_left = valid;
    else shift_left = 1'b0;
  end

  always_comb begin : Shift_Calculate
    shift_operand = shift_left ? operand_a_rev : operand_a;
    shift_result = shift_operand >> shift_amt;
    for (int unsigned i = 0; i < 32; i++) begin
      shift_result_rev[i] = shift_result[31-i];
    end
    shift_result = shift_left ? shift_result_rev : shift_result;
  end

  always @(posedge clk or negedge rstn) begin
      if(!rstn) begin
        valid <= 1;
      end
      else begin
        if(alu_rotate) begin
          valid <= !valid;
        end
      end
  end
  always_comb begin
    if(!valid && alu_rotate) temp_shift_value = shift_result;
    else if(valid && alu_rotate) temp_shift_value = temp_shift_value;
    else temp_shift_value = 0;
  end
  ///////////////////////////////////
  // perform arithmethic operation //
  ///////////////////////////////////
  always_comb begin
    result = 32'b0;
    if (alu_add)     result = operand_a + operand_b;
    if (alu_sub)     result = operand_a - operand_b;
    if (alu_xor)     result = operand_a ^ operand_b;
    if (alu_or)      result = operand_a | operand_b;                     ;
    if (alu_slt)     result = ($signed(operand_a) < $signed(operand_b));
    if (alu_sltu)    result = (operand_a < operand_b);
    if (alu_and)     result = (operand_a & operand_b);
    if (alu_sll)     result = (operand_a << operand_b[4:0]);
    if (alu_srl)     result = (operand_a >> operand_b[4:0]);
    if (alu_sra)     result = ($signed(operand_a) >>> operand_b[4:0]);
    if (alu_eq)      result = (operand_a == operand_b);
    if (alu_neq)     result = (operand_a != operand_b);
    if (alu_ge)      result = ($signed(operand_a) >= $signed(operand_b));
    if (alu_geu)     result = (operand_a >= operand_b);
    if (alu_sh1add)  result = ((operand_a << 1) + operand_b);
    if (alu_sh2add)  result = ((operand_a << 2) + operand_b);
    if (alu_sh3add)  result = ((operand_a << 3) + operand_b);
    if (alu_xnor)    result = ~(operand_a ^ operand_b);
    if (alu_zext_h)  result = {15'b0,operand_a[15:0]};
    if (alu_andn)    result = (operand_a & ~operand_b);
    if (alu_clz || alu_cpop || alu_ctz) result = {26'b0, bitcnt_partial[31]};    
    if (alu_max)     result = ($signed(operand_a) >= $signed(operand_b)) ? operand_a : operand_b;
    if (alu_maxu)    result = (operand_a >= operand_b) ? operand_a : operand_b;
    if (alu_min)     result = ($signed(operand_a) < $signed(operand_b)) ? operand_a : operand_b;
    if (alu_minu)    result = (operand_a < operand_b) ? operand_a : operand_b;
    if (alu_orc_b) begin
        result[7:0] = (operand_a[7:0] == 8'h00) ? 8'h00 : 8'hFF;
        result[15:8] = (operand_a[15:8] == 8'h00) ? 8'h00 : 8'hFF;
        result[23:16] = (operand_a[23:16] == 8'h00) ? 8'h00 : 8'hFF;
        result[31:24] = (operand_a[31:24] == 8'h00) ? 8'h00 : 8'hFF;
    end   
    if (alu_orn)     result = operand_a | (~operand_b);
    if (alu_rev8) begin
        result[7:0] =   operand_a[31:24];
        result[15:8] =  operand_a[23:16];
        result[23:16] = operand_a[15:8];
        result[31:24] = operand_a[7:0];
    end    
    if (alu_ror || alu_rol)     result = shift_result | temp_shift_value;
    if (alu_sext_b)  result = {{24{operand_a[7]}}, operand_a[7:0]};
    if (alu_sext_h)  result = {{16{operand_a[15]}}, operand_a[15:0]};
    if (alu_clmul)   result = clmul_result_raw;
    if (alu_clmulr)  result = clmul_result_rev;
    if (alu_clmulh)  result = {1'b0, clmul_result_rev[31:1]};
    if (alu_bclr)    result = operand_a & (~(1 << operand_b[4:0]));
    if (alu_bext)    result = ((operand_a >> operand_b[4:0]) & 1) ;
    if (alu_binv)    result = operand_a ^ (1 << operand_b[4:0]);
    if (alu_bset)    result = operand_a | (1 << operand_b[4:0]);
  end
  
  ///////////////////////////////////////
  // determine rd and next value of PC //
  ///////////////////////////////////////
  always @* begin
    //flush this stage along with the previous stages
    flush      = prev_flush;
    rd_wdata_d = 0;
    change_pc  = 0;
    next_pc    = 0;
    a_pc       = prev_pc;

    if (!prev_flush) begin
      if (opcode_rtype || opcode_itype) rd_wdata_d = result;
      // branch if ALU result (operand_a and operand_b comparision) is true
      if (opcode_branch && (result == `TRUE)) begin
        next_pc    = pc_plus_imm;
        // change PC when clk_en for this stage is high
        change_pc  = prev_clk_en;  // change_pc is valid
        flush      = prev_clk_en;
      end
      // jalr jump to new PC, which stored in rs1
      if (opcode_jal || opcode_jalr) begin
        if (opcode_jalr) 
        a_pc       = prev_rs1_data;
        next_pc    = pc_plus_imm;
        // change PC when clk_en for this stage is high
        change_pc  = prev_clk_en;
        flush      = prev_clk_en;
        // store next pc to rd
        rd_wdata_d = pc_plus_4;
      end
    end

    if (opcode_lui)   rd_wdata_d = prev_imm;
    if (opcode_auipc) rd_wdata_d = pc_plus_imm;
  end

  /////////////////////////////////////
  // Determine read, write, stall //
  /////////////////////////////////////
  always @* begin : check_exception_write_rd_enable
    // always write to the destination reg except when instruction is BRANCH or STORE or SYSTEM (except CSR system instruction)  
    if ((opcode_system && (prev_funct3 == 0)) || opcode_branch || opcode_store || opcode_fence) // funct3 == 0 are the non-csr system instructions         
      rd_w_en_d = 0; 
    else 
      rd_w_en_d = 1;
  end

  always @* begin : check_valid
    // value of rd for load and CSR write is not yet available at this stage
    if (opcode_load || (opcode_system && funct3 != 0))
      rd_valid_d = 0;  
    else 
      rd_valid_d = 1;
  end

  always @* begin : check_stall
    // stall logic (stall when upper stages are stalled, when forced to stall, or when needs to flush previous stages but are still stalled)
    stall = ((!prev_flush) && (prev_stall || force_stall)) || rotate_stall;  //stall when alu needs wait time
  end
endmodule

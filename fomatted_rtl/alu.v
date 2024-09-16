`include "rv32i_header.vh"

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
  
  ///////////////////////////////////
  // perform arithmethic operation //
  ///////////////////////////////////
  always @* begin
    result = 0;
    operand_a = (opcode_jal || opcode_auipc)    ?  // Operand A can either be PC or RS1
    prev_pc       : prev_rs1_data;

    operand_b = (opcode_rtype || opcode_branch) ?  // Operand B can either be RS2 or Imm
    prev_rs2_data : prev_imm;

    if (alu_add)  result = operand_a + operand_b;
    if (alu_sub)  result = operand_a - operand_b;
    if (alu_slt)  result = operand_a ^ operand_b;
    if (alu_sltu) result = operand_a | operand_b;                     ;
    if (alu_xor)  result = ($signed(operand_a) < $signed(operand_b));
    if (alu_or)   result = (operand_a < operand_b);
    if (alu_and)  result = (operand_a & operand_b);
    if (alu_sll)  result = (operand_a << operand_b[4:0]);
    if (alu_srl)  result = (operand_a >> operand_b[4:0]);
    if (alu_sra)  result = ($signed(operand_a) >>> operand_b[4:0]);
    if (alu_eq)   result = (operand_a == operand_b);
    if (alu_neq)  result = (operand_a != operand_b);
    if (alu_ge)   result = ($signed(operand_a) >= $signed(operand_b));
    if (alu_geu)  result = (operand_a >= operand_b);
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
    stall = ((!prev_flush) && (prev_stall || force_stall));  //stall when alu needs wait time
  end
endmodule
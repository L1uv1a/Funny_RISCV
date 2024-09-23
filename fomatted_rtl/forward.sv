`include "rv32i_header.svh"

module forward (
    // current read data of rs1 and rs1 from regs
    input wire [31:0] rs1_rdata,
    input wire [31:0] rs2_rdata,

    // rs1 and rs2 address are extracted from instruction (from [Decode] stage, which also used in [ALU] stage)
    input wire [4:0]  rs1,
    input wire [4:0]  rs2,

    output reg        alu_force_stall,  // force [ALU] stage to stall

    // rs1 and rs2 data to Operand forwarding to [Execute] stage
    output reg [31:0] fwd_rs1_rdata,
    output reg [31:0] fwd_rs2_rdata,

    // [Mem] stage 4
    input wire [ 4:0] alu_rd,        // rd addr
    input wire        alu_rd_w_en,   // rd will be written
    input wire        alu_rd_valid,  // rd is valid at this stage (no LOAD or CSR instr)
    input wire [31:0] alu_rd_data,   // rd value in stage 4
    input wire        mem_en,        // high if stage 4 is enabled

    // [Writeback] stage 5
    input wire [ 4:0] mem_rd,             // rd addr
    input wire        mem_rd_w_en,        // rd will be written
    input wire [31:0] writeback_rd_data,  // rd value in stage 5
    input wire        writeback_en        // high if stage 4 is enabled
);

  /* Data hazard:
        Register value is about to be overwritten by previous instructions but
        are still on the pipeline and are not yet written to base registers (x).
        To avoid this, make sure the updated value of rs1 or rs2 is used by
        either stall the pipeline until the base registers is updated or use
        operand forwarding.
  */
  wire next_rs1_is_on_stage4 = (rs1 == alu_rd) && alu_rd_w_en && mem_en;
  wire next_rs1_is_on_stage5 = (rs1 == mem_rd) && mem_rd_w_en && writeback_en;

  wire next_rs2_is_on_stage4 = (rs2 == alu_rd) && alu_rd_w_en && mem_en;
  wire next_rs2_is_on_stage5 = (rs2 == mem_rd) && mem_rd_w_en && writeback_en;

  always @* begin : forward_and_stall
    fwd_rs1_rdata   = rs1_rdata;
    fwd_rs2_rdata   = rs2_rdata;
    alu_force_stall = 0;

    // Operand forwarding for rs1
    // if next value of rs1 is currently on stage 4
    if (next_rs1_is_on_stage4) begin
      // forward the stage 4 rs1 data
      fwd_rs1_rdata = alu_rd_data;

      /* if next stage value of rs1 comes from Load or CSR instruction then
      we must stall from [ALU] stage and wait until stage 4 [Mem] becomes disabled,
      which means next value of rs1 is already at stage 5 */
      if (!alu_rd_valid) alu_force_stall = 1;
    end 
    else if (next_rs1_is_on_stage5) begin
      fwd_rs1_rdata = writeback_rd_data;
    end

    // Operand forwarding for rs2
    // if next value of rs2 is currently on stage 4
    if (next_rs2_is_on_stage4) begin
      // forward the stage 4 rs1 data
      fwd_rs2_rdata = alu_rd_data;

      /* if next stage value of rs1 comes from Load or CSR instruction then
      we must stall from [ALU] stage and wait until stage 4 [Mem] becomes disabled,
      which means next value of rs1 is already at stage 5 */
      if (!alu_rd_valid) alu_force_stall = 1;
    end 
    else if (next_rs2_is_on_stage5) begin
      fwd_rs2_rdata = writeback_rd_data;
    end

    // No oprand forward necessary when rs = 0 (try to operand on x0 (zero) register)
    if (rs1 == `ZERO_REG_ADDR) fwd_rs1_rdata = 0;
    if (rs2 == `ZERO_REG_ADDR) fwd_rs2_rdata = 0;
  end
endmodule

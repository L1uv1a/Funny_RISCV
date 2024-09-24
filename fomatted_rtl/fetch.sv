/* retrieves instruction from the memory [FETCH STAGE]
*/
module fetch #(
    parameter PC_RESET = 0
) (
    input clk,
    input rstn,

    output reg [31:0] pc,         // PC value of current instruction
    output reg [31:0] instr_send, // instruction sent to pipeline

    output logic        instr_req_o,    // req
    input  logic        instr_gnt_i,    // gnt
    output logic [31:0] instr_addr_o,   // addr
    input  logic [31:0] instr_rdata_i,  // rdata
    input  logic        instr_err_i,    // err    // fix NO USE
    input  logic        instr_rvalid_i, // valid  // fix NO USE

    // PC control
    input        writeback_change_pc,  // high when pc needs to change (trap/return from trap)
    input [31:0] writeback_next_pc,    // next PC due to trap
    input        alu_change_pc,        // high when pc needs to change (branch/jump)
    input [31:0] alu_next_pc,          // next PC due to branch/jump

    // Pipeline control
    output reg clk_en,  // output clk enable for pipeline stalling of next state
    input      stall,   // stall logic for whole pipeline
    input      flush    // flush this stage
);

  wire        instr_req;  // request for instruction
  reg  [31:0] instr_addr;  // instruction memory address
  wire [31:0] instr_mem;  // instruction from memory
  wire        instr_ack;  // high if new instruction is now on the bus

  assign instr_req_o = instr_req;
  assign instr_ack = instr_gnt_i;
  assign instr_addr_o  = instr_addr;
  assign instr_mem = instr_rdata_i;



  reg  [31:0] prev_pc;
  reg  [31:0] stalled_instr;
  reg  [31:0] stalled_pc;
  reg         r_clk_en;
  reg         r_clk_en_d;
  reg         stall_fetch;
  reg         stall_q;


  reg [31:0] r_instr_addr;
  /* Stall conditions
  stall this stage when:
  - next stages are stalled
  - request but no ack yet
  - no request at all (no instruction to execute for this stage)
  */
  wire        stall_bit = (stall_fetch ||  // stall fetch
 stall ||  // stall
 (instr_req && !instr_ack) ||  // request but no ack
 !instr_req  // no request
);
  assign instr_req = r_clk_en;  // request for new instruction if this stage is enabled

  // clk enable logic for fetch stage
  wire disable_next_stage = ((alu_change_pc || writeback_change_pc) && !(stall || stall_fetch));
  always @(posedge clk, negedge rstn) begin
    if (!rstn) r_clk_en <= 0;
    // do pipeline bubble when need to change pc so that next stage will be disable
    // and will not execute the instructions already inside the pipeline
    else
      r_clk_en <= !disable_next_stage;
  end

  /* Update registers conditions
  update registers only if this stage is enable and next stages are not stalled
  */
  wire enable_update_registers = ((!stall_bit && r_clk_en) ||  //
  (stall_bit && !clk_en && r_clk_en) ||  //
  (writeback_change_pc)  //
  );
  always @(posedge clk, negedge rstn) begin
    if (!rstn) begin
      clk_en        <= 0;
      instr_addr    <= PC_RESET;
      prev_pc       <= PC_RESET;
      stalled_instr <= 0;
      pc            <= 0;
    end else begin
      // update registers only if this stage is enabled and next stages are not stalled
      if (enable_update_registers) begin
        instr_addr <= r_instr_addr;
        pc <= stall_q ? stalled_pc : prev_pc;
        instr_send <= stall_q ? stalled_instr : instr_mem;
      end
      // flush this stage (only when not stalled) so that clock-enable of next stage is disabled at next clock cycle
      if (!stall_bit && flush) clk_en <= 0;
      //clock-enable will change only when not stalled
      else if (!stall_bit) clk_en <= r_clk_en_d;
      //if this stage is stalled but next stage is not, disable clock enable of next stage at next clock cycle (pipeline bubble)
      else if (stall_bit && !stall) clk_en <= 0;

      // raise stall when any of 5 stages is stalled
      stall_q <= (stall || stall_fetch);

      // store instruction and PC before stalling so we can come back to these values when we need to return from stall
      if (stall_bit && !stall_q) begin
        stalled_pc <= prev_pc;
        stalled_instr <= instr_mem;
      end

      // first delay to align the PC to the pipeline
      prev_pc <= instr_addr;
    end
  end


    // pc and pipeline clk enable control logic

  always @* begin
    r_instr_addr = 0;
    r_clk_en_d   = 0;
    stall_fetch  = stall;  // stall when retrieving instructions need wait time prepare next PC when changing pc, then do a pipeline bubble to disable the ce of next stage
    if (writeback_change_pc) begin
      r_instr_addr = writeback_next_pc;
      r_clk_en_d   = 0;
    end else if (alu_change_pc) begin
      r_instr_addr = alu_next_pc;
      r_clk_en_d   = 0;
    end else begin
      r_instr_addr = instr_addr + 4;
      r_clk_en_d   = r_clk_en;
    end
  end


endmodule

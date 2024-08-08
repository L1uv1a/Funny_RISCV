module fetch #(
    parameter PC_RESET = 0
) (
    input clk,
    input rst,

    // region control Main Memory's Instruction Memory
    // Instruction Memory
    output reg [31:0] main_memory_instr_addr,  // instruction memory address
    input      [31:0] main_memory_instr,       // instruction from memory
    output            main_memory_instr_req,   // request for instruction
    input             main_memory_instr_ack,   // ack
    // endregion control Main Memory's Instruction Memory

    output reg [31:0] fetch_instr,  // fetched instruction sent to pipeline

    // region PC control
    output reg [31:0] pc,  // PC value of current instruction

    // control from [STAGE 5 WRITEBACK]
    input        writeback_change_pc,  // high when pc needs to change (trap/return from trap)
    input [31:0] writeback_next_pc,    // next PC due to trap

    // control from [STAGE 3 EXECUTE]
    input        execute_change_pc,  // high when pc needs to change (branch/jump)
    input [31:0] execute_next_pc,    // next PC due to branch/jump
    // endregion PC control


    // region Pipeline control
    input      stall,       // stall this stage
    input      flush,       // flush this stage
    output reg next_clk_en  // clk enable for pipeline stalling of next state ([STAGE 2 DECODE])
    // endregion Pipeline control
);

  reg r_clk_en;
  reg r_clk_en_d;
  reg r_stall;

  /* Stall conditions
  stall this stage when:
  - stall this stage
  - memory instruction requested but no ack yet
  - no memory instruction request at all (no instruction to execute for this stage)
  */
  wire stall_bit = (  // stall conditions check bit
  stall ||  // stall this stage
  (main_memory_instr_req && (!main_memory_instr_ack)) || // memory instruction requested but no ack yet
  (!main_memory_instr_req) // no memory instruction request at all (no instruction to execute for this stage)
  );

  assign main_memory_instr_req = r_clk_en;  // request for new instruction if this stage is enabled

  // region clk enable logic for [STAGE 1 FETCH]
  // need to change pc ? (change pc signals from [STAGE 3 EXECUTE] or [STAGE 5 WRITEBACK])
  wire change_pc = (execute_change_pc || writeback_change_pc);

  // disable next stage if need to change pc not stall
  wire disable_next_stage = (change_pc && !(stall));

  always @(posedge clk, posedge rst) begin
    if (rst) r_clk_en <= 0;
    // do pipeline bubble when need to change pc so that the next stage will be disable
    // and will not execute the instruction already inside the pipeline
    else
      r_clk_en <= (!disable_next_stage);
  end
  // endregion clk enable logic for [STAGE 1 FETCH]

  // region update internal registers
  reg [31:0] prev_pc;
  reg [31:0] stalled_instr;
  reg [31:0] stalled_pc;

  /* Update registers conditions:
  - this stage is not stalled and currently enabled
  - this stage is stalled and currently enable but the next stage is disabled.
  - writeback change pc
  */
  wire enable_update_registers = (  //
  ((!stall) && r_clk_en) ||  // this stage is not stalled and currently enabled
  (stall && r_clk_en && (!next_clk_en))|| // this stage is stalled and currently enable but the next stage is disabled.
  writeback_change_pc  // writeback change pc
  );

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      next_clk_en            <= 0;
      main_memory_instr_addr <= PC_RESET;
      prev_pc                <= PC_RESET;
      stalled_instr          <= 0;
      pc                     <= 0;
      fetch_instr            <= 0;
      r_stall                <= 0;
      stalled_pc             <= 0;
    end else begin
      // update registers
      if (enable_update_registers) begin
        main_memory_instr_addr <= r_main_memory_instr_addr;
        pc                     <= r_stall ? stalled_pc : prev_pc;
        fetch_instr            <= r_stall ? stalled_instr : main_memory_instr;
      end
      // flush next stage (only when not stalled) so that clock-enable of next stage is disabled at next clock cycle
      if ((!stall_bit) && flush) next_clk_en <= 0;
      // next stage enable only when not stalled
      else if (!stall_bit) next_clk_en <= r_clk_en_d;
      // if this stage is not forced to be stalled but still be stall from other conditions (mem ack, no mem req), disable next stage (make a pipeline bubble)
      else if (stall_bit && (!stall)) next_clk_en <= 0;

      // raise stall when any of 5 stages is stalled
      r_stall <= stall;

      // store pc and instr before stalling the stage so we can come back to the prev values when we need to return from stall
      if (stall_bit && (!r_stall)) begin
        stalled_pc <= prev_pc;
        stalled_instr <= main_memory_instr;
      end

      // first delay to align PC to the pipeline
      prev_pc <= main_memory_instr_addr;
    end
  end
  // endregion update internal registers

  // region pc and pipeline clk enable control logic
  reg [31:0] r_main_memory_instr_addr;

  always_comb begin
    r_main_memory_instr_addr = 0;
    r_clk_en_d               = 0;
    if (writeback_change_pc) begin
      r_main_memory_instr_addr = writeback_next_pc;
      r_clk_en_d               = 0;
    end else if (execute_change_pc) begin
      r_main_memory_instr_addr = execute_next_pc;
      r_clk_en_d               = 0;
    end else begin
      r_main_memory_instr_addr = main_memory_instr_addr + 4;
      r_clk_en_d               = r_clk_en;
    end
  end
  // endregion pc and pipeline clk enable control logic

endmodule

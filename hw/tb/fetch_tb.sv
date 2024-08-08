// compile_verilog rtl/fetch.sv rtl/main_memory.sv tb/fetch_tb.sv

`timescale 1ns / 1ps
`define VCD_FILE "./vcds/fetch_tb.vcd"
`define MEMORY "./hexs/add.hex"
// `define DETAILS
`define ICARUS_SIM

module fetch_tb ();

  integer totals = 0;
  integer errors = 0;

  localparam PC_RESET = 0;

  // parameter MEMORY_HEX = "./hexs/add.hex";

  parameter MEMORY_BYTES = 1024;
  localparam ADDR_WIDTH = $clog2(MEMORY_BYTES);
  localparam MEMORY_DEPTH = MEMORY_BYTES / 4;

  initial begin
`ifdef ICARUS_SIM
    $dumpfile(`VCD_FILE);
    $dumpvars;
`endif
  end

  parameter CLK_PERIOD = 10;  // 100 MHz clk
  parameter CLK_PERIOD_HALF = CLK_PERIOD / 2;
  parameter CLK_PERIOD_QUAR = CLK_PERIOD / 4;
  always #(CLK_PERIOD_HALF) clk = !clk;

  initial begin
    clk                 <= 0;
    rst                 <= 0;
    writeback_change_pc <= 0;
    writeback_next_pc   <= 0;
    execute_change_pc   <= 0;
    execute_next_pc     <= 0;
    stall               <= 0;
    flush               <= 0;
  end

  initial begin
    // Reset
    reset(1);
    // Test fetch and change pc
    test_fetch();
    $finish;
  end

  task automatic test_fetch;
    begin
      totals = 0;
      errors = 0;
      // Test fetch and change pc
      instruction_fetch(10);
      test_execute_change_pc('h10);  // Execute change pc to 0x10
      instruction_fetch(5);
      test_writeback_change_pc('h18);  // Writeback change pc to 0x18
      instruction_fetch(MEMORY_DEPTH);
      $display(  //
          "[%-6s] Done test_fetch with %0d\/%0d errors",  //
          (errors == 0) ? "\033[92mOK\033[00m" : "\033[91mFAILED\033[00m",  //
          errors,  //
          totals  //
      );
    end
  endtask  //automatic

  task automatic instruction_fetch;
    input integer number_of_instructions;
    reg match;
    reg [31:0] instr_in_mem;
    begin
      @(negedge clk);
      repeat (number_of_instructions) begin
        @(posedge clk);
        #1;  // Wait for signals to change
        instr_in_mem = main_memory_inst.memory[pc>>2];
        match = (fetch_instr == instr_in_mem);
`ifdef DETAILS
        $write(  //
            "[%-6s] pc = %4d, instr_fetch = %8h, instr_in_mem = %8h",  //
            (match) ? "\033[92mOK\033[00m" : "\033[91mFAILED\033[00m",  //
            pc, fetch_instr, instr_in_mem  //
        );
        $write(  //
            "%s >>> [Info] stall_bit = %1b, stall = %1b, flush = %1b, next_clk_en = %1b\033[00m\n",  //
            ((!fetch_inst.stall_bit) && (!stall) && (!flush) && next_clk_en) ? "\033[00m" : "\033[95m",  //
            fetch_inst.stall_bit, stall, flush, next_clk_en  //
        );
`endif
        totals = totals + 1;
        if (!match) errors = errors + 1;
      end
    end
  endtask  //automatic

  task automatic test_execute_change_pc;
    input [31:0] next_pc;
    begin
      @(negedge clk);
      execute_change_pc <= 1;
      execute_next_pc   <= next_pc;
      @(posedge clk);
      #(CLK_PERIOD_QUAR);
      execute_change_pc <= 0;
`ifdef DETAILS
      $display(  //
          "\033[94m>>> PC changed to %1d due to Execute\033[00m",  //
          next_pc  //
      );
`endif
    end
  endtask  //automatic

  task automatic test_writeback_change_pc;
    input [31:0] next_pc;
    begin
      @(negedge clk);
      writeback_change_pc <= 1;
      writeback_next_pc   <= next_pc;
      @(posedge clk);
      #(CLK_PERIOD_QUAR);
      writeback_change_pc <= 0;
`ifdef DETAILS
      $display(  //
          "\033[94m>>> PC changed to %1d due to Writeback\033[00m",  //
          next_pc  //
      );
`endif
    end
  endtask  //automatic

  task automatic reset;
    input integer clk_period;
    begin
      @(negedge clk);
      rst <= 1;
      repeat (clk_period) @(posedge clk);
      @(negedge clk);
      rst <= 0;
`ifdef DETAILS
      $display("\033[94m>>> Reset released...\033[00m");
`endif
    end
  endtask  //automatic

  reg         clk;
  reg         rst;

  wire [31:0] main_memory_instr_addr;
  wire [31:0] main_memory_instr;
  wire        main_memory_instr_req;
  wire        main_memory_instr_ack;

  wire [31:0] fetch_instr;

  wire [31:0] pc;

  reg         writeback_change_pc;
  reg  [31:0] writeback_next_pc;

  reg         execute_change_pc;
  reg  [31:0] execute_next_pc;

  reg         stall;
  reg         flush;
  wire        next_clk_en;


  fetch #(
      .PC_RESET(PC_RESET)
  ) fetch_inst (
      .clk(clk),
      .rst(rst),

      .main_memory_instr_addr(main_memory_instr_addr),
      .main_memory_instr     (main_memory_instr),
      .main_memory_instr_req (main_memory_instr_req),
      .main_memory_instr_ack (main_memory_instr_ack),

      .fetch_instr(fetch_instr),

      .pc(pc),

      .writeback_change_pc(writeback_change_pc),
      .writeback_next_pc  (writeback_next_pc),

      .execute_change_pc(execute_change_pc),
      .execute_next_pc  (execute_next_pc),

      .stall      (stall),
      .flush      (flush),
      .next_clk_en(next_clk_en)
  );


  reg         wb_cyc = 0;
  reg         wb_stb = 0;
  reg         wb_wr_en = 0;
  reg  [31:0] wb_addr = 0;
  reg  [31:0] wb_wr_data = 0;
  reg  [ 3:0] wb_wr_sel = 0;
  wire        wb_ack;
  wire        wb_stall;
  wire [31:0] wb_rd_data;

  main_memory #(
      .MEMORY_HEX  (`MEMORY),
      .MEMORY_BYTES(MEMORY_BYTES)
  ) main_memory_inst (
      .clk(clk),

      .instr_addr(main_memory_instr_addr),
      .instr     (main_memory_instr),
      .instr_stb (main_memory_instr_req),
      .instr_ack (main_memory_instr_ack),

      .wb_cyc    (wb_cyc),
      .wb_stb    (wb_stb),
      .wb_wr_en  (wb_wr_en),
      .wb_addr   (wb_addr),
      .wb_wr_data(wb_wr_data),
      .wb_wr_sel (wb_wr_sel),
      .wb_ack    (wb_ack),
      .wb_stall  (wb_stall),
      .wb_rd_data(wb_rd_data)
  );
endmodule

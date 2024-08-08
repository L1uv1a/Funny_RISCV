// compile_verilog rtl/main_memory.sv tb/main_memory_tb.sv
// compile_verilog rtl/main_memory.sv tb/main_memory_tb.sv -DDETAILS

`timescale 1ns / 1ps
`define VCD_FILE "./vcds/main_memory_tb.vcd"

// `define DETAILS

module main_memory_tb ();

  integer totals = 0;
  integer fails = 0;

  parameter MEMORY_HEX = "./hexs/add.hex";
  parameter MEMORY_BYTES = 1024;
  localparam ADDR_WIDTH = $clog2(MEMORY_BYTES);
  localparam MEMORY_DEPTH = MEMORY_BYTES / 4;

  initial begin
    $dumpfile(`VCD_FILE);
    $dumpvars;
  end

  parameter CLK_PERIOD = 10;  // 100 MHz clk
  parameter CLK_PERIOD_HALF = CLK_PERIOD / 2;
  parameter CLK_PERIOD_QUAR = CLK_PERIOD / 4;
  always #(CLK_PERIOD_HALF) clk = !clk;

  initial begin
    clk = 0;
    instr_addr = 0;
    instr_stb = 0;
    wb_cyc = 0;
    wb_stb = 0;
    wb_wr_en = 0;
    wb_addr = 0;
    wb_wr_data = 0;
    wb_wr_sel = 0;
  end

  initial begin
    test_instruction_read();
    test_data_write_read();

    #(CLK_PERIOD * 10);
    $finish;
  end

  task automatic instruction_read;
    input [31:0] read_from_address;
    output match;
    reg [31:0] expected_instr;
    begin
      @(negedge clk);
      instr_stb  <= 1;
      instr_addr <= read_from_address;
      @(posedge clk);
      #(CLK_PERIOD_QUAR);
      expected_instr = main_memory_inst.memory[read_from_address[ADDR_WIDTH-1:2]];
      match = (instr == expected_instr);
`ifdef DETAILS
      $display(  //
          "[%-6s] Read Instruction %h from address %h, expected %h",  //
          match ? "\033[92mOK\033[00m" : "\033[91mFAILED\033[00m",  //
          instr,  //
          instr_addr,  //
          expected_instr  //
      );
`endif
      instr_stb <= 0;
    end
  endtask  //automatic

  task automatic data_read;
    input [31:0] read_from_address;
    input [31:0] expected_data;
    output match;
    begin
      @(negedge clk);
      wb_stb  <= 1;
      wb_cyc  <= 1;
      wb_addr <= read_from_address;
      @(posedge clk);
      #(CLK_PERIOD_QUAR);
      match = (wb_rd_data == expected_data);
`ifdef DETAILS
      $display(  //
          "[%-6s] Read Data %h from address %h, expected %h",  //
          (match) ? "\033[92mOK\033[00m" : "\033[91mFAILED\033[00m",  //
          wb_rd_data,  //
          wb_addr,  //
          expected_data  //
      );
`endif
      wb_stb <= 0;
      wb_cyc <= 0;
    end
  endtask  //automatic

  task automatic data_write;
    input [31:0] write_to_address;
    input [31:0] write_data;
    input [3:0] write_select;
    output match;
    begin
      @(negedge clk);
      wb_wr_en   <= 1;
      wb_stb     <= 1;
      wb_cyc     <= 1;
      wb_addr    <= write_to_address;
      wb_wr_data <= write_data;
      wb_wr_sel  <= write_select;
      @(posedge clk);
      #(CLK_PERIOD_QUAR);
      match = (write_data == main_memory_inst.memory[write_to_address>>2]);
      if (!match) fails = fails + 1;
`ifdef DETAILS
      $display(  //
          "[%-6s] Write Data %h to address %h",  //
          match ? "\033[92mOK\033[00m" : "\033[91mFAILED\033[00m",  //
          wb_wr_data,  //
          wb_addr  //
      );
`endif
      wb_wr_en <= 0;
      wb_stb   <= 0;
      wb_cyc   <= 0;
    end
  endtask  //automatic

  task automatic test_instruction_read;
    integer address;
    integer errors = 0;
    integer tests = 0;
    reg match;
    begin
      for (address = 0; address < MEMORY_DEPTH; address = address + 1) begin
        instruction_read(address << 2, match);
        if (!match) errors = errors + 1;
        tests = tests + 1;
      end
      $display(  //
          "[%-6s] Done test_instruction_read with %0d\/%0d errors",  //
          (errors == 0) ? "\033[92mOK\033[00m" : "\033[91mFAILED\033[00m",  //
          errors,  //
          tests  //
      );
    end
  endtask  //automatic

  task automatic test_data_write_read;
    integer address;
    integer errors = 0;
    integer tests = 0;
    reg [31:0] random_data;
    reg match;
    begin
      for (address = 'h10 << 2; address < MEMORY_DEPTH; address = address + 1) begin
        random_data = $urandom;
        data_write(address, random_data, 4'b1111, match);
        if (!match) errors = errors + 1;
        tests = tests + 1;
        data_read(address, random_data, match);
        if (!match) errors = errors + 1;
        tests = tests + 1;
      end
      $display(  //
          "[%-6s] Done test_data_write_read with %0d\/%0d errors",  //
          (errors == 0) ? "\033[92mOK\033[00m" : "\033[91mFAILED\033[00m",  //
          errors,  //
          tests  //
      );
    end
  endtask  //automatic

  reg         clk;

  reg  [31:0] instr_addr;
  wire [31:0] instr;
  reg         instr_stb;
  wire        instr_ack;

  reg         wb_cyc;
  reg         wb_stb;
  reg         wb_wr_en;
  reg  [31:0] wb_addr;
  reg  [31:0] wb_wr_data;
  reg  [ 3:0] wb_wr_sel;
  wire        wb_ack;
  wire        wb_stall;
  wire [31:0] wb_rd_data;

  main_memory #(
      .MEMORY_HEX  (MEMORY_HEX),
      .MEMORY_BYTES(MEMORY_BYTES)
  ) main_memory_inst (
      .clk(clk),

      .instr_addr(instr_addr),
      .instr     (instr),
      .instr_stb (instr_stb),
      .instr_ack (instr_ack),

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

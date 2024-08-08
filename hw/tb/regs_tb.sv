// compile_verilog rtl/regs.sv tb/regs_tb.sv
// compile_verilog rtl/regs.sv tb/regs_tb.sv -DDETAILS

`timescale 1ns / 1ps
`define VCD_FILE "./vcds/regs_tb.vcd"

// `define DETAILS
`define ICARUS_SIM

module regs_tb ();

  integer totals = 0;
  integer fails = 0;

  initial begin
`ifdef ICARUS_SIM
    $dumpfile(`VCD_FILE);
    $dumpvars;
`endif
  end

  initial begin
    clk = 0;
    rst = 0;
    rs_rd_en = 0;
    rs1 = 0;
    rs2 = 0;
    rd = 0;
    rd_wr_data = 0;
    rd_wr_en = 0;
  end

  initial begin
    rst <= 1;
    #(CLK_PERIOD * 5);
    rst <= 0;
    test();
    #(CLK_PERIOD * 10);
    $finish;
  end

  task automatic test;
    integer i;
    for (i = 0; i < 32; i = i + 1) begin
      totals = totals + 1;
      write_rd(i, $urandom);
    end
    for (i = 0; i < 32 / 2; i = i + 1) begin
      totals = totals + 1;
      read_rs(i * 2, i * 2 + 1);
    end
    $display(  //
        "[%1s] DONE AUTO TESTING, SCORE: %1d/%1d (FAILED: %1d)",  //
        fails ? "\033[91mFAILED\033[00m" : "\033[92mOK\033[00m",  //
        totals - fails,  //
        totals,  //
        fails  //
    );
  endtask  //automatic

  task automatic print_x;
    integer i;
    begin
      for (i = 1; i < 32; i = i + 1) begin
        $display("x[%2d] = %h", i, regs_inst.x[i]);
      end
    end
  endtask  //automatic

  task automatic read_rs;
    input [4:0] rs1_addr;
    input [4:0] rs2_addr;
    reg [31:0] expected_rs1_data;
    reg [31:0] expected_rs2_data;
    reg match_rs1;
    reg match_rs2;
    begin
      @(negedge clk);
      rs_rd_en <= 1;
      rs1      <= rs1_addr;
      rs2      <= rs2_addr;
      @(posedge clk);
      #(CLK_PERIOD_QUAR);
      expected_rs1_data = (rs1_addr == 0) ? 0 : regs_inst.x[rs1_addr];
      match_rs1 = (rs1_rd_data == expected_rs1_data);
      if (!match_rs1) fails = fails + 1;
`ifdef DETAILS
      $display(  //
          "[%-6s] Read %08h from rs1 = x[%2d], expected %08h",  //
          match_rs1 ? "\033[92mOK\033[00m" : "\033[91mFAILED\033[00m",  //
          rs1_rd_data,  //
          rs1_addr,  //
          expected_rs1_data  //
      );
`endif
      expected_rs2_data = (rs2_addr == 0) ? 0 : regs_inst.x[rs2_addr];
      match_rs2 = (rs2_rd_data == expected_rs2_data);
      if (!match_rs2) fails = fails + 1;
`ifdef DETAILS
      $display(  //
          "[%-6s] Read %08h from rs2 = x[%2d], expected %08h",  //
          match_rs2 ? "\033[92mOK\033[00m" : "\033[91mFAILED\033[00m",  //
          rs2_rd_data,  //
          rs2_addr,  //
          expected_rs2_data  //
      );
`endif
      rs_rd_en <= 0;
    end
  endtask  //automatic

  task automatic write_rd;
    input [4:0] rd_addr;
    input [31:0] rd_write_data;
    begin
      @(negedge clk);
      rd_wr_en   <= 1;
      rd         <= rd_addr;
      rd_wr_data <= rd_write_data;
      @(posedge clk);
      #(CLK_PERIOD_QUAR);
`ifdef DETAILS
      $display("[\033[92mOK\033[00m]Write %h to rd = x[%2d]", rd_write_data, rd_addr);
`endif
      rd_wr_en <= 0;
    end
  endtask  //automatic


  parameter CLK_PERIOD = 10;  // 100 MHz clk
  parameter CLK_PERIOD_HALF = CLK_PERIOD / 2;
  parameter CLK_PERIOD_QUAR = CLK_PERIOD / 4;
  always #(CLK_PERIOD_HALF) clk = !clk;

  reg         clk;
  reg         rst;

  reg         rs_rd_en;
  reg  [ 4:0] rs1;
  reg  [ 4:0] rs2;

  reg  [ 4:0] rd;
  reg  [31:0] rd_wr_data;
  reg         rd_wr_en;

  wire [31:0] rs1_rd_data;
  wire [31:0] rs2_rd_data;

  regs regs_inst (
      .clk(clk),
      .rst(rst),

      .rs_rd_en(rs_rd_en),
      .rs1     (rs1),
      .rs2     (rs2),

      .rd        (rd),
      .rd_wr_data(rd_wr_data),
      .rd_wr_en  (rd_wr_en),

      .rs1_rd_data(rs1_rd_data),
      .rs2_rd_data(rs2_rd_data)
  );

endmodule

module soc #(
    // main_memory
    parameter MEMORY_HEX   = "",
    parameter MEMORY_BYTES = 1024,
    parameter PC_RESET     = 0,
    parameter TRAP_ADDR    = 0
) (
    input clk,
    input rst
);

  // Instruction Memory
  wire [31:0] main_memory_instr_addr;
  wire [31:0] main_memory_instr;
  wire        main_memory_instr_req;
  wire        main_memory_instr_ack;

  // Data Memory
  //   wire        main_memory_wb_cyc;
  //   wire        main_memory_wb_stb;
  //   wire        main_memory_wb_wr_en;
  //   wire [31:0] main_memory_wb_addr;
  //   wire [31:0] main_memory_wb_wr_data;
  //   wire [ 3:0] main_memory_wb_wr_sel;
  //   wire        main_memory_wb_ack;
  //   wire        main_memory_wb_stall;
  //   wire [31:0] main_memory_wb_rd_data;

  // Interrupts
  wire        external_interrupt;  //! interrupt from external source
  wire        software_interrupt;  //! interrupt from software (inter-processor interrupt)
  wire        timer_interrupt;  //! interrupt from timer


  core #(
      .PC_RESET (PC_RESET),
      .TRAP_ADDR(TRAP_ADDR)
  ) core_dut (
      .clk(clk),
      .rst(rst),

      .main_memory_instr_addr(main_memory_instr_addr),
      .main_memory_instr     (main_memory_instr),
      .main_memory_instr_req (main_memory_instr_req),
      .main_memory_instr_ack (main_memory_instr_ack),

      .main_memory_wb_cyc    (memory_wrapper_wb_cyc),
      .main_memory_wb_stb    (memory_wrapper_wb_stb),
      .main_memory_wb_wr_en  (memory_wrapper_wb_wr_en),
      .main_memory_wb_addr   (memory_wrapper_wb_addr),
      .main_memory_wb_wr_data(memory_wrapper_wb_wr_data),
      .main_memory_wb_wr_sel (memory_wrapper_wb_wr_sel),
      .main_memory_wb_ack    (memory_wrapper_wb_ack),
      .main_memory_wb_stall  (memory_wrapper_wb_stall),
      .main_memory_wb_rd_data(memory_wrapper_wb_rd_data),

      // Interrupts
      .external_interrupt(external_interrupt),
      .software_interrupt(software_interrupt),
      .timer_interrupt   (timer_interrupt)
  );

  // Data Memory
  wire        memory_wrapper_wb_cyc;
  wire        memory_wrapper_wb_stb;
  wire        memory_wrapper_wb_wr_en;
  wire [31:0] memory_wrapper_wb_addr;
  wire [31:0] memory_wrapper_wb_wr_data;
  wire [ 3:0] memory_wrapper_wb_wr_sel;
  wire        memory_wrapper_wb_ack;
  wire        memory_wrapper_wb_stall;
  wire [31:0] memory_wrapper_wb_rd_data;

  memory_wrapper memory_wrapper_dut (
      .i_wb_cyc  (memory_wrapper_wb_cyc),
      .i_wb_stb  (memory_wrapper_wb_stb),
      .i_wb_we   (memory_wrapper_wb_wr_en),
      .i_wb_addr (memory_wrapper_wb_addr),
      .i_wb_data (memory_wrapper_wb_wr_data),
      .i_wb_sel  (memory_wrapper_wb_wr_sel),
      .o_wb_ack  (memory_wrapper_wb_ack),
      .o_wb_stall(memory_wrapper_wb_stall),
      .o_wb_data (memory_wrapper_wb_rd_data),

      .o_device0_wb_cyc  (main_memory_wb_cyc),
      .o_device0_wb_stb  (main_memory_wb_stb),
      .o_device0_wb_we   (main_memory_wb_wr_en),
      .o_device0_wb_addr (main_memory_wb_addr),
      .o_device0_wb_data (main_memory_wb_wr_data),
      .o_device0_wb_sel  (main_memory_wb_wr_sel),
      .i_device0_wb_ack  (main_memory_wb_ack),
      .i_device0_wb_stall(main_memory_wb_stall),
      .i_device0_wb_data (main_memory_wb_rd_data)
  );

  // Data Memory
  wire        main_memory_wb_cyc;
  wire        main_memory_wb_stb;
  wire        main_memory_wb_wr_en;
  wire [31:0] main_memory_wb_addr;
  wire [31:0] main_memory_wb_wr_data;
  wire [ 3:0] main_memory_wb_wr_sel;
  wire        main_memory_wb_ack;
  wire        main_memory_wb_stall;
  wire [31:0] main_memory_wb_rd_data;

  main_memory #(
      .MEMORY_HEX  (MEMORY_HEX),
      .MEMORY_BYTES(MEMORY_BYTES)
  ) main_memory_dut (
      .clk(clk),

      .instr_addr(main_memory_instr_addr),
      .instr     (main_memory_instr),
      .instr_stb (main_memory_instr_req),
      .instr_ack (main_memory_instr_ack),

      .wb_cyc    (main_memory_wb_cyc),
      .wb_stb    (main_memory_wb_stb),
      .wb_wr_en  (main_memory_wb_wr_en),
      .wb_addr   (main_memory_wb_addr),
      .wb_wr_data(main_memory_wb_wr_data),
      .wb_wr_sel (main_memory_wb_wr_sel),
      .wb_ack    (main_memory_wb_ack),
      .wb_stall  (main_memory_wb_stall),
      .wb_rd_data(main_memory_wb_rd_data)
  );




endmodule

module core #(
    parameter PC_RESET = 0,
    parameter TRAP_ADDR = 0,
    parameter ZICSR_EXTENSION = 1
) (
    input clk,
    input rst,

    // region main_memory
    // Instruction Memory
    output [31:0] instr_addr,
    input  [31:0] instr,
    output        instr_stb,   // request for instruction
    input         instr_ack,   // ack

    // Data Memory
    output        wb_cyc,
    output        wb_stb,
    output        wb_we,
    output [31:0] wb_addr,
    output [31:0] wb_wr_data,
    output [ 3:0] wb_sel,
    input         wb_ack,
    input         wb_stall,
    input  [31:0] wb_rd_data,
    // endregion main_memory

    // Interrputs
    input external_interrupt,
    input software_interrupt,
    input timer_interrupt
);

  // region regs
  // region control by [STAGE 2 DECODE]
  wire        rs_rd_en;  // source registers read enable
  wire [ 4:0] rs1;  // source register 1 address
  wire [ 4:0] rs2;  // source register 2 address
  // endregion control by [STAGE 2 DECODE]

  // region control by [STAGE 5 WRITEBACK]
  wire [ 4:0] rd;  // destination register address
  wire [31:0] rd_wr_data;  // data to be written to destination register
  wire        rd_wr_en;  // destination register write enable
  // endregion control by [STAGE 5 WRITEBACK]

  wire [31:0] rs1_rd_data;  // source register 1 value
  wire [31:0] rs2_rd_data;  // source register 2 value


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
  // endregion regs


endmodule

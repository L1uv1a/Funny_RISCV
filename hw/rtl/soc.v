module soc #(
    // main_memory
    parameter MEMORY_HEX   = "",
    parameter MEMORY_DEPTH = 1024
) (
    input clk,
    input rst
);

  // region main_memory
  // region control by [STAGE 1 FETCH]
  // Instruction Memory
  wire [31:0] instr_addr;
  wire [31:0] instr;
  wire        instr_stb;
  wire        instr_ack;
  // endregion control by [STAGE 1 FETCH]

  // region control by [STAGE 4 MEMORY]
  // Data Memory

  // bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
  wire        wb_cyc;

  wire        wb_stb;  // request for read/write access to data memory
  wire        wb_wr_en;  // write-enable (1 = write, 0 = read)
  wire [31:0] wb_addr;  // address of data memory for store/load
  wire [31:0] wb_wr_data;  // data to be stored to memory

  // byte strobe for write (1 = write the byte) {byte3, byte2, byte1, byte0}
  wire [ 3:0] wb_wr_sel;

  // ack by data memory (high when data to be read is ready or when write data is already written)
  wire        wb_ack;

  wire        wb_stall;  // stall by data memory
  wire [31:0] wb_rd_data;  // data retrieved from memory
  // endregion control by [STAGE 4 MEMORY]

  main_memory #(
      .MEMORY_HEX  (MEMORY_HEX),
      .MEMORY_DEPTH(MEMORY_DEPTH)
  ) main_memory_inst (
      .clk(clk),

      // Instruction Memory
      .instr_addr(instr_addr),
      .instr     (instr),
      .instr_stb (instr_stb),
      .instr_ack (instr_ack),

      // Data Memory
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
  // endregion main_memory

endmodule

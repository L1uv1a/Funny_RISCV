`include "rv32i_header.svh"

/* regfile controller for the 32 integer base registers
*/

module regs (
  input wire         clk,

  input wire         clk_en_read,  // clock enable for reading from registers [Stage 2] (Decode)

  input wire  [4:0]  rs1,          // register source 1
  input wire  [4:0]  rs2,          // register source 2

  input wire  [4:0]  rd,           // register destination
  input wire  [31:0] rd_wdata,     // data to be written to rd
  input wire         w_en,         // write enable

  output wire [31:0] rs1_rdata,    // read data from rs1
  output wire [31:0] rs2_rdata     // read data from rs2
);

  reg [4:0] r_rs1;      // registered rs1
  reg [4:0] r_rs2;      // registered rs2

  reg [31:0] x [31:1];  // regs from x1 to x31

  // only need to write if not try to write to x0 (zero)
  wire need_write  = w_en && (rd != `ZERO_REG_ADDR);

  // if reg is write and read at the same time, return write data as read data
  assign rs1_rdata = (r_rs1 == `ZERO_REG_ADDR) ? 0 : x[r_rs1];  // if read x0, return 0
  assign rs2_rdata = (r_rs2 == `ZERO_REG_ADDR) ? 0 : x[r_rs2];  // if read x0, return 0

  // region write to x
  // syn write with clk and only write if [Stage 5] (Writeback) is preivously enable.
  always @(posedge clk) begin : write_x
    // [Info] output of [Stage 5] (Writeback) is registered so delayed by 1 clk
    if (need_write) 
      x[rd]        <= rd_wdata;
  end
  // endregion write to x

  // region read from x
  // syn read with clk
  always @(posedge clk) begin : read_x
    if (clk_en_read) begin
      // [Info] only read the register if [Stage 2] (Decode) is enabled
      r_rs1        <= rs1;  // update r_rs1 so the rs1_rdata = x[r_rs1] will also be updated
      r_rs2        <= rs2;  // update r_rs2 so the rs2_rdata = x[r_rs2] will also be updated
    end
  end
  // endregion read from x

endmodule


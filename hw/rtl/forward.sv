`include "rv32i_header.vh"

module forward (
    //! current rs1 datas read from regs
    input [31:0] regs_rs1_rd_data,
    //! current rs2 datas read from regs
    input [31:0] regs_rs2_rd_data,

    //! rs1 addresses decoded from decode module [Decode]
    input [4:0] decode_r_rs1,
    //! rs2 addresses decoded from decode module [Decode]
    input [4:0] decode_r_rs2,

    //! force execute module [Execute] to be stalled
    output reg force_stall_execute,

    //! Forwarding rs1 data to [Execute]
    output reg [31:0] forward_rs1_data,
    //! Forwarding rs2 data to [Execute]
    output reg [31:0] forward_rs2_data,

    // [Execute/Memory] forwarding
    input [ 4:0] execute_rd,          //! rd address
    input        execute_rd_wr_en,    //! high if want to write to rd address
    input [31:0] execute_rd_wr_data,  //! value to be written back to destination register
    input        execute_rd_valid,    //! high if rd is already valid (not LOAD nor CSR instructions)
    input        memory_clk_en,       //! memory_clk_en

    // [Memory/Writeback] forwarding
    input [ 4:0] memory_rd,             //! rd address
    input        memory_rd_wr_en,       //! high if want to write to rd address
    input [31:0] writeback_rd_wr_data,  //! value to be written back to destination register
    input        writeback_clk_en       //! memory_clk_en
);

  wire next_rs1_is_on_execute_memory = (  //
 (decode_r_rs1 == execute_rd) &&  // decoded instruction need data from rs1
 execute_rd_wr_en && memory_clk_en  // execute need to write to rd and memory enabled
);
  wire next_rs2_is_on_execute_memory = (  //
 (decode_r_rs2 == execute_rd) &&  // decoded instruction need data from rs2
 execute_rd_wr_en && memory_clk_en  // execute need to write to rd and memory enabled
);

  wire next_rs1_is_on_memory_writeback = (  //
 (decode_r_rs1 == memory_rd) &&  //
 memory_rd_wr_en && writeback_clk_en  //
);

  wire next_rs2_is_on_memory_writeback = (  //
 (decode_r_rs2 == memory_rd) &&  //
 memory_rd_wr_en && writeback_clk_en  //
);

  //! Data hazard:
  //! Register value is about to be overwritten by previous instructions but
  //! are still on the pipeline and are not yet written to base registers (x).
  //! To avoid this, make sure the updated value of rs1 or rs2 is used by
  //! either stall the pipeline until the base registers is updated or use
  //! operand forwarding.
  always_comb begin : forward_and_stall
    forward_rs1_data    = regs_rs1_rd_data;
    forward_rs2_data    = regs_rs2_rd_data;
    force_stall_execute = 0;

    // Operand forwarding for rs1
    // if need rs1 data from execute/memory
    if (next_rs1_is_on_execute_memory) begin : forwarding_rs1_from_execute_memory
      // forward execute/memory data
      forward_rs1_data = execute_rd_wr_data;
      /* if next stage value of rs1 comes from Load or CSR instruction then
      we must stall from [Execute] stage and wait until stage [Memory] becomes disabled,
      which means next value of rs1 is already at stage [Writeback] */
      if (!execute_rd_valid) force_stall_execute = 1;
    end else if (next_rs1_is_on_memory_writeback) begin : forwarding_rs1_from_memory_writeback
      forward_rs1_data = writeback_rd_wr_data;
    end

    // Operand forwarding for rs2
    // if need rs1 data from execute/memory
    if (next_rs2_is_on_execute_memory) begin : forwarding_rs2_from_execute_memory
      // forward execute/memory data
      forward_rs2_data = execute_rd_wr_data;
      /* if next stage value of rs1 comes from Load or CSR instruction then
        we must stall from [Execute] stage and wait until stage [Memory] becomes disabled,
        which means next value of rs1 is already at stage [Writeback] */
      if (!execute_rd_valid) force_stall_execute = 1;
    end else if (next_rs2_is_on_memory_writeback) begin : forwarding_rs2_from_memory_writeback
      forward_rs2_data = writeback_rd_wr_data;
    end

    // No operand forward needed when rs1, rs2 address = 0 (try to read from Zero reg is pointless)
    if (decode_r_rs1 == `ZERO_REG_ADDR) forward_rs1_data = `ZERO_REG_DATA;
    if (decode_r_rs2 == `ZERO_REG_ADDR) forward_rs1_data = `ZERO_REG_DATA;
  end

endmodule

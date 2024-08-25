`include "rv32i_header.vh"
`include "decode_header.vh"

module writeback (
    input [              2:0] memory_funct3,      //! funct3 (byte, halfword, word store/load operation)asmndmabsmdmnasbdmasmdmamnsdmnasmndmnanmsdmnaabsdmnbamsdbmabsmndban
    input [             31:0] memory_data_load,   //! data to be loaded to base reg (z-or-s extended)
    input [`OPCODE_WIDTH-1:0] memory_opcode_type, //! opcode type from previous stage (memory). Extract load or system opcode type to determine if this stage need to handle the instruction

    // region CRS Register operation
    input [31:0] csr_data,  //! CSR data to be loaded to base reg (ZICSR extension)
    // endregion CRS Register operation

    // region Base reg control
    input             memory_rd_wr_en,      //! enable write rd from previous stage (memory)
    output reg        writeback_rd_wr_en,   //! write rd to the base reg if enabled
    input      [ 4:0] memory_rd,            //! rd write address from previous stage (memory)
    output reg [ 4:0] writeback_rd,         //! address for destination register
    input      [31:0] memory_rd_wr_data,    //! rd write data from previous stage (memory)
    output reg [31:0] writeback_rd_wr_data, //! data to be written back to destination register
    // endregion Base reg control

    // region PC control
    input      [31:0] memory_pc,           //! pc value from previous stage (memory)
    output reg [31:0] writeback_pc,        //! new pc value
    output reg        writeback_change_pc, //! PC need to jump due to writeback
    // endregion PC control

    // region Trap handler
    input wire        go_to_trap,        // trap (exception/interrupt detected)
    input wire        return_from_trap,  // high before returning from trap (via mret)
    input wire [31:0] return_addr,       // mepc CSR
    input wire [31:0] trap_addr,         // mtvec CSR
    // endregion Trap handler


    // region Pipeline control
    input      clk_en,      //! control by previous stage (memory)
    output reg next_stall,  //! stalls the pipeline
    output reg next_flush   //! flushes previous stages
    // endregion Pipeline control
);

  wire opcode_load = memory_opcode_type[`LOAD];  //! load type
  wire opcode_system = memory_opcode_type[`SYSTEM];  //! system type

  //! handling writeback
  always_comb begin : writeback_process
    next_stall           = 0;
    next_flush           = 0;
    writeback_rd_wr_en   = (clk_en && memory_rd_wr_en && (!next_stall));  // default rd write enable when this stage is enable, previous stage raise rd write signal and the pipeline is not being stalled
    writeback_rd         = memory_rd;
    writeback_rd_wr_data = 0;
    writeback_pc         = memory_pc;
    writeback_change_pc  = 0;

    // Go to trap
    if (go_to_trap) begin
      writeback_change_pc = 1;
      writeback_pc        = trap_addr;  // interrupt or exception detected so go to trap address (mtvec value)
      next_flush          = clk_en;
      writeback_rd_wr_en  = 0;
      // Return from trap
    end else if (return_from_trap) begin
      writeback_change_pc = 1;
      writeback_pc        = return_addr;  // return from trap via mret (mepc value)
      next_flush          = clk_en;
      writeback_rd_wr_en  = 0;
      // Else
    end else begin
      // handling operations need writeback
      if (opcode_load) writeback_rd_wr_data = memory_data_load;  // load data from memory to base reg
      else if (opcode_system && (memory_funct3 != 0)) writeback_rd_wr_data = csr_data;  // load csr data to base reg
      else writeback_rd_wr_data = memory_rd_wr_data;  // rd value computed at execute stage
    end
  end



endmodule

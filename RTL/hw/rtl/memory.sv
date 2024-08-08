/* The memory module: memory access stage of the 5 stages pipelined risc-v softcore processor.
This module handles:
- load and store instructions
- passing necessary signals to next stage and to the whole pipeline
- handling pipeline stalls and flushes when required.
Module functionalities:
- Generating data to be stored and its mask (which byte(s) to be stored)
- Generating the right data loaded from memory coresponding to byte/half/word accesses

Details:
- Address and data, mask handling for store operations:
  + uses address that caculated from execute stage (execute_result) to passing it as the appropriate data memory address we want to store
  the data into (main_memory_wb_addr)
  + uses data contained in (execute_rs2_data) to generate the data to be stored into the memory (wr_data) based on byte/half/word operations
  and passing it to memory using (main_memory_wb_wr_data)
  + based on byte/half/word operations (execute_funct3) and the store address (execute_result)'s last few bits, also generate mask (wr_sel)
  for which byte(s) of the data to be stored into memory (main_memory_wb_wr_sel). The mask is used to control which part of the data memory
  will be written during store operation.
*/

`include "rv32i_header.vh"

module memory (
    input clk,
    input rst,

    input [31:0] execute_rs2_data,  //! Data to be stored to memory is always rs2 data
    input [31:0] execute_result,    //! Result from execute (load/store address)

    input      [2:0] execute_funct3,  //! funct3 from previous stage [STAGE 3 EXECUTE]
    output reg [2:0] memory_funct3,   //! funct3 (byte, halfword, word store/load operation)

    input      [`OPCODE_WIDTH-1:0] execute_opcode_type,  //! opcode type from prev [STAGE 3 EXECUTE]
    output reg [`OPCODE_WIDTH-1:0] memory_opcode_type,   //! opcode type

    input      [31:0] execute_pc,  //! pc from previous stage (execute)
    output reg [31:0] memory_pc,   //! registered pc for this stage (memory)

    // region Base Registers control
    input             execute_rd_wr_en,    //! enable write rd from previous stage (execute)
    output reg        memory_rd_wr_en,     //! write rd to the base reg if enabled
    input      [ 4:0] execute_rd,          //! rd write address from previous stage (execute)
    output reg [ 4:0] memory_rd,           //! address for destination register
    input      [31:0] execute_rd_wr_data,  //! rd write data from previous stage (execute)
    output reg [31:0] memory_rd_wr_data,   //! data to be written back to destination register
    // endregion Base Registers control

    // region Data Memory control
    //! bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
    output reg        main_memory_wb_cyc,
    output reg        main_memory_wb_stb,      //! request for read/write access to data memory
    output reg        main_memory_wb_wr_en,    //! write-enable (1 = write, 0 = read)
    output reg [31:0] main_memory_wb_addr,     //! data memory address
    output reg [31:0] main_memory_wb_wr_data,  //! data to be stored to memory
    //! byte select for write {byte3, byte2, byte1, byte0}
    output reg [ 3:0] main_memory_wb_wr_sel,
    //! ack by data memory (high when data to be read is ready or when write data is already written)
    input             main_memory_wb_ack,
    input             main_memory_wb_stall,    //! stall by data memory (1 = data memory is busy)
    input      [31:0] main_memory_wb_rd_data,  //! data retrieve from data memory
    // endregion Data Memory control

    output reg [31:0] memory_data_load,  //! data to be loaded to base reg (z-or-s extended)

    // region Pipeline control
    input      stall_from_execute,  //! Execute tell to prepare to stall for load/store instruction
    input      clk_en,              //! control by previous stage [STAGE 3 EXECUTE]
    output reg next_clk_en,         //! clk enable for pipeline stalling
    input      stall,               //! stall this stage
    output reg next_stall,          //! stalls the pipeline
    input      flush,               //! flush this stage
    output reg next_flush           //! flushes previous stages
    // endregion Pipeline control
);

  reg  [31:0] wr_data;  //! data to be stored to memory
  reg  [31:0] rd_data;  //! data to be loaded to basereg
  reg  [ 3:0] wr_sel;  //! byte mask
  reg         pend_req;  //! is there still a pending request (not yet ack req)

  wire [ 1:0] which_byte = execute_result[1:0];  //! last 2 bits of data memory address => which byte to load (0, 1, 2, 3)
  wire        which_half = execute_result[1];  //! 2 bits from lsb of data memory address => which half to load (0, 1)

  wire        stall_bit = (stall || next_stall);  //! stall bit for this stage

  // region registered the outputs
  //! registed signals for this stage
  always @(posedge clk, posedge rst) begin : registered_signals
    if (rst) begin
      memory_rd_wr_en      <= 0;

      main_memory_wb_cyc   <= 0;
      main_memory_wb_stb   <= 0;
      main_memory_wb_wr_en <= 0;

      next_clk_en          <= 0;

      pend_req             <= 0;
    end else begin
      // wishbone cycle will only be high if this stage is enabled
      main_memory_wb_cyc <= clk_en;

      // pending request completed after ack
      if (main_memory_wb_ack) pend_req <= 0;  // no longer pending

      // update registers only if this stage is enabled and not stalled (after load/store operation)
      if (clk_en && (!stall_bit)) begin
        memory_funct3      <= execute_funct3;
        memory_opcode_type <= execute_opcode_type;
        memory_pc          <= execute_pc;

        memory_rd_wr_en    <= execute_rd_wr_en;
        memory_rd          <= execute_rd;
        memory_rd_wr_data  <= execute_rd_wr_data;

        memory_data_load   <= rd_data;
      end

      // update request to memory only if this stage is enabled and no pending request yet
      if (clk_en && (!pend_req)) begin
        main_memory_wb_stb     <= (execute_opcode_type[`LOAD] || execute_opcode_type[`STORE]);
        main_memory_wb_wr_en   <= execute_opcode_type[`STORE];
        main_memory_wb_wr_sel  <= wr_sel;
        main_memory_wb_addr    <= execute_result;
        main_memory_wb_wr_data <= wr_data;
        pend_req               <= (execute_opcode_type[`LOAD] || execute_opcode_type[`STORE]);
      end

      // if there is pending request but no stall from memory: idle the stb line
      if (pend_req && (!main_memory_wb_stall)) main_memory_wb_stb <= 0;

      // if this stage is disabled, idle the stb line
      if (!clk_en) main_memory_wb_stb <= 0;

      // disable next stage if this stage is not stalled but flushed.
      if ((!stall_bit) && flush) next_clk_en <= 0;
      // change based on prev clk en only if not stalled this stage
      else if (!stall_bit) next_clk_en <= clk_en;
      // If this stage is not stalled but next stage is, disable next stage
      else if ((!stall) && next_stall) next_clk_en <= 0;
    end
  end
  // endregion registered the outputs

  wire [ 7:0] rd_data_byte0 = main_memory_wb_rd_data[(0+1)*8-1:0*8];  // byte 0
  wire [ 7:0] rd_data_byte1 = main_memory_wb_rd_data[(1+1)*8-1:1*8];  // byte 1
  wire [ 7:0] rd_data_byte2 = main_memory_wb_rd_data[(2+1)*8-1:2*8];  // byte 2
  wire [ 7:0] rd_data_byte3 = main_memory_wb_rd_data[(3+1)*8-1:3*8];  // byte 3

  wire [15:0] rd_data_half0 = main_memory_wb_rd_data[(0+1)*16-1:0*16];  // half 0
  wire [15:0] rd_data_half1 = main_memory_wb_rd_data[(1+1)*16-1:1*16];  // half 1

  wire [31:0] rd_data_word = main_memory_wb_rd_data;  // word

  //! determine data to be loaded to base regs or stored to data memory
  always @* begin : write_load_data_processing
    // stall while data memory has not yet acknowledged i.e.write data is not yet written or
    // read data is not yet available (no ack yet). Don't stall when need to flush by next stage
    next_stall = (stall_from_execute && clk_en && (!main_memory_wb_ack) || stall) && (!flush);
    next_flush = flush;
    wr_data    = 0;
    rd_data    = 0;
    wr_sel     = 0;

    case (execute_funct3)
      // LOAD STORE BYTE SIGNED
      `FUNCT3_LOAD_STORE_BYTE: begin : load_store_byte
        // LOAD SIGNED
        case (which_byte)  // Choose which of the 4 byte will be loaded to base regs
          2'd0:
          rd_data = {
            {`BYTE_SIGN_EXPAND_BITS{rd_data_byte0[`BYTE_SIGN_BIT]}},  // sign expand
            rd_data_byte0  // byte data
          };
          2'd1:
          rd_data = {
            {`BYTE_SIGN_EXPAND_BITS{rd_data_byte1[`BYTE_SIGN_BIT]}},  // sign expand
            rd_data_byte1  // byte data
          };
          2'd2:
          rd_data = {
            {`BYTE_SIGN_EXPAND_BITS{rd_data_byte2[`BYTE_SIGN_BIT]}},  // sign expand
            rd_data_byte2  // byte data
          };
          2'd3:
          rd_data = {
            {`BYTE_SIGN_EXPAND_BITS{rd_data_byte3[`BYTE_SIGN_BIT]}},  // sign expand
            rd_data_byte3  // byte data
          };
          default: rd_data = 0;  // error happend, load dummy data
        endcase

        // STORE
        wr_sel  = (4'b0001 << which_byte);  // mask 1 of the 4 byte
        wr_data = (execute_rs2_data << {which_byte, 3'b000});  // align data to mask
      end

      // LOAD STORE HALF SIGNED
      `FUNCT3_LOAD_STORE_HALF: begin : load_store_half
        // LOAD SIGNED
        case (which_half)  // choose which of the 2 half will be loaded to base register
          1'd0:
          rd_data = {
            {`HALF_SIGN_EXPAND_BITS{rd_data_half0[`HALF_SIGN_BIT]}},  // sign expand
            rd_data_half0  // half data
          };
          1'd1:
          rd_data = {
            {`HALF_SIGN_EXPAND_BITS{rd_data_half1[`HALF_SIGN_BIT]}},  // sign expand
            rd_data_half1  // half data
          };
          default: rd_data = 0;  // error happend, load dummy data
        endcase

        // STORE
        wr_sel  = ({4'b0011 << {which_half, 1'b0}});  // mask 1 of the 2 halfs
        wr_data = (execute_rs2_data << {which_half, 4'b0000});  // align data to mask
      end

      // LOAD SIGNED/UNSIGNED (all the same)
      `FUNCT3_LOAD_STORE_WORD: begin : load_store_word
        rd_data = rd_data_word;

        // STORE
        wr_sel  = 4'b1111;  // mask word
        wr_data = execute_rs2_data;  // no need to align, store as a word
      end

      // LOAD BYTE UNSIGNED
      `FUNCT3_LOAD_BYTE_U: begin : load_byte_unsigned
        // LOAD UNSIGNED
        case (which_byte)  // Choose which of the 4 byte will be loaded to base regs
          2'd0:
          rd_data = {
            {`BYTE_SIGN_EXPAND_BITS{1'b0}},  // unsign expand
            rd_data_byte0  // byte data
          };
          2'd1:
          rd_data = {
            {`BYTE_SIGN_EXPAND_BITS{1'b0}},  // unsign expand
            rd_data_byte1  // byte data
          };
          2'd2:
          rd_data = {
            {`BYTE_SIGN_EXPAND_BITS{1'b0}},  // unsign expand
            rd_data_byte2  // byte data
          };
          2'd3:
          rd_data = {
            {`BYTE_SIGN_EXPAND_BITS{1'b0}},  // unsign expand
            rd_data_byte3  // byte data
          };
          default: rd_data = 0;  // error happend, load dummy data
        endcase
      end

      // LOAD HALF UNSIGNED
      `FUNCT3_LOAD_HALF_U: begin : load_half_unsigned
        // LOAD UNSIGNED
        case (which_half)  // choose which of the 2 half will be loaded to base register
          1'd0:
          rd_data = {
            {`HALF_SIGN_EXPAND_BITS{1'b0}},  // unsign expand
            rd_data_half0  // half data
          };
          1'd1:
          rd_data = {
            {`HALF_SIGN_EXPAND_BITS{1'b0}},  // unsign expand
            rd_data_half1  // half data
          };
          default: rd_data = 0;  // error happend, load dummy data
        endcase
      end

      // Default case
      default: begin
        wr_sel  = 0;
        wr_data = 0;
        rd_data = 0;
      end
    endcase
  end

endmodule

/* The mem module serves as the memory access stage of the 
 pipelined processor. This module primarily handles data memory access for
 load and store instructions, as well as passing the necessary information 
 to subsequent pipeline stas. The module is responsible for generating 
 appropriate data memory addresses, data to be stored, and write masks for
 different load/store operations, as well as handling pipeline stalls and 
 flushes when required. Key functionalities of the mem module
 include:
 - Address and data handling for load/store operations: The module uses the 
 incoming address (alu_result) to generate the appropriate data memory address (wb_addr_data)
 and stores it in the o_data_store register. It also selects the correct byte, 
 halfword, or word data from the data memory input (wb_rdata_data) based on the 
 instruction's funct3 field and stores it in the data_load register. The 
 write mask (wb_sel_data) is generated based on the address and the size of the 
 operation (byte, halfword, or word). The mask is used to control which part
 of the data memory will be written during store operations.
 - Register writeback control: The module i_wb_stall_datadetermines whether a destination register
 should be written (rd_w_en) based on the input prev_rd_w_en signal. It passes the 
 destination register address (o_rd_addr) and the data to be written (rd_wdata) to the 
 next stage.
 - Data memory control: The module controls the data memory read/write requests by
 generating the o_stb_data signal, which indicates a request for data memory access.
It also generates the wb_w_r_en_data signal, which indicates whether a write operation 
should be performed on the data memory.
 - Pipeline control: The module can stall the pipeline by asserting the stall signal
 if the data memory access is not yet acknowledged (wb_ack_data) or if there is a stall
 request from the ALU stage (stall_from_alu). It can also flush the current stage and
 previous stages using the flush signal based on the input prev_flush signal. The module 
 controls the clock enable signals (clk_en) for the next stage based on the stall and flush 
 conditions.
*/

`include "../rv32i_header.vh"

module mem (
    input wire clk,
    input wire rstn,

    input wire [31:0] rs2_wdata,  // data to be stored to memory is always rs2_wdata
    input wire [31:0] alu_result, // result from ALU (mem address to load/store)

    input  wire [2:0] prev_funct3,  //funct3 from previous stage
    output reg  [2:0] funct3,       //funct3 (byte,halfword,word)

    input  wire [`OPCODE_WIDTH-1:0] prev_opcode_type,  // opcode type from previous stage
    output reg  [`OPCODE_WIDTH-1:0] opcode_type,       //opcode type

    input  wire [31:0] prev_pc,  //PC from previous stage
    output reg  [31:0] pc,       //PC value

    // Basereg Control
    input  wire        prev_rd_w_en,   // write rd to base reg is enabled (from memoryaccess stage)
    output reg         rd_w_en,        // write rd to the base reg if enabled
    input  wire [ 4:0] prev_rd,        // address for destination register (from previous stage)
    output reg  [ 4:0] rd,             // address for destination register
    input  wire [31:0] prev_rd_wdata,  // value to be written back to destination reg
    output reg  [31:0] rd_wdata,       // value to be written back to destination register

    // Data Memory Control
    //bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
    output reg         wb_bus_cyc_data,
    output reg         wb_stb_data,      //request for read/write access to data memory
    output reg         wb_w_r_en_data,   //write-enable (1 = write, 0 = read)
    output reg  [31:0] wb_addr_data,     //data memory address
    output reg  [31:0] wb_wdata_data,    //data to be stored to memory
    output reg  [ 3:0] wb_sel_data,      //byte select for write {byte3, byte2, byte1, byte0}
    //ack by data memory (high when data to be read is ready or when write data is already written)
    input  wire        wb_ack_data,
    input  wire        wb_stall_data,    //stall by data memory (1 = data memory is busy)
    input  wire [31:0] wb_rdata_data,    //data retrieve from data memory
    output reg  [31:0] data_load,        //data to be loaded to base reg (z-or-s extended)

    /// Pipeline Control ///
    input  wire stall_from_alu,  //stalls this stage when incoming instruction is a load/store
    input  wire prev_clk_en,     // input clk enable for pipeline stalling of this stage
    output reg  clk_en,          // output clk enable for pipeline stalling of next stage
    input  wire prev_stall,      //informs this stage to stall
    output reg  stall,           //informs pipeline to stall
    input  wire prev_flush,      //flush this stage
    output reg  flush            //flush previous stages
);

  reg [31:0] data_store_d;  // data to be stored to memory
  reg [31:0] data_load_d;  // data to be loaded to basereg
  reg [3:0] wr_mask_d;
  reg pending_request;  // high if there is still a pending request (not yet acknowledged request)
  wire [1:0] addr_2 = alu_result[1:0];  //last 2  bits of data memory address

  wire stall_bit = (prev_stall || stall);

   wire [ 7:0] wb_data_data_byte0 = wb_rdata_data[(0+1)*8-1:0*8];  // byte0
  wire [ 7:0] wb_data_data_byte1 = wb_rdata_data[(1+1)*8-1:1*8];  // byte1
  wire [ 7:0] wb_data_data_byte2 = wb_rdata_data[(2+1)*8-1:2*8];  // byte2
  wire [ 7:0] wb_data_data_byte3 = wb_rdata_data[(3+1)*8-1:3*8];  // byte3

  wire [15:0] wb_data_data_half0 = wb_rdata_data[(0+1)*16-1:0*16];  // half0
  wire [15:0] wb_data_data_half1 = wb_rdata_data[(1+1)*16-1:1*16];  // half1

  wire [31:0] wb_data_data_word = wb_rdata_data;  // word

  //register the outputs of this module
  always @(posedge clk, negedge rstn) begin
    if (!rstn) begin
      rd_w_en         <= 0;
      wb_w_r_en_data  <= 0;
      clk_en          <= 0;
      wb_stb_data     <= 0;
      pending_request <= 0;
      wb_bus_cyc_data <= 0;
    end else begin
      // wishbone cycle will only be high if this stage is enabled
      wb_bus_cyc_data <= prev_clk_en;
      // request completed after ack
      if (wb_ack_data) begin
        pending_request <= 0;  // not pending any more
      end

      // update register only if this stage is enabled and not stalled (after load/store operation)
      if (prev_clk_en && !stall_bit) begin
        rd          <= prev_rd;
        funct3      <= prev_funct3;
        opcode_type <= prev_opcode_type;
        pc          <= prev_pc;
        rd_w_en     <= prev_rd_w_en;
        rd_wdata    <= prev_rd_wdata;
        data_load   <= data_load_d;
      end

      // update request to memory when no pending request yet
      if ((!pending_request) && prev_clk_en) begin
        // stb goes high when instruction is a load/store and when
        // request is not already high (request lasts for 1 clk cycle only)
        wb_stb_data     <= prev_opcode_type[`LOAD] || prev_opcode_type[`STORE];
        wb_sel_data     <= wr_mask_d;
        wb_w_r_en_data  <= prev_opcode_type[`STORE];
        pending_request <= prev_opcode_type[`LOAD] || prev_opcode_type[`STORE];
        wb_addr_data    <= alu_result;
        wb_wdata_data   <= data_store_d;
      end

      // if there is pending request but no stall from memory: idle the stb line
      if (pending_request && !wb_stall_data) wb_stb_data <= 0;


      if (!prev_clk_en) wb_stb_data <= 0;


      // flush this stage so clock-enable of next stage is disabled at next clock cycle
      if ((!stall_bit) && prev_flush) clk_en <= 0;
      // clock-enable will change only when not stalled
      else if (!stall_bit) clk_en <= prev_clk_en;
      // if this stage is stalled but this stage is not directly stalled, disable clock enable of next stage at next clock cycle (pipeline bubble)
      // else if (stall_bit && (!prev_stall)) clk_en <= 0;
      else if (stall && (!prev_stall)) clk_en <= 0;
    end

  end



  //determine data to be loaded to basereg or stored to data memory
  always @* begin
    //stall while data memory has not yet acknowledged i.e.write data is not yet written or
    //read data is not yet available (no ack yet). Don't stall when need to flush by next stage
    stall        = ((stall_from_alu && prev_clk_en && !wb_ack_data) || prev_stall) && !prev_flush;
    flush        = prev_flush;  //flush this stage along with previous stages
    data_store_d = 0;
    data_load_d  = 0;
    wr_mask_d    = 0;

    case (prev_funct3)
      `FUNCT3_LOAD_STORE_BYTE: begin : load_store_byte
        // LOAD SIGNED
        case (addr_2)  // choose which of the 4 byte will be loaded to base register
          2'b00:
          data_load_d = {
            {`BYTE_SIGN_EXPAND_BITS{wb_data_data_byte0[`BYTE_SIGN_BIT]}},  // sign expand
            wb_data_data_byte0  // byte data
          };
          2'b01:
          data_load_d = {
            {`BYTE_SIGN_EXPAND_BITS{wb_data_data_byte1[`BYTE_SIGN_BIT]}},  // sign expand
            wb_data_data_byte1  // byte data
          };
          2'b10:
          data_load_d = {
            {`BYTE_SIGN_EXPAND_BITS{wb_data_data_byte2[`BYTE_SIGN_BIT]}},  // sign expand
            wb_data_data_byte2  // byte data
          };
          2'b11:
          data_load_d = {
            {`BYTE_SIGN_EXPAND_BITS{wb_data_data_byte3[`BYTE_SIGN_BIT]}},  // sign expand
            wb_data_data_byte3  // byte data
          };
          default: data_load_d = 0;
        endcase

        // STORE
        wr_mask_d    = 4'b0001 << addr_2;  // mask 1 of the 4 bytes
        data_store_d = rs2_wdata << {addr_2, 3'b000};  // align data to mask
      end

      `FUNCT3_LOAD_STORE_HALF: begin : load_store_half
        // LOAD SIGNED
        case (addr_2[1])  // choose which of the 2 half will be loaded to base register
          1'b0:
          data_load_d = {
            {`HALF_SIGN_EXPAND_BITS{wb_data_data_half0[`HALF_SIGN_BIT]}},  // sign expand
            wb_data_data_half0  // half data
          };
          1'b1:
          data_load_d = {
            {`HALF_SIGN_EXPAND_BITS{wb_data_data_half1[`HALF_SIGN_BIT]}},  // sign expand
            wb_data_data_half1  // half data
          };
          default: data_load_d = 0;  // some error happend, don't load
        endcase

        // STORE
        wr_mask_d    = 4'b0011 << {addr_2[1], 1'b0};  // mask 1 of the 2 halfs
        data_store_d = rs2_wdata << {addr_2[1], 4'b0000};  //  align data to mask
      end

      `FUNCT3_LOAD_STORE_WORD: begin : load_store_word
        // LOAD SIGNED/UNSIGNED (all the same)
        data_load_d  = wb_data_data_word;

        // STORE
        wr_mask_d    = 4'b1111;  //mask all 4 bytes as a word
        data_store_d = rs2_wdata;  // no need to align data to mask, store all bytes as a word
      end

      `FUNCT3_LOAD_BYTE_U: begin : load_byte_unsigned
        // LOAD UNSIGNED
        case (addr_2)  // choose which of the 4 byte will be loaded to base register
          2'b00:
          data_load_d = {
            {`BYTE_SIGN_EXPAND_BITS{1'b0}},  // sign expand
            wb_data_data_byte0  // byte data
          };
          2'b01:
          data_load_d = {
            {`BYTE_SIGN_EXPAND_BITS{1'b0}},  // sign expand
            wb_data_data_byte1  // byte data
          };
          2'b10:
          data_load_d = {
            {`BYTE_SIGN_EXPAND_BITS{1'b0}},  // sign expand
            wb_data_data_byte2  // byte data
          };
          2'b11:
          data_load_d = {
            {`BYTE_SIGN_EXPAND_BITS{1'b0}},  // sign expand
            wb_data_data_byte3  // byte data
          };
          default: data_load_d = 0;  // some error happend, don't load
        endcase
      end

      `FUNCT3_LOAD_HALF_U: begin : load_half_unsigned
        // LOAD UNSIGNED
        case (addr_2[1])  // choose which of the 2 half will be loaded to base register
          1'b0:
          data_load_d = {
            {`HALF_SIGN_EXPAND_BITS{1'b0}},  // unsign expand
            wb_data_data_half0  // half data
          };
          1'b1:
          data_load_d = {
            {`HALF_SIGN_EXPAND_BITS{1'b0}},  // unsign expand
            wb_data_data_half1  // half data
          };
          default: data_load_d = 0;  // some error happend, don't load
        endcase
      end

      default: begin
        data_load_d  = 0;
        wr_mask_d    = 0;
        data_store_d = 0;
      end
    endcase
  end
endmodule


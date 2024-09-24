/* The mem module serves as the memory access stage of the 
 pipelined processor. This module primarily handles data memory access for
 load and store instructions, as well as passing the necessary information 
 to subsequent pipeline stas. The module is responsible for generating 
 appropriate data memory addresses, data to be stored, and write masks for
 different load/store operations, as well as handling pipeline stalls and 
 flushes when required. Key functionalities of the mem module
 include:
 - Address and data handling for load/store operations: The module uses the 
 incoming address (execute_result_i) to generate the appropriate data memory address (memory_addr_o)
 and stores it in the o_data_store register. It also selects the correct byte, 
 halfword, or word data from the data memory input (memory_rdata_i) based on the 
 instruction's funct3 field and stores it in the memory_data_load_i register. The 
 write mask (memory_be_o) is generated based on the address and the size of the 
 operation (byte, halfword, or word). The mask is used to control which part
 of the data memory will be written during store operations.
 - Register writeback control: The module i_wb_stall_datadetermines whether a destination register
 should be written (memory_rd_write_en_o) based on the input execute_rd_write_en_i signal. It passes the 
 destination register address (o_rd_addr) and the data to be written (memory_rd_wdata_o) to the 
 next stage.
 - Data memory control: The module controls the data memory read/write requests by
 generating the o_stb_data signal, which indicates a request for data memory access.
It also generates the memory_we_o signal, which indicates whether a write operation 
should be performed on the data memory.
 - Pipeline control: The module can stall the pipeline by asserting the stall signal
 if the data memory access is not yet acknowledged (memory_ack_i) or if there is a stall
 request from the ALU stage (stall_from_alu). It can also flush the current stage and
 previous stages using the flush signal based on the input memory_flush_i signal. The module 
 controls the clock enable signals (writeback_en_o) for the next stage based on the stall and flush 
 conditions.
*/

`include "rv32i_header.svh"

module mem (
  input  wire clk,
  input  wire rstn,

  input  wire [31:0] execute_rs2_wdata_i,                // data to be stored to memory is always execute_rs2_wdata_i
  input  wire [31:0] execute_result_i,                   // result from ALU (mem address to load/store)

  input  wire [2:0]  execute_funct3_i,                   // funct3 from previous stage
  output reg  [2:0]  memory_funct3_o,                    // memory_funct3_o (byte,halfword,word)

  input  wire [`OPCODE_WIDTH-1:0] execute_opcode_type_i, // opcode type from previous stage
  output reg  [`OPCODE_WIDTH-1:0] memory_opcode_type_o,  // opcode type

  input  wire [31:0] execute_pc_i,                       // PC from previous stage
  output reg  [31:0] memory_pc_o,                        // memory_pc_o value

  // Basereg Control
  input  wire        execute_rd_write_en_i,              // write rd to base reg is enabled (from memoryaccess stage)
  output reg         memory_rd_write_en_o,               // write rd to the base reg if enabled
  input  wire [ 4:0] execute_rd_addr_i,                  // address for destination register (from previous stage)
  output reg  [ 4:0] memory_rd_addr_o,                   // address for destination register
  input  wire [31:0] execute_rd_wdata_i,                 // value to be written back to destination reg
  output reg  [31:0] memory_rd_wdata_o,                  // value to be written back to destination register

  // Data Memory Interface
  // bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
  //output reg         memory_bus_cyc_data_o,             
  output reg         memory_req_o,                       // request for read/write access to data memory
  output reg         memory_we_o,                        // write-enable (1 = write, 0 = read)
  output reg  [31:0] memory_addr_o,                      // data memory address
  output reg  [31:0] memory_wdata_o,                     // data to be stored to memory
  output reg  [ 3:0] memory_be_o,                        // byte enable for write {byte3, byte2, byte1, byte0}
  // ack by data memory (high when data to be read is ready or when write data is already written)
  input  wire        memory_ack_i,                                                                              
  //input  wire        memory_stall_i,                     // stall by data memory (1 = data memory is busy)
  input  wire [31:0] memory_rdata_i,                     // data retrieve from data memory
  output reg  [31:0] memory_data_load_i,                 // data to be loaded to base reg (z-or-s extended)

  /// Pipeline Control ///
  input  wire        stall_from_alu,                     // stalls this stage when incoming instruction is a load/store
  input  wire        memory_en_i,                        // input clk enable for pipeline stalling of this stage
  output reg         writeback_en_o,                     // output clk enable for pipeline stalling of next stage
  input  wire        memory_stall_i,                     // informs this stage to stall
  output reg         memory_pipeline_stall_o,            // informs pipeline to stall
  input  wire        memory_flush_i,                     // flush this stage
  output reg         memory_pipeline_flush_o             // flush previous stages
);

  reg [31:0] data_store_d;  // data to be stored to memory
  reg [31:0] data_load_d;  // data to be loaded to basereg
  reg [3:0] wr_mask_d;
  reg pending_request;  // high if there is still a pending request (not yet acknowledged request)
  wire [1:0] addr_2 = execute_result_i[1:0];  //last 2 bits of data memory address

  wire stall_bit = (memory_stall_i || memory_pipeline_stall_o);


            /////////////////////////////////
            // Pipelined Register Updating //
            /////////////////////////////////
  always @(posedge clk, negedge rstn) begin
    if (!rstn) begin
      memory_rd_write_en_o  <= 0;
      memory_we_o           <= 0;
      writeback_en_o        <= 0;
      memory_req_o          <= 0;
      pending_request       <= 0;
      //memory_bus_cyc_data_o <= 0;
    end else begin
      // wishbone cycle will only be high if this stage is enabled
      //memory_bus_cyc_data_o <= memory_en_i;
      // request completed after grant
      if (memory_ack_i) begin
        pending_request      <= 0;  // not pending any more
      end

      // update register only if this stage is enabled and not stalled (after load/store operation)
      if (memory_en_i && !stall_bit) begin
        memory_rd_addr_o     <= execute_rd_addr_i;
        memory_funct3_o      <= execute_funct3_i;
        memory_opcode_type_o <= execute_opcode_type_i;
        memory_pc_o          <= execute_pc_i;
        memory_rd_write_en_o <= execute_rd_write_en_i;
        memory_rd_wdata_o    <= execute_rd_wdata_i;
        memory_data_load_i   <= data_load_d;
      end

      // update request to memory when no pending request yet
      if ((!pending_request) && memory_en_i) begin
        // req goes high when instruction is a load/store and when
        // request is not already high (request lasts for 1 clk cycle only)
        memory_req_o    <= execute_opcode_type_i[`LOAD] || execute_opcode_type_i[`STORE];
        memory_be_o     <= wr_mask_d;
        memory_we_o     <= execute_opcode_type_i[`STORE];
        pending_request <= execute_opcode_type_i[`LOAD] || execute_opcode_type_i[`STORE];
        memory_addr_o   <= execute_result_i;
        memory_wdata_o  <= data_store_d;
      end

      // if there is pending request but no stall from memory: idle the stb line
      if (pending_request && !memory_stall_i) memory_req_o <= 0;


      if (!memory_en_i) memory_req_o <= 0;


      // flush this stage so clock-enable of next stage is disabled at next clock cycle
      if ((!stall_bit) && memory_flush_i) writeback_en_o <= 0;
      // clock-enable will change only when not stalled
      else if (!stall_bit) writeback_en_o <= memory_en_i;
      // if this stage is stalled but this stage is not directly stalled, disable clock enable of next stage at next clock cycle (pipeline bubble)
      // else if (stall_bit && (!memory_stall_i)) writeback_en_o <= 0;
      else if (memory_pipeline_stall_o && (!memory_stall_i)) writeback_en_o <= 0;
    end

  end


            /////////////////////////////////
            // Logic to handle byte enable //
            /////////////////////////////////

  wire [ 7:0] wb_data_data_byte0 = memory_rdata_i[(0+1)*8-1:0*8];  // byte0
  wire [ 7:0] wb_data_data_byte1 = memory_rdata_i[(1+1)*8-1:1*8];  // byte1
  wire [ 7:0] wb_data_data_byte2 = memory_rdata_i[(2+1)*8-1:2*8];  // byte2
  wire [ 7:0] wb_data_data_byte3 = memory_rdata_i[(3+1)*8-1:3*8];  // byte3

  wire [15:0] wb_data_data_half0 = memory_rdata_i[(0+1)*16-1:0*16];  // half0
  wire [15:0] wb_data_data_half1 = memory_rdata_i[(1+1)*16-1:1*16];  // half1

  wire [31:0] wb_data_data_word = memory_rdata_i;  // word

  //determine data to be loaded to basereg or stored to data memory
  always @* begin
    //stall while data memory has not yet acknowledged i.e.write data is not yet written or
    //read data is not yet available (no ack yet). Don't stall when need to flush by next stage
    memory_pipeline_stall_o        = ((stall_from_alu && memory_en_i && !memory_ack_i) || memory_stall_i) && !memory_flush_i;
    memory_pipeline_flush_o        = memory_flush_i;  //flush this stage along with previous stages
    data_store_d = 0;
    data_load_d  = 0;
    wr_mask_d    = 0;

    case (execute_funct3_i)
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
        data_store_d = execute_rs2_wdata_i << {addr_2, 3'b000};  // align data to mask
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
        data_store_d = execute_rs2_wdata_i << {addr_2[1], 4'b0000};  //  align data to mask
      end

      `FUNCT3_LOAD_STORE_WORD: begin : load_store_word
        // LOAD SIGNED/UNSIGNED (all the same)
        data_load_d  = wb_data_data_word;

        // STORE
        wr_mask_d    = 4'b1111;  //mask all 4 bytes as a word
        data_store_d = execute_rs2_wdata_i;  // no need to align data to mask, store all bytes as a word
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

      
      
  
      
      
  
      
      
      
      
      
      
  
      
      
      
      
      
  
      
      
      
      
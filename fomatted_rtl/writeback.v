/* The rv32i_writeback module serves as the writeback stage of the pipelined 
processor. This stage is responsible for determining the next value of the
program counter (PC), writing data back to the destination register, and 
handling trap-related operations (interrupts and exceptions). In addition,
the module manages pipeline control, such as stalling and flushing previous 
stages. Key functionalities of the rv32i_writeback module include:
 - Determining the next value of the program counter (PC) and updating the
    next_pc register: If an interrupt or exception is detected, the module
    sets the PC to the trap address (trap_addr) and asserts the change_pc 
    signal. If the processor is returning from a trap, the module sets the PC to 
    the return address (return_addr) and asserts the change_pc signal. In
    normal operation, the PC value from the previous stage (prev_pc) is passed through.
 - Handling writeback to destination registers: The module writes data back to the
    destination register based on the opcode and funct3 fields of the instruction:
    If the instruction is a load operation, the data from the memory (data_load) 
    is written back. If the instruction is a CSR write operation, the CSR value 
    (csr_out) is written back. In other cases, the data is computed at the ALU
    stage (prev_rd_wdata) and is written back. The rd_w_en signal is set based on the prev_rd_w_en 
    input and the current pipeline control state (prev_clk_en and stall). The destination 
    register address (rd) is passed through from the prev_rd input.
 - Trap-handler control: The module handles interrupts and exceptions by checking the
    go_to_trap and return_from_trap input signals. When the processor goes to a 
    trap, the next_pc register is set to the trap address (trap_addr) and the
    pipeline is flushed. When the processor returns from a trap, the next_pc register
    is set to the return address (return_addr) and the pipeline is flushed.
 - Pipeline control: The module can stall the pipeline by asserting the stall signal
    when necessary. It can also flush the current stage and previous stages by asserting
    the flush signal based on the state of the pipeline and trap-related operations.
*/

//logic controller for the next PC and rd value [WRITEBACK STAGE]

`timescale 1ns / 1ps
`include "rv32i_header.vh"

module writeback (
    input wire [ 2:0]           funct3,        //function type 
    input wire [31:0]           data_load,     //data to be loaded to base reg
    input wire [31:0]           csr_out,       //CSR value to be loaded to basereg
    input wire                  opcode_load,
    input wire                  opcode_system,
              
    // Basereg Control          
    input  wire                 prev_rd_w_en,   // write rd to basereg if enabled (from previous stage)
    output reg                  rd_w_en,        // write rd to the base reg if enabled
    input  wire [ 4:0]          prev_rd,        // address for destination register (from previous stage)
    output reg  [ 4:0]          rd,             // address for destination register
    // value to be written back to destination register (from previous stage)
    input  wire [31:0]          prev_rd_wdata,
    output reg  [31:0]          rd_wdata,       // value to be written back to destination register

    // PC Control
    input  wire [31:0]          prev_pc,   // pc value (from previous stage)
    output reg  [31:0]          next_pc,   // new pc value
    output reg                  change_pc, // high if PC needs to jump

    // Trap-Handler
    input wire                  go_to_trap,        // trap (exception/interrupt detected)
    input wire                  return_from_trap,  // high before returning from trap (via mret)
    input wire [31:0]           return_addr,       // mepc CSR
    input wire [31:0]           trap_addr,         // mtvec CSR

    /// Pipeline Control ///
    input  wire                 prev_clk_en,  // input clk enable for pipeline stalling of this stage
    output reg                  stall,        // informs pipeline to stall
    output reg                  flush         // flush previous stages
);

  // determine next value of pc and rd_wdata
  always @* begin
    stall                       = 0;  // stall when this stage needs wait time
    flush                       = 0;  // flush this stage along with previous stages when changing PC
    rd_w_en                     = (prev_rd_w_en && prev_clk_en && (!stall));
    rd                          = prev_rd;
    rd_wdata                    = 0;
    next_pc                     = prev_pc;
    change_pc                   = 0;

    if (go_to_trap) begin
      change_pc                 = 1;  // change PC only when ce of this stage is high (change_pc is valid)
      next_pc                   = trap_addr;  // interrupt or exception detected so go to trap address (mtvec value)
      flush                     = prev_clk_en;
      rd_w_en                   = 0;
    end else if (return_from_trap) begin
      change_pc                 = 1;  //change PC only when ce of this stage is high (change_pc is valid)
      next_pc                   = return_addr;  //return from trap via mret (mepc value)
      flush                     = prev_clk_en;
      rd_w_en                   = 0;
    end else begin  //normal operation
      //load data from memory to basereg
      if (opcode_load) 
        rd_wdata = data_load;
      //CSR write
      else if (opcode_system && (funct3 != 0)) 
        rd_wdata                = csr_out;
      else rd_wdata             = prev_rd_wdata;  //rd value is already computed at ALU stage
    end

  end
endmodule

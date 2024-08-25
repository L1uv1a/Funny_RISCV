// Core plus memory

`timescale 1ns / 1ps
// `default_nettype none

//complete package containing the rv32i_core, RAM, and IO peripherals (I2C and UART)
module soc #(
    parameter CLK_FREQ_MHZ = 12,
    PC_RESET = 32'h00_00_00_00,
    TRAP_ADDRESS = 32'h00_00_00_00,
    ZICSR_EXTENSION = 1,
    MEMORY_DEPTH = 81920,
    GPIO_COUNT = 12
) (
    input wire i_clk,
    input wire i_rst
);


  //Instruction Memory Interface
  wire [31:0] inst;
  wire [31:0] iaddr;
  wire i_stb_inst;
  wire o_ack_inst;

  //Data Memory Interface
  wire [31:0] i_wb_data_data;  //data retrieved from memory
  wire [31:0] o_wb_data_data;  //data to be stored to memory
  wire [31:0] wb_addr_data;  //address of data memory for store/load
  wire [3:0] wb_sel_data;  //byte strobe for write (1 = write the byte) {byte3,byte2,byte1,byte0}
  wire wb_we_data;  //write-enable (1 = write, 0 = read) 
  wire wb_stb_data;  //request for read/write access to data memory
  wire wb_ack_data; //ack by data memory (high when data to be read is ready or when write data is already written
  wire wb_cyc_data; //bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
  wire wb_stall_data;  //stall by data memory

  //Interrupts
  wire i_external_interrupt = 0;  //interrupt from external source
  wire o_timer_interrupt;  //interrupt from CLINT
  wire o_software_interrupt;  //interrupt from CLINT

  //Memory Wrapper
  wire device0_wb_cyc;
  wire device0_wb_stb;
  wire device0_wb_we;
  wire [31:0] device0_wb_addr;
  wire [31:0] o_device0_wb_data;
  wire [3:0] device0_wb_sel;
  wire device0_wb_ack;
  wire device0_wb_stall;
  wire [31:0] i_device0_wb_data;

  core #(
      .PC_RESET       (PC_RESET),
      .TRAP_ADDRESS   (TRAP_ADDRESS),
      .ZICSR_EXTENSION(ZICSR_EXTENSION)
  ) core_inst (  //main RV32I core
      .i_clk  (i_clk),
      .i_rst_n(!i_rst),

      //Instruction Memory Interface
      .i_inst    (inst),        //32-bit instruction
      .o_iaddr   (iaddr),       //address of instruction 
      .o_stb_inst(i_stb_inst),  //request for read access to instruction memory
      .i_ack_inst(o_ack_inst),  //ack (high if new instruction is ready)

      //Data Memory Interface
      //bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
      .o_wb_cyc_data  (wb_cyc_data),
      .o_wb_stb_data  (wb_stb_data),     //request for read/write access to data memory
      .o_wb_we_data   (wb_we_data),      //write-enable (1 = write, 0 = read)
      .o_wb_addr_data (wb_addr_data),    //address of data memory for store/load
      .o_wb_data_data (o_wb_data_data),  //data to be stored to memory
      //byte strobe for write (1 = write the byte) {byte3,byte2,byte1,byte0}
      .o_wb_sel_data  (wb_sel_data),
      //ack by data memory (high when read data is ready or when write data is already written)
      .i_wb_ack_data  (wb_ack_data),
      .i_wb_stall_data(wb_stall_data),   //stall by data memory
      .i_wb_data_data (i_wb_data_data),  //data retrieved from memory

      //Interrupts
      //interrupt from external source
      .i_external_interrupt(i_external_interrupt),
      //interrupt from software (inter-processor interrupt)
      .i_software_interrupt(o_software_interrupt),
      //interrupt from timer
      .i_timer_interrupt   (o_timer_interrupt)
  );

  memory_wrapper memory_wrapper_inst (  //decodes address and access the corresponding memory-mapped device
      //RISC-V Core
      .i_wb_cyc  (wb_cyc_data),
      .i_wb_stb  (wb_stb_data),
      .i_wb_we   (wb_we_data),
      .i_wb_addr (wb_addr_data),
      .i_wb_data (o_wb_data_data),
      .i_wb_sel  (wb_sel_data),
      .o_wb_ack  (wb_ack_data),
      .o_wb_stall(wb_stall_data),
      .o_wb_data (i_wb_data_data),

      //Device 0 Interface (RAM)x
      .o_device0_wb_cyc  (device0_wb_cyc),
      .o_device0_wb_stb  (device0_wb_stb),
      .o_device0_wb_we   (device0_wb_we),
      .o_device0_wb_addr (device0_wb_addr),
      .o_device0_wb_data (o_device0_wb_data),
      .o_device0_wb_sel  (device0_wb_sel),
      .i_device0_wb_ack  (device0_wb_ack),
      .i_device0_wb_stall(device0_wb_stall),
      .i_device0_wb_data (i_device0_wb_data)
  );

  // DEVICE 0
  main_memory #(
      .MEMORY_DEPTH(MEMORY_DEPTH)
  ) main_memory_inst (  //Instruction and Data memory (combined memory) 
      .i_clk(i_clk),

      // Instruction Memory
      .i_inst_addr(iaddr[$clog2(MEMORY_DEPTH)-1:0]),
      .o_inst_out (inst),
      .i_stb_inst (i_stb_inst),
      .o_ack_inst (o_ack_inst),

      // Data Memory
      .i_wb_cyc  (device0_wb_cyc),
      .i_wb_stb  (device0_wb_stb),
      .i_wb_we   (device0_wb_we),
      .i_wb_addr (device0_wb_addr[$clog2(MEMORY_DEPTH)-1:0]),
      .i_wb_data (o_device0_wb_data),
      .i_wb_sel  (device0_wb_sel),
      .o_wb_ack  (device0_wb_ack),
      .o_wb_stall(device0_wb_stall),
      .o_wb_data (i_device0_wb_data)
  );
endmodule

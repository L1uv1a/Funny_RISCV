
/* The rv32i_core module represents the top module for an RV32I RISC-V processor 
core. The module is structured around a 5-stage pipeline architecture, which 
includes Fetch, Decode, Execute, Memory Access, and Writeback stages. The RV32I 
core is designed to interface with separate instrruction and data memories, and 
supports external, software, and timer interrupts. The module contains several 
sub-modules that carry out various functions within the processor:
 - rv32i_forwarding: This sub-module is responsible for handling operand
    forwarding. It ensures that the correct operand values are used in the ALU 
    stage, even when they have not yet been written back to the register file.
    Operand forwarding helps to reduce pipeline stalls caused by data 
    dependencies.
 - rv32i_basereg: This sub-module serves as a controller for the 32 integer 
    base registers. It manages reading from and writing to the register file, 
    with read operations occurring during the Decode stage and write operations 
    during the Writeback stage.
 - rv32i_fetch: This sub-module is responsible for fetching instrructions from
    the instrruction memory. It generates the instrruction address, retrieves 
    the instrruction, and controls the program counter (PC). It also manages 
    the pipeline stall and flush signals for the Fetch stage.
 - rv32i_decoder: This sub-module takes care of decoding the fetched 32-bit
    instrruction. It extracts various fields from the instrruction, such as opcode,
    function type, immediate value, and register addresses. The sub-module also 
    detects exceptions, manages pipeline stall and flush signals for the Decode 
    stage, and provides a clock enable signal for the next stage.
 - rv32i_alu: This sub-module is the Arithmetic Logic Unit (ALU) of the core.
    It performs arithmetic and logical operations based on the opcode and 
    function type provided by the decoder. It also controls the program counter
    for branches and jumps, manages register write enable signals, and handles 
    pipeline stall and flush signals for the Execute stage.
 - rv32i_memoryaccess: This sub-module controls data memory access for load and 
    store operations. It computes the memory address based on the ALU output 
    and provides the appropriate signals for reading from or writing to the data
    memory. The sub-module also manages pipeline stall and flush signals for 
    the Memory Access stage.
 - rv32i_writeback: This sub-module is responsible for writing the results of ALU 
    and load operations back to the register file. It also manages the program 
    counter for returning from traps and provides clock enable signals for the 
    Writeback stage.
 - rv32i_csr: This sub-module manages the Control and Status Registers (CSRs) in 
    the core. It handles traps and exceptions, updates CSR values, and controls 
    the program counter for trap handling.
*/

`timescale 1ns / 1ps
// `default_nettype none
`include "rv32i_header.svh"

module core #(
    parameter PC_RESET     = 32'h00_00_00_00,
    parameter TRAP_ADDRESS = 0
) (
  input wire         i_clk,
                     i_rst_n,

  // instrruction Memory Interface (32 bit rom)
  input  wire [31:0] i_instr,                // 32-bit instrruction
  input  wire [31:0] i_instr_addr;
  output wire [31:0] o_iaddr,               // address of instrruction
  output wire        o_stb_instr,            // request for read access to instrruction memory
  input  wire        i_ack_instr,            // ack (high if new instrruction is ready)

  // Data Memory Interface (32 bit ram)
  // bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
  //output wire        o_wb_cyc_data,
  output wire        o_wb_stb_data,         // request for read/write access to data memory
  output wire        o_wb_we,          // write-enable (1 = write, 0 = read)
  output wire [31:0] o_wb_addr_data,        // address of data memory for store/load
  output wire [31:0] o_wb_data_data,        // data to be stored to memory
  // byte strobe for write (1 = write the byte) {byte3,byte2,byte1,byte0}
  output wire [ 3:0] o_wb_sel_data,
  // ack by data memory (high when read data is ready or when write data is already written)
  input  wire        i_wb_ack,
  //input  wire        i_wb_stall,       // stall by data memory
  input  wire [31:0] i_wb_data_data,        // data retrieve from memory
  // Interrupts
  input  wire        i_external_interrupt,  // interrupt from external source
  input  wire        i_software_interrupt,  // interrupt from software (inter-processor interrupt)
  input  wire        i_timer_interrupt      // interrupt from timer
);

  localparam ZICSR_EXTENSION = 1;

  //wires for rv32i_fetch
  wire [31:0]                 fetch_pc;
  wire [31:0]                 fetch_instr, instr_decompressed;

  //wires for rv32i_decoder
  wire [`ALU_WIDTH-1:0]       decoder_alu;
  wire [`OPCODE_WIDTH-1:0]    decoder_opcode;
  wire [31:0]                 decoder_pc;
  wire [4:0]                  decoder_rs1_addr, decoder_rs2_addr;
  wire [4:0]                  decoder_rs1_addr_q, decoder_rs2_addr_q;
  wire [4:0]                  decoder_rd_addr;
  wire [31:0]                 decoder_imm;
  wire [2:0]                  decoder_funct3;
  wire [`EXCEPTION_WIDTH-1:0] decoder_exception;
  wire                        decoder_ce;
  wire                        decoder_flush;

  //wires for rv32i_alu
  wire [`OPCODE_WIDTH-1:0]    alu_opcode;
  wire [4:0]                  alu_rs1_addr;
  wire [31:0]                 alu_rs1;
  wire [31:0]                 alu_rs2;
  wire [11:0]                 alu_imm;
  wire [2:0]                  alu_funct3;
  wire [31:0]                 alu_y;
  wire [31:0]                 alu_pc;
  wire [31:0]                 alu_next_pc;
  wire                        alu_change_pc;
  wire                        alu_wr_rd;
  wire [4:0]                  alu_rd_addr;
  wire [31:0]                 alu_rd;
  wire                        alu_rd_valid;
  wire [`EXCEPTION_WIDTH-1:0] alu_exception;
  wire                        alu_ce;
  wire                        alu_flush;
  wire                        alu_force_stall;

  //wires for rv32i_memoryaccess
  wire [`OPCODE_WIDTH-1:0]    memoryaccess_opcode;
  wire [2:0]                  memoryaccess_funct3;
  wire [31:0]                 memoryaccess_pc;
  wire                        memoryaccess_wr_rd;
  wire [4:0]                  memoryaccess_rd_addr;
  wire [31:0]                 memoryaccess_rd;
  wire [31:0]                 memoryaccess_data_load;
  wire                        memoryaccess_wr_mem;
  wire                        memoryaccess_ce;
  wire                        memoryaccess_flush;
  wire                        o_stall_from_alu;
  //wires for rv32i_writeback
  wire                        writeback_wr_rd;
  wire [4:0]                  writeback_rd_addr;
  wire [31:0]                 writeback_rd;
  wire [31:0]                 writeback_next_pc;
  wire                        writeback_change_pc;
  wire                        writeback_ce;
  wire                        writeback_flush;

  //wires for rv32i_csr
  wire [31:0]                 csr_out;               // CSR value to be stored to basereg
  wire [31:0]                 csr_return_address;    // mepc CSR
  wire [31:0]                 csr_trap_address;      // mtvec CSR
  wire                        csr_go_to_trap;        // high before going to trap (if exception/interrupt detected)
  wire                        csr_return_from_trap;  // high before returning from trap (via mret)

  wire                        stall_decoder;
  wire                        stall_alu;
  wire                        stall_memoryaccess;
  wire                        stall_writeback;       // control stall of each pipeline stages

  //wires for basereg
  wire [31:0]                 rs1_orig, rs2_orig;
  wire [31:0]                 rs1, rs2;
  wire                        ce_read;

  assign ce_read = decoder_ce && !stall_decoder;     // reads basereg only decoder is not stalled

  /////////////////////////////
  // int32 reg file instance //
  /////////////////////////////
  regs m0 (
       // int32 reg file control logic  
      .clk        (i_clk),

      .clk_en_read(ce_read),                      // clock enable for reading from basereg [STAGE 2]
      .rs1        (decoder_rs1_addr),             // source register 1 address
      .rs2        (decoder_rs2_addr),             // source register 2 address

      .rd         (writeback_rd_addr),            // destination register address
      .rd_wdata   (writeback_rd),                 // data to be written to destination register
      .w_en       (writeback_wr_rd),              // write enable

      .rs1_rdata  (rs1_orig),                     // source register 1 value
      .rs2_rdata  (rs2_orig)                      // source register 2 value
  );


  ////////////////////
  // fetch instance //
  ////////////////////
  fetch #(
      .PC_RESET(PC_RESET)
  ) m1 (  
       // logic for fetch stage [STAGE 1]
      .clk                (i_clk),
      .rstn               (i_rst_n),

      .pc                 (fetch_pc),             // PC value of o_instr
      .instr_send         (fetch_instr),          // instrruction

      .instr_addr_o(o_iaddr),     //Instruction address
      .instr_addr_i(i_instr_addr),
      .instr_rdata_i(i_instr),      // retrieved instruction from Memory
      .instr_req_o(o_stb_instr),  // request for instruction
      .instr_gnt_i(i_ack_instr),  //ack (high if new instruction is ready)
      .instr_err_i(1'b0),    // err    // fix NO USE
      .instr_rvalid_i(1'b0), // ack (high if new instrruction is ready)

      // PC Control
      .writeback_change_pc(writeback_change_pc),  // high when PC needs to change when going to trap or returning from trap
      .writeback_next_pc  (writeback_next_pc),    // next PC due to trap
      .alu_change_pc      (alu_change_pc),        // high when PC needs to change for taken branches and jumps
      .alu_next_pc        (alu_next_pc),          // next PC due to branch or jump

      /// Pipeline Control 
      .clk_en(decoder_ce),                                                             // output clk enable for pipeline stalling of next stage
      .stall ((stall_decoder || stall_alu || stall_memoryaccess || stall_writeback)),  // informs this stage to stall
      .flush (decoder_flush)                                                           // flush this stage
  );



  ////////////////////////////////////
  // compressed instruction decoder //         turn 16 bit instruction to 32 bit
  ////////////////////////////////////
  
  ibex_compressed_decoder compressed_decoder_i (
    .clk_i          (i_clk),
    .rst_ni         (i_rst_n),
    .valid_i        (1'b1),
    .instr_i        (fetch_instr),
    .instr_o        (instr_decompressed)
    //.is_compressed_o(instr_is_compressed),
    // .illegal_instr_o(illegal_c_insn)
  );


  /////////////////////
  // decode instance //
  /////////////////////
  instr_de m2 (  
      // logic for decode stage [STAGE 2]
      .clk              (i_clk),                                                                              
      .rstn             (i_rst_n),                                                                              

      .fetch_instr_i           (instr_decompressed),                                          // 32 bit instrruction
      .fetch_pc_i          (fetch_pc),                                             // PC value from fetch stage

      .decode_pc_o              (decoder_pc),                                           // PC value

      .rf_rs1_addr_o              (decoder_rs1_addr),                                     // address for register source 1
      .fw_rs1_addr_o            (decoder_rs1_addr_q),                                   // registered address for register source 1
      .rf_rs2_addr_o              (decoder_rs2_addr),                                     // address for register source 2
      .fw_rs2_addr_o            (decoder_rs2_addr_q),                                   // registered address for register source 2
      .decode_rd_addr_o               (decoder_rd_addr),                                      // address for destination register
      .decode_imm_o              (decoder_imm),                                          // extended value for immediate
      .decode_funct3_o           (decoder_funct3),                                       // function type

      .decode_alu_operation_o               (decoder_alu), 
      .decode_opcode_type_o      (decoder_opcode),                                       // opcode type
      .decode_exception_o        (decoder_exception),                                    // exceptions: illegal instr, ecall, ebreak, mret

      /// Pipeline Control //
      .decode_en_i      (decoder_ce),                                           // input clk enable for pipeline stalling of this stage
      .execute_en_o           (alu_ce),                                               // output clk enable for pipeline stalling of next stage
      .decode_stall_i           ((stall_alu || stall_memoryaccess || stall_writeback)), // informs this stage to stall
      .decode_pipeline_stall_o           (stall_decoder),                                        // informs pipeline to stall
      .decode_flush_i      (alu_flush),                                            // flush this stage
      .decode_pipeline_flush_o            (decoder_flush)                                         // flushes previous stages
  );
  
  //////////////////
  // alu instance //
  //////////////////
  alu m3 (  
      // ALU combinational logic (execute stage) [STAGE 3]
      .clk              (i_clk),                                                       
      .rstn             (i_rst_n),                                                    
      .alu_operation    (decoder_alu),                                   // alu operation type
      .prev_rs1         (decoder_rs1_addr_q),                            // address for register source 1
      .rs1              (alu_rs1_addr),                                  // address for register source 1
      .prev_rs1_data    (rs1),                                           // Source register 1 value
      .rs1_data         (alu_rs1),                                       // Source register 1 value
      .prev_rs2_data    (rs2),                                           // Source Register 2 value
      .rs2_data         (alu_rs2),                                       // Source Register 2 value
      .prev_imm         (decoder_imm),                                   // Immediate value from previous stage
      .imm              (alu_imm),                                       // Immediate value
      .prev_funct3      (decoder_funct3),                                // function type from decoder stage
      .funct3           (alu_funct3),                                    // function type
      .prev_opcode_type (decoder_opcode),                                // opcode type from previous stage
      .opcode_type      (alu_opcode),                                    // opcode type
      .prev_exception   (decoder_exception),                             // exception from decoder stage
      .exception        (alu_exception),                                 // exception: illegal instr,ecall,ebreak,mret
      .alu_result       (alu_y),                                         // result of arithmetic operation
      // PC Control
      .prev_pc          (decoder_pc),                                    // pc from decoder stage
      .pc               (alu_pc),                                        //  current pc 
      .next_pc          (alu_next_pc),                                   // next pc 
      .change_pc        (alu_change_pc),                                 // change pc if high
      // Basereg Control
      .rd_w_en          (alu_wr_rd),                                     // write rd to basereg if enabled
      .prev_rd          (decoder_rd_addr),                               // address for destination register (from previous stage)
      .rd               (alu_rd_addr),                                   // address for destination register
      .rd_wdata         (alu_rd),                                        // value to be written back to destination register
      .rd_valid         (alu_rd_valid),                                  // high if o_rd is valid (not load nor csr instrruction)
      /// Pipeline Control //
      .stall_from_alu   (o_stall_from_alu),                              // prepare to stall next stage(memory-access stage) for load/store instrruction
      .prev_clk_en      (alu_ce),                                        // input clk enable for pipeline stalling of this stage
      .clk_en           (memoryaccess_ce),                               // output clk enable for pipeline stalling of next stage
      .prev_stall       ((stall_memoryaccess || stall_writeback)),       // informs this stage to stall
      .force_stall      (alu_force_stall),                               // force this stage to stall
      .stall            (stall_alu),                                     // informs pipeline to stall
      .prev_flush       (memoryaccess_flush),                            // flush this stage
      .flush            (alu_flush)                                      // flushes previous stages
  );

  ////////////////// 
  // mem instance //
  //////////////////
  mem m4 (  
      //logic controller for data memory access (load/store) (memory stage) [STAGE 4]
      .clk              (i_clk),                                                                              
      .rstn             (i_rst_n),                                                                              
      .execute_rs2_wdata_i        (alu_rs2),                                       // data to be stored to memory is always rs2
      .execute_result_i       (alu_y),                                         // y value from ALU (address of data to memory be stored or loaded)
      .execute_funct3_i      (alu_funct3),                                    // funct3 from previous stage
      .memory_funct3_o           (memoryaccess_funct3),                           // funct3 (byte,halfword,word)
      .execute_opcode_type_i (alu_opcode),                                    // opcode type from previous stage
      .memory_opcode_type_o      (memoryaccess_opcode),                           // opcode type
      .execute_pc_i          (alu_pc),                                        // PC from previous stage
      .memory_pc_o               (memoryaccess_pc),                               // PC value
      // Basereg Control
      .execute_rd_write_en_i     (alu_wr_rd),                                     // write rd to base reg is enabled (from memoryaccess stage)
      .memory_rd_write_en_o          (memoryaccess_wr_rd),                            // write rd to the base reg if enabled
      .execute_rd_addr_i          (alu_rd_addr),                                   // address for destination register (from previous stage)
      .memory_rd_addr_o               (memoryaccess_rd_addr),                          // address for destination register
      .execute_rd_wdata_i    (alu_rd),                                        // value to be written back to destination reg
      .memory_rd_wdata_o         (memoryaccess_rd),                               // value to be written back to destination register
      // Data Memory Control
      //.memory_bus_cyc_data_o  (o_wb_cyc_data),                                 // bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
      .memory_req_o  (o_wb_stb_data),                                 // request for read/write access to data memory
      .memory_we_o  (o_wb_we),                                  // write-enable (1 = write, 0 = read)
      .memory_addr_o  (o_wb_addr_data),                                // data memory address
      .memory_wdata_o  (o_wb_data_data),                                // data to be stored to memory (mask-aligned)
      .memory_be_o  (o_wb_sel_data),                              // byte strobe for write (1 = write the byte) {byte3,byte2,byte1,byte0}
      .memory_ack_i  (i_wb_ack),                                 // ack by data memory (high when read data is ready or when write data is already written
      //.memory_stall_i  (i_wb_stall),                               // stall by data memory (1 = data memory is busy)
      .memory_rdata_i  (i_wb_data_data),                                // data retrieve from data memory 
      .memory_data_load_i        (memoryaccess_data_load),                        // data to be loaded to base reg (z-or-s extended) 
      /// Pipeline Control ///
      .stall_from_alu   (o_stall_from_alu),                              // stalls this stage when incoming instrruction is a load/store
      .memory_en_i      (memoryaccess_ce),                               // input clk enable for pipeline stalling of this stage
      .writeback_en_o           (writeback_ce),                                  // output clk enable for pipeline stalling of next stage
      .memory_stall_i       (stall_writeback),                               // informs this stage to stall
      .memory_pipeline_stall_o            (stall_memoryaccess),                            // informs pipeline to stall
      .memory_flush_i       (writeback_flush),                               // flush this stage
      .memory_pipeline_flush_o            (memoryaccess_flush)                             // flushes previous stages
  );

  ////////////////////////
  // writeback instance //
  ////////////////////////
  writeback m5 (  
      // logic controller for the next PC and rd value (write-back stage) [STAGE 5]                                                                        
      .funct3           (memoryaccess_funct3),                           // function type
      .data_load        (memoryaccess_data_load),                        // data to be loaded to base reg (from previous stage)
      .csr_out          (csr_out),                                       // CSR value to be loaded to basereg
      .opcode_load      (memoryaccess_opcode[`LOAD]),                          
      .opcode_system    (memoryaccess_opcode[`SYSTEM]),                                                                               
      // Basereg Control
      .prev_rd_w_en     (memoryaccess_wr_rd),                            // write rd to base reg is enabled (from memoryaccess stage)
      .rd_w_en          (writeback_wr_rd),                               // write rd to the base reg if enabled
      .prev_rd          (memoryaccess_rd_addr),                          // address for destination register (from previous stage)
      .rd               (writeback_rd_addr),                             // address for destination register
      .prev_rd_wdata    (memoryaccess_rd),                               // value to be written back to destination reg
      .rd_wdata         (writeback_rd),                                  // value to be written back to destination register
      // PC Control
      .prev_pc          (memoryaccess_pc),                               // pc value
      .next_pc          (writeback_next_pc),                             // new PC value
      .change_pc        (writeback_change_pc),                           // high if PC needs to jump
      // Trap-Handler
      .go_to_trap       (csr_go_to_trap),                                // high before going to trap (if exception/interrupt detected)
      .return_from_trap (csr_return_from_trap),                          // high before returning from trap (via mret)
      .return_addr      (csr_return_address),                            // mepc CSR
      .trap_addr        (csr_trap_address),                              // mtvec CSR
      /// Pipeline Control ///
      .prev_clk_en      (writeback_ce),                                  //  input clk enable for pipeline stalling of this stage
      .stall            (stall_writeback),                               // informs pipeline to stall
      .flush            (writeback_flush)                                // flushes previous stages 
  );

  //////////////////
  // forward instr //
  //////////////////
  forward forward_instr (  
      // logic for operand forwarding
      .rs1_rdata        (rs1_orig),                                      // current rs1 value saved in basereg
      .rs2_rdata        (rs2_orig),                                      // current rs2 value saved in basereg
      .rs1              (decoder_rs1_addr_q),                            // address of operand rs1 used in ALU stage
      .rs2              (decoder_rs2_addr_q),                            // address of operand rs2 used in ALU stage
      .alu_force_stall  (alu_force_stall),                               // high to force ALU stage to stall
      .fwd_rs1_rdata    (rs1),                                           // rs1 value with Operand Forwarding
      .fwd_rs2_rdata    (rs2),                                           // rs2 value with Operand Forwarding
      // memory access [STAGE 4]
      .alu_rd           (alu_rd_addr),                                   // destination register address
      .alu_rd_w_en      (alu_wr_rd),                                     // high if rd_addr will be written
      .alu_rd_valid     (alu_rd_valid),                                  // high if rd is already valid at this stage (not LOAD nor CSR instrruction)
      .alu_rd_data      (alu_rd),                                        // rd value in stage 4
      .mem_en           (memoryaccess_ce),                               // high if stage 4 is enabled
      // write back [STAGE %]
      .mem_rd           (memoryaccess_rd_addr),                          // destination register address
      .mem_rd_w_en      (memoryaccess_wr_rd),                            // high if rd_addr will be written
      .writeback_rd_data(writeback_rd),                                  // rd value in stage 5
      .writeback_en     (writeback_ce)                                   // high if stage 4 is enabled
  );
   
  ////////////////////// 
  // zicsr extensions //    this extension is removeable 
  //////////////////////
  if (ZICSR_EXTENSION == 1) begin : zicsr
    rv32i_csr # (
        .TRAP_ADDRESS (TRAP_ADDRESS)
    ) m6 (  
        // control logic for Control and Status Registers  (CSR) [STAGE 4]
        .i_clk               (i_clk),                                                                               
        .i_rst_n             (i_rst_n),                                                                               
        // Interrupts
        .i_external_interrupt(i_external_interrupt),                     // interrupt from external source
        .i_software_interrupt(i_software_interrupt),                     // interrupt from software (inter-processor interrupt)
        .i_timer_interrupt   (i_timer_interrupt),                        // interrupt from timer
        /// Exceptions ///
        .i_is_inst_illegal   (alu_exception[`ILLEGAL]),                 // illegal instrruction
        .i_is_ecall          (alu_exception[`ECALL]),                    // ecall instrruction
        .i_is_ebreak         (alu_exception[`EBREAK]),                   // ebreak instrruction
        .i_is_mret           (alu_exception[`MRET]),                     // mret (return from trap) instrruction
        /// Load/Store Misaligned Exception///
        .i_opcode            (alu_opcode),                               // opcode type from alu stage
        .i_y                 (alu_y),                                    // y value from ALU (address used in load/store/jump/branch)
        /// CSR instrruction ///
        .i_funct3            (alu_funct3),                               // CSR instrruction operation
        .i_csr_index         (alu_imm),                                  // immediate value decoded by decoder
        .i_imm               ({27'b0, alu_rs1_addr}),                    // unsigned immediate for immediate  type of CSR instrruction (new value to be stored to CSR)
        .i_rs1               (alu_rs1),                                  // Source register 1 value (new value to be stored to CSR)
        .o_csr_out           (csr_out),                                  // CSR value to be loaded to basereg
        // Trap-Handler 
        .i_pc                (alu_pc),                                   // Program Counter  (three stages had already been filled [fetch -> decode -> execute ])
        .writeback_change_pc (writeback_change_pc),                      // high if writeback will issue change_pc (which will override this stage)
        .o_return_address    (csr_return_address),                       // mepc CSR
        .o_trap_address      (csr_trap_address),                         // mtvec CSR
        .o_go_to_trap_q      (csr_go_to_trap),                           // high before going to trap (if exception/interrupt detected)
        .o_return_from_trap_q(csr_return_from_trap),                     // high before returning from trap (via mret)
        .i_minstret_inc      (writeback_ce),                            // high for one clock cycle at the end of every instrruction
        /// Pipeline Control ///
        .i_ce                (memoryaccess_ce),                          // input clk enable for pipeline stalling of this stage
        .i_stall             ((stall_writeback || stall_memoryaccess))   // informs this stage to stall
    );
  end else begin : zicsr
    assign csr_out = 0;
    assign csr_return_address = 0;
    assign csr_trap_address = 0;
    assign csr_go_to_trap = 0;
    assign csr_return_from_trap = 0;
  end

endmodule

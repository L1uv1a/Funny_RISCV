`include "decode_header.vh"
`include "rv32i_header.vh"

module core #(
    parameter PC_RESET  = 0,
    parameter TRAP_ADDR = 0
) (
    input clk,  //! positive edge triggered system clock
    input rst,  //! asynchronous reset

    // region main_memory
    // Instruction Memory
    output [31:0] main_memory_instr_addr,  //! address for instruction
    input  [31:0] main_memory_instr,       //! instruction
    output        main_memory_instr_req,   //! request for read instruction
    input         main_memory_instr_ack,   //! ack

    // Data Memory
    output        main_memory_wb_cyc,      //! bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
    output        main_memory_wb_stb,      //! request for read/write access to data memory
    output        main_memory_wb_wr_en,    //! write-enable (1 = write, 0 = read)
    output [31:0] main_memory_wb_addr,     //! address of data memory for store/load
    output [31:0] main_memory_wb_wr_data,  //! data to be stored to memory
    output [ 3:0] main_memory_wb_wr_sel,   //! byte strobe for write (1 = write the byte) {byte3,byte2,byte1,byte0}
    input         main_memory_wb_ack,      //! ack by data memory (high when read data is ready or when write data is already written)
    input         main_memory_wb_stall,    //! stall by data memory
    input  [31:0] main_memory_wb_rd_data,  //! data retrieve from memory
    // endregion main_memory

    // region Interrupts
    input external_interrupt,  //! interrupt from external source
    input software_interrupt,  //! interrupt from software (inter-processor interrupt)
    input timer_interrupt      //! interrupt from timer
    // endregion Interrupts


    //rvfi outputs 
    output logic                         rvfi_valid,

    output logic [ 4:0]                  rvfi_rs1_addr,
    output logic [ 4:0]                  rvfi_rs2_addr,
    output logic [ 4:0]                  rvfi_rs3_addr,
    output logic [31:0]                  rvfi_rs1_rdata,
    output logic [31:0]                  rvfi_rs2_rdata,
    output logic [31:0]                  rvfi_rs3_rdata,
    output logic [ 4:0]                  rvfi_rd_addr,
    output logic [31:0]                  rvfi_rd_wdata,
    output logic [31:0]                  rvfi_pc_rdata,
    output logic [31:0]                  rvfi_pc_wdata,
    output logic [31:0]                  rvfi_mem_addr,
    output logic [ 3:0]                  rvfi_mem_rmask,
    output logic [ 3:0]                  rvfi_mem_wmask,
    output logic [31:0]                  rvfi_mem_rdata,
    output logic [31:0]                  rvfi_mem_wdata,
    output logic [31:0]                  rvfi_ext_pre_mip,
    output logic [31:0]                  rvfi_ext_post_mip,
    output logic                         rvfi_ext_nmi,
    output logic                         rvfi_ext_nmi_int,
    output logic                         rvfi_ext_debug_req,
    output logic                         rvfi_ext_debug_mode,
    output logic                         rvfi_ext_rf_wr_suppress,
    output logic [63:0]                  rvfi_ext_mcycle,
    output logic [31:0]                  rvfi_ext_mhpmcounters [10],
    output logic [31:0]                  rvfi_ext_mhpmcountersh [10],
    output logic                         rvfi_ext_ic_scr_key_valid,
    output logic                         rvfi_ext_irq_valid,
);

  // region R regs
  // region control by [STAGE 2 DECODE]
  wire        rs_rd_en = (decode_clk_en && (!decode_stall));  // source registers read enable
  //   wire [ 4:0] rs1;  // source register 1 address
  //   wire [ 4:0] rs2;  // source register 2 address
  // endregion control by [STAGE 2 DECODE]

  // region control by [STAGE 5 WRITEBACK]
  //   wire [ 4:0] rd;  // destination register address
  //   wire [31:0] rd_wr_data;  // data to be written to destination register
  //   wire        rd_wr_en;  // destination register write enable
  // endregion control by [STAGE 5 WRITEBACK]

  wire [31:0] regs_rs1_rd_data;  // source register 1 value
  wire [31:0] regs_rs2_rd_data;  // source register 2 value


  regs regs_dut (
      .clk(clk),
      .rst(rst),

      .rs_rd_en(rs_rd_en),
      .rs1     (decode_rs1),
      .rs2     (decode_rs2),

      .rd        (writeback_rd),
      .rd_wr_data(writeback_rd_wr_data),
      .rd_wr_en  (writeback_rd_wr_en),

      .rs1_rd_data(regs_rs1_rd_data),
      .rs2_rd_data(regs_rs2_rd_data)
  );
  // endregion R regs

  // region R fetch
  // region control Main Memory's Instruction Memory
  // Instruction Memory
  //   wire [31:0] main_memory_instr_addr;  // instruction memory address
  //   wire [31:0] main_memory_instr;  // instruction from memory
  //   wire        main_memory_instr_req;  // request for instruction
  //   wire        main_memory_instr_ack;  // ack
  // endregion control Main Memory's Instruction Memory

  wire [31:0] fetch_instr;  // fetched instruction sent to pipeline

  // region PC control
  wire [31:0] fetch_pc;  // PC value of current instruction

  // control from [STAGE 5 WRITEBACK]
  //   wire        writeback_change_pc;  // high when pc needs to change (trap/return from trap)
  //   wire [31:0] writeback_next_pc;  // next PC due to trap

  // control from [STAGE 3 EXECUTE]
  //   wire        execute_change_pc;  // high when pc needs to change (branch/jump)
  //   wire [31:0] execute_next_pc;  // next PC due to branch/jump
  // endregion PC control


  // region Pipeline control
  wire        stall_fetch = (decode_stall || execute_stall || memory_stall || writeback_stall);  // stall this stage
  //   wire        decode_flush;  // flush this stage
  wire        decode_clk_en;  // clk enable for pipeline stalling of next state ([STAGE 2 DECODE])
  // endregion Pipeline control

  fetch #(
      .PC_RESET(PC_RESET)
  ) fetch_dut (
      .clk(clk),
      .rst(rst),

      .main_memory_instr_addr(main_memory_instr_addr),
      .main_memory_instr     (main_memory_instr),
      .main_memory_instr_req (main_memory_instr_req),
      .main_memory_instr_ack (main_memory_instr_ack),

      .fetch_instr(fetch_instr),
      .pc         (fetch_pc),

      .writeback_change_pc(writeback_change_pc),
      .writeback_next_pc  (writeback_next_pc),

      .execute_change_pc(execute_change_pc),
      .execute_next_pc  (execute_next_pc),

      .stall      (stall_fetch),
      .flush      (decode_flush),
      .next_clk_en(decode_clk_en)
  );
  // endregion R fetch

  // region R decode
  // region control from [STAGE 1 FETCH]
  //   wire [                31:0] fetch_instr;  // fetched instruction from [STAGE 1 FETCH]
  //   wire [                31:0] fetch_pc;  // pc from [STAGE 1 FETCH] (previous stage)
  // endregion control from [STAGE 1 FETCH]

  // region decoded signals
  wire [                31:0] decode_pc;  // pc from [STAGE 2 DECODE] (current stage)

  wire [                 4:0] decode_rs1;  // source register 1 address
  wire [                 4:0] decode_r_rs1;  // registed source register 1 address
  wire [                 4:0] decode_rs2;  // source register 2 address
  wire [                 4:0] decode_r_rs2;  // registed source register 2 address
  wire [                 4:0] decode_r_rd;  // registed destination register address
  wire [                31:0] decode_imm;  // extended value for immediate
  wire [                 2:0] decode_funct3;  // function type

  wire [      `ALU_WIDTH-1:0] decode_alu_type;  // alu operation type
  wire [   `OPCODE_WIDTH-1:0] decode_opcode_type;  // opcode type
  wire [`EXCEPTION_WIDTH-1:0] decode_exception;  // illegal instr, ecall, ebreak, mret
  // endregion decoded signals

  // region Pipeline control
  //   wire                        clk_en;  // control by previous stage ([STAGE 1 FETCH])
  wire                        execute_clk_en;  // clk enable for pipeline stalling of next state ([STAGE 3 EXECUTE])
  wire                        stall_decode = (execute_stall || memory_stall || writeback_stall);  // stall this stage
  wire                        decode_stall;  // stalls the pipeline
  //   wire                        execute_flush;  // flush this stage
  wire                        decode_flush;  // flushes previous stages
  // endregion Pipeline control
  decode decode_dut (
      .clk(clk),
      .rst(rst),

      .fetch_instr(fetch_instr),
      .fetch_pc   (fetch_pc),

      .decode_pc(decode_pc),

      .decode_rs1   (decode_rs1),
      .decode_r_rs1 (decode_r_rs1),
      .decode_rs2   (decode_rs2),
      .decode_r_rs2 (decode_r_rs2),
      .decode_r_rd  (decode_r_rd),
      .decode_imm   (decode_imm),
      .decode_funct3(decode_funct3),

      .decode_alu_type   (decode_alu_type),
      .decode_opcode_type(decode_opcode_type),
      .decode_exception  (decode_exception),

      .clk_en     (decode_clk_en),
      .next_clk_en(execute_clk_en),
      .stall      (stall_decode),
      .next_stall (decode_stall),
      .flush      (execute_flush),
      .next_flush (decode_flush)
  );
  // endregion R decode

  // region R execute
  //   wire [      `ALU_WIDTH-1:0] decode_alu_type;

  //   wire [                 4:0] decode_r_rs1;
  wire [                 4:0] execute_rs1;

  //   wire [                31:0] forward_rs1_data;
  wire [                31:0] execute_rs1_data;

  //   wire [                31:0] forward_rs2_data;
  wire [                31:0] execute_rs2_data;

  //   wire [                 4:0] decode_r_rd;
  wire [                 4:0] execute_rd;

  //   wire [                31:0] decode_imm;
  wire [                11:0] execute_imm;

  //   wire [                2:0] decode_funct3;
  wire [                 2:0] execute_funct3;

  //   wire [   `OPCODE_WIDTH-1:0] decode_opcode_type;
  wire [   `OPCODE_WIDTH-1:0] execute_opcode_type;

  //   wire [`EXCEPTION_WIDTH-1:0] decode_exception;
  wire [`EXCEPTION_WIDTH-1:0] execute_exception;

  wire [                31:0] execute_result;  // alu operation result

  // region PC control
  //   wire [                31:0] decode_pc;  // pc
  wire [                31:0] execute_pc;  // pc in pipeline
  wire [                31:0] execute_next_pc;  // new pc
  wire                        execute_change_pc;  // need to jump
  // endregion PC control

  // region base registers control
  wire                        execute_rd_wr_en;
  wire [                31:0] execute_rd_wr_data;
  wire                        execute_rd_valid;
  // endregion base registers control

  // region Pipeline control
  wire                        stall_from_execute;  // stall next stage ([STAGE 4 MEMORY] for load/store instructions)
  //   wire                        clk_en;  // control by previous stage ([STAGE 2 DECODE])
  wire                        memory_clk_en;  // clk enable for pipeline stalling of next state ([STAGE 4 MEMORY])
  wire                        stall_execute = (memory_stall || writeback_stall);  // stall this stage
  //   wire                        force_stall_execute;  // force this stage to stall
  wire                        execute_stall;  // stalls the pipeline
  //   wire                        memory_flush;  // flush this stage
  wire                        execute_flush;  // flushes previous stages
  // endregion Pipeline control

  execute execute_dut (
      .clk(clk),
      .rst(rst),

      .decode_alu_type(decode_alu_type),

      .decode_r_rs1(decode_r_rs1),
      .execute_rs1 (execute_rs1),

      .forward_rs1_data(forward_rs1_data),
      .execute_rs1_data(execute_rs1_data),

      .forward_rs2_data(forward_rs2_data),
      .execute_rs2_data(execute_rs2_data),

      .decode_r_rd(decode_r_rd),
      .execute_rd (execute_rd),

      .decode_imm (decode_imm),
      .execute_imm(execute_imm),

      .decode_funct3 (decode_funct3),
      .execute_funct3(execute_funct3),

      .decode_opcode_type (decode_opcode_type),
      .execute_opcode_type(execute_opcode_type),

      .decode_exception (decode_exception),
      .execute_exception(execute_exception),

      .execute_result(execute_result),

      .decode_pc        (decode_pc),
      .execute_pc       (execute_pc),
      .execute_next_pc  (execute_next_pc),
      .execute_change_pc(execute_change_pc),

      .execute_rd_wr_en  (execute_rd_wr_en),
      .execute_rd_wr_data(execute_rd_wr_data),
      .execute_rd_valid  (execute_rd_valid),

      .stall_from_execute(stall_from_execute),
      .clk_en            (execute_clk_en),
      .next_clk_en       (memory_clk_en),
      .stall             (stall_execute),
      .force_stall       (force_stall_execute),
      .next_stall        (execute_stall),
      .flush             (memory_flush),
      .next_flush        (execute_flush)
  );
  // endregion R execute

  // region R memory
  //   wire [             31:0] execute_rs2_data;  // Data to be stored to memory is always rs2 data
  //   wire [             31:0] execute_result;  // Result from execute (load/store address)

  //   wire [              2:0] execute_funct3;  // funct3 from previous stage [STAGE 3 EXECUTE]
  wire [              2:0] memory_funct3;  // funct3 (byte, halfword, word store/load operation)

  //   wire [`OPCODE_WIDTH-1:0] execute_opcode_type;  // opcode type from prev [STAGE 3 EXECUTE]
  wire [`OPCODE_WIDTH-1:0] memory_opcode_type;  // opcode type

  //   wire [             31:0] execute_pc;  // pc from previous stage (execute)
  wire [             31:0] memory_pc;  // registered pc for this stage (memory)

  // region Base Registers control
  //   wire                     execute_rd_wr_en;  // enable write rd from previous stage (execute)
  wire                     memory_rd_wr_en;  // write rd to the base reg if enabled
  //   wire [              4:0] execute_rd;  // rd write address from previous stage (execute)
  wire [              4:0] memory_rd;  // address for destination register
  //   wire [             31:0] execute_rd_wr_data;  // rd write data from previous stage (execute)
  wire [             31:0] memory_rd_wr_data;  // data to be written back to destination register
  // endregion Base Registers control

  // region Data Memory control
  // bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
  //   wire                     main_memory_wb_cyc;
  //   wire                     main_memory_wb_stb;  // request for read/write access to data memory
  //   wire                     main_memory_wb_wr_en;  // write-enable (1 = write, 0 = read)
  //   wire [             31:0] main_memory_wb_addr;  // data memory address
  //   wire [             31:0] main_memory_wb_wr_data;  // data to be stored to memory
  // byte select for write {byte3, byte2, byte1, byte0}
  //   wire [              3:0] main_memory_wb_wr_sel;
  // ack by data memory (high when data to be read is ready or when write data is already written)
  //   wire                     main_memory_wb_ack;
  //   wire                     main_memory_wb_stall;  // stall by data memory (1 = data memory is busy)
  //   wire [             31:0] main_memory_wb_rd_data;  // data retrieve from data memory
  // endregion Data Memory control

  wire [             31:0] memory_data_load;  // data to be loaded to base reg (z-or-s extended)

  // region Pipeline control
  //   wire                     stall_from_execute;  // Execute tell to prepare to stall for load/store instruction
  //   wire      clk_en;              // control by previous stage [STAGE 3 EXECUTE]
  wire                     writeback_clk_en;  // clk enable for pipeline stalling
  wire                     stall_memory = (writeback_stall);  // stall this stage
  wire                     memory_stall;  // stalls the pipeline
  //   wire                     writeback_flush;  // flush this stage
  wire                     memory_flush;  // flushes previous stages
  // endregion Pipeline control

  memory memory_dut (
      .clk(clk),
      .rst(rst),

      .execute_rs2_data(execute_rs2_data),
      .execute_result  (execute_result),

      .execute_funct3(execute_funct3),
      .memory_funct3 (memory_funct3),

      .execute_opcode_type(execute_opcode_type),
      .memory_opcode_type (memory_opcode_type),

      .execute_pc(execute_pc),
      .memory_pc (memory_pc),

      .execute_rd_wr_en(execute_rd_wr_en),
      .memory_rd_wr_en (memory_rd_wr_en),

      .execute_rd(execute_rd),
      .memory_rd (memory_rd),

      .execute_rd_wr_data(execute_rd_wr_data),
      .memory_rd_wr_data (memory_rd_wr_data),

      .main_memory_wb_cyc    (main_memory_wb_cyc),
      .main_memory_wb_stb    (main_memory_wb_stb),
      .main_memory_wb_wr_en  (main_memory_wb_wr_en),
      .main_memory_wb_addr   (main_memory_wb_addr),
      .main_memory_wb_wr_data(main_memory_wb_wr_data),
      .main_memory_wb_wr_sel (main_memory_wb_wr_sel),
      .main_memory_wb_ack    (main_memory_wb_ack),
      .main_memory_wb_stall  (main_memory_wb_stall),
      .main_memory_wb_rd_data(main_memory_wb_rd_data),

      .memory_data_load(memory_data_load),

      .stall_from_execute(stall_from_execute),
      .clk_en            (memory_clk_en),
      .next_clk_en       (writeback_clk_en),
      .stall             (stall_memory),
      .next_stall        (memory_stall),
      .flush             (writeback_flush),
      .next_flush        (memory_flush)
  );
  // endregion R memory

  // region R writeback
  //   wire [              2:0] memory_funct3;      // funct3 (byte, halfword, word store/load operation)asmndmabsmdmnasbdmasmdmamnsdmnasmndmnanmsdmnaabsdmnbamsdbmabsmndban
  //   wire [             31:0] memory_data_load;   // data to be loaded to base reg (z-or-s extended)
  //   wire [`OPCODE_WIDTH-1:0] memory_opcode_type; // opcode type from previous stage (memory). Extract load or system opcode type to determine if this stage need to handle the instruction

  // region CRS Register operation
  wire [31:0] csr_data;  // CSR data to be loaded to base reg (ZICSR extension)
  // endregion CRS Register operation

  // region Base reg control
  //   wire             memory_rd_wr_en;      // enable write rd from previous stage (memory)
  wire        writeback_rd_wr_en;  // write rd to the base reg if enabled
  //   wire      [ 4:0] memory_rd;            // rd write address from previous stage (memory)
  wire [ 4:0] writeback_rd;  // address for destination register
  //   wire      [31:0] memory_rd_wr_data;    // rd write data from previous stage (memory)
  wire [31:0] writeback_rd_wr_data;  // data to be written back to destination register
  // endregion Base reg control

  // region PC control
  //   wire      [31:0] memory_pc;           // pc value from previous stage (memory)
  wire [31:0] writeback_pc;  // new pc value
  wire        writeback_change_pc;  // PC need to jump due to writeback
  // endregion PC control

  // region Trap handler from csr
  //   wire        go_to_trap;  // trap (exception/interrupt detected)
  //   wire        return_from_trap;  // high before returning from trap (via mret)
  //   wire [31:0] return_addr;  // mepc CSR
  //   wire [31:0] trap_addr;  // mtvec CSR
  // endregion Trap handler from csr


  // region Pipeline control
  //   wire        clk_en;  // control by previous stage (memory)
  wire        writeback_stall;  // stalls the pipeline
  wire        writeback_flush;  // flushes previous stages
  // endregion Pipeline control

  writeback writeback_dut (
      .memory_funct3     (memory_funct3),
      .memory_data_load  (memory_data_load),
      .memory_opcode_type(memory_opcode_type),

      .csr_data(csr_out),

      .memory_rd_wr_en   (memory_rd_wr_en),
      .writeback_rd_wr_en(writeback_rd_wr_en),

      .memory_rd   (memory_rd),
      .writeback_rd(writeback_rd),

      .memory_rd_wr_data   (memory_rd_wr_data),
      .writeback_rd_wr_data(writeback_rd_wr_data),

      .memory_pc          (memory_pc),
      .writeback_pc       (writeback_pc),
      .writeback_change_pc(writeback_change_pc),

      .go_to_trap      (go_to_trap),
      .return_from_trap(return_from_trap),
      .return_addr     (return_addr),
      .trap_addr       (trap_addr),

      .clk_en   (writeback_clk_en),
      .next_stall(writeback_stall),
      .next_flush(writeback_flush)
  );
  // endregion R writeback

  // region R forward
  // current rs1 datas read from regs
  //   wire [31:0] regs_rs1_rd_data;
  // current rs2 datas read from regs
  //   wire [31:0] regs_rs2_rd_data;

  // rs1 addresses decoded from decode module [Decode]
  input [4:0] decode_r_rs1;
  // rs2 addresses decoded from decode module [Decode]
  input [4:0] decode_r_rs2;

  // force execute module [Execute] to be stalled
  wire        force_stall_execute;

  // Forwarding rs1 data to [Execute]
  wire [31:0] forward_rs1_data;
  // Forwarding rs2 data to [Execute]
  wire [31:0] forward_rs2_data;

  // [Execute/Memory] forwarding
  input [4:0] execute_rd;  // rd address
  input execute_rd_wr_en;  // high if want to write to rd address
  input [31:0] execute_rd_wr_data;  // value to be written back to destination register
  input execute_rd_valid;  // high if rd is already valid (not LOAD nor CSR instructions)
  input memory_clk_en;  // memory_clk_en

  // [Memory/Writeback] forwarding
  input [4:0] memory_rd;  // rd address
  input memory_rd_wr_en;  // high if want to write to rd address
  input [31:0] writeback_rd_wr_data;  // value to be written back to destination register
  input writeback_clk_en;  // memory_clk_en

  forward forward_dut (
      .regs_rs1_rd_data(regs_rs1_rd_data),
      .regs_rs2_rd_data(regs_rs2_rd_data),

      .decode_r_rs1(decode_r_rs1),
      .decode_r_rs2(decode_r_rs2),

      .force_stall_execute(force_stall_execute),

      .forward_rs1_data(forward_rs1_data),
      .forward_rs2_data(forward_rs2_data),

      .execute_rd        (execute_rd),
      .execute_rd_wr_en  (execute_rd_wr_en),
      .execute_rd_wr_data(execute_rd_wr_data),
      .execute_rd_valid  (execute_rd_valid),
      .memory_clk_en     (memory_clk_en),

      .memory_rd           (memory_rd),
      .memory_rd_wr_en     (memory_rd_wr_en),
      .writeback_rd_wr_data(writeback_rd_wr_data),
      .writeback_clk_en    (writeback_clk_en)
  );

  // endregion R forward

  // region csr
  //   wire                     external_interrupt;
  //   wire                     software_interrupt;
  //   wire                     timer_interrupt;
  wire        is_illegal_instr = execute_exception[`ILLEGAL];
  wire        is_ecall_instr = execute_exception[`ECALL];
  wire        is_ebreak_instr = execute_exception[`EBREAK];
  wire        is_mret_instr = execute_exception[`MRET];
  //   wire [`OPCODE_WIDTH-1:0] execute_opcode_type;
  //   wire [             31:0] execute_result;
  //   wire [              2:0] execute_funct3;
  //   wire [             11:0] execute_imm;
  //   wire [              4:0] execute_rs1;
  //   wire [             31:0] execute_rs1_data;
  wire [31:0] csr_out;
  //   wire [             31:0] execute_pc;
  //   wire                     writeback_change_pc;
  wire [31:0] return_addr;
  wire [31:0] trap_addr;
  wire        go_to_trap;
  wire        return_from_trap;
  wire        minstret_inc = writeback_clk_en;
  wire        csr_clk_en = memory_clk_en;
  wire        stall_csr = (writeback_stall || memory_stall);

  csr #(
      .TRAP_ADDR(TRAP_ADDR)
  ) csr_dut (
      .clk(clk),
      .rst(rst),

      .external_interrupt(external_interrupt),
      .software_interrupt(software_interrupt),
      .timer_interrupt   (timer_interrupt),

      .is_illegal_instr(is_illegal_instr),
      .is_ecall_instr  (is_ecall_instr),
      .is_ebreak_instr (is_ebreak_instr),
      .is_mret_instr   (is_mret_instr),

      .execute_opcode_type(execute_opcode_type),
      .execute_result(execute_result),
      .execute_funct3(execute_funct3),
      .execute_imm(execute_imm),
      .execute_rs1(execute_rs1),
      .execute_rs1_data(execute_rs1_data),
      .csr_out(csr_out),
      .execute_pc(execute_pc),
      .writeback_change_pc(writeback_change_pc),
      .return_addr(return_addr),
      .trap_addr(trap_addr),
      .go_to_trap_q(go_to_trap),
      .return_from_trap_q(return_from_trap),
      .minstret_inc(minstret_inc),
      .clk_en(csr_clk_en),
      .stall(stall_csr)
  );

  // endregion csr


endmodule

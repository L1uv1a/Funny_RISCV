`timescale 1ns / 1ps
`define VCD_FILE "./vcds/decode_tb.vcd"
`define ICARUS_SIM

module decode_tb ();

  integer totals = 0;
  integer errors = 0;

  localparam PC_RESET = 0;

  localparam TEST_INSTRUCTION = 32'h09600113;

  parameter MEMORY_HEX = "";
  parameter MEMORY_BYTES = 1024 * 4;
  localparam ADDR_WIDTH = $clog2(MEMORY_BYTES);
  localparam MEMORY_DEPTH = MEMORY_BYTES / 4;

  initial begin
`ifdef ICARUS_SIM
    $dumpfile(`VCD_FILE);
    $dumpvars;
`endif
  end

  parameter CLK_PERIOD = 10;  // 100 MHz clk
  parameter CLK_PERIOD_HALF = CLK_PERIOD / 2;
  parameter CLK_PERIOD_QUAR = CLK_PERIOD / 4;
  always #(CLK_PERIOD_HALF) clk = !clk;

  initial begin
    clk                        <= 0;
    rst                        <= 0;
    writeback_change_pc        <= 0;
    writeback_next_pc          <= 0;
    execute_change_pc          <= 0;
    execute_next_pc            <= 0;
    main_memory_inst.memory[0] <= 'h402081b3;  // sub  x3, x1, x2      [RTYPE]
    main_memory_inst.memory[1] <= 'hf6a04113;  // xori x2, x0, -150    [ITYPE]
    main_memory_inst.memory[2] <= 'h00008103;  // lb   x2,  0(x1)      [LOAD]
    main_memory_inst.memory[3] <= 'hfe311e23;  // sh   x3, -4(x2)      [STORE]
    main_memory_inst.memory[4] <= 'hfe1158e3;  // bge  x2, x1, -16     [BRANCH]
    main_memory_inst.memory[5] <= 'h010000ef;  // jal  x1, 16          [JAL]
    main_memory_inst.memory[6] <= 'h018081e7;  // jalr x3, 24(x1)      [JALR]
    main_memory_inst.memory[7] <= 'habcde097;  // auipc x1, -344866    [AUIPC]
    main_memory_inst.memory[8] <= 'h30556473;  // csrrsi x8, mtvec, 10 [SYSTEM]
    main_memory_inst.memory[9] <= 'h0ff0000f;  // fence iorw, iorw     [FENCE]
    for (integer i = 10; i < MEMORY_DEPTH; i = i + 1) begin
      main_memory_inst.memory[i] <= 0;
    end
  end

  initial begin
    // Reset
    reset(1);
    // Test fetch, decode and change pc
    test_decode();
    $finish;
  end

  task automatic test_decode;
    begin
      instruction_decode(11);
      test_execute_change_pc('h18);
      instruction_decode(11);
      test_writeback_change_pc('h10);
      instruction_decode(11);
    end
  endtask  //automatic

  task automatic instruction_decode;
    input integer number_of_test;
    string opcode_type, alu_type;
    reg [31:0] previous_fetch_instr;
    begin
      repeat (number_of_test) begin
        // Fetch
        instruction_fetch(1, previous_fetch_instr);
`ifdef DETAILS
        $write(  //
            "[%sDECODE\033[00m] decode_pc = %4d, decode_instr = \033[96m%8h\033[00m",  //
            (|decode_opcode_type) ? "\033[92m" : "\033[91m",  //
            decode_pc, previous_fetch_instr  //
        );
        $write(  //
            "%s >>> [Info] stall_bit = %1b, decode_clk_en = %1b, execute_clk_en = %1b, stall_decode = %1b, decode_stall = %1b, flush_decode = %1b, decode_flush = %1b\033[00m\n",  //
            ((!decode_inst.stall_bit) && decode_clk_en && execute_clk_en && (!stall_decode) && (!decode_stall) && (!flush_decode) && (!decode_flush)) ? "\033[00m" : "\033[95m",  //
            decode_inst.stall_bit, decode_clk_en, execute_clk_en, stall_decode, decode_stall,
            flush_decode, decode_flush,  //
        );

        $display("%s", get_opcode_type(decode_opcode_type));
        get_decode_info();
        $display("\n");
`endif
      end
    end
  endtask  //automatic

  function string get_opcode_type;
    input [`OPCODE_WIDTH-1:0] decode_opcode_type;
    integer number_of_bit_1;
    begin
      number_of_bit_1 = 0;
      for (integer check_bit = 0; check_bit < `OPCODE_WIDTH; check_bit = check_bit + 1) begin
        if (decode_opcode_type[check_bit]) number_of_bit_1 = number_of_bit_1 + 1;
      end
      if (number_of_bit_1 > 1) begin
        get_opcode_type = "\033[91mERROR (DECODED MORE THAN 1 TYPE)\033[00m";
      end else if (decode_opcode_type[`RTYPE]) begin
        get_opcode_type = "\033[92mRTYPE\033[00m";
      end else if (decode_opcode_type[`ITYPE]) begin
        get_opcode_type = "\033[92mITYPE\033[00m";
      end else if (decode_opcode_type[`LOAD]) begin
        get_opcode_type = "\033[92mLOAD\033[00m";
      end else if (decode_opcode_type[`STORE]) begin
        get_opcode_type = "\033[92mSTORE\033[00m";
      end else if (decode_opcode_type[`BRANCH]) begin
        get_opcode_type = "\033[92mBRANCH\033[00m";
      end else if (decode_opcode_type[`JAL]) begin
        get_opcode_type = "\033[92mJAL\033[00m";
      end else if (decode_opcode_type[`JALR]) begin
        get_opcode_type = "\033[92mJALR\033[00m";
      end else if (decode_opcode_type[`LUI]) begin
        get_opcode_type = "\033[92mLUI\033[00m";
      end else if (decode_opcode_type[`AUIPC]) begin
        get_opcode_type = "\033[92mAUIPC\033[00m";
      end else if (decode_opcode_type[`SYSTEM]) begin
        get_opcode_type = "\033[92mSYSTEM\033[00m";
      end else if (decode_opcode_type[`FENCE]) begin
        get_opcode_type = "\033[92mFENCE\033[00m";
      end else begin
        get_opcode_type = "\033[91mILLEGAL (NO TYPE DECODED)\033[00m";
      end
    end
  endfunction


  function string get_alu_type;
    input [`ALU_WIDTH-1:0] decode_alu_type;
    input [2:0] decode_funct3;
    begin
      if (decode_alu_type[`ADD]) get_alu_type = "\033[92mADD\033[00m";
      else if (decode_alu_type[`SUB]) get_alu_type = "\033[92mSUB\033[00m";
      else if (decode_alu_type[`SLT]) get_alu_type = "\033[92mSLT\033[00m";
      else if (decode_alu_type[`SLTU]) get_alu_type = "\033[92mSLTU\033[00m";
      else if (decode_alu_type[`XOR]) get_alu_type = "\033[92mXOR\033[00m";
      else if (decode_alu_type[`OR]) get_alu_type = "\033[92mOR\033[00m";
      else if (decode_alu_type[`AND]) get_alu_type = "\033[92mAND\033[00m";
      else if (decode_alu_type[`SLL]) get_alu_type = "\033[92mSLL\033[00m";
      else if (decode_alu_type[`SRL]) get_alu_type = "\033[92mSRL\033[00m";
      else if (decode_alu_type[`SRA]) get_alu_type = "\033[92mSRA\033[00m";
      else if (decode_alu_type[`EQ]) get_alu_type = "\033[92mEQ\033[00m";
      else if (decode_alu_type[`NEQ]) get_alu_type = "\033[92mNEQ\033[00m";
      else if (decode_alu_type[`GE]) get_alu_type = "\033[92mGE\033[00m";
      else if (decode_alu_type[`GEU]) get_alu_type = "\033[92mGEU\033[00m";
      else get_alu_type = "\033[91mINVALID ALU \033[00m";
    end
  endfunction

  function string get_decode_info;
    begin
      // REG
      if (decode_opcode_type[`RTYPE]) begin
        if (decode_inst.funct7_bit6) $write("\033[92mSUB\033[00m");
        else $write("\033[92mADD\033[00m");
        $write(" x%1d, x%1d, x%1d", decode_r_rd, decode_r_rs1, decode_r_rs2);
        // IMM
      end else if (decode_opcode_type[`ITYPE]) begin
        $write("%s\033[92mI\033[00m x%1d, x%1d, %1d", get_alu_type(decode_alu_type, decode_funct3),
               decode_r_rd, decode_r_rs1, decode_imm);
        // LOAD
      end else if (decode_opcode_type[`LOAD]) begin
        if (decode_funct3 == `FUNCT3_LB) $write("\033[92mLB\033[00m");
        else if (decode_funct3 == `FUNCT3_LBU) $write("\033[92mLBU\033[00m");
        else if (decode_funct3 == `FUNCT3_LH) $write("\033[92mLH\033[00m");
        else if (decode_funct3 == `FUNCT3_LHU) $write("\033[92mLHU\033[00m");
        else if (decode_funct3 == `FUNCT3_LW) $write("\033[92mLW\033[00m");
        else $write("\033[91mInvalid\033[00m");
        $write(" x%1d, %1d(x%1d)", decode_r_rd, decode_imm, decode_r_rs1);
        // STORE
      end else if (decode_opcode_type[`STORE]) begin
        if (decode_funct3 == `FUNCT3_SB) $write("\033[92mSB\033[00m");
        else if (decode_funct3 == `FUNCT3_SH) $write("\033[92mSH\033[00m");
        else if (decode_funct3 == `FUNCT3_SW) $write("\033[92mSW\033[00m");
        else $write("\033[91mInvalid\033[00m");
        $write(" x%1d, %1d(x%1d)", decode_r_rs2, decode_imm, decode_r_rs1);
        // BRANCH
      end else if (decode_opcode_type[`BRANCH]) begin
        if (decode_funct3 == `FUNCT3_EQ) $write("\033[92mBEQ\033[00m");
        else if (decode_funct3 == `FUNCT3_NEQ) $write("\033[92mBNE\033[00m");
        else if (decode_funct3 == `FUNCT3_LT) $write("\033[92mBLT\033[00m");
        else if (decode_funct3 == `FUNCT3_GE) $write("\033[92mBGE\033[00m");
        else if (decode_funct3 == `FUNCT3_LTU) $write("\033[92mBLTU\033[00m");
        else if (decode_funct3 == `FUNCT3_GEU) $write("\033[92mBGEU\033[00m");
        else $write("\033[91mInvalid\033[00m");
        $write(" x%1d, x%1d, %1d", decode_r_rs1, decode_r_rs2, $signed(decode_imm));
        // JAL
      end else if (decode_opcode_type[`JAL]) begin
        $write("\033[92mJAL\033[00m x%1d, %1d", decode_r_rd, $signed(decode_imm));
        // JALR
      end else if (decode_opcode_type[`JALR]) begin
        $write("\033[92mJALR\033[00m x%1d, x%1d, %1d", decode_r_rd, decode_r_rs1, $signed(
                                                                                      decode_imm));
      end else
      // LUI
      if (decode_opcode_type[`LUI]) begin
        $write("\033[92mLUI\033[00m x%1d, %1d", decode_r_rd, $signed(decode_imm));
      end else
      // AUIPC
      if (decode_opcode_type[`AUIPC]) begin
        $write("\033[92mAUIPC\033[00m x%1d, %1d", decode_r_rd, $signed(decode_imm) >>> 12);
      end else
      // SYSTEM
      if (decode_opcode_type[`SYSTEM]) begin
        // TODO
      end else
      // FENCE
      if (decode_opcode_type[`FENCE]) begin
        // TODO
      end else begin
        // TODO
      end
    end
  endfunction

  task automatic instruction_fetch;
    input integer number_of_instructions;
    output [31:0] previous_fetch_instr;
    reg match;
    reg [31:0] instr_in_mem;
    begin
      @(negedge clk);
      repeat (number_of_instructions) begin
        previous_fetch_instr = fetch_instr;
        @(posedge clk);
        #1;  // Wait for signals to change
        instr_in_mem = main_memory_inst.memory[fetch_pc>>2];
        match = (fetch_instr == instr_in_mem);
`ifdef DETAILS
        $write(  //
            "[%-6s] fetch_pc  = %4d, instr_fetch  = \033[96m%8h\033[00m",  //
            (match) ? "\033[92mFETCH \033[00m" : "\033[91mFETCH \033[00m",  //
            fetch_pc, fetch_instr  //
        );
        $write(  //
            "%s >>> [Info] stall_bit = %1b, stall_fetch = %1b, decode_flush = %1b, decode_clk_en = %1b\033[00m\n",  //
            ((!fetch_inst.stall_bit) && (!stall_fetch) && (!decode_flush) && decode_clk_en) ? "\033[00m" : "\033[95m",  //
            fetch_inst.stall_bit, stall_fetch, decode_flush, decode_clk_en  //
        );
`endif
      end
    end
  endtask  //automatic

  task automatic test_execute_change_pc;
    input [31:0] next_pc;
    begin
      @(negedge clk);
      execute_change_pc <= 1;
      execute_next_pc   <= next_pc;
      @(posedge clk);
      #(CLK_PERIOD_QUAR);
      execute_change_pc <= 0;
`ifdef DETAILS
      $display(  //
          "\033[94m>>> fetch_pc changed to %1d due to Execute\033[00m",  //
          next_pc  //
      );
`endif
    end
  endtask  //automatic

  task automatic test_writeback_change_pc;
    input [31:0] next_pc;
    begin
      @(negedge clk);
      writeback_change_pc <= 1;
      writeback_next_pc   <= next_pc;
      @(posedge clk);
      #(CLK_PERIOD_QUAR);
      writeback_change_pc <= 0;
`ifdef DETAILS
      $display(  //
          "\033[94m>>> fetch_pc changed to %1d due to Writeback\033[00m",  //
          next_pc  //
      );
`endif
    end
  endtask  //automatic

  task automatic reset;
    input integer clk_period;
    begin
      @(negedge clk);
      rst <= 1;
      repeat (clk_period) @(posedge clk);
      @(negedge clk);
      rst <= 0;
`ifdef DETAILS
      $display("\033[94m>>> Reset released...\033[00m");
`endif
    end
  endtask  //automatic

  reg                         clk;
  reg                         rst;

  wire [                31:0] decode_pc;
  wire [                 4:0] decode_rs1;
  wire [                 4:0] decode_r_rs1;
  wire [                 4:0] decode_rs2;
  wire [                 4:0] decode_r_rs2;
  wire [                 4:0] decode_r_rd;
  wire [                31:0] decode_imm;
  wire [                 2:0] decode_funct3;
  wire [      `ALU_WIDTH-1:0] decode_alu_type;
  wire [   `OPCODE_WIDTH-1:0] decode_opcode_type;
  wire [`EXCEPTION_WIDTH-1:0] decode_exception;

  wire                        execute_clk_en;
  wire                        stall_decode = 0;
  wire                        decode_stall;
  wire                        flush_decode = 0;
  wire                        decode_flush;


  decode decode_inst (
      .clk(clk),
      .rst(rst),

      .fetch_instr(fetch_instr),
      .fetch_pc(fetch_pc),

      .decode_pc   (decode_pc),
      .decode_rs1  (decode_rs1),
      .decode_r_rs1(decode_r_rs1),
      .decode_rs2  (decode_rs2),
      .decode_r_rs2(decode_r_rs2),
      .decode_r_rd (decode_r_rd),
      .decode_imm  (decode_imm),

      .decode_funct3     (decode_funct3),
      .decode_alu_type   (decode_alu_type),
      .decode_opcode_type(decode_opcode_type),
      .decode_exception  (decode_exception),

      .clk_en     (decode_clk_en),
      .next_clk_en(execute_clk_en),
      .stall      (stall_decode),
      .next_stall (decode_stall),
      .flush      (flush_decode),
      .next_flush (decode_flush)
  );

  wire [31:0] main_memory_instr_addr;
  wire [31:0] main_memory_instr;
  wire        main_memory_instr_req;
  wire        main_memory_instr_ack;

  wire [31:0] fetch_instr;

  wire [31:0] fetch_pc;

  reg         writeback_change_pc;
  reg  [31:0] writeback_next_pc;

  reg         execute_change_pc;
  reg  [31:0] execute_next_pc;

  wire        stall_fetch = decode_stall;
  wire        decode_clk_en;


  fetch #(
      .PC_RESET(PC_RESET)
  ) fetch_inst (
      .clk(clk),
      .rst(rst),

      .main_memory_instr_addr(main_memory_instr_addr),
      .main_memory_instr     (main_memory_instr),
      .main_memory_instr_req (main_memory_instr_req),
      .main_memory_instr_ack (main_memory_instr_ack),

      .fetch_instr(fetch_instr),

      .pc(fetch_pc),

      .writeback_change_pc(writeback_change_pc),
      .writeback_next_pc  (writeback_next_pc),

      .execute_change_pc(execute_change_pc),
      .execute_next_pc  (execute_next_pc),

      .stall      (stall_fetch),
      .flush      (decode_flush),
      .next_clk_en(decode_clk_en)
  );


  reg         wb_cyc = 0;
  reg         wb_stb = 0;
  reg         wb_wr_en = 0;
  reg  [31:0] wb_addr = 0;
  reg  [31:0] wb_wr_data = 0;
  reg  [ 3:0] wb_wr_sel = 0;
  wire        wb_ack;
  wire        wb_stall;
  wire [31:0] wb_rd_data;

  main_memory #(
      .MEMORY_HEX  (MEMORY_HEX),
      .MEMORY_BYTES(MEMORY_BYTES)
  ) main_memory_inst (
      .clk(clk),

      .instr_addr(main_memory_instr_addr),
      .instr     (main_memory_instr),
      .instr_stb (main_memory_instr_req),
      .instr_ack (main_memory_instr_ack),

      .wb_cyc    (wb_cyc),
      .wb_stb    (wb_stb),
      .wb_wr_en  (wb_wr_en),
      .wb_addr   (wb_addr),
      .wb_wr_data(wb_wr_data),
      .wb_wr_sel (wb_wr_sel),
      .wb_ack    (wb_ack),
      .wb_stall  (wb_stall),
      .wb_rd_data(wb_rd_data)
  );
endmodule

`include "rv32i_header.vh"

module execute (
    input clk,
    input rst,

    input [`ALU_WIDTH-1:0] decode_alu_type,

    input      [4:0] decode_r_rs1,
    output reg [4:0] execute_rs1,

    input      [31:0] forward_rs1_data,
    output reg [31:0] execute_rs1_data,

    input      [31:0] forward_rs2_data,
    output reg [31:0] execute_rs2_data,

    input      [4:0] decode_r_rd,
    output reg [4:0] execute_rd,

    input      [31:0] decode_imm,
    output reg [11:0] execute_imm,

    input      [2:0] decode_funct3,
    output reg [2:0] execute_funct3,

    input      [`OPCODE_WIDTH-1:0] decode_opcode_type,
    output reg [`OPCODE_WIDTH-1:0] execute_opcode_type,

    input      [`EXCEPTION_WIDTH-1:0] decode_exception,
    output reg [`EXCEPTION_WIDTH-1:0] execute_exception,

    output reg [31:0] execute_result,  // alu operation result

    // region PC control
    input      [31:0] decode_pc,         // pc
    output reg [31:0] execute_pc,        // pc in pipeline
    output reg [31:0] execute_next_pc,   // new pc
    output reg        execute_change_pc, // need to jump
    // endregion PC control

    // region base registers control
    output reg        execute_rd_wr_en,
    output reg [31:0] execute_rd_wr_data,
    output reg        execute_rd_valid,
    // endregion base registers control

    // region Pipeline control
    output reg stall_from_execute,  // stall next stage ([STAGE 4 MEMORY] for load/store instructions)
    input clk_en,  // control by previous stage ([STAGE 2 DECODE])
    output reg next_clk_en,  // clk enable for pipeline stalling of next state ([STAGE 4 MEMORY])
    input stall,  // stall this stage
    input force_stall,  // force this stage to stall
    output reg next_stall,  // stalls the pipeline
    input flush,  // flush this stage
    output reg next_flush  // flushes previous stages
    // endregion Pipeline control
);

  // region alu operation
  wire        alu_add = decode_alu_type[`ADD];
  wire        alu_sub = decode_alu_type[`SUB];
  wire        alu_slt = decode_alu_type[`SLT];
  wire        alu_sltu = decode_alu_type[`SLTU];
  wire        alu_xor = decode_alu_type[`XOR];
  wire        alu_or = decode_alu_type[`OR];
  wire        alu_and = decode_alu_type[`AND];
  wire        alu_sll = decode_alu_type[`SLL];
  wire        alu_srl = decode_alu_type[`SRL];
  wire        alu_sra = decode_alu_type[`SRA];
  wire        alu_eq = decode_alu_type[`EQ];
  wire        alu_neq = decode_alu_type[`NEQ];
  wire        alu_ge = decode_alu_type[`GE];
  wire        alu_geu = decode_alu_type[`GEU];
  // endregion alu operation

  // region opcode type
  wire        opcode_rtype = decode_opcode_type[`RTYPE];
  wire        opcode_itype = decode_opcode_type[`ITYPE];
  wire        opcode_load = decode_opcode_type[`LOAD];
  wire        opcode_store = decode_opcode_type[`STORE];
  wire        opcode_branch = decode_opcode_type[`BRANCH];
  wire        opcode_jal = decode_opcode_type[`JAL];
  wire        opcode_jalr = decode_opcode_type[`JALR];
  wire        opcode_lui = decode_opcode_type[`LUI];
  wire        opcode_auipc = decode_opcode_type[`AUIPC];
  wire        opcode_system = decode_opcode_type[`SYSTEM];
  wire        opcode_fence = decode_opcode_type[`FENCE];
  // endregion opcode type

  // region alu registered values
  reg  [31:0] op_a;  // operand a
  reg  [31:0] op_b;  // operand b
  reg  [31:0] result;  // result

  reg         rd_wr_en;  // enable writeback rd data to base registers (x) at address rd.
  reg  [31:0] rd_wr_data;  // next value to be written back to rd address
  reg         rd_valid;  // writeback rd data is valid (not Load or CSR instructions)

  reg  [31:0] base_pc;  // pc to jump to (stored in rs1)
  wire [31:0] base_pc_plus_imm = base_pc + decode_imm;  // pc in rs1 + offset value in imm
  wire [31:0] pc_plus_4 = decode_pc + 4;  // default pc + 4

  // stall when this stage is stalled or pipeline need to be stalled
  wire        stall_bit = (stall || next_stall);

  always @(posedge clk, posedge rst) begin : update_registers
    if (rst) begin
      execute_opcode_type <= 0;
      execute_exception   <= 0;
      execute_result      <= 0;
      execute_rs1         <= 0;
      execute_rs1_data    <= 0;
      execute_rs2_data    <= 0;
      execute_rd          <= 0;
      execute_imm         <= 0;
      execute_funct3      <= 0;
      execute_rd_wr_en    <= 0;
      execute_rd_wr_data  <= 0;
      execute_rd_valid    <= 0;
      stall_from_execute  <= 0;
      execute_pc          <= 0;
    end  // update registers only if this stage is enable and pipeline is not stalled
    else if ((!stall_bit) && clk_en) begin
      execute_opcode_type <= decode_opcode_type;
      execute_exception   <= decode_exception;
      execute_result      <= result;
      execute_rs1         <= decode_r_rs1;
      execute_rs1_data    <= forward_rs1_data;
      execute_rs2_data    <= forward_rs2_data;
      execute_rd          <= decode_r_rd;
      execute_imm         <= decode_imm[11:0];
      execute_funct3      <= decode_funct3;
      execute_rd_wr_en    <= rd_wr_en;
      execute_rd_wr_data  <= rd_wr_data;
      execute_rd_valid    <= rd_valid;

      stall_from_execute  <= (decode_opcode_type[`STORE] || decode_opcode_type[`LOAD]);
      execute_pc          <= decode_pc;
    end
  end
  // endregion alu registered values

  // region update stages control
  always @(posedge clk, posedge rst) begin : update_stage_control
    if (rst) next_clk_en <= 0;
    else begin
      // flush this stage also disable next stage
      if ((!stall_bit) && flush) next_clk_en <= 0;
      // update next stage enable/disable only when not stalled
      else if ((!stall_bit)) next_clk_en <= clk_en;
      // if this stage is stalled but not by forced stall, disable next stage
      else if (stall_bit && (!stall)) next_clk_en <= 0;
    end
  end
  // endregion update stages control

  // region alu operations
  wire [31:0] result_add = op_a + op_b;
  wire [31:0] result_sub = op_a - op_b;
  wire [31:0] result_xor = op_a ^ op_b;
  wire [31:0] result_or = op_a | op_b;
  wire [31:0] result_slt = ($signed(op_a) < $signed(op_b));
  wire [31:0] result_sltu = (op_a < op_b);
  wire [31:0] result_and = (op_a & op_b);
  wire [31:0] result_sll = (op_a << op_b[4:0]);
  wire [31:0] result_srl = (op_a >> op_b[4:0]);
  wire [31:0] result_sra = ($signed(op_a) >>> op_b[4:0]);
  wire [31:0] result_eq = (op_a == op_b);
  wire [31:0] result_neq = (op_a != op_b);
  wire [31:0] result_ge = ($signed(op_a) >= $signed(op_b));
  wire [31:0] result_geu = (op_a >= op_b);

  always_comb begin : compute_result
    result = 0;
    op_a   = (opcode_jal || opcode_auipc) ?  // Operand A can either be PC or RS1
 decode_pc : forward_rs1_data;

    op_b   = (opcode_rtype || opcode_branch) ?  // Operand B can either be RS2 or Imm
 forward_rs2_data : decode_imm;

    if (alu_add) result = result_add;
    if (alu_sub) result = result_sub;
    if (alu_slt) result = result_slt;
    if (alu_sltu) result = result_sltu;
    if (alu_xor) result = result_xor;
    if (alu_or) result = result_or;
    if (alu_and) result = result_and;
    if (alu_sll) result = result_sll;
    if (alu_srl) result = result_srl;
    if (alu_sra) result = result_sra;
    if (alu_eq) result = result_eq;
    if (alu_neq) result = result_neq;
    if (alu_ge) result = result_ge;
    if (alu_geu) result = result_geu;
  end
  // endregion alu operations

  // region compute writeback rd and next pc
  `define TRUE 1
  `define FALSE 0
  always_comb begin : compute_writeback_rd_and_next_pc
    // default values
    // flush this stage along with previous stages
    next_flush = flush;
    rd_wr_data = 0;
    execute_change_pc = 0;
    execute_next_pc = 0;
    base_pc = decode_pc;

    if (!flush) begin
      if (opcode_rtype || opcode_itype) rd_wr_data = result;

      // branch if result (op_a and op_b comparision) is true
      if (opcode_branch && (result == `TRUE)) begin
        execute_next_pc = base_pc_plus_imm;
        // change pc if this stage is enable
        execute_change_pc = clk_en;  // change pc is valid
        next_flush = clk_en;
      end

      if (opcode_jal || opcode_jalr) begin
        if (opcode_jalr) base_pc = forward_rs1_data;
        execute_next_pc = base_pc_plus_imm;
        // change pc if this stage is enable
        execute_change_pc = clk_en;  // change pc is valid
        next_flush = clk_en;
        // store next pc to rd
        rd_wr_data = pc_plus_4;
      end
    end

    if (opcode_lui) rd_wr_data = decode_imm;

    if (opcode_auipc) rd_wr_data = base_pc_plus_imm;
  end
  // endregion compute writeback rd and next pc

  // region check
  /*Disable write to rd when enocunter those instruction types:
  - Non-CSR System
  - Branch
  - Store
  - Fence
  */
  wire disable_write_rd = (  // check logic for disable write to rd
  // funct3 == 0 are the non-csr system instructions
  (opcode_system && (decode_funct3 == 0)) ||  // non-csr system instruction
  opcode_branch ||  // branch
  opcode_store ||  // store
  opcode_fence  // fence
  );

  always_comb begin : check_exception_wrire_rd
    // always write to the destination reg except when instruction is BRANCH or STORE or SYSTEM (except CSR system instruction)
    rd_wr_en = disable_write_rd ? 0 : 1;
  end

  always_comb begin : check_valid
    // value of rd for load and CSR write is not yet available at this stage
    rd_valid = (opcode_load || (opcode_system && (decode_funct3 != 0))) ? 0 : 1;
  end

  always_comb begin : check_stall
    // stall logic (stall when upper stages are stalled, when forced to stall, or when needs to flush previous stages but are still stalled)
    next_stall = ((!flush) && (stall || force_stall));  // stall when alu need wait time
  end
  // endregion check

endmodule

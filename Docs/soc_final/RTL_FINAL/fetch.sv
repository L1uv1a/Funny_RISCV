/* retrieves instruction from the memory [FETCH STAGE]
*/
module fetch #(
    parameter PC_RESET = 0,
    parameter int unsigned NUM_REQS = 2
) (
    input clk,
    input rstn,

    output logic [31:0] pc,         // PC value of current instruction
    output reg [31:0] instr_send, // instruction sent to pipeline

    output logic        instr_req_o,    // req
    input  logic        instr_gnt_i,    // gnt
    output logic [31:0] instr_addr_o,   // addr to get instr
    input  logic [31:0] instr_rdata_i,  // rdata
    input  logic        instr_err_i,    // err    // fix NO USE
    input  logic        instr_rvalid_i, // valid  // fix NO USE

    // PC control
    input        writeback_change_pc,  // high when PC needs to change (trap/return from trap)
    input [31:0] writeback_next_pc,    // next PC due to trap
    input        alu_change_pc,        // high when PC needs to change (branch/jump)
    input [31:0] alu_next_pc,          // next PC due to branch/jump

    // Pipeline control
    output reg clk_en,  // output clk enable for pipeline stalling of next state
    input      stall,   // stall logic for whole pipeline
    input      flush    // flush this stage
);
  localparam int unsigned DEPTH = NUM_REQS + 1;

  wire        instr_ack;  // high if new instruction is now on the bus
  logic       instr_req;

  reg         r_clk_en;
  reg         r_clk_en_d;
  reg         stall_fetch;
  reg         stall_q;  

  logic [DEPTH-1:0] [31:0] rdata_d,      rdata_q;
  reg   [DEPTH-1:0] [31:0] instr_addr_d, instr_addr_q; // fifo data in wire (d) and register (q)
  logic [DEPTH-1:0]        valid_pushed, valid_popped;
  logic [DEPTH-1:0]        occupied_d,   occupied_q; // check if this depth occupied or not 
  logic [DEPTH-1:0]        entry_en;
  logic [DEPTH-1:0]        lowest_free_entry;
  logic             [31:0] rdata, rdata_unaligned;
  logic             [31:0] hold_next_addr;
  logic                    aligned_is_compressed, unaligned_is_compressed;
  logic             [31:0] instr_addr_next;

  assign instr_ack = instr_gnt_i;
  assign instr_req_o = instr_req;
  assign instr_addr_o = instr_addr_q[0];
  assign rdata = occupied_q[0] ? rdata_q[0] : instr_rdata_i;

  /*
                                           WIDTH = 32
                         >------------------------------------------<
                         | 31               16 | 15               0 |  v               
  Back  --> FIFO entry 2 | Instr 3 [15:0]      | Instr 3 [15:0]     |  |       
            FIFO entry 1 | Instr 2 [15:0]      | Instr 1 [31:16]    |  |  DEPTH
  Front --> FIFO entry 0 | Instr 1 [15:0]      | Instr 0 [31:16]    |  |       
                                                                       ^       

  In queue, data move from back (depth 2) to front (depth 0), instr_addr_q are wires that feed data to the queue, 
  and transmit data from depth i to depth i + 1. Entry are write pointer to write to each depth of fifo, it is only possible
  to write new data to depth i if entry i exist
  */

  /////////////////////
  // FIFO management //
  /////////////////////

  assign pop_fifo = (~aligned_is_compressed | pc[1]);   // if there is a 16 bit instruction at 1st half, or a 32 bit instruction then fifo ready for pop

  for (genvar i = 0; i < DEPTH - 1; i++) begin : g_fifo_next
    // Calculate lowest free entry (write pointer)
    if (i == 0) begin : g_ent0
      assign lowest_free_entry[i] = ~occupied_q[i];                   // depth = 0 is already lowest depth                
    end else begin : g_ent_others
      assign lowest_free_entry[i] = ~occupied_q[i] & occupied_q[i-1]; // if this depth is not occupied and the lower depth is occupied, then this stage is the lowest entry 
    end

    // An entry is set when an incoming request chooses the lowest available entry
    assign valid_pushed[i] = lowest_free_entry[i] |                             // check if it is valid for data to be pushed to lower depth
                             occupied_q[i];
    // Popping the FIFO shifts all entries down
    assign valid_popped[i] = pop_fifo ? valid_pushed[i+1] : valid_pushed[i];    // check if data successfully removed from higher depth (and received as this depth)

    /* Basically, valid_popped it just an intermediate logic to track data transmit between depth levels. We can just removed it and valid_pushed still does the trick*/

    // All entries are wiped out on a clear
    assign occupied_d[i] = valid_popped[i] & ~flush;                          // check if data occupied in this depth or not

    // data flops are enabled if there is new data to shift into it, or
    assign entry_en[i] = (valid_pushed[i+1] & pop_fifo) | 
                         // a new request is incoming and this is the lowest free entry
                         (lowest_free_entry[i] & ~pop_fifo);

    // take the next entry or the incoming data
    assign rdata_d[i]  = occupied_q[i+1] ? rdata_q[i+1] : instr_rdata_i; // if higher depth is occupied, shift down. Otherwise get new data
  end

    // The top entry is similar but with simpler muxing
  assign lowest_free_entry[DEPTH-1] = ~occupied_q[DEPTH-1] & occupied_q[DEPTH-2];
  assign valid_pushed     [DEPTH-1] = occupied_q[DEPTH-1] | lowest_free_entry[DEPTH-1];
  assign valid_popped     [DEPTH-1] = pop_fifo ? 1'b0 : valid_pushed[DEPTH-1];
  assign occupied_d [DEPTH-1]       = valid_popped[DEPTH-1] & ~flush;
  assign entry_en[DEPTH-1]          = lowest_free_entry[DEPTH-1];
  assign rdata_d [DEPTH-1]          = instr_rdata_i;

  ///////////////////////////
  // Construct output data //
  ///////////////////////////

  // Construct the output data for an unaligned instruction
  assign rdata_unaligned = occupied_q[1] ? {rdata_q[1][15:0], rdata[31:16]} :
                                        {instr_rdata_i[15:0], rdata[31:16]};
  // An uncompressed unaligned instruction is only valid if both parts are available
  assign valid_unaligned = occupied_q[1] ? 1'b1 :
                                        (occupied_q[0]);

  // If there is an error, rdata is unknown
  assign unaligned_is_compressed = rdata[17:16] != 2'b11;  // check if 2nd half is a compress instruction (by checking opcode)
  assign aligned_is_compressed   = rdata[ 1: 0] != 2'b11;  // check if 1st half is a compress instruction (by checking opcode)

  // Instruction aligner (if unaligned)
  always_comb begin
    if (pc[1]) begin
      // unaligned case
      instr_send    = rdata_unaligned;
    end else begin
      // aligned case
      instr_send    = rdata;
    end
  end

  //////////////////////////////////////////////
  // Calculate address of current instruction //
  //////////////////////////////////////////////

  // Update the address on branches and every time an instruction is driven



  // Increment the address by two every time a compressed instruction is popped
  assign addr_incr_two = instr_addr_q[1] ? unaligned_is_compressed :
                                           aligned_is_compressed; 

  assign instr_addr_next [31:1] = (instr_addr_q [0] [31:1] +
                            // Increment address by 4 or 2
                            {29'd0,~addr_incr_two,addr_incr_two});
  assign instr_addr_next [0:0] = 1'b0;

  assign instr_addr_d = hold_next_addr;

  assign pc[31:0]      =  instr_addr_q [0] [31:0];

  ////////////////////
  // FIFO registers //
  ////////////////////
  

  /* Stall conditions
  stall this stage when:
  - next stages are stalled
  - request but no ack yet
  - no request at all (no instruction to execute for this stage)
  */
  assign instr_req = r_clk_en;  // request for new instruction if this stage is enabled
  
  wire        stall_bit = (stall_fetch ||  // stall fetch
  stall ||  // stall
  (instr_req && !instr_ack) ||  // request but no ack
  !instr_req  // no request
  );
  
  // clk enable logic for fetch stage
  wire disable_next_stage = ((alu_change_pc || writeback_change_pc) && !(stall || stall_fetch));
  always @(posedge clk, negedge rstn) begin
    if (!rstn) r_clk_en <= 1;
    // do pipeline bubble when need to change PC so that next stage will be disable
    // and will not execute the instructions already inside the pipeline
    else
      r_clk_en <= !disable_next_stage;
  end

  wire enable_update_registers = ((!stall_bit && r_clk_en) ||  //
  (stall_bit && !clk_en && r_clk_en) ||  //
  (writeback_change_pc)  //
  );

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      occupied_q               <= '0;
      instr_addr_q[0]          <= PC_RESET;
      instr_addr_q[1]          <= '0;
      instr_addr_q[2]          <= '0;
      rdata_q                  <= '0;
      hold_next_addr           <= PC_RESET;
    end else begin
      if (enable_update_registers) begin 
        occupied_q <= stall_bit ? occupied_q : occupied_d;
        instr_addr_q <= instr_addr_d;
        hold_next_addr <= instr_addr_next;
      end
    end
    if (writeback_change_pc) begin
      occupied_q               <= '0;
      instr_addr_q[0]          <= writeback_next_pc;
      instr_addr_q[1]          <= '0;
      instr_addr_q[2]          <= '0;
      rdata_q                  <= '0;
      hold_next_addr           <= writeback_next_pc;
    end else if (alu_change_pc) begin
      occupied_q               <= '0;
      instr_addr_q[0]          <= alu_next_pc;
      instr_addr_q[1]          <= '0;
      instr_addr_q[2]          <= '0;
      rdata_q                  <= '0;
      hold_next_addr <= alu_next_pc;
    end 
  end
  
  for (genvar i = 0; i < DEPTH; i++) begin : g_fifo_regs
    begin : g_rdata_nr
      always_ff @(posedge clk) begin
        if (entry_en[i]) begin
          if (enable_update_registers) rdata_q[i] <= stall_bit ? rdata_q[i] : rdata_d[i];
        end
      end
    end
  end

// PC and pipeline clk enable control logic

  always @* begin
    r_clk_en_d   = 0;
    stall_fetch  = stall;  // stall when retrieving instructions need wait time prepare next PC when changing PC, then do a pipeline bubble to disable the ce of next stage
    if (writeback_change_pc) begin
      r_clk_en_d   = 0;
    end else if (alu_change_pc) begin
      r_clk_en_d   = 0;
    end else begin
      r_clk_en_d   = r_clk_en;
    end
  end

  /* Update registers conditions
  update registers only if this stage is enable and next stages are not stalled
  */
  
  always @(posedge clk, negedge rstn) begin
    if (!rstn) begin
      clk_en              <= 0;
    end else begin
      if (!stall_bit && flush) clk_en <= 0;
      //clock-enable will change only when not stalled
      else if (!stall_bit) clk_en <= r_clk_en_d;
      //if this stage is stalled but next stage is not, disable clock enable of next stage at next clock cycle (pipeline bubble)
      else if (stall_bit && !stall) clk_en <= 0;
      // raise stall when any of 5 stages is stalled
      stall_q <= (stall || stall_fetch);

    end
  end


  


endmodule

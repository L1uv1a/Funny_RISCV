// region Description

//! ### Reading instructions from memory
//! When reading from memory, ignore the last 2 bits of the address because memory is accessed by word.
//! 
//! ``` verilog
//!   always @(posedge clk) begin : sync_read_process
//!
//!    instr_ack  <= instr_stb;  // read ack go high next cycle after receiving stb (request)
//!
//!    instr      <= memory[instr_addr>>2];  // read instruction
//!
//!  end
//! ```
//! In the example below, passing instr_addr = h'4 (*b'100*) or instr_addr = h'5 (*b'101*) accesses the same memory slot at h'4 (*b'100*).

//! {
//!   signal: [
//!     {name: "clk", wave: "P......", period: 1},
//!     [
//!       "handshake",
//!       {name: "instr_stb", wave: "0.1..0."},
//!       {name: "instr_ack", wave: "0..1..0"}
//!     ],
//!     {},
//!     [
//!       "access",
//!       {name: "instr_addr [ADDR_WIDTH-1:0]", wave: "x.555x.", data: ["h'30", "h'34", "h'35"]},
//!       {name: "instr [31:0]",                wave: "x..55.x", data: ["h'ab", "h'cd"]}
//!     ],
//!     {},
//!   ],
//!   head: {
//!     text: "read instruction",
//!     tick: 0
//!   },
//!   config: { hscale: 1}
//! }

//! ###### Data in memory:
//! {reg: [
//!   {bits: 1, name: "Value",   attr: [". . .", "h'000000ab", "h'000000cd", ". . ."]},
//!   {bits: 1, name: "Address", attr: [". . .", "h'00000030", "h'00000034", ". . ."]},
//! ], config:{bits: 2}}

//! ### Reading data from memory

//! When reading from memory, ignore the last 2 bits of the address because memory is accessed by word.
//!
//! **wb_cyc**: bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
//!
//! **wb_stb**: request for read/write access to data memory
//!
//! **wb_ack**: only when ack = 1, **wb_rd_data** is valid

//! ``` verilog
//!   always @(posedge clk) begin : sync_read_process
//!
//!    wb_ack     <= (wb_stb && wb_cyc);  // read ack go high next cycle after receiving stb (request)
//!
//!    wb_rd_data <= memory[wb_addr>>2];  // read data
//!
//!  end
//! ```
//! In the example below, the **wb_rd_data** at **clk** = 3, 6, 7 is valid, any other data changes should be ignore.

//! {
//!   signal: [
//!     {name: "clk", wave: "P...........", period: 1},
//!     [
//!       "handshake",
//!       {name: "wb_cyc", wave: "0.1.....0..."},
//!       {name: "wb_stb", wave: "0.10.1.0.10."},
//!       {name: "wb_ack", wave: "0..10.1.0..."}
//!     ],
//!     {},
//!     [
//!       "access",
//!       {name: "wb_addr [ADDR_WIDTH-1:0]", wave: "x5...5555...", data: ["h'30", "h'34", "h'38", "h'30", "h'34"]},
//!       {name: "wb_rd_data [31:0]",        wave: "x.5...5555..", data: ["h'ab", "h'cd", "h'ef", "h'ab", "h'cd"]}
//!     ],
//!     {},
//!   ],
//!   head: {
//!     text: "read data",
//!     tick: 0
//!   },
//!   config: { hscale: 1}
//! }

//! ###### Data in memory:
//! {reg: [
//!   {bits: 1, name: "Value",   attr: [". . .", "h'000000ab", "h'000000cd", "h'000000ef", ". . ."]},
//!   {bits: 1, name: "Address", attr: [". . .", "h'00000030", "h'00000034", "h'00000038", ". . ."]},
//! ], config:{bits: 2}}

//! ### Writing data to memory
//! When writing to memory, ignore the last 2 bits of the address because memory is accessed by word.
//!
//! **wb_wr_en**: write enable (wb_wr_en = 1: write, wb_wr_en = 0: read)
//!
//! **wb_cyc**: bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
//!
//! **wb_stb**: request for read/write access to data memory
//!
//! **wb_addr**: address to write
//!
//! **wb_wr_data**: data need to be write
//!
//! **wb_wr_sel**: select which byte(s) in **wb_wr_data** need to be stored at the coresponding byte slots at the address

//!  ``` verilog
//!   always @(posedge clk) begin : sync_write_process
//!
//!    if (wb_wr_en && wb_stb && wb_cyc) begin
//!
//!      if (wb_wr_sel[0]) memory[wb_addr>>2][7:0] <= wb_wr_data[7:0];
//!
//!      if (wb_wr_sel[1]) memory[wb_addr>>2][15:8] <= wb_wr_data[15:8];
//!
//!      if (wb_wr_sel[2]) memory[wb_addr>>2][23:16] <= wb_wr_data[23:16];
//!
//!      if (wb_wr_sel[3]) memory[wb_addr>>2][31:24] <= wb_wr_data[31:24];
//!
//!    end
//!
//!  end
//!  ```

//! {
//!   signal: [
//!     {name: "clk", wave: "P..........", period: 1},
//!     [
//!       "handshake",
//!       {name: "wb_wr_en", wave: "0.10.1.0.10"},
//!       {name: "wb_cyc",   wave: "0.1.....0.."},
//!       {name: "wb_stb",   wave: "0.10.1.0.10"},
//!     ],
//!     {},
//!     [
//!       "access",
//!       {name: "wb_addr [ADDR_WIDTH-1:0]", wave: "x.5..55..5.", data: ["h'30", "h'35", "h'3A", "h'30", "h'34"]},
//!       {name: "wb_wr_sel [3:0]",         wave: "x.5.5.5...x", data: ["b'0100", "b'1100", "b'1111", "b'0010"]},
//!       {name: "wb_wr_data [31:0]",        wave: "x.5..5...5.", data: ["h'abcdef01", "h'12345678", "h'ef120021", "h'ab", "h'cd"]},
//!       {name: "info",                     wave: "d..7d.77d..", data: ["[1]", "[2]", "[3]", "[4]", "[5]"]},
//!     ],
//!     {},
//!   ],
//!   head: {
//!     text: "write data",
//!     tick: 0
//!   },
//!   config: { hscale: 1}
//! }

//! [1]: Write h'--cd---- to h'30 (h'cd to h'32)
//!
//! [2]: Write h'1234---- to h'34 (h'34 to h'36, h'12 to h'37)
//!
//! [3]: Write h'12345678 to h'38

//! ###### Data in memory:
//! {reg: [
//!   {bits: 1, name: "Value",   attr: [". . .", "h'00cd0000", "h'12340000", "h'12345678", ". . ."]},
//!   {bits: 1, name: "Address", attr: [". . .", "h'00000030", "h'00000034", "h'00000038", ". . ."]},
//! ], config:{bits: 2}}


// endregion Description



module main_memory #(
    parameter MEMORY_HEX   = "",   //! Hex file to load into memory
    parameter MEMORY_BYTES = 1024  //! Number of bytes in memory
) (
    input clk,  //! positive edge triggered system clock

    // region control by [STAGE 1 FETCH]
    // Instruction Memory
    input      [ADDR_WIDTH-1:0] instr_addr,  //! instruction memory address
    output reg [          31:0] instr,       //! instruction from memory
    input                       instr_stb,   //! request for instruction
    output reg                  instr_ack,   //! read ack
    // endregion control by [STAGE 1 FETCH]

    // region control by [STAGE 4 MEMORY]
    // Data Memory
    //! bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
    input                       wb_cyc,
    //! request for read/write access to data memory
    input                       wb_stb,
    //! write enable (wb_wr_en = 1: write, wb_wr_en = 0: read)
    input                       wb_wr_en,
    input      [ADDR_WIDTH-1:0] wb_addr,     //! memory address
    input      [          31:0] wb_wr_data,  //! data to write to memory
    input      [           3:0] wb_wr_sel,   //! mask to write data to memory
    output reg                  wb_ack,      //! read ack
    output                      wb_stall,
    output reg [          31:0] wb_rd_data   //! data read from memory
    // endregion control by [STAGE 4 MEMORY]
);
  localparam ADDR_WIDTH = $clog2(MEMORY_BYTES);  //! Address bits
  localparam MEMORY_DEPTH = MEMORY_BYTES / 4;  //! Memory slots
  reg [31:0] memory[0:MEMORY_DEPTH-1];  //! Storing memory

  assign wb_stall = 0;  // never stall wb

  initial begin
    instr_ack  <= 0;
    wb_ack     <= 0;
    instr      <= 0;
    wb_rd_data <= 0;
    if (MEMORY_HEX != "") begin
      $readmemh(MEMORY_HEX, memory);
      $display("Init Memory from %s", MEMORY_HEX);
    end else $display("No Memory Init");
  end

  //! syn read with clk
  always @(posedge clk) begin : sync_read_process
    instr_ack  <= instr_stb;  // ack go high next cycle after receiving stb (request)
    instr      <= memory[instr_addr>>2];  // read instruction

    wb_ack     <= (wb_stb && wb_cyc);  // ack go high next cycle after receiving stb (request)
    wb_rd_data <= memory[wb_addr>>2];  // read data
  end

  //! syn write with clk
  always @(posedge clk) begin : sync_write_process
    if (wb_wr_en && wb_stb && wb_cyc) begin
      if (wb_wr_sel[0]) memory[wb_addr>>2][7:0] <= wb_wr_data[7:0];
      if (wb_wr_sel[1]) memory[wb_addr>>2][15:8] <= wb_wr_data[15:8];
      if (wb_wr_sel[2]) memory[wb_addr>>2][23:16] <= wb_wr_data[23:16];
      if (wb_wr_sel[3]) memory[wb_addr>>2][31:24] <= wb_wr_data[31:24];
    end
  end

endmodule

## Names should be descriptive and avoid abbreviations:
Assume that, we have a module mydesign and want to create an instance of it in module top
```
mydesign mydesign_inst ([port_list]); // is an ok name
```
```
mydesign mydesign_i ([port_list]); // can be messed with input signal named "my_design", otherwise somewhat acceptable
```
```
mydesign d0 ([port_list]); // not descriptive, not a suitable name
```
## Non-ASCII characters are forbidden
## Indentation uses spaces, no tabs. Indentation is two spaces for nesting, four spaces for line continuation
```
always_comb begin
  unique case (1'b1) // indent 2 space
    // case list // another 2 space, 4 total
    default: ;
  endcase
end
```
## Place a space between if and the parenthesis in conditional expressions.
```
if (condition) begin // space between "if" and "(condition)"
  // do sth
end
```
## Use horizontal whitespace around operators, and avoid trailing whitespace at the end of lines
```
input a;  // ok
input b ; // trailing white space
```
```
assign a = b + c; // ok
assign a=b+c; // operator without white space around
```
## Maintain consistent and good punctuation, spelling, and grammar (within comments)
## Signals must be declared before use, and all should stay in a code block with command:
OK:
```
///////////
// Adder //
///////////

logic        adder_op_a_shift1;
logic        adder_op_a_shift2;
... // other signals

// prepare operand a
always_comb begin
  unique case (1'b1)
    // case list here
  endcase
end
... // other blocks here
```
Messed order:
```
///////////
// Adder //
///////////

// prepare operand a
always_comb begin
  unique case (1'b1)
    // case list here
  endcase
end
... // other blocks here
logic        adder_op_a_shift1;
logic        adder_op_a_shift2;

... // other signals
```
Not in a same block: 
```
///////////
// Adder //
///////////

logic        adder_op_a_shift1;
logic        adder_op_a_shift2;
... // other signals

............................ // Many line between declare and usage

// prepare operand a
always_comb begin
  unique case (1'b1)
    // case list here
  endcase
end
... // other blocks here
```
## Data type, width and names should stay in the same column:
This is ok:
```
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
```
This is wrong format:
```
  //wires for rv32i_decoder
  wire [`ALU_WIDTH-1:0] decoder_alu;
  wire [`OPCODE_WIDTH-1:0] decoder_opcode;
  wire [31:0] decoder_pc;
  wire [4:0] decoder_rs1_addr, decoder_rs2_addr;
  wire [4:0] decoder_rs1_addr_q, decoder_rs2_addr_q;
  wire [4:0] decoder_rd_addr;
  wire [31:0] decoder_imm;
  wire [2:0] decoder_funct3;
  wire [`EXCEPTION_WIDTH-1:0] decoder_exception;
  wire decoder_ce;
  wire decoder_flush;
```

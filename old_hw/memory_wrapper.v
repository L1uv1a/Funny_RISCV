module memory_wrapper (  //decodes address and access the corresponding memory-mapped device
    //RISC-V Core
    input wire i_wb_cyc,
    input wire i_wb_stb,
    input wire i_wb_we,
    input wire [31:0] i_wb_addr,
    input wire [31:0] i_wb_data,
    input wire [3:0] i_wb_sel,
    output reg o_wb_ack,
    output reg o_wb_stall,
    output reg [31:0] o_wb_data,

    //Device 0 Interface (RAM)
    output reg o_device0_wb_cyc,
    output reg o_device0_wb_stb,
    output reg o_device0_wb_we,
    output reg [31:0] o_device0_wb_addr,
    output reg [31:0] o_device0_wb_data,
    output reg [3:0] o_device0_wb_sel,
    input wire i_device0_wb_ack,
    input wire i_device0_wb_stall,
    input wire [31:0] i_device0_wb_data
);


  always @* begin
    o_wb_ack = 0;
    o_wb_stall = 0;
    o_wb_data = 0;

    o_device0_wb_cyc = 0;
    o_device0_wb_stb = 0;
    o_device0_wb_we = 0;
    o_device0_wb_addr = 0;
    o_device0_wb_data = 0;
    o_device0_wb_sel = 0;

    // Access RAM
    o_device0_wb_cyc = i_wb_cyc;
    o_device0_wb_stb = i_wb_stb;
    o_device0_wb_we = i_wb_we;
    o_device0_wb_addr = i_wb_addr;
    o_device0_wb_data = i_wb_data;
    o_device0_wb_sel = i_wb_sel;
    o_wb_ack = i_device0_wb_ack;
    o_wb_stall = i_device0_wb_stall;
    o_wb_data = i_device0_wb_data;
  end
endmodule

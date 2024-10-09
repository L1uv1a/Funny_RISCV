// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// The Ibex demo system, which instantiates and connects the following blocks:
// - Memory bus.
// - Ibex top module.
// - RAM memory to contain code and data.
// - GPIO driving logic.
// - UART for serial communication.
// - Timer.
// - Debug module.
// - SPI for driving LCD screen.
module ibex_top_myself #(
  parameter int                 GpiWidth       = 8,
  parameter int                 GpoWidth       = 16,
  parameter int                 PwmWidth       = 12,
  parameter int unsigned        ClockFrequency = 50_000_000,
  parameter int unsigned        BaudRate       = 115_200,
  //parameter ibex_pkg::regfile_e RegFile        = ibex_pkg::RegFileFPGA,
  parameter                     SRAMInitFile   = ""
) (
  input  logic clk_sys_i,
  input  logic rst_sys_ni,

  input  logic [GpiWidth-1:0] gp_i,
  output logic [GpoWidth-1:0] gp_o,
  output logic [PwmWidth-1:0] pwm_o
  // input  logic                uart_rx_i,
  // output logic                uart_tx_o,
  // input  logic                spi_rx_i,
  // output logic                spi_tx_o,
  // output logic                spi_sck_o

  // input  logic        tck_i,    // JTAG test clock pad
  // input  logic        tms_i,    // JTAG test mode select pad
  // input  logic        trst_ni,  // JTAG test reset pad
  // input  logic        td_i,     // JTAG test data input pad
  // output logic        td_o      // JTAG test data output pad
);
  localparam logic [31:0] MEM_SIZE      = 64 * 1024; // 64 KiB
  localparam logic [31:0] MEM_START     = 32'h00100000;
  localparam logic [31:0] MEM_MASK      = ~(MEM_SIZE-1);

  localparam logic [31:0] GPIO_SIZE     =  4 * 1024; //  4 KiB
  localparam logic [31:0] GPIO_START    = 32'h80000000;
  localparam logic [31:0] GPIO_MASK     = ~(GPIO_SIZE-1);

  localparam logic [31:0] DEBUG_SIZE    = 64 * 1024; // 64 KiB
  localparam logic [31:0] DEBUG_START   = 32'h1a110000;
  localparam logic [31:0] DEBUG_MASK    = ~(DEBUG_SIZE-1);

  localparam logic [31:0] UART_SIZE     =  4 * 1024; //  4 KiB
  localparam logic [31:0] UART_START    = 32'h80001000;
  localparam logic [31:0] UART_MASK     = ~(UART_SIZE-1);

  localparam logic [31:0] TIMER_SIZE    =  4 * 1024; //  4 KiB
  localparam logic [31:0] TIMER_START   = 32'h80002000;
  localparam logic [31:0] TIMER_MASK    = ~(TIMER_SIZE-1);

  localparam logic [31:0] PWM_SIZE      =  4 * 1024; //  4 KiB
  localparam logic [31:0] PWM_START     = 32'h80003000;
  localparam logic [31:0] PWM_MASK      = ~(PWM_SIZE-1);
  localparam int PwmCtrSize = 8;

  parameter logic [31:0] SPI_SIZE       =  1 * 1024; //  1 KiB
  parameter logic [31:0] SPI_START      = 32'h80004000;
  parameter logic [31:0] SPI_MASK       = ~(SPI_SIZE-1);

  parameter logic [31:0] SIM_CTRL_SIZE  =  1 * 1024; //  1 KiB
  parameter logic [31:0] SIM_CTRL_START = 32'h20000;
  parameter logic [31:0] SIM_CTRL_MASK  = ~(SIM_CTRL_SIZE-1);

  // Debug functionality is optional.
  localparam bit DBG = 0;
  localparam int unsigned DbgHwBreakNum = (DBG == 1) ?    2 :    0;
  localparam bit          DbgTriggerEn  = (DBG == 1) ? 1'b1 : 1'b0;

  typedef enum int {
    CoreD,
    DbgHost
  } bus_host_e;

  typedef enum int {
    Ram,
    Gpio,
    Pwm,
    Uart,
    Timer,
    Spi,
    SimCtrl,
    DbgDev
  } bus_device_e;

  localparam int NrDevices = DBG ? 8 : 7;
  localparam int NrHosts   = DBG ? 2 : 1;

  // Interrupts.
  logic timer_irq;
  logic uart_irq;

  // Host signals.
  logic        host_req      [NrHosts];
  logic        host_gnt      [NrHosts];
  logic [31:0] host_addr     [NrHosts];
  logic        host_we       [NrHosts];
  logic [ 3:0] host_be       [NrHosts];
  logic [31:0] host_wdata    [NrHosts];
  logic        host_rvalid   [NrHosts];
  logic [31:0] host_rdata    [NrHosts];
  logic        host_err      [NrHosts];

  // Device signals.
  logic        device_req    [NrDevices];
  logic [31:0] device_addr   [NrDevices];
  logic        device_we     [NrDevices];
  logic [ 3:0] device_be     [NrDevices];
  logic [31:0] device_wdata  [NrDevices];
  logic        device_rvalid [NrDevices];
  logic [31:0] device_rdata  [NrDevices];
  logic        device_err    [NrDevices];

  // Instruction fetch signals.
  logic        core_instr_req;
  logic        core_instr_gnt;
  logic        core_instr_rvalid;
  logic [31:0] core_instr_addr;
  logic [31:0] core_instr_rdata;
  logic        core_instr_sel_dbg;

  logic        mem_instr_req;
  logic [31:0] mem_instr_rdata;
  logic        dbg_instr_req;

  logic        dbg_device_req;
  logic [31:0] dbg_device_addr;
  logic        dbg_device_we;
  logic [ 3:0] dbg_device_be;
  logic [31:0] dbg_device_wdata;
  logic        dbg_device_rvalid;
  logic [31:0] dbg_device_rdata;

  // Internally generated resets cause IMPERFECTSCH warnings
  /* verilator lint_off IMPERFECTSCH */
  logic rst_core_n;
  logic ndmreset_req;
  //logic dm_debug_req;

  // Device address mapping.
  logic [31:0] cfg_device_addr_base [NrDevices];
  logic [31:0] cfg_device_addr_mask [NrDevices];

  assign cfg_device_addr_base[Ram]     = MEM_START;
  assign cfg_device_addr_mask[Ram]     = MEM_MASK;
  assign cfg_device_addr_base[Gpio]    = GPIO_START;
  assign cfg_device_addr_mask[Gpio]    = GPIO_MASK;
  assign cfg_device_addr_base[Pwm]     = PWM_START;
  assign cfg_device_addr_mask[Pwm]     = PWM_MASK;
  assign cfg_device_addr_base[Uart]    = UART_START;
  assign cfg_device_addr_mask[Uart]    = UART_MASK;
  assign cfg_device_addr_base[Timer]   = TIMER_START;
  assign cfg_device_addr_mask[Timer]   = TIMER_MASK;
  assign cfg_device_addr_base[Spi]     = SPI_START;
  assign cfg_device_addr_mask[Spi]     = SPI_MASK;
  assign cfg_device_addr_base[SimCtrl] = SIM_CTRL_START;
  assign cfg_device_addr_mask[SimCtrl] = SIM_CTRL_MASK;

  // if (DBG) begin : g_dbg_device_cfg
  //   assign cfg_device_addr_base[DbgDev] = DEBUG_START;
  //   assign cfg_device_addr_mask[DbgDev] = DEBUG_MASK;
  //   assign device_err[DbgDev] = 1'b0;
  // end

  // Tie-off unused error signals.
  assign device_err[Ram]     = 1'b0;
  assign device_err[Gpio]    = 1'b0;
  assign device_err[Pwm]     = 1'b0;
  assign device_err[Uart]    = 1'b0;
  assign device_err[Spi]     = 1'b0;
  assign device_err[SimCtrl] = 1'b0;

  bus #(
    .NrDevices    ( NrDevices ),
    .NrHosts      ( NrHosts   ),
    .DataWidth    ( 32        ),
    .AddressWidth ( 32        )
  ) u_bus (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .host_req_i   (host_req     ),
    .host_gnt_o   (host_gnt     ),
    .host_addr_i  (host_addr    ),
    .host_we_i    (host_we      ),
    .host_be_i    (host_be      ),
    .host_wdata_i (host_wdata   ),
    .host_rvalid_o(host_rvalid  ),
    .host_rdata_o (host_rdata   ),
    .host_err_o   (host_err     ),

    .device_req_o   (device_req   ),
    .device_addr_o  (device_addr  ),
    .device_we_o    (device_we    ),
    .device_be_o    (device_be    ),
    .device_wdata_o (device_wdata ),
    .device_rvalid_i(device_rvalid),
    .device_rdata_i (device_rdata ),
    .device_err_i   (device_err   ),

    .cfg_device_addr_base,
    .cfg_device_addr_mask
  );

  assign mem_instr_req =
      core_instr_req & ((core_instr_addr & cfg_device_addr_mask[Ram]) == cfg_device_addr_base[Ram]);

  assign dbg_instr_req =
      core_instr_req & ((core_instr_addr & cfg_device_addr_mask[DbgDev]) == cfg_device_addr_base[DbgDev]);

  assign core_instr_gnt = mem_instr_req | (dbg_instr_req & ~device_req[DbgDev]);

  always @(posedge clk_sys_i or negedge rst_sys_ni) begin
    if (!rst_sys_ni) begin
      core_instr_rvalid  <= 1'b0;
      core_instr_sel_dbg <= 1'b0;
    end else begin
      core_instr_rvalid  <= core_instr_gnt;
      core_instr_sel_dbg <= dbg_instr_req;
    end
  end

  // assign core_instr_rdata = core_instr_sel_dbg ? dbg_device_rdata : mem_instr_rdata;
  assign core_instr_rdata = 1'b0 ? dbg_device_rdata : mem_instr_rdata;

  assign rst_core_n = rst_sys_ni;
  // & ~ndmreset_req;

    core #(
      .PC_RESET    (32'h00100080),
      .TRAP_ADDRESS(32'b0)
  ) core_inst (  //main RV32I core
      .i_clk  (clk_sys_i),
      .i_rst_n(rst_core_n),

      //Instruction Memory Interface
      .i_instr    (core_instr_rdata),        //32-bit instruction
      .o_iaddr   (core_instr_addr),       //address of instruction 
      .o_stb_instr(core_instr_req),  //request for read access to instruction memory
      .i_ack_instr(core_instr_rvalid),  //ack (high if new instruction is ready)

      //Data Memory Interface
      //bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
      .o_wb_stb_data  (host_req[CoreD]),     //request for read/write access to data memory
      .o_wb_we   (host_we[CoreD]),      //write-enable (1 = write, 0 = read)
      .o_wb_addr_data (host_addr[CoreD]),    //address of data memory for store/load
      .o_wb_data_data (host_wdata[CoreD]),  //data to be stored to memory
      //byte strobe for write (1 = write the byte) {byte3,byte2,byte1,byte0}
      .o_wb_sel_data  (host_be[CoreD]),
      //ack by data memory (high when read data is ready or when write data is already written)
      .i_wb_ack  (host_rvalid[CoreD]),
      //.i_wb_stall_data(1'b0),   //stall by data memory
      .i_wb_data_data (host_rdata[CoreD]),  //data retrieved from memory

      //Interrupts
      //interrupt from external source
      .i_external_interrupt(1'b0),
      //interrupt from software (inter-processor interrupt)
      .i_software_interrupt(1'b0),
      //interrupt from timer
      .i_timer_interrupt   (timer_irq)
  );

  ram_2p #(
      .Depth       ( MEM_SIZE / 4 ),
      .MemInitFile ( SRAMInitFile )
  ) u_ram (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .a_req_i   (device_req[Ram]),
    .a_we_i    (device_we[Ram]),
    .a_be_i    (device_be[Ram]),
    .a_addr_i  (device_addr[Ram]),
    .a_wdata_i (device_wdata[Ram]),
    .a_rvalid_o(device_rvalid[Ram]),
    .a_rdata_o (device_rdata[Ram]),

    .b_req_i   (mem_instr_req),
    .b_we_i    (1'b0),
    .b_be_i    (4'b0),
    .b_addr_i  (core_instr_addr),
    .b_wdata_i (32'b0),
    .b_rvalid_o(),
    .b_rdata_o (mem_instr_rdata)
  );

  gpio #(
    .GpiWidth ( GpiWidth ),
    .GpoWidth ( GpoWidth )
  ) u_gpio (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .device_req_i   (device_req[Gpio]),
    .device_addr_i  (device_addr[Gpio]),
    .device_we_i    (device_we[Gpio]),
    .device_be_i    (device_be[Gpio]),
    .device_wdata_i (device_wdata[Gpio]),
    .device_rvalid_o(device_rvalid[Gpio]),
    .device_rdata_o (device_rdata[Gpio]),

    .gp_i,
    .gp_o
  );

  pwm_wrapper #(
    .PwmWidth     ( PwmWidth   ),
    .PwmCtrSize   ( PwmCtrSize ),
    .BusAddrWidth ( 32         )
  ) u_pwm (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .device_req_i   (device_req[Pwm]),
    .device_addr_i  (device_addr[Pwm]),
    .device_we_i    (device_we[Pwm]),
    .device_be_i    (device_be[Pwm]),
    .device_wdata_i (device_wdata[Pwm]),
    .device_rvalid_o(device_rvalid[Pwm]),
    .device_rdata_o (device_rdata[Pwm]),

    .pwm_o
  );

  // uart #(
  //   .ClockFrequency ( ClockFrequency ),
  //   .BaudRate       ( BaudRate       )
  // ) u_uart (
  //   .clk_i (clk_sys_i),
  //   .rst_ni(rst_sys_ni),

  //   .device_req_i   (device_req[Uart]),
  //   .device_addr_i  (device_addr[Uart]),
  //   .device_we_i    (device_we[Uart]),
  //   .device_be_i    (device_be[Uart]),
  //   .device_wdata_i (device_wdata[Uart]),
  //   .device_rvalid_o(device_rvalid[Uart]),
  //   .device_rdata_o (device_rdata[Uart]),

  //   .uart_rx_i,
  //   .uart_irq_o     (uart_irq),
  //   .uart_tx_o
  // );

  // spi_top #(
  //   .ClockFrequency ( ClockFrequency ),
  //   .CPOL           ( 0          ),
  //   .CPHA           ( 1          )
  // ) u_spi (
  //   .clk_i (clk_sys_i),
  //   .rst_ni(rst_sys_ni),

  //   .device_req_i   (device_req[Spi]),
  //   .device_addr_i  (device_addr[Spi]),
  //   .device_we_i    (device_we[Spi]),
  //   .device_be_i    (device_be[Spi]),
  //   .device_wdata_i (device_wdata[Spi]),
  //   .device_rvalid_o(device_rvalid[Spi]),
  //   .device_rdata_o (device_rdata[Spi]),

  //   .spi_rx_i(spi_rx_i), // Data received from SPI device.
  //   .spi_tx_o(spi_tx_o), // Data transmitted to SPI device.
  //   .sck_o   (spi_sck_o), // Serial clock pin.

  //   .byte_data_o() // Unused.
  // );

  // `ifdef VERILATOR
  //   simulator_ctrl #(
  //     .LogName ( "ibex_demo_system.log" )
  //   ) u_simulator_ctrl (
  //     .clk_i (clk_sys_i),
  //     .rst_ni(rst_sys_ni),

  //     .req_i   (device_req[SimCtrl]),
  //     .we_i    (device_we[SimCtrl]),
  //     .be_i    (device_be[SimCtrl]),
  //     .addr_i  (device_addr[SimCtrl]),
  //     .wdata_i (device_wdata[SimCtrl]),
  //     .rvalid_o(device_rvalid[SimCtrl]),
  //     .rdata_o (device_rdata[SimCtrl])
  //   );
  // `endif

  // timer #(
  //   .DataWidth    ( 32 ),
  //   .AddressWidth ( 32 )
  // ) u_timer (
  //   .clk_i (clk_sys_i),
  //   .rst_ni(rst_sys_ni),

  //   .timer_req_i   (device_req[Timer]),
  //   .timer_we_i    (device_we[Timer]),
  //   .timer_be_i    (device_be[Timer]),
  //   .timer_addr_i  (device_addr[Timer]),
  //   .timer_wdata_i (device_wdata[Timer]),
  //   .timer_rvalid_o(device_rvalid[Timer]),
  //   .timer_rdata_o (device_rdata[Timer]),
  //   .timer_err_o   (device_err[Timer]),
  //   .timer_intr_o  (timer_irq)
  // );

  // assign dbg_device_req        = device_req[DbgDev] | dbg_instr_req;
  // assign dbg_device_we         = device_req[DbgDev] & device_we[DbgDev];
  // assign dbg_device_addr       = device_req[DbgDev] ? device_addr[DbgDev] : core_instr_addr;
  // assign dbg_device_be         = device_be[DbgDev];
  // assign dbg_device_wdata      = device_wdata[DbgDev];
  // assign device_rvalid[DbgDev] = dbg_device_rvalid;
  // assign device_rdata[DbgDev]  = dbg_device_rdata;

  // always @(posedge clk_sys_i or negedge rst_sys_ni) begin
  //   if (!rst_sys_ni) begin
  //     dbg_device_rvalid <= 1'b0;
  //   end else begin
  //     dbg_device_rvalid <= device_req[DbgDev];
  //   end
  // end

  // if (DBG) begin : gen_dm_top
  //   dm_top #(
  //     .NrHarts      ( 1                              ),
  //     .IdcodeValue  ( jtag_id_pkg::RV_DM_JTAG_IDCODE )
  //   ) u_dm_top (
  //     .clk_i        (clk_sys_i),
  //     .rst_ni       (rst_sys_ni),
  //     .testmode_i   (1'b0),
  //     .ndmreset_o   (ndmreset_req),
  //     .dmactive_o   (),
  //     .debug_req_o  (), //dm_debug_req
  //     .unavailable_i(1'b0),

  //     // Bus device with debug memory (for execution-based debug).
  //     .device_req_i  (dbg_device_req),
  //     .device_we_i   (dbg_device_we),
  //     .device_addr_i (dbg_device_addr),
  //     .device_be_i   (dbg_device_be),
  //     .device_wdata_i(dbg_device_wdata),
  //     .device_rdata_o(dbg_device_rdata),

  //     // Bus host (for system bus accesses, SBA).
  //     .host_req_o    (host_req[DbgHost]),
  //     .host_add_o    (host_addr[DbgHost]),
  //     .host_we_o     (host_we[DbgHost]),
  //     .host_wdata_o  (host_wdata[DbgHost]),
  //     .host_be_o     (host_be[DbgHost]),
  //     .host_gnt_i    (host_gnt[DbgHost]),
  //     .host_r_valid_i(host_rvalid[DbgHost]),
  //     .host_r_rdata_i(host_rdata[DbgHost]),

  //     .tck_i,
  //     .tms_i,
  //     .trst_ni,
  //     .td_i,
  //     .td_o
  //   );
  // end else begin : gen_no_dm
  //   assign dm_debug_req = 1'b0;
  //   assign ndmreset_req = 1'b0;
  // end

  // `ifdef VERILATOR

  //   export "DPI-C" function mhpmcounter_num;

  //   function automatic int unsigned mhpmcounter_num();
  //     return u_top.u_ibex_core.cs_registers_i.MHPMCounterNum;
  //   endfunction

  //   export "DPI-C" function mhpmcounter_get;

  //   function automatic longint unsigned mhpmcounter_get(int index);
  //     return u_top.u_ibex_core.cs_registers_i.mhpmcounter[index];
  //   endfunction
  // `endif
endmodule

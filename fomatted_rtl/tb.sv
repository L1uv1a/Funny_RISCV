`timescale 1ns / 1ns
`include "rv32i_header.svh"
module testbench (
);
    reg clk_i, rstn_i;
    always #5 clk_i=!clk_i; //100MHz clock
    initial begin 
        $readmemh("/home/khai/Funny_RISCV/old_hw/hexs/sw.hex",u_ibex_top.u_ram.u_ram.u_impl_generic.mem, 32); //write instruction and data to memory
        //u_ibex_top.main_memory_inst.memory_regfile[{32'h0000_1000>>2}] = 32'h12345678; //initial data memory 
    end
    // initial begin
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[32] = 32'h123450b7; //lui x1, 74565
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[33] = 32'h67808093; //addi x1, x1, 1656
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[34] = 32'h00001137; //lui x2, 1
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[36] = 32'h00112023; //sw x1, 0(x2)
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[35] = 32'h00810113; //addi x2, x2, 8
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[37] = 32'h00000013;
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[38] = 32'h00000013;
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[39] = 32'h00000013;
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[40] = 32'h00000013;
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[41] = 32'h00000013;
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[42] = 32'h00000013;
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[43] = 32'h00000013;
    //   u_ibex_top.u_ram.u_ram.u_impl_generic.mem[44] = 32'h0000005d;
    // end
    initial begin //2nd reset to test resetting while core is executing instruction
      $dumpfile("/home/khai/test_core/test.vcd");
      $dumpvars(0,testbench);
        clk_i = 0;
        #20;
        rstn_i = 0;
        #50;
        rstn_i = 1;
    end  

    ibex_top_myself #(
    .GpiWidth     ( 8            ),
    .GpoWidth     ( 8            ),
    .PwmWidth     ( 12           ),
    .SRAMInitFile ( "" )
    ) u_ibex_top (
    //Input
    .clk_sys_i (clk_i),
    .rst_sys_ni(rstn_i),
    .gp_i('b0),
    .gp_o(),
    .pwm_o()
  );

  
    always @(posedge clk_i) begin
        if(u_ibex_top.core_inst.m0.x[17] == 32'h5d) begin //Exit test using RISC-V International's riscv-tests pass/fail criteria
                if(u_ibex_top.core_inst.m0.x[10] == 0)
                    $display("\n\033[92mPASS\033[00m: exit code = 0x%h \n[%0d instructions in %0d clk cycles]\n",u_ibex_top.core_inst.m0.x[10]>>1,u_ibex_top.core_inst.zicsr.m6.minstret,u_ibex_top.core_inst.zicsr.m6.mcycle);
                else begin
                    $display("\n\033[91mFAIL\033[00m: exit code = 0x%h \n[%0d instructions in %0d clk cycles]\n",u_ibex_top.core_inst.m0.x[10]>>1,u_ibex_top.core_inst.zicsr.m6.minstret,u_ibex_top.core_inst.zicsr.m6.mcycle);
                end
                $display("u_ibex_top.core_inst.m0.x[0] = %h", u_ibex_top.core_inst.m0.x[0]);
                $display("u_ibex_top.core_inst.m0.x[1] = %h", u_ibex_top.core_inst.m0.x[1]);
                $display("u_ibex_top.core_inst.m0.x[2] = %h", u_ibex_top.core_inst.m0.x[2]);
                $display("u_ibex_top.core_inst.m0.x[3] = %h", u_ibex_top.core_inst.m0.x[3]);
                $display("u_ibex_top.core_inst.m0.x[4] = %h", u_ibex_top.core_inst.m0.x[4]);
                $display("u_ibex_top.core_inst.m0.x[5] = %h", u_ibex_top.core_inst.m0.x[5]);
                $display("u_ibex_top.core_inst.m0.x[6] = %h", u_ibex_top.core_inst.m0.x[6]);
                $display("u_ibex_top.core_inst.m0.x[7] = %h", u_ibex_top.core_inst.m0.x[7]);
                $display("u_ibex_top.core_inst.m0.x[8] = %h", u_ibex_top.core_inst.m0.x[8]);
                $display("u_ibex_top.core_inst.m0.x[9] = %h", u_ibex_top.core_inst.m0.x[9]);
                $display("u_ibex_top.core_inst.m0.x[10] = %h", u_ibex_top.core_inst.m0.x[10]);
                $display("u_ibex_top.core_inst.m0.x[11] = %h", u_ibex_top.core_inst.m0.x[11]);
                $display("u_ibex_top.core_inst.m0.x[12] = %h", u_ibex_top.core_inst.m0.x[12]);
                $display("u_ibex_top.core_inst.m0.x[13] = %h", u_ibex_top.core_inst.m0.x[13]);
                $display("u_ibex_top.core_inst.m0.x[14] = %h", u_ibex_top.core_inst.m0.x[14]);
                $display("u_ibex_top.core_inst.m0.x[15] = %h", u_ibex_top.core_inst.m0.x[15]);
                $display("u_ibex_top.core_inst.m0.x[16] = %h", u_ibex_top.core_inst.m0.x[16]);
                $display("u_ibex_top.core_inst.m0.x[17] = %h", u_ibex_top.core_inst.m0.x[17]);
                $display("u_ibex_top.core_inst.m0.x[18] = %h", u_ibex_top.core_inst.m0.x[18]);
                $display("u_ibex_top.core_inst.m0.x[19] = %h", u_ibex_top.core_inst.m0.x[19]);
                $display("u_ibex_top.core_inst.m0.x[20] = %h", u_ibex_top.core_inst.m0.x[20]);
                $display("u_ibex_top.core_inst.m0.x[21] = %h", u_ibex_top.core_inst.m0.x[21]);
                $display("u_ibex_top.core_inst.m0.x[22] = %h", u_ibex_top.core_inst.m0.x[22]);
                $display("u_ibex_top.core_inst.m0.x[23] = %h", u_ibex_top.core_inst.m0.x[23]);
                $display("u_ibex_top.core_inst.m0.x[24] = %h", u_ibex_top.core_inst.m0.x[24]);
                $display("u_ibex_top.core_inst.m0.x[25] = %h", u_ibex_top.core_inst.m0.x[25]);
                $display("u_ibex_top.core_inst.m0.x[26] = %h", u_ibex_top.core_inst.m0.x[26]);
                $display("u_ibex_top.core_inst.m0.x[27] = %h", u_ibex_top.core_inst.m0.x[27]);
                $display("u_ibex_top.core_inst.m0.x[28] = %h", u_ibex_top.core_inst.m0.x[28]);
                $display("u_ibex_top.core_inst.m0.x[29] = %h", u_ibex_top.core_inst.m0.x[29]);
                $display("u_ibex_top.core_inst.m0.x[30] = %h", u_ibex_top.core_inst.m0.x[30]);
                $display("u_ibex_top.core_inst.m0.x[31] = %h", u_ibex_top.core_inst.m0.x[31]);
                $display("mem[0x1000] = %h", u_ibex_top.u_ram.u_ram.u_impl_generic.mem[1024]);
                 $display("mem[0x1008] = %h", u_ibex_top.u_ram.u_ram.u_impl_generic.mem[1026]);
                  $display("mem[0x1010] = %h", u_ibex_top.u_ram.u_ram.u_impl_generic.mem[1028]);

                $finish;
            end
            //else $display("\nUNKNOWN: basereg[17] = 0x%h (must be 0x0000005d)",u_ibex_top.core_inst.m0.x[17]);
        
      end

endmodule
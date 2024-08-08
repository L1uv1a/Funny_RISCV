# Test hw behavior simulation

# regs
# compile_verilog rtl/regs.sv tb/regs_tb.sv -DDETAILS > logs/regs.txt
# compile_verilog rtl/regs.sv tb/regs_tb.sv > logs/regs.txt
# cat logs/regs.txt

# main_memory
# compile_verilog rtl/main_memory.sv tb/main_memory_tb.sv -DDETAILS > logs/main_memory.txt
# compile_verilog rtl/main_memory.sv tb/main_memory_tb.sv > logs/main_memory.txt
# cat logs/main_memory.txt

# fetch
# compile_verilog rtl/fetch.sv rtl/main_memory.sv tb/fetch_tb.sv -DDETAILS > logs/fetch.txt
# compile_verilog rtl/fetch.sv rtl/main_memory.sv tb/fetch_tb.sv > logs/fetch.txt
# cat logs/fetch.txt

# decode
# compile_verilog rtl/decode.sv rtl/fetch.sv rtl/main_memory.sv tb/decode_tb.sv -DDETAILS > logs/decode.txt
# compile_verilog rtl/decode.sv rtl/fetch.sv rtl/main_memory.sv tb/decode_tb.sv > logs/decode.txt
# cat logs/decode.txt

# execute
# compile_verilog rtl/decode.sv rtl/fetch.sv rtl/main_memory.sv rtl/regs.sv rtl/forward.sv rtl/execute.sv tb/execute_tb.sv -DDETAILS -DINIT_MEM > logs/execute.txt
# compile_verilog rtl/decode.sv rtl/fetch.sv rtl/main_memory.sv rtl/regs.sv rtl/forward.sv rtl/execute.sv tb/execute_tb.sv -DDETAILS > logs/execute.txt
# cat logs/execute.txt

# memory
compile_verilog rtl/decode.sv rtl/fetch.sv rtl/main_memory.sv rtl/regs.sv rtl/forward.sv rtl/execute.sv rtl/memory.sv tb/memory_tb.sv -DDETAILS -DINIT_MEM > logs/memory.txt
# compile_verilog rtl/decode.sv rtl/fetch.sv rtl/main_memory.sv rtl/regs.sv rtl/forward.sv rtl/execute.sv rtl/memory.sv tb/memory_tb.sv -DDETAILS > logs/memory.txt
cat logs/memory.txt
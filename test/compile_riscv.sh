# Using this bash cmd file to compile risc-v asm files using riscv32-unknown-elf-gcc compiler
# Example useage: ./compile_riscv.sh /home/khai/Funny_RISCV/Compile/asms/sh1add.s 1024 default_linker.lds
# >>> THIS FILE MUST BE FIXED, IT MAY NOT WORKING ON YOUR COMPUTER. USE WITH CAUTION

#!/bin/bash

# Function definitions for colored output
prRed() {
    echo -e "\033[91m $1\033[00m"
}

prGreen() {
    echo -e "\033[92m $1\033[00m"
}

prYellow() {
    echo -e "\033[93m $1\033[00m"
}

prLightPurple() {
    echo -e "\033[94m $1\033[00m"
}

prPurple() {
    echo -e "\033[95m $1\033[00m"
}

prCyan() {
    echo -e "\033[96m $1\033[00m"
}

prLightGray() {
    echo -e "\033[97m $1\033[00m"
}

prBlack() {
    echo -e "\033[98m $1\033[00m"
}

prLightPurple "Auto compile RISC-V (RV32I) script"
prCyan "by Dao Quyet"

# Function to check the exit status of a command and exit if it fails
check_command() {
    if [ $? -ne 0 ]; then
        echo "Command failed: $1"
        exit 1
    fi
}

# Check if the correct number of arguments is provided
if [ "$#" -lt 1 ] || [ "$#" -gt 3 ]; then
    echo "Usage: $0 <assembly_file_path> [<memory_size>|<linker_script_path>]"
    exit 1
fi

assembly_file_path=$1

# Get the directory of this script
script_dir=$(dirname "$0")

# Default values
memory_size=1024
linker_script_path="$script_dir/compile_riscv_files/default_linker.lds"
is_default_memory_size=1
is_default_linker_script_path=1

# Check the number of arguments and assign values accordingly
if [ "$#" -eq 2 ]; then
    if [[ "$2" =~ ^[0-9]+$ ]]; then
        memory_size=$2
        is_default_memory_size=0
    elif [ -f "$2" ]; then
        linker_script_path=$2
        is_default_linker_script_path=0
    else
        echo "Invalid argument: $2"
        exit 1
    fi
elif [ "$#" -eq 3 ]; then
    if [[ "$2" =~ ^[0-9]+$ ]]; then
        memory_size=$2
        linker_script_path=$3
        is_default_memory_size=0
        is_default_linker_script_path=0
    elif [[ "$3" =~ ^[0-9]+$ ]]; then
        memory_size=$3
        linker_script_path=$2
        is_default_memory_size=0
        is_default_linker_script_path=0
    else
        echo "Invalid argument combination: $2 $3"
        exit 1
    fi
fi

if [ $is_default_memory_size -eq 1 ]; then
    prYellow "[Info] Using default memory size: $memory_size"
else
    prPurple "[Info] Custom memory size: $memory_size"
fi

if [ $is_default_linker_script_path -eq 1 ]; then
    prYellow "[Info] Using default linker script: $linker_script_path"
else
    prPurple "[Info] Custom linker script: $linker_script_path"
fi

# Check if the assembly file exists
if [ ! -f "$assembly_file_path" ]; then
    prRed "[Error] Assembly file not found: $assembly_file_path"
    exit 1
else
    prPurple "[Info] Included assembly file: $assembly_file_path"
fi

# Check if the linker script exists
if [ ! -f "$linker_script_path" ]; then
    prRed "[Error] Linker script not found: $linker_script_path"
    exit 1
else
    prPurple "[Info] Included linker script: $linker_script_path"
fi

# Extract the base name without extension
base_name=$(basename "$assembly_file_path" .s)

# Get the directory of the assembly file
asm_dir=$(dirname "$assembly_file_path")

# Create a folder to store compiled files if it doesn't exist
output_folder="$asm_dir/compiled_files"
mkdir -p "$output_folder"

# Step 1: Compile the assembly code
riscv32-unknown-elf-gcc -c -mabi=ilp32 -march=rv32im_zba_zbb_zbc_zbs_zicsr_zifencei -o "$output_folder/${base_name}.o" "$assembly_file_path"
check_command "riscv32-unknown-elf-gcc -c -mabi=ilp32 -march=rv32im_zba_zbb_zbc_zbs_zicsr_zifencei -o $output_folder/${base_name}.o $assembly_file_path"

# Step 2: Link the object file to create an ELF executable
riscv32-unknown-elf-gcc -Og -mabi=ilp32 -march=rv32im_zba_zbb_zbc_zbs_zicsr_zifencei -ffreestanding -nostdlib -o "$output_folder/${base_name}.elf" -Wl,--build-id=none,-Bstatic,-T,"$linker_script_path",-Map,"$output_folder/${base_name}.map",--strip-debug "$output_folder/${base_name}.o" -lgcc
check_command "riscv32-unknown-elf-gcc -Og -mabi=ilp32 -march=rv32im_zba_zbb_zbc_zbs_zicsr_zifencei -ffreestanding -nostdlib -o $output_folder/${base_name}.elf -Wl,--build-id=none,-Bstatic,-T,${linker_script_path},-Map,${output_folder}/${base_name}.map,--strip-debug ${output_folder}/${base_name}.o -lgcc"

# Step 3: Disassemble the ELF file to a human-readable format
riscv32-unknown-elf-objdump -d "$output_folder/${base_name}.elf" > "$output_folder/${base_name}.dump"
check_command "riscv32-unknown-elf-objdump -d ${output_folder}/${base_name}.elf > ${output_folder}/${base_name}.dump"

# Step 4: Convert the ELF file to a binary file
riscv32-unknown-elf-objcopy -O binary "$output_folder/${base_name}.elf" "$output_folder/${base_name}.bin"
check_command "riscv32-unknown-elf-objcopy -O binary ${output_folder}/${base_name}.elf ${output_folder}/${base_name}.bin"

# Step 5: Generate a hex file from the binary file
python3 "/home/khai/Funny_RISCV/Compile/makehex.py" "$output_folder/${base_name}.bin" "$memory_size" > "$asm_dir/${base_name}.hex"
check_command "python3 $script_dir/makehex.py ${output_folder}/${base_name}.bin $memory_size > ${asm_dir}/${base_name}.hex"

prGreen "[OK] All steps completed successfully."

# RV32I 5 Stages Pipelined

## Tracking tasks:

||Design|Behavior<br>Simulation Test|Documentation|
|:-|:-:|:-:|:-:|
|regs|:heart:|:heart:|:heart:|
|main_memory|:heart:|:heart:|:heart:|
|memory_wrapper|:white_heart:|:white_heart:|:white_heart:|
|forward|:heart:|:heart:|:white_heart:|
|<br>|
|fetch|:heart:|:heart:|:white_heart:|
|decode|:heart:|:heart:|:white_heart:|
|execute|:heart:|:heart:|:white_heart:|
|memory|:heart:|:heart:|:white_heart:|
|writeback|:heart:|:white_heart:|:white_heart:|
|<br>|
|core|:white_heart:|:white_heart:|:white_heart:|
|soc (top module)|:white_heart:|:white_heart:|:white_heart:|
|<br>|
|csr (ZICSR)|:heart:|:white_heart:|:white_heart:|
|Integer Multiplication<br>and Division (M)|:white_heart:|:white_heart:|:white_heart:|
|Bit Manipulation<br>and Division (B)|:white_heart:|:white_heart:|:white_heart:|



## Modules
### Main Memory

|[Details](./docs/modules/main_memory/main_memory.md)|[Code](./hw/rtl/main_memory.sv)|
|-|-|

### Base Registers

|[Details](./docs/modules/regs/regs.md)|[Code](./hw/rtl/regs.sv)|
|-|-|
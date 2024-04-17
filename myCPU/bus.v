`ifndef RISCV_BUS_V_
`define RISCV_BUS_V_

// address bus
`define ADDR_BUS 31:0 //地址总线口线具体位置
`define ADDR_BUS_WIDTH 32 //地址总线宽度

// instruction bus
`define INST_BUS 31:0 //指令总线口线具体位置
`define INST_BUS_WIDTH 32 //指令总线宽度

// data bus
`define DATA_BUS 31:0 //数据总线口线具体位置
`define DATA_BUS_WIDTH 32 //数据总线宽度

//Reg Index
`define RFIDX_WIDTH 5 //寄存器索引宽度 
`define RFIDX_BUS 4:0  //寄存器索引总线口线具体位置
`endif // RISCV_BUS_V_
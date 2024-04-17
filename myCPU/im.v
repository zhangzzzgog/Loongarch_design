
// instruction memory
module im(
      input  [11:2]  addr,
      output [31:0] dout 
);
      reg  [31:0] RAM[20470:0]; // 8KB储存容量 32位输出，2048位字地址
      assign dout = RAM[addr]; // 指令储存器中字对齐，因为每条指令长度都是一个字
endmodule  

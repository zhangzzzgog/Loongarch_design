`include"bus.v"

// flop with reset and clear control(触发器，输入信号为clk,rst,clear)
module floprc #(parameter WIDTH = 8)
              (input                  clk, reset, clear,
               input      [WIDTH-1:0] d, //输入数据
               output reg [WIDTH-1:0] q);//输出数据

  always @(posedge clk, posedge reset)
    if (reset)      q <= 0;
    else if (clear) q <= 0;
    else            q <= d;
endmodule


module floprc_negedge #(parameter WIDTH = 8)
              (input                  clk, reset, clear,
               input      [WIDTH-1:0] d, //输入数据
               output reg [WIDTH-1:0] q);//输出数据

  always @(negedge clk, posedge reset)
    if (reset)      q <= 0;
    else if (clear) q <= 0;
    else            q <= d;
endmodule

// flop with reset, Enable and clear control(触发器，输入信号为en,clk,rst,clear)
module flopenrc #(parameter WIDTH = 8)
                 (input                  clk, reset,
                  input                  en, clear,
                  input      [WIDTH-1:0] d, 
                  output reg [WIDTH-1:0] q);
 
  always @(posedge clk, posedge reset)
    if      (reset) q <= 0;
    else if (clear) q <= 0;
    else if (en)    q <= d;
endmodule


module flopenrc_ins #(parameter WIDTH = 8)
                 (input                  clk, reset,
                  input                  en, clear,
                  input      [WIDTH-1:0] d, 
                  output reg [WIDTH-1:0] q);
 
  always @(posedge clk, posedge reset)
    if      (reset) q <= 0;
    else if (clear) q <= 32'h03400000;
    else if (en)    q <= d;
endmodule

/**************************************************************************************************************************/
//IFID
module IFID(
	//control input
	input clk,
	input rst,
	input enable,
	input flushD,
	//input
	input  [`ADDR_BUS] addr_in, //输入指令地址
	input  [`ADDR_BUS] addrplus4_in, //输入指令地址+4
	input  [`INST_BUS] inst_in, //输入指令
	input fs_esubcode, //输入异常码
	input [5:0] fs_ecode, //输入异常码
	input fs_ex, 
	input branchPredictF,
	//output 
	output [`ADDR_BUS] addr_out,
	output [`ADDR_BUS] addrplus4_out,
	output [`INST_BUS] inst_out,
	output ds_esubcode, 
	output [5:0] ds_ecode,
	output ds_ex,
	output branchPredictD
);
flopenrc #(`ADDR_BUS_WIDTH) ff_addr(
	//input 
	clk, 
	rst,
	enable, //en
	0, //clear
	addr_in, 
	//output
	addr_out
);//pc
flopenrc #(`ADDR_BUS_WIDTH) ff_addr_p4(
	//input
	clk, 
	rst,
	enable, 
	flushD,
	addrplus4_in,
	//output
	addrplus4_out
);//pc+4
flopenrc #(`ADDR_BUS_WIDTH) ff_branchPredict(
	//input
	clk, 
	rst,
	enable, 
	flushD,
	branchPredictF,
	//output
	branchPredictD
);//pc+4
flopenrc_ins #(`INST_BUS_WIDTH) ff_inst(
	//input 
	clk,
	rst,
	enable,
	flushD, //flushD清成nop指令而非0
	inst_in, 
	//output
	inst_out
);
flopenrc #(8) csr_in(
	//input
	clk,
	rst,
	enable,
	flushD,
	{fs_esubcode, fs_ecode, fs_ex},
	//output
	{ds_esubcode, ds_ecode, ds_ex});
endmodule 
//IFID



/**************************************************************************************************************************/
//IDEX
//reg SignalsD={regwriteD, memwriteD, memtoregD, lwhbD, swhbD, lunsignedD, alusrcaD, alusrcbD, aluctrlD, aluctrl1D, jD, bD, data_ram_weD};
//reg SignalsE={regwriteE, memwriteE, memtoregE, lwhbE, swhbE, luE,alusrcaE, alusrcbE, aluctrlE, aluctrl1E, jE, bE, data_ram_weE};
module IDEX(
	input clk,
	input rst,
	input enable,
	input flushE,
	//input data1 in ID stage 
	//output data1 in EX stage 
    input  [`DATA_BUS] rdata1D,
	output [`DATA_BUS] rdata1E,
	//input data2 in ID stage
	//output data2 in EX stage
    input  [`DATA_BUS] rdata2D,
	output [`DATA_BUS] rdata2E,
	//input imm in ID stage
	//output imm in EX stage
    input  [`DATA_BUS] immoutD,
	output [`DATA_BUS] immoutE,
	//input rs1 in ID stage
	//output rs1 in EX stage
    input  [`RFIDX_BUS] rs1D,
	output [`RFIDX_BUS] rs1E,
	//input rs2 in ID stage
	//output rs2 in EX stage
    input  [`RFIDX_BUS] rs2D,
	output [`RFIDX_BUS] rs2E,
	//input rd in ID stage
	//output rd in EX stage
	input  [`RFIDX_BUS] rdD,
	output [`RFIDX_BUS] rdE,
	//input pc in ID stage
	//output pc in EX stage
    input  [`ADDR_BUS] pcD,
	output [`ADDR_BUS] pcE,
	input  [`ADDR_BUS] pcplus4D,
	output [`ADDR_BUS] pcplus4E,
	//input control signals in ID stage
	//output control signals in EX stage
    input  [17:0]      SignalsD,
    output [17:0]      SignalsE,
	input  [153:0] csr_in,
	output [153:0] csr_out,
	input [7:0] plv,
	output [7:0] plvE
);
flopenrc #(`DATA_BUS_WIDTH) 	rs1dataDE(clk, rst, enable, flushE, rdata1D, rdata1E);        // rs1.data
flopenrc #(`DATA_BUS_WIDTH) 	rs2dataDE(clk, rst, enable, flushE, rdata2D, rdata2E);        // rs2.data
flopenrc #(`DATA_BUS_WIDTH) 	immDE    (clk, rst, enable, flushE, immoutD, immoutE);       //imm
flopenrc #(`RFIDX_WIDTH)  	rs1numDE(clk, rst, enable, flushE, rs1D, rs1E);              // rs1.number
flopenrc #(`RFIDX_WIDTH)  	rs2numDE(clk, rst, enable, flushE, rs2D, rs2E);              // rs2.number
flopenrc #(`RFIDX_WIDTH)  	rdnumDE(clk, rst, enable, flushE, rdD, rdE);                 // rd.number
flopenrc #(`ADDR_BUS_WIDTH)	pcDE(clk, rst, enable, flushE, pcD, pcE);                 // pc
flopenrc #(`ADDR_BUS_WIDTH)	pcp4DE(clk, rst, enable, flushE, pcplus4D, pcplus4E);     // pc+4
flopenrc #(18) SignalsDE(clk, rst, enable, flushE, SignalsD, SignalsE);                //Signals 
flopenrc #(122) csrDE(clk, rst, enable, flushE, csr_in, csr_out);                      //csr    
flopenrc #(8) plvDE(clk, rst, enable, flushE, plv, plvE);                              //plv             
endmodule //IDEX



/**************************************************************************************************************************/
//EXMEM
//CtrE={writenE,regwriteE, memwriteE, memtoregE, lwhbE, luE, swhbE, jE, bE, data_ram_weE};
//CtrM={writenM,regwriteM, memwriteM, memtoregM, lwhbM, luM, swhbM, jM, bM, data_ram_weM};
module EXMEM(
	input clk,
	input reset,
	input flushM,
	input  [7:0] CtrE,
	output [7:0] CtrM,
	input  writenE,
	output writenM,
	input  [`DATA_BUS] srcb1E,
	output [`DATA_BUS] srcb1M,
	input  [`DATA_BUS] aluoutE,
	output [`DATA_BUS] aluoutM,
	input  [`DATA_BUS] srcb,
	output [`DATA_BUS] writedataM,
	input  [`RFIDX_BUS] rdE,
	output [`RFIDX_BUS] rdM,
	input  [`ADDR_BUS] pcE,
	output [`ADDR_BUS] pcM,
	input  [`ADDR_BUS] pcplus4E,
	output [`ADDR_BUS] pcplus4M,
	input  [`ADDR_BUS] PCoutE,
	output [`ADDR_BUS] PCoutM,
	input  [153:0] csr_in,
	output [153:0] csr_out,
	input  [7:0] plvE,
	output [7:0] plvM
);
floprc #(8) 		  SignalsEM	(clk, reset, flushM,CtrE,CtrM);//Signals
floprc #(`DATA_BUS_WIDTH) regM		(clk, reset, flushM,srcb1E,srcb1M);//???
floprc #(1) writenEM	(clk, reset, flushM,writenE,writenM);//ALU_Out
floprc #(`DATA_BUS_WIDTH) ALU_OutEM	(clk, reset, flushM, aluoutE, aluoutM);	   //ALU_Out
floprc #(`DATA_BUS_WIDTH) MemWDataEM	(clk, reset, flushM, srcb, writedataM);    //MemWriteData
floprc #(`RFIDX_WIDTH) 	  RdNumEM	(clk, reset, flushM, rdE, rdM);		   //rd.num
floprc #(`ADDR_BUS_WIDTH) pcEM		(clk, reset, flushM, pcE, pcM);            // pc
floprc #(`ADDR_BUS_WIDTH) pcp4EM	(clk, reset, flushM, pcplus4E, pcplus4M);  // pc+4
floprc #(`ADDR_BUS_WIDTH) PCOutEM	(clk, reset, flushM, PCoutE, PCoutM);      //pcOut
floprc #(154) csrEM		(clk, reset, flushM, csr_in, csr_out);       //csr
floprc #(8) plvEM		(clk, reset, flushM, plvE, plvM);           //plv
endmodule//EXMEM



/**************************************************************************************************************************/
//MEMWB
//CtrM={regwriteM, memtoregM, jM, bM};
//CtrW={regwriteW, memtoregW, jW, bW};
module MEMWB(
	input clk,
	input reset,
	input flushW,
	input  [3:0] CtrMW,
	output [3:0] CtrWB,
	input  [`DATA_BUS] dmoutM,
	output [`DATA_BUS] dmoutW,
	input  [`DATA_BUS] aluoutM,
	output [`DATA_BUS] aluoutW,
	input  [`RFIDX_BUS] rdM,
	output [`RFIDX_BUS] rdW,
	input  [`ADDR_BUS] pcM,
	output [`ADDR_BUS] pcWt,
	input  [`ADDR_BUS] pcplus4M,
	output [`ADDR_BUS] pcplus4W,
	input  [`ADDR_BUS] PCoutM,
	output [`ADDR_BUS] PCoutW,
	input  [153:0] csr_in,
	output [153:0] csr_out,
	input  [7:0] plvM,
	output [7:0] plvW
);
floprc #(4) 		        SignalsMW(clk, reset, flushW, CtrMW, CtrWB);//Ctr Signals
floprc #(`DATA_BUS_WIDTH)   DMOutMW(clk, reset, flushW, dmoutM, dmoutW);   //Data memory output
floprc #(`DATA_BUS_WIDTH)   pr1W(clk, reset, flushW, aluoutM, aluoutW);	   //ALU_OUT
floprc #(`RFIDX_WIDTH)      pr2W(clk, reset, flushW, rdM, rdW);		   //Rd.num
floprc #(`ADDR_BUS_WIDTH)   pr3W(clk, reset, flushW, pcM, pcWt);            // pc
floprc #(`ADDR_BUS_WIDTH)   pr4W(clk, reset, flushW, pcplus4M, pcplus4W);  // pc+4
floprc #(`ADDR_BUS_WIDTH)   regpcW(clk, reset, flushW, PCoutM, PCoutW);	   //PCOut: To support jalr
floprc #(154)               csr_inW(clk, reset, flushW, csr_in, csr_out);  //csr_in
floprc #(8) plvMW		(clk, reset, flushM, plvM, plvW);  
endmodule//MEMWB

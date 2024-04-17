`include "ctrl_encode_def.v"
`include "bus.v"
//如果不能一次写完或者读完（如要访问整个字，而地址却为对齐）
module dm_weaGen //DM_wea generator
(
   //input
   input          DMWr,
   input  [2:0]   DMType,
   input  [1:0]   Addr_loc,
   input  [`DATA_BUS] DM_DataRaw,
   //output
   output [`DATA_BUS]  DM_WriteData,
   output [3:0]   wea
);

   assign DM_WriteData=(Addr_loc==2'b00)?DM_DataRaw:
                        ((Addr_loc==2'b01)?{DM_DataRaw[23:0],{8'b0}}:
                           ((Addr_loc==2'b10)?{DM_DataRaw[15:0],{16'b0}}:
                              {DM_DataRaw[7:0],{24'b0}}
                           )
                        );

   //用组合逻辑来实现wea信号的构成
   // //字长
   // `define dm_word 3'b000
   // `define dm_halfword 3'b001
   // `define dm_halfword_unsigned 3'b010
   // `define dm_byte 3'b011
   // `define dm_byte_unsigned 3'b100

   wire wea_tmp0=(DMType<=4)&&DMWr;
   wire wea_tmp1=(DMType<=2)&&DMWr;
   wire wea_tmp2=(DMType==0)&&DMWr;
   wire wea_tmp3=wea_tmp2;

   assign wea[0]=(Addr_loc==2'b00)?wea_tmp0:1'b0;
   assign wea[1]=(Addr_loc==2'b00)?wea_tmp1:((Addr_loc==2'b01)?wea_tmp0:1'b0);
   assign wea[2]=(Addr_loc==2'b00)? wea_tmp2:
                     ((Addr_loc==2'b01)? wea_tmp1:
                        ( (Addr_loc==2'b10)?
                           wea_tmp0:1'b0));
   assign wea[3]=(Addr_loc<=2'b01)? wea_tmp2:
                     ( (Addr_loc==2'b10)? wea_tmp1:
                        wea_tmp0);


endmodule

// data memory
module dm_controller //DM_Controller
(
   //input
   input  [2:0]   DMType,
   input  [`DATA_BUS]  dout_raw,
   input  [1:0]  Addr_loc,
   //output
   output [`DATA_BUS]  dout
);
   wire [`DATA_BUS] dout_processed = (Addr_loc==2'b00)?dout_raw:
                                     ((Addr_loc==2'b01)?{{8'b0},dout_raw[31:8]}:
                                       ((Addr_loc==2'b10)?{{16'b0},dout_raw[31:16]}:
                                        {{24'b0},dout_raw[31:24]}
                                       )
                                     );

   //符号扩展
	assign dout[7:0] = dout_processed[7:0];
	assign dout[15:8] = (DMType==`dm_byte_unsigned)?8'b0:((DMType==`dm_byte)?{8{dout_processed[7]}}:dout_processed[15:8]);
	assign dout[31:16] = (DMType==`dm_halfword_unsigned||DMType==`dm_byte_unsigned)?16'b0
                        :( (DMType==`dm_halfword)?{16{dout_processed[15]}}:
                           ((DMType==`dm_byte)?{16{dout_processed[7]}}:dout_processed[31:16])
                        );
endmodule





//数据内存读写模块:自制，已经不用，仅供参考
module my__dm(input           	         clk,
            input  [3:0]			      wea,  //读入使能信号
            input  [`ADDR_BUS]	      addr, //读入地址
            input  [`DATA_BUS]         wd,   //写入数据
            output [`DATA_BUS]         rd);  //读出数据

  // 内存大小定义，8KB，这里是字节编址，所以是8192个字节
  reg  [7:0] RAM[8191:0];

  //输出数据暂存器
  reg [31:0] rtmp;

  //输出永远为从当前地址开始一个字的数据
  always @(*)
		rtmp <= {RAM[addr+3],RAM[addr+2],RAM[addr+1],RAM[addr]};

  assign rd = rtmp; // 输出赋值

  always @(posedge clk)//写入对应信号有效的字节数据
      begin
		if(wea[0])
			RAM[addr] <= wd[7:0];
		if(wea[1])
			RAM[addr+1] <= wd[15:8];
		if(wea[2])
			RAM[addr+2] <= wd[23:16];
		if(wea[3])
			RAM[addr+3] <= wd[31:24];
		end
endmodule

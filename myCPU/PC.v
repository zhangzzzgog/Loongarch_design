`include "bus.v"
module PC( clk, rst,en, NPC, PC,this_pc,pcplus4);
  input              clk;
  input              rst;
  input              en;
  input       [31:0] NPC;
  output reg  [31:0] PC;
  output reg  [31:0] this_pc;
  output [31:0] pcplus4;
  
  PCPlus4 U_PCPlus4(PC, pcplus4);
  
  initial
    begin
        PC <= 32'h1c00_0000;
        this_pc<=PC;
end
  
  // always @(posedge clk, posedge rst)
  //   if(en)
  //   this_pc<=PC;
  
  always @(posedge clk, posedge rst) begin
    //this_pc<=PC;
    //$display("PC%h",PC);
    //$display("NPC%h",NPC);
    if (rst) 
      PC <= 32'h1c00_0000;
//      PC <= 32'h0000_3000;
    else if(en)
    begin
      this_pc<=PC;
      PC <= NPC;
    end
end
endmodule

//Simplified from NPC module in single cycle cpu.
module PCPlus4(PC,pcplus4);
   input  [31:0] PC;        // pc
   output reg [31:0] pcplus4;   // pc+4
   wire [31:0] PCPLUS4;
   assign PCPLUS4 = PC + 4; // pc + 4
   always @(*) begin
        pcplus4 = PCPLUS4;
   end // end always
endmodule

module pc_imm(
  //input 
  input [`ADDR_BUS] pcE,
  input [`DATA_BUS]immoutE,
  //output 
  output reg [31:0] PCoutE
  );//adder


  wire [`ADDR_BUS] PCPLUS;   
  assign PCPLUS = pcE + immoutE; // pc + 4
  always @(*) begin
       PCoutE = PCPLUS;
  end // end always
endmodule

module UPC(
    input  clk,
    input  rst,
    input  [31:0]PCR,
    input  PC_WRITE,
    output reg[31:0]PC_IF_out,
    output reg[31:0]this_pc
    );

    always @(posedge clk,posedge rst)
        if(rst)
            PC_IF_out <= 32'h1c00_0000;
        else if(PC_WRITE)
            PC_IF_out<=PCR;
    
    always @(posedge clk,posedge rst)
            this_pc<=PC_IF_out;
            

endmodule

module PC_src_exception(
  input [5:0]ecode,
  input esubcode,
  input plv6,
  input plv3,

  input [13:0]csr_numW,
  input [13:0]csr_numM,

  input [7:0]plvM,
  input [7:0]plvE,
  input [31:0]ertn_addressM,
  input [31:0]ertn_addressW,

  input [`ADDR_BUS]ertn_address,
  input [`ADDR_BUS]eentry,
  input [`ADDR_BUS]nextpc,
  output [`ADDR_BUS] pc_complete
);
wire Isexception;
wire ertn ; 
wire pcforward_ertn_ws;
wire pcforward_ertn_ms;
wire [1:0]pcforward_ertn;

wire [31:0]ertn_address_complete;

assign ertn = ~plv3&plv6;
assign Isexception=({ecode,esubcode}!=7'b0000001);
wire [1:0] select ={ertn,Isexception};

wire csrM=~plvM[0]&plvM[1]&~plvM[2]&(plvM[3]|plvM[4])&~plvM[5]&~plvM[6]&~plvM[7];
wire csrE=~plvE[0]&plvE[1]&~plvE[2]&(plvE[3]|plvE[4])&~plvE[5]&~plvE[6]&~plvE[7];

assign pcforward_ertn_es=ertn&(csrE&csr_numW==6);
assign pcforward_ertn_ms=ertn&(csrM&csr_numM==6);
assign pcforward_ertn={pcforward_ertn_ms,pcforward_ertn_es};

mux3 #(`ADDR_BUS_WIDTH) ertn_address_forward(
  .s(pcforward_ertn),
  .d0(ertn_address),
  .d1(ertn_addressM),
  .d2(ertn_addressW),
  .Out(ertn_address_complete)
);

mux3 #(`ADDR_BUS_WIDTH) U_expection_mux(
  .s(select),
  .d0(nextpc),
  .d1(eentry),
  .d2(ertn_address_complete),
  .Out(pc_complete));

endmodule

module PC_remain(
  input clk,
  input rst,
  input flushD,
  input flush_ins,
  input [`ADDR_BUS]pc_now,
  output reg [`ADDR_BUS]pc_remain_one_cycle,
  output reg [`ADDR_BUS]pc_remain_two_cycles
);

always @(posedge clk,negedge rst)
begin
  if(rst)
  begin
   pc_remain_one_cycle=0;
   pc_remain_two_cycles=0;
  end
  else
  begin
      pc_remain_two_cycles=pc_remain_one_cycle;
      pc_remain_one_cycle=pc_now;
      
  end
end



endmodule
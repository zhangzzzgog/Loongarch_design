// //Exception unit
// module EXC(
//     input clk,
//     input rst,

//     input IPI,  //1个核间中断
//     input TI,  //1个定时器中断
//     input [2:0] HWI,  //8个硬中断
//     input SWI,  //2个软中断

//     input PIL,
//     input PIS,
//     input PIF,
//     input PME,
//     input PPI,


//     input [31:0] PC,
//     input [31:0] addr,
//     input load_or_store,

//     input syscall,
//     input break,

//     input INE,
//     input IPE,


//     input TLBR,

//     output Fs_new_ex,
//     output es_adem_ex,
//     output ale_ex,
//     output reg [5:0] ecode,
//     output reg       esubcode
// );
// integer Isize;
// integer Dsize;


// always @(*) begin
//     if(IPI||TI||HWI||SWI)
//     begin
//         ecode=6'b000000;
//         esubcode=1'b0;
//     end
//     else if(PIL)
//     begin
//         ecode=6'b000001;
//         esubcode=1'b0;
//     end
//     else if(PIS)
//     begin
//         ecode=6'b000010;
//         esubcode=1'b0;
//     end
//     else if(PIF)
//     begin
//         ecode=6'b000011;
//         esubcode=1'b0;
//     end
//     else if(PME)
//     begin
//         ecode=6'b000100;
//         esubcode=1'b0;
//     end
//     else if(PPI)
//     begin
//         ecode=6'b000111;
//         esubcode=1'b0;
//     end
//     else if(PC>Isize||PC<0)
//     begin
//         ecode=6'b001000;
//         esubcode=1'b0;
//     end
//     else if(load_or_store&&addr>Dsize||addr<0)
//     begin
//         ecode=6'b001000;
//         esubcode=1'b1;
//     end
//     else if((PC[31]!=0)||(PC[1:0]!=2'b00)||(load_or_store&&addr[1:0]!=2'b00))
//     begin
//         ecode=6'b001001;
//         esubcode=1'b0;  
//     end
//     else if(syscall)
//     begin
//         ecode=6'b001011;
//         esubcode=1'b0;
//     end
//     else if(break)
//     begin
//         ecode=6'b001100;
//         esubcode=1'b0;
//     end
//     else if(INE)
//     begin
//         ecode=6'b001101;
//         esubcode=1'b0;
//     end
//     else if(IPE)
//     begin
//         ecode=6'b001110;
//         esubcode=1'b0;
//     end
//     else if(TLBR)
//     begin
//         ecode=6'b111111;
//         esubcode=1'b0;
//     end
//     else //未发生中断例外
//     begin
//         ecode=6'b000000;
//         esubcode=1'b1;
//     end
// end

//     wire adef_ex = (ecode==6'b001000)&&(esubcode==1'b0);
//     wire fs_tlb_invalid_ex= (ecode==6'b000011)&&(esubcode==1'b0);
//     wire fs_tlb_ppe_ex= (ecode==6'b000111)&&(esubcode==1'b0);
//     wire fs_tlb_refill_ex= (ecode==6'b111111)&&(esubcode==1'b0);
//     assign Fs_new_ex           = adef_ex | fs_tlb_invalid_ex | fs_tlb_ppe_ex | fs_tlb_refill_ex;
//     assign es_adem_ex = (ecode==6'b001000)&&(esubcode==1'b1);
//     assign ale_ex = (ecode==6'b001001)&&(esubcode==1'b0);
// endmodule

//Exception unit
/*
    This module is used to handle exceptions.(Control Signal Generation)
*/
//Exception unit
/*
    This module is used to handle exceptions.(Control Signal Generation)
*/
module exception(
    input clk,
    input rst,
    input IPI,          //1个核间中断
    input TI,           //1 timer interruptions
    input [2:0] HWI,    //8 hard interruptions
    input [1:0]SWI,          //2 soft interruptions

    input PIL,
    input PIS,
    input PIF,
    input PME,
    input PPI,


    input [31:0] PC,
    input [31:0] addr,
    input load_or_store,

    input syscall,
    input break,

    input INE,
    input IPE,


    input TLBR,
    
    output Fs_new_ex,
    output Ds_new_ex,
    output Es_new_ex,
    output Ms_new_ex,
    output Ws_new_ex,

    output es_adem_ex,
    output ale_ex,

    output [5:0] Fs_ecode,
    output  Fs_esubcode,

    output [5:0] Ds_new_ecode,
    output  Ds_new_esubcode,

    output [5:0] Es_new_ecode,
    output  Es_new_esubcode,

    output [5:0] Ms_new_ecode,
    output  Ms_new_esubcode,

    output [5:0] Ws_new_ecode,
    output  Ws_new_esubcode
);
integer Isize;
integer Dsize;

wire [5:0] ecode;
wire esubcode;

initial begin
    Isize=32'h7fff_ffff;
    Dsize=32'h7fff_ffff;
end

//To CSR, csr operate exeception code
wire F_ex = ~(TI|SWI[0]|SWI[1]);//|IPI||HWI[0]|HWI[1]|HWI[2]|SWI;

assign Fs_ecode[5]=0;
assign Fs_ecode[4]=0;
assign Fs_ecode[3]=0;
assign Fs_ecode[2]=0;
assign Fs_ecode[1]=0;
assign Fs_ecode[0]=0;

assign Fs_esubcode=F_ex;
assign Fs_new_ex=({Fs_ecode,Fs_esubcode}!=7'b0000001);

wire D_ex=~(syscall|break|INE);
assign Ds_new_ecode[5]=0;
assign Ds_new_ecode[4]=0;
assign Ds_new_ecode[3]=syscall|break|INE;
assign Ds_new_ecode[2]=break|INE;
assign Ds_new_ecode[1]=syscall;
assign Ds_new_ecode[0]=syscall|INE;

assign Ds_new_esubcode=D_ex;
assign Ds_new_ex=({Ds_new_ecode,Ds_new_esubcode}!=7'b0000001);

assign Es_new_ecode[5]=0;
assign Es_new_ecode[4]=0;
assign Es_new_ecode[3]=0;
assign Es_new_ecode[2]=0;
assign Es_new_ecode[1]=0;
assign Es_new_ecode[0]=0;

assign Es_new_esubcode=1;
assign Es_new_ex=({Es_new_ecode,Es_new_esubcode}!=7'b0000001);

assign Ms_new_ecode[5]=0;
assign Ms_new_ecode[4]=0;
assign Ms_new_ecode[3]=0;
assign Ms_new_ecode[2]=0;
assign Ms_new_ecode[1]=0;
assign Ms_new_ecode[0]=0;

assign Ms_new_esubcode=1;
assign Ms_new_ex=({Ms_new_ecode,Ms_new_esubcode}!=7'b0000001);

assign Ws_new_ecode[5]=0;
assign Ws_new_ecode[4]=0;
assign Ws_new_ecode[3]=0;
assign Ws_new_ecode[2]=0;
assign Ws_new_ecode[1]=0;
assign Ws_new_ecode[0]=0;

assign Ws_new_esubcode=1;
assign Ws_new_ex=({Ws_new_ecode,Ws_new_esubcode}!=7'b0000001);

assign ds_ex=Ds_new_ecode!=6'b000000;
assign es_ex=Es_new_ecode!=6'b000000;
assign ms_ex=Ms_new_ecode!=6'b000000;
assign ws_ex=Ws_new_ecode!=6'b000000;


assign ecode[5]=TLBR;
assign ecode[4]=TLBR;
assign ecode[3]=syscall|break|INE|IPE|TLBR;//|(PC>Isize||PC<0)|(load_or_store&&addr>Dsize||addr<0)|((PC[31]!=0)||(PC[1:0]!=2'b00)||(load_or_store&&addr[1:0]!=2'b00))
assign ecode[2]=PME|PPI|break|INE|IPE|TLBR;
assign ecode[1]=PIS|PIF|PPI|syscall|IPE|TLBR;
assign ecode[0]=PIL|PIF|PPI|syscall|INE|TLBR;//|((PC[31]!=0)||(PC[1:0]!=2'b00)||(load_or_store&&addr[1:0]!=2'b00))

assign esubcode=    //To CSR, csr operate exeception subcode
    ~(IPI||TI||HWI||SWI)&
    ~PIL&
    ~PIS&
    ~PIF&
    ~PME&
    ~PPI&
    //~(PC>Isize||PC<0)&        //1'b1:address illegal
    //~((PC[31]!=0)||(PC[1:0]!=2'b00)||(load_or_store&&addr[1:0]!=2'b00))&
    ~syscall&
    ~break&
    ~INE&
    ~IPE&
    ~TLBR&
    1'b1|(load_or_store&&addr>Dsize||addr<0)  ;          //No exception
    

    //Exception
    wire adef_ex = (ecode==6'b001000)&&(esubcode==1'b0);
    wire fs_tlb_invalid_ex= (ecode==6'b000011)&&(esubcode==1'b0);
    wire fs_tlb_ppe_ex= (ecode==6'b000111)&&(esubcode==1'b0);
    wire fs_tlb_refill_ex= (ecode==6'b111111)&&(esubcode==1'b0);
  
    assign ex = ((ecode==6'b001000)&&(esubcode==1'b0)) 
                | ((ecode==6'b000011)&&(esubcode==1'b0)) 
                |(ecode==6'b000111)&&(esubcode==1'b0) 
                | (ecode==6'b111111)&&(esubcode==1'b0);
     
    assign es_adem_ex = (ecode==6'b001000)&&(esubcode==1'b1);
    assign ale_ex = (ecode==6'b001001)&&(esubcode==1'b0);
endmodule


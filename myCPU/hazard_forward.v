`include "bus.v"
module hazard (
    input clk,
    input [1:0] WDSel,
    input [`RFIDX_BUS] rdE,
    input [`RFIDX_BUS] rs1D,
    input [`RFIDX_BUS] rs2D,
    input writenM,
    output reg writen
);  //load from memory and then use the value, should stall 1 cycle.
  always @*
    if ((WDSel == 2'b01) && (rdE == rs1D || rdE == rs2D) && writenM) writen <= 1'b0;
    else writen <= 1'b1;
endmodule


module Forward(
    input regwriteM,
    input memreadM,

    input regwriteW,
    input memreadW,

    input regwriteE,
    input memreadE,


    input [4:0] rs1D,
    input [4:0] rs2D,
    input [4:0] rs1E,
    input [4:0] rs2E, 
    input [4:0] rdE,
    input [4:0] rdM,
    input [4:0] rdW,

    input [7:0] plvE,
    input [7:0] plvM,
  


    output reg [2:0]forwardA, //alu source1
    output reg [2:0]forwardB, //alu source2
    output reg [2:0]forwardC, //branch source1
    output reg [2:0]forwardD,  //branch source2 
    input IsJtype
);
//forwardA, forwardB
// 00 : No Forwarding
// 01 : Forward from EX_MEM ALU result
// 10 : Forward from MEM_WB ALU result
// 11 : Forward from MEM_WB MEM result


//forwardC, forwardD
// 00 : No Forwarding
// 01 : Forward from EX_MEM ALU result
// 10 : Forward from MEM_WB MEM result

wire csrE=~plvE[0]&plvE[1]&(plvE[2]|plvE[3]|plvE[4])&~plvE[5]&~plvE[6]&~plvE[7];

wire csrM=~plvM[0]&plvM[1]&(plvM[2]|plvM[3]|plvM[4])&~plvM[5]&~plvM[6]&~plvM[7];



always @(*) begin
    if(regwriteM&(~csrM) && (rdM != 0) && (rdM == rs1E)) begin
        forwardA = 3'b010;
    end
    else if((regwriteW|memreadW&(~csrM)) && (rdW != 0) && (rdW == rs1E)) begin
        forwardA = 3'b001;
    end
    else if(csrM&& (rdM != 0) && (rdM == rs1E))
    begin
        forwardA = 3'b011;
    end
    else begin
        forwardA = 3'b000;
    end

    if (regwriteM&(~csrM) && (rdM != 0) && (rdM == rs2E)) begin
        forwardB = 3'b010;
    end
    else if((regwriteW|memreadW&(~csrM))  && (rdW != 0) && (rdW == rs2E)) begin
        forwardB = 3'b001;
    end
    else if(csrM&& (rdM != 0) && (rdM == rs2E))
    begin
        forwardB = 3'b011;
    end
    else begin
        forwardB = 3'b000;
    end

    if ((regwriteE&~memreadE&(~csrE)) && (rdE != 0) && (rdE == rs1D)) begin
        forwardC = 3'b001;
    end
    else if((regwriteM&~memreadM&(~csrM&~csrE))&&(rdM != 0) && (rdM == rs1D)) begin
        forwardC = 3'b010;
    end
    else if((memreadM&(~csrM&~csrE)) && (rdM != 0) && (rdM == rs1D)) begin
        forwardC = 3'b011;
    end
    else if(csrE&&(rdE==rs1D))
    begin
        forwardC=3'b101;
    end
    else if(csrM&&(rdM==rs1D))
    begin
        forwardC=3'b110;
    end
    else if(IsJtype)begin
        forwardC=3'b100;
    end    
    else begin
        forwardC = 3'b000;
    end

    if((regwriteE&~memreadE&(~csrE)) && (rdE != 0) && (rdE == rs2D)) begin
        forwardD = 3'b001;
    end
    else if((regwriteM&~memreadM&(~csrM&~csrE))&&(rdM != 0) && (rdM == rs2D)) begin
        forwardD = 3'b010;
    end
    else if((memreadM&(~csrM&~csrE)) && (rdM != 0) && (rdM == rs2D)) begin
        forwardD = 3'b011;
    end
    else if(csrE&&(rdE==rs2D))
        begin
            forwardD=3'b101;
        end
    else if(csrM&&(rdM==rs2D))
    begin
        forwardD=3'b110;
    end
    else if(IsJtype)begin
        forwardD = 3'b100;
    end

    else begin
        forwardD = 3'b000;
    end

end
endmodule
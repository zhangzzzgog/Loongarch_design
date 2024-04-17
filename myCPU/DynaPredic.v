`include "bus.v"
/*Dynamic Prediction Module: be installed in IF stage, use information in ID stage.*/
module DynaPredic(
    input wire clk,
    input wire reset,
    /*Update Port*/
    input wire branchD,       /*Is branch type instruction*/
    input wire PredictErrorD, /*(Really_branchD&PredictD)|(~Really_branchD&~PredictD)*/
    input wire Really_branchD,
    input wire [`ADDR_BUS] SourcePCD,/*For update an entry,branch source pc*/
    input wire [`ADDR_BUS] TargetPCD,/*For update an entry,branch target pc,always put branch target(if branch)*/
    /*Predict Port*/
    input  wire [`ADDR_BUS] SearchPCF,/*IF stage:predict*/
    output wire [`ADDR_BUS] TargetPCF,
    output wire branchPredictF/*for PC module*/
    /*if(PredictErrorD) PC<=RealPC;if(branchPredictF&&~PredictErrorD) PC<=TargetPCF; else PC<=PC+4*/
    );
    
    //Storage Table: 31 entries
    reg [31:0] validReg;
    reg [1:0] PredictionReg[31:0]; //2bit Prediction
    reg [22:0]  SourcePCReg_tag[31:0]; //Source PC
    reg [31:4]  TargetPCReg[31:0]; //Target PC
    
    //Predict Input port
    wire [4:0]  SearchPC_index = SearchPCF[8:4]; //INDEX
    wire [22:0] SearchPC_tag   = SearchPCF[31:9];//TAG
    
    //Predict Output
    assign branchPredictF=(SearchPC_tag==SourcePCReg_tag[SearchPC_index]&&validReg[SearchPC_index])?
                          PredictionReg[SearchPC_index][1]:0;
    assign TargetPCF=(SearchPC_tag==SourcePCReg_tag[SearchPC_index]&&validReg[SearchPC_index])?
                          {TargetPCReg[SearchPC_index],4'h0}:32'b00000000; //TargetPCF
    
    //Update Input port
    wire [4:0] SourcePCD_index = SourcePCD[8:4];//INDEX
    wire [22:0] SourcePCD_tag  = SourcePCD[31:9];//TAG
    
    //Controlled by branchD signal(Update only if it is a br ins)
    reg [4:0] SourcePCD_index_buf;//INDEX_buffer
    reg [22:0] SourcePCD_tag_buf;//TAG_buffer
    reg [31:4] TargetPCD_buf_h28;
    reg PredictErrorD_buf,branchD_buf,Really_branchD_buf;
    
  integer i;
  always @(posedge clk, posedge reset)begin
    if (reset) begin    //  reset
      for (i=0; i<32; i=i+1)
        validReg[i]=1'b0;
        PredictionReg[i]=2'b00;
        SourcePCReg_tag[i]=23'h000000;
        TargetPCReg[i]=28'h0000000;
    end
    else begin
        //Write Update buffer
        SourcePCD_index_buf=SourcePCD_index;
        SourcePCD_tag_buf=SourcePCD_tag;
        PredictErrorD_buf=PredictErrorD;
        branchD_buf=branchD;
        TargetPCD_buf_h28=TargetPCD[31:4];
        Really_branchD_buf=Really_branchD;
    end    
  end
    
  wire MatchiD=(SourcePCD_tag_buf==SourcePCReg_tag[SourcePCD_index_buf])&&validReg[SourcePCD_index_buf];
  always @(negedge clk)begin //Update entries
    if(PredictErrorD_buf&&(PredictionReg[SourcePCD_index_buf][1]==1'b0)&&MatchiD) 
        PredictionReg[i]=PredictionReg[SourcePCD_index_buf]+1;
    else if(PredictErrorD_buf&&(PredictionReg[SourcePCD_index_buf][1]==1'b1)&&MatchiD) 
        PredictionReg[SourcePCD_index_buf]=PredictionReg[SourcePCD_index_buf]-1;
    else if(~PredictErrorD_buf&&(PredictionReg[SourcePCD_index_buf]==2'b01&&MatchiD)) 
        PredictionReg[SourcePCD_index_buf]=2'b00;
    else if(~PredictErrorD_buf&&(PredictionReg[SourcePCD_index_buf]==2'b10)&&MatchiD) 
        PredictionReg[SourcePCD_index_buf]=2'b11;
    
    //Record the first time
    if(branchD&&~MatchiD)begin
        TargetPCReg[SourcePCD_index_buf]=TargetPCD_buf_h28;
        validReg[SourcePCD_index_buf]=1'b1;
        SourcePCReg_tag[SourcePCD_index_buf] = SourcePCD_tag_buf;
        PredictionReg[SourcePCD_index_buf]=Really_branchD_buf?2'b01:2'b00;
    end
  end
  
endmodule

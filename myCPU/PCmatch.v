module PCmatch(
    input clk,
    input rst,
    input [31:0] pcF,
    input flush,
    input en,
    output reg [31:0] pcF_match
);
always @(posedge clk)
    if(!rst)
    begin
        if(flush)
        pcF_match<=0;
        else if(en)
        pcF_match<=pcF;
    end 


endmodule
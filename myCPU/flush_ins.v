module Flush_ins(
    input clk,
    input rst,
    input flush,
    output reg flush_ins
);

always @(posedge clk)
if(rst)
    flush_ins<=0;
else
    flush_ins<=flush;
endmodule
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/06/19 17:42:29
// Design Name: 
// Module Name: mux
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module mux2 #(parameter WIDTH = 8)
             (input  [WIDTH-1:0] d0, d1, 
              input              s, 
              output [WIDTH-1:0] Out);

  assign Out = s ? d1 : d0;  // 1'b0 -> d0, 1'b1 -> d1
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  [WIDTH-1:0] d0, d1, d2,
              input  [1:0]       s, 
              output [WIDTH-1:0] Out);

  assign  Out = s[1] ? d2 : (s[0] ? d1 : d0); // 2'b00 -> d0, 2'b01 -> d1, 2'b10 -> d2
endmodule

module mux4 #(parameter WIDTH = 8)
             (input  [WIDTH-1:0] d0, d1, d2, d3,
              input  [1:0]       s, 
              output [WIDTH-1:0] Out);

  assign  Out = s[1] ? (s[0] ? d3 : d2) : (s[0] ? d1 : d0); // 2'b00 -> d0, 2'b01 -> d1, 2'b10 -> d2, 2'b11 -> d3
endmodule

module
  mux5 #(parameter WIDTH = 8)
       (input  [WIDTH-1:0] d0, d1, d2, d3, d4,
        input  [2:0]       s, 
        output [WIDTH-1:0] Out);
        
  assign  Out = s[2]?d4:(s[1]?(s[0]?d3:d2):(s[0]?d1:d0)); // 3'b000 -> d0, 3'b001 -> d1, 3'b010 -> d2, 3'b011 -> d3, 3'b100 -> d4
endmodule

module 
mux6 #(parameter WIDTH = 8)
       (input  [WIDTH-1:0] d0, d1, d2, d3, d4,d5,
        input  [2:0]       s, 
        output [WIDTH-1:0] Out);
        
  assign  Out = s[2]?(s[0]?d5:d4):(s[1]?(s[0]?d3:d2):(s[0]?d1:d0)); // 3'b000 -> d0, 3'b001 -> d1, 3'b010 -> d2, 3'b011 -> d3, 3'b100 -> d4
endmodule

module 
mux7 #(parameter WIDTH = 8)
       (input  [WIDTH-1:0] d0, d1, d2, d3, d4,d5,d6,
        input  [2:0]       s, 
        output [WIDTH-1:0] Out);
        
  assign  Out = s[2]?(s[1]?d6:(s[0]?d5:d4)):(s[1]?(s[0]?d3:d2):(s[0]?d1:d0)); // 3'b000 -> d0, 3'b001 -> d1, 3'b010 -> d2, 3'b011 -> d3, 3'b100 -> d4
endmodule


module
    adder(
        input [31:0] a,
        input [31:0] b,
        output reg [31:0] sum
    );
    always @(*)
        sum <= a + b;

endmodule
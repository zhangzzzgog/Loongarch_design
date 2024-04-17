`include "ctrl_encode_def.v"
`include "bus.v"

module alu(
   input  signed [31:0] A, B,
   input         [4:0]  ALUOp,
   input         [`ADDR_BUS] PC,
   output signed [31:0] C_out,
   output Zero,
   output       	Overflow,
   output           Lt,
   output           Ge
);
   wire signed [63:0] A_dw={{32{1'b0}},A};
   wire signed [63:0] B_dw={{32{1'b0}},B};
   reg [31:0] C;
   integer    i;
       
   always @( * ) begin
      case ( ALUOp )
`ALUOp_nop:C=A;	//00000
`ALUOp_lui:C=B; //00001
`ALUOp_auipc:C=PC+B; //00010
`ALUOp_add:C=A+B;	//00011
`ALUOp_sub:C=A-B;	//00100
`ALUOp_slt:C={31'b0,(A<B)};				//01010
`ALUOp_sltu:C={31'b0,($unsigned(A)<$unsigned(B))};	//01011
`ALUOp_xor:C=A^B;	//01100
`ALUOp_or:C=A|B;	//01101
`ALUOp_and:C=A&B;	//01110
`ALUOp_sll:C=A<<B;	//01111
`ALUOp_srl:C=A>>B;	//10000
`ALUOp_sra:C=A>>>B;	//10001
//100010 is taken by beq
`ALUOp_nor:C=~(A|B);	//10011
`ALUOp_divw:C=A/B;	//10100
`ALUOp_divwu:C=$unsigned(A)/$unsigned(B);	//10101
`ALUOp_modw:C=A%B;	//10110
`ALUOp_modwu:C=$unsigned(A)%$unsigned(B);	//10111
`ALUOp_mulw:C=A*B;	//11000
`ALUOp_mulhw:C={{32{A[31]}},A}*{{32{B[31]}},B}>>32;	//11001
`ALUOp_mulhwu:C=($unsigned(A_dw)*$unsigned(B_dw))>>32;	//11010
      endcase
      //$display("B = 0x%8X", B); // used for debug
   end // end always
   assign Zero = (C == 32'b0);
   assign Overflow = ( A[31]&&B[31]|| ( (A[31]||B[31])&&!C[31] ) )?1:0;
   assign Lt =  C[`DATA_BUS_WIDTH-1];
   assign Ge = ~C[`DATA_BUS_WIDTH-1];
assign C_out=C;
endmodule
    

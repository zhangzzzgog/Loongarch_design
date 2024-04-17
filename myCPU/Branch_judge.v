`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// IsBranch_Dompany: 
// Engineer: 
// 
// IsBranch_Dreate Date: 2023/06/28 15:09:41
// Design Name: 
// Module Name: Branch_judge
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File IsBranch_Dreated
// Additional IsBranch_Domments:
// 
//////////////////////////////////////////////////////////////////////////////////



`include "ctrl_encode_def.v"

module Branch_judge(Branch_in1D, Branch_in2D, Branch_judge_ctrl, IsBranch_D);
           
   input  signed [31:0] Branch_in1D, Branch_in2D;
   input         [4:0]  Branch_judge_ctrl;
   output signed  IsBranch_D;

   
   reg  IsBranch_D;
   integer    i;
       
   always @( * ) begin
      case ( Branch_judge_ctrl )
      `ALUOp_beq:IsBranch_D=(Branch_in1D!=Branch_in2D);
      `ALUOp_bne:IsBranch_D=(Branch_in1D==Branch_in2D);
      `ALUOp_blt:IsBranch_D=(Branch_in1D>=Branch_in2D);
      `ALUOp_bge:IsBranch_D=(Branch_in1D<Branch_in2D);
      `ALUOp_bltu:IsBranch_D=($unsigned(Branch_in1D)>=$unsigned(Branch_in2D));
      `ALUOp_bgeu:IsBranch_D=($unsigned(Branch_in1D)<$unsigned(Branch_in2D));
      default:IsBranch_D=1'b1;
      endcase
   end // end always
   


endmodule
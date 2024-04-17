`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/06/29 17:46:37
// Design Name: 
// Module Name: IF
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
`include "bus.v"


module IF(
    input clk,
    input PCSrc,
    input PC_WRITE,    
    input PC,
    input [31:0] PC_IF_in_jump,

    input rst,
    output [31:0] PC_IF,
    output [31:0]this_pc
);


wire [31:0] PC_IF_in_next;
wire [31:0] PCR;

mux2 #(`ADDR_BUS_WIDTH) mux2_pc(
    .d0(PC_IF_in_next),
    .d1(PC_IF_in_jump),
    .s(PCSrc),
    .Out(PCR)
    );


UPC pc(
    .clk(clk),
    .rst(rst),
    .PCR(PCR),
    .PC_IF_out(PC_IF),
    .PC_WRITE(PC_WRITE),
    .this_pc(this_pc)
    );


adder adder(
    .a(PC_IF),
    .b(4),
    .sum(PC_IF_in_next)
    );





endmodule


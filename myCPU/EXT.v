`include "ctrl_encode_def.v"
module EXT(
	input [31:0] instrD,
	input	[5:0]			EXTOp,

	output	reg [31:0] 	       immout);


	// immediate generate
	wire [4:0]   iimm_shamt=instrD[14:10];
	wire [11:0]  iimm = instrD[21:10];
	wire [11:0]  simm	= instrD[21:10];
	wire [15:0]  bimm	= instrD[25:10];
	wire [19:0]  uimm	= instrD[24:5];
	wire [25:0]  jimm	= {instrD[9:0],instrD[25:10]};  //b,bl
   
always  @(*) begin
	 case (EXTOp)
		`EXT_CTRL_ITYPE_SHAMT:   immout<={27'b0,iimm_shamt[4:0]};
		`EXT_CTRL_ITYPE:	immout <= {{{32-12}{iimm[11]}}, iimm[11:0]};
		`EXT_CTRL_ITYPE_ZERO:	immout <= {{{32-12}{1'b0}}, iimm[11:0]};
		`EXT_CTRL_STYPE:	immout <= {{{32-12}{simm[11]}}, simm[11:0]};
		`EXT_CTRL_BTYPE:    immout <= {{{32-18}{bimm[15]}}, bimm[15:0], 2'b00};
		`EXT_CTRL_UTYPE:	immout <= {uimm[19:0], 12'b0}; //???????????12??0
		`EXT_CTRL_JTYPE:	immout <= {{{32-28}{jimm[25]}}, jimm[25:0], 2'b00};
		default:	        immout <= 32'b0; 
	endcase
end
endmodule

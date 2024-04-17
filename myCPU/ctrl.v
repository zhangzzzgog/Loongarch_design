// `include "ctrl_encode_def.v"

module LOONGARCH_CTRL(
               input [31:0] INSTR,
               input [31:0]pcF,
               input Jump,  // control signal for jump

              output MemRead,
              output       RegWrite, // control signal for register write
              output       MemWrite, // control signal for memory write
              output [5:0] EXTOp,    // control signal to signed extension
              output [4:0] ALUOp,    // ALU opertion
              output [1:0] NPCOp,    // next pc operation
              //如果要区分b和bl，需要在这里加一位
              
              //   output [1:0] counterOp;// counter operation
              output [1:0] ALUSrcA,
              output       ALUSrcB,   // ALU source for B
              output [2:0] DMType,   //DataMemoryType(W/h/b)
              output [1:0] GPRSel,   // general purpose register selection
              output [1:0] WDSel,   // (register) write data selection
              output       rs2Zero,  // Don't need rs2;
              output [17:0] SignalsD  // control signals for D stage
            ,
              output [7:0]      plv,      // priviledge level
            // exception,
            // exp_code,
              output [13:0] csr_num,   // csr number
            // cacopcode,
            // cache_target,
            // cache_op,
            // invlib_op,
              output syscall,
              output break,
              output rdcntidw_yes, // read counter id width
            //  output [14:0] idle_level // idle level
              output INE
            );




  //  output  exception; // exception signal
  //  output [14:0] exp_code;  // exception code

  //  output [4:0] cacopcode;  // cache opcode
  //  output [2:0] cache_target; // cache target
  //  output [1:0] cache_op; // cache operation
  //  output [4:0] invlib_op; // invlib operation



assign INE=(INSTR==32'h94080000)||(INSTR==32'hf36e0000)||(INSTR==32'h88000200)||(INSTR==32'h887b8000)||(INSTR==32'hf9000000)||(INSTR==32'hffff_ffff)||((INSTR==32'h0)&&(pcF!=32'h1c00_0000)&&(pcF!=32'h1c00_0004));
assign invlib=0;
   


//**********INSTR segmentation**********//

  wire [21:0] op2R=INSTR[31:10];
  wire [16:0] op3R=INSTR[31:15];
 // wire [11:0] op4R=INSTR[31:20]; //float instrs
  wire [13:0] op2R8I=INSTR[31:18];
  wire [9:0] op2R12I=INSTR[31:22];
  wire [7:0] op2R14I=INSTR[31:24];
  wire [6:0] op1R20I=INSTR[31:25];
  wire [5:0] op2R16I=INSTR[31:26];  //same as 1R21I and 26I

  wire [4:0] r1=INSTR[4:0];
  wire [4:0] r2=INSTR[9:5];
  wire [4:0] r3=INSTR[14:10];

  wire [4:0] ui5=INSTR[14:10];
  wire [11:0] si12=INSTR[21:10];
  wire [13:0] si14=INSTR[23:10];
  wire [19:0] si20=INSTR[24:5];

  wire [15:0] B_offs_L=INSTR[25:10];
  wire [9:0] B_offs_H=INSTR[9:0];

//**********INSTR segmentation end**********//

//**********INSTR**********//
//INTERGER
  //Arithmetic

  //ADD.W
  wire addw=(op3R==17'b00000000000100000);
  //SUB.W
  wire subw=(op3R==17'b00000000000100010);
  //ADDI.W
  wire addiw=(op2R12I==10'b0000001010);
      //NOP andi r0,r0,0
  //LU12I.W
  wire lu12iw=(op1R20I==7'b0001010);
  //SLT
  wire slt=(op3R==17'b00000000000100100);
  //SLTU
  wire sltu=(op3R==17'b00000000000100101);
  //SLTI
  wire slti=(op2R12I==10'b0000001000);
  //SLTUI
  wire sltui=(op2R12I==10'b0000001001);
  //PCADDU12I
  wire pcaddu12i=(op1R20I==7'b0001110);
  //AND
  wire iand=(op3R==17'b00000000000101001);  //i ahead means instructor (incase of conflict with "and" logistic oper)
  //OR
  wire ior=(op3R==17'b00000000000101010);
  //XOR
  wire ixor=(op3R==17'b00000000000101011);
  //NOR
  wire inor=(op3R==17'b00000000000101000);
  //ANDI
  wire andi=(op2R12I==10'b0000001101);
  //ORI
  wire ori=(op2R12I==10'b0000001110);
  //XORI
  wire xori=(op2R12I==10'b0000001111);
  //MUL.W
  wire mulw=(op3R==17'b00000000000111000);
  //MULH.W
  wire mulhw=(op3R==17'b00000000000111001);
  //MULH.WU
  wire mulhwu=(op3R==17'b00000000000111010);
  //DIV.W
  wire divw=(op3R==17'b00000000001000000);
  //DIV.WU
  wire divwu=(op3R==17'b00000000001000010);
  //MOD.W
  wire modw=(op3R==17'b00000000001000001);
  //MOD.WU
  wire modwu=(op3R==17'b00000000001000011);
  wire Arithmetic=addw|subw|addiw|lu12iw|slt|sltu|slti|sltui|pcaddu12i|iand|ior|ixor|inor|andi|ori|xori|mulw|mulhw|mulhwu|divw|divwu|modw|modwu;


  //Shift

  //SLL.W
  wire sllw=(op3R==17'b00000000000101110);
  //SRL.W
  wire srlw=(op3R==17'b00000000000101111);
  //SRA.W
  wire sraw=(op3R==17'b00000000000110000);
  //SLLI.W
  wire slliw=(op2R8I==14'b00000000010000)&(INSTR[17:15]==3'b001);
  //SRLI.W
  wire srliw=(op2R8I==14'b00000000010001)&(INSTR[17:15]==3'b001);
  //SRAI.W
  wire sraiw=(op2R8I==14'b00000000010010)&(INSTR[17:15]==3'b001);
  wire Shift=sllw|srlw|sraw|slliw|srliw|sraiw;

  wire reg_imm=addiw|andi|ori|xori|slti|sltui;  //as itype_r

  //Jump
  //JIRL
  wire jirl=(op2R16I==6'b010011);
  //B

  wire b=(op2R16I==6'b010100);
  //BL
  wire bl=(op2R16I==6'b010101);
  //BEQ
  wire beq=(op2R16I==6'b010110);
  //BNE
  wire bne=(op2R16I==6'b010111);
  //BLT
  wire blt=(op2R16I==6'b011000);
  //BGE
  wire bge=(op2R16I==6'b011001);
  //BLTU
  wire bltu=(op2R16I==6'b011010);
  //BGEU
  wire bgeu=(op2R16I==6'b011011);

  wire sb=beq|bne|blt|bge|bltu|bgeu;


  //MEM

  //LD.B
  wire ldb=(op2R12I==10'b0010100000);
  //LD.H
  wire ldh=(op2R12I==10'b0010100001);
  //LD.W
  wire ldw=(op2R12I==10'b0010100010);

  //ST.B
  wire stb=(op2R12I==10'b0010100100);
  //ST.H
  wire sth=(op2R12I==10'b0010100101);
  //ST.W
  wire stw=(op2R12I==10'b0010100110);
  //LD.BU
  wire ldbu=(op2R12I==10'b0010101000);
  //LD.HU
  wire ldhu=(op2R12I==10'b0010101001);
  //PRELD
  wire preld=(op2R12I==10'b0010101011);

  wire ld=ldh|ldw|ldbu|ldhu|ldb;
  wire st=sth|stw|stb;


  //ITOM_MEM
  //LL.W
  wire llw=(op2R14I==8'b00100000);
  //SC.W
  wire scw=(op2R14I==8'b00100001);

  //FENCE
  //DBAR
  wire dbar=(op3R==17'b00111000011100100);
  //IBAR
  wire ibar=(op3R==17'b00111000011100101);
  wire [14:0] hint=INSTR[14:0];

  //OTHER
  //BREAK
  assign break=(op3R==17'b00000000001010100);
  //SYSCALL
  assign syscall=(op3R==17'b00000000001010110);

  //counter
  //RDCNTID.W
  wire rdcntidw=(op2R==22'b0000000000000000011000)&(r1==5'b00000);
  //RDCNTVL.W
  wire rdcntvlw=(op2R==22'b0000000000000000011000)&(r2==5'b00000);
  //RDCNTVH.W
  wire rdcntvhw=(op2R==22'b0000000000000000011001)&(r2==5'b00000);
  wire rdc=rdcntidw|rdcntvlw|rdcntvhw;

//PLV
  //CSR
  wire csrrd=(op2R14I==8'b00000100)&(r2==5'b00000);
  wire csrwr=(op2R14I==8'b00000100)&(r2==5'b00001);
  wire csrxchg=(op2R14I==8'b00000100)&(r2!=5'b00000)&(r2!=5'b00001);
  wire csr=csrrd|csrwr|csrxchg;

  //Cache maintenance
  wire cacop=(op2R12I==10'b0000011000);

  //TLB maintenance
  wire tlbsrch=(op2R==22'b0000011001001000001010)&(r2==5'b00000)&(r1==5'b00000);
  wire tlbrd=(op2R==22'b0000011001001000001011)&(r2==5'b00000)&(r1==5'b00000);
  wire tlbwr=(op2R==22'b0000011001001000001100)&(r2==5'b00000)&(r1==5'b00000);
  wire tlbfill=(op2R==22'b0000011001001000001101)&(r2==5'b00000)&(r1==5'b00000);
  wire invtlb=(op3R==17'b00000110010010011);


  //other
  wire ertn=(op2R==22'b0000011001001000001110)&(r2==5'b00000)&(r1==5'b00000);
  wire idle=(op3R==17'b00000110010010001);

  wire ine=~(cacop|tlbsrch|tlbrd|tlbwr|tlbfill|invlib|ertn|idle|csr|rdc|break|syscall|
              dbar|ibar|preld|llw|scw|st|ld|sb|jirl|Shift|Arithmetic);

//**********INSTR end**********//

//**********CONTROL signals**********//

  // generate control signals
  assign MemRead    = ld|llw;
  assign RegWrite   = Arithmetic|Shift|jirl|bl|ld|llw|rdcntvlw|rdcntvhw|rdcntidw|csr;//|csr; // register write
  assign MemWrite   = st|scw|bl;                           // memory write
  assign ALUSrcA    = lu12iw ? 2'b01 :(pcaddu12i ? 2'b10 : 2'b00);
  assign ALUSrcB    = reg_imm | st | jirl | b | bl |lu12iw|pcaddu12i | ld|slliw|srliw|sraiw;   // ALU B is from instruction immediate

  // signed extension
  // EXT_CTRL_ITYPE_SHAMT     6'b100000
  // EXT_CTRL_ITYPE	      6'b010000
  // EXT_CTRL_STYPE	      6'b001000
  // EXT_CTRL_BTYPE	      6'b000100
  // EXT_CTRL_UTYPE	      6'b000010
  // EXT_CTRL_JTYPE	      6'b000001
  assign EXTOp[5]    =  slliw|sraiw|srliw;  //5
  assign EXTOp[4]    =  reg_imm| ld|preld|cacop;  //12
  assign EXTOp[3]    = st; //12
  assign EXTOp[2]    = sb; //16
  assign EXTOp[1]    = lu12iw|pcaddu12i;   //20
  assign EXTOp[0]    = b|bl|andi|ori|xori;         //26
  

  //counterop
  //counterop_readid   2'b00
  //counterop_readlow  2'b01
  //counterop_readhigh 2'b10
  // assign counterop[0]=rdcntvlw;
  // assign counterop[1]=rdcntvhw;
  
  
  // WDSel_FromALU 2'b00
  // WDSel_FromMEM 2'b01
  // WDSel_FromPC  2'b10 
  assign WDSel[0] = ld|llw;
  assign WDSel[1] = b|bl|jirl;

  // NPC_PLUS4   3'b000
  // NPC_BRANCH  3'b001
  // NPC_JUMP    3'b010
  // NPC_JALR	3'b100
  //����Branch ����Ҫ��EX����MEM�׶ζ����ж�Zero��

  assign NPCOp[0] = sb & Jump| jirl;
  assign NPCOp[1] = b|bl|jirl;
  //assign NPCOp[2] = b|bl;用于区分b和bl
  
/*
`define ALUOp_beq  5'b10010
`define ALUOp_bne  5'b00101
`define ALUOp_blt  5'b00110
`define ALUOp_bge  5'b00111
`define ALUOp_bltu 5'b01000
`define ALUOp_bgeu 5'b01001
*/
 
	assign ALUOp[0] = jirl|ld|st|addiw|addw|ori|ior|sllw|slliw|sraw|sraiw|sltu|sltui|lu12iw|bne|bge|bgeu|inor|divwu|modwu|mulhw;
	assign ALUOp[1] = jirl|ld|st|addiw|addw|iand|andi|sllw|slliw|slt|slti|sltu|sltui|pcaddu12i|blt|bge|beq|inor|modw|modwu|mulhwu;
	assign ALUOp[2] = andi|iand|ori|ior|subw|ixor|xori|sllw|slliw|bne|blt|bge|divw|divwu|modw|modwu;
	assign ALUOp[3] = andi|iand|ori|ior|ixor|xori|sllw|slliw|slt|slti|sltu|sltui|bltu|bgeu|mulw|mulhw|mulhwu;
	assign ALUOp[4] = sraw|sraiw|srlw|srliw|beq|inor|divw|divwu|modw|modwu|mulw|mulhw|mulhwu;

   //DMType
   assign DMType[2]=ldbu;
   assign DMType[1]=ldb | ldhu| stb;
   assign DMType[0]=ldh | ldb |sth| stb;  

   //rs2Zero
   assign rs2Zero=ld |reg_imm | lu12iw|pcaddu12i|b|bl;

  assign SignalsD={NPCOp[1], NPCOp[0],ALUSrcA, ALUSrcB, ALUOp,RegWrite, MemWrite,MemRead, DMType,WDSel};//如果要区分b和bl，需要加一位
   //PLV type
   //PLV_exception 8'b00000001
   //PLV_csr       8'b00000010
   //PLV_cache     8'b00000100
   //PLV_tlb       8'b00001000
   //PLV_invlib    8'b00010000
   //PLV_idle      8'b00100000
   //PLV_ertn      8'b01000000
   assign plv[0]=break|syscall|ine;
   assign plv[1]=csrrd|csrwr|csrxchg|break;
   assign plv[2]=cacop|syscall|csrrd;
   assign plv[3]=tlbsrch|tlbrd|tlbwr|tlbfill|csrwr|csrxchg;
   assign plv[4]=invtlb|csrxchg|tlbsrch;
   assign plv[5]=0;//idle_level|tlbrd;
   assign plv[6]=ertn|tlbwr;
   assign plv[7]=tlbfill;


   //PLV_relate_data
//  assign exception=break|syscall; //重复
//  assign exp_code=INSTR[14:0];  //exception模块内处理
 assign csr_num=INSTR[23:10];   //csr模块内处理
//  assign cacopcode=r1;            //下面三条cache模块内处理
//  assign cache_target=cacopcode[2:0]; //code[2:0]=0 表示操作一级私有指令 Cache，code[2:0]=1 表示操作一级私有数据 Cache，code[2:0]=2 表示操作二级共享混合 Cache
//  assign cache_op=cacopcode[4:3]; //
//  assign invlib_op=r1; //ref p59  
// assign idle_level=INSTR[14:0];

  assign rdcntidw_yes=rdcntidw;
  
  
  //**********CONTROL signals end**********//

endmodule
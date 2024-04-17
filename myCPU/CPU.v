//=====================================================================
// Xgriscv_CPU
// Designer   : Zhentao He, Xiaoyi Lin, Zhangyi Hu,Yilin Zhang
// Adapted from and refined Professor Zhaohui Cai and Yili Gong's original code. 
// Description:
// 	As part of the project of Computer Organization Experiments, Wuhan University
// 	In July 2023
// 	A pipelined CPU with full forwarding and all hazards units. It branches in ID unit, and has the static prediction of not taken.
//	Without Cache.
// ====================================================================
`include "bus.v"
`include "ctrl_encode_def.v"

module CPU (
    //input 
    input              clk,
    input              reset,
    input  [`INST_BUS] instr,       // from instructon memory
    input  [`DATA_BUS] readdataM,   // from data memory: read data(raw)

    input  TLBR,                    // from tlb: tlb refill
    //output
    output wire [`ADDR_BUS] pcF,         // to instruction memory
    output [`ADDR_BUS] this_pcF,     // last PC (for debug use)
    output [`DATA_BUS] Addr_out,    // to data memory: address
    output [`DATA_BUS] writedataM,  // to data memory: write data
    output             memwriteM,   // to data memory: write enable
    output [      3:0] wea,         // to data memory: this time write length
    output inst_sram_en,            // to instruction memory: enable signal
    //debug use
    output  [`ADDR_BUS] pcW_out,
    output  [      3:0] wb_rf_we,    // RF write enable
    output  [      4:0] reg_sel,     // register selection (for debug use)
    output  [`DATA_BUS] reg_data     // selected register data (for debug use)
);



///////////////////////////////////////////////////////////////////////////////////
  wire regwriteW;
  assign wb_rf_we = {4{regwriteW}};
  wire [`DATA_BUS]compare_data1D,compare_data2D;
  wire [17:0] SignalsD;
  wire flush_ins;
  wire [31:0] aluoutE;
  wire [`DATA_BUS] aluoutM;
  assign Addr_out = aluoutE;



/////////////////////exception unit///////////////////////////
  wire fs_ex;
  wire [5:0] fs_ecode;
  wire [5:0] fs_new_ecode;
  wire fs_esubcode;
  wire fs_new_esubcode;


  wire ds_ex;
  wire ds_new_ex;
  wire [5:0] ds_ecode;
  wire [5:0] ds_new_ecode;
  wire ds_esubcode;
  wire ds_new_esubcode;

  wire es_ex;
  wire es_new_ex;  
  wire [5:0] es_ecode;
  wire [5:0] es_new_ecode;
  wire es_esubcode;
  wire es_new_esubcode;

  wire ms_ex;
  wire ms_new_ex;
  wire [5:0] ms_ecode;
  wire [5:0] ms_new_ecode;
  wire ms_esubcode;
  wire ms_new_esubcode;

  wire ws_ex;
  wire ws_new_ex;
  wire [5:0] ws_ecode;
  wire [5:0] ws_new_ecode;
  wire ws_esubcode;
  wire ws_new_esubcode;

  wire [7:0]plv;
  wire [7:0]plvE;
  wire [7:0]plvM;
  wire [7:0]plvW;

  wire [31:0]csr_rvalueD;
  wire [31:0]csr_rvalueE;
  wire [31:0]csr_rvalueM;

  wire ertn;



wire [13:0] es_csr_num;
wire [13:0] ms_csr_num;
wire [13:0] ws_csr_num;


wire TI;
wire INE;
wire [1:0]SWI;
////////////////////PC///////////////
  wire  pcsrc, writenE, writenM;
  wire flushM = 0;
  wire flushD = pcsrc;
  wire [`ADDR_BUS] pcE, pcplus4E;

////////////////////EX////////////////
 wire [`DATA_BUS] srca1E, srcb1E, immoutE, srcaE, srcbE;
 wire [`DATA_BUS] srca, srcb;


////////////////forward//////////
  wire [2:0] forwardA, forwardB;
 

/////////////////////////exc///////////////////
wire syscall;
wire break;

/////////////////////////Dynamic Prediction///////////////////
wire branchTypeD=(ALUOpD==`ALUOp_beq)||(ALUOpD==`ALUOp_bne)||(ALUOpD==`ALUOp_blt)||(ALUOpD==`ALUOp_bge)||(ALUOpD==`ALUOp_bltu)||(ALUOpD==`ALUOp_bgeu);
wire PredictErrorD;
wire [`ADDR_BUS] TargetPCF;
wire branchPredictF;
DynaPredic U_DynaPredic(
 .clk(clk),.reset(reset),
 /*Update Port*/
 .branchD(branchTypeD),       /*Is branch type instruction,Input*/
 .PredictErrorD(PredictErrorD), /*(Really_branchD&PredictD)|(~Really_branchD&~PredictD),Input*/
 .Really_branchD(Is_Branch_D),
 .SourcePCD(pcD),/*For update an entry,branch source pc,Input*/
 .TargetPCD(pcbranchD),/*For update an entry,branch target pc,always put branch target(if branch),Input*/
 /*Predict Port*/
 .SearchPCF(this_pcF),/*IF stage:predict.Input*/
 .TargetPCF(TargetPCF),//Output
 .branchPredictF(branchPredictF)/*for PC module,Output*/
 /*if(PredictErrorD) PC<=RealPC;if(branchPredictF&&~PredictErrorD) PC<=TargetPCF; else PC<=PC+4*/
);

  // ====================================================================
  //IF Stage
  // ====================================================================

  assign ertn =plv[6]&~plv[3];
  assign inst_sram_en=~(reset)&&writenE;
  wire [`INST_BUS] instrF = (flush_ins|ds_ex|ertn)?32'h03400000:instr;
  // next PC logic (operates in fetch and decode)
  wire [`ADDR_BUS] pcplus4F, nextpcF, pcbranchD, pcadder2aD, pcadder2bD, pcbranch0D;  //?
  assign nextpcF=pcsrc? pcbranchD:pcplus4F;
  assign TLBR = 0;
  assign tlb_reflush = 0;

  //assign final_nextpc = TLBR ?           csr_tlbrentry_rvalue :
  //                    tlb_reflush ?                   refetch_pc : //tlb 更新时使pc重装
  //                    final_ex & ~TLBR?                   nextpc :
  //                    (br_taken_buf | ex_buf_valid) ? nextpc_buf : 
  //                                                        nextpc;
//     IF U_IF (
//       //input 
//       .clk(clk),
//       .rst(reset),
//       .PC_WRITE(writenE),
//       .PCSrc(pcsrc),
//       .PC_IF_in_jump(pcbranchD)
//      // .PC_IF(pcF),
//       //.this_pc(this_pcF)
//       //output
//   );
  wire [`ADDR_BUS] pcW;  //The pc of ins which has been fetched.
  // Fetch stage logic
  wire [31:0] csr_rvalue,ex_entry,ex_era;
  wire [31:0]pc_complete;
  PC_src_exception U_PC_src_exception (
      //input 
      .plv6(plv[6]),
      .plv3(plv[3]),
      .csr_numW(ws_csr_num),
      .csr_numM(ms_csr_num),
      .plvE(plvE),
      .plvM(plvM),
      .ertn_addressM(aluoutM),
      .ertn_addressW(reg_data),
      .ertn_address(ex_era),
      .ecode(ds_ecode),
      .esubcode(ds_esubcode),
      .eentry(ex_entry),
      .nextpc(nextpcF),
      //output
      .pc_complete(pc_complete)
  );


  PC U_PC (
      //input 
      .clk(clk),
      .rst(reset),
      .en(writenE),  //允许写入下一个PC的来自EX阶段的使能Signal
      .NPC(pc_complete), //传入下一个PC 32位宽的address,来自IF阶段产生下一个PC的功能单元NPC
      //output
      .PC(pcF),           //产生IF阶段的PC信号,for instruction fetch
      .this_pc(this_pcF),  //当前PC
      .pcplus4(pcplus4F)
  );  //当前PC+4,作用于IF阶段
  
  //需要逻辑链接（里面的信号也需要声明）

    wire adef_ex;

  wire [31:0] pc_remain_one_cycle;
  wire [31:0] pc_remain_two_cycles;
  PC_remain U_pc_remain(
    .clk(clk),
    .rst(reset),
    .flushD(flushD|fs_ex|ds_ex|ertn),
    .flush_ins(flush_ins|ds_ex|ertn),
    .pc_now(pcW),
    .pc_remain_one_cycle(pc_remain_one_cycle),
    .pc_remain_two_cycles(pc_remain_two_cycles)
  );

  ///////////////////////////////////////////////////////////////////////////////////
  // IF/ID pipeline registers
  ///////////////////////////////////////////////////////////////////////////////////
  wire [`INST_BUS] instrD;
  wire [`ADDR_BUS] pcD, pcplus4D;
  wire branchPredictD;

  Flush_ins U_flushins(
    .clk(clk),
    .rst(reset),
    .flush(flushD|fs_ex|ds_ex|ertn),
    .flush_ins(flush_ins)
  );

  //Three Signals


wire [31:0]pc_in_final;

assign pc_in_final=flushD?(flush_ins?this_pcF:pcD):(flush_ins?pcE:this_pcF);

  //Install IF/ID
  IFID IFIDReg (
      //control input 
      .clk(clk),
      .rst(reset),
      .enable(writenE),
      .flushD(flushD),
      //input
      .addr_in(pc_in_final),
      .addrplus4_in(pcF),
      .inst_in(instrF),

      .fs_ex(fs_ex),
      .branchPredictF(branchPredictF),
      //output
      .addr_out(pcD),
      .addrplus4_out(pcplus4D),
      .inst_out(instrD),
      .branchPredictD(branchPredictD)
     );

  // ====================================================================
  //ID Stage
  // ====================================================================   

  // from controller
  wire [5:0] EXTOp;
  //wire       jalD, jalrD, jD, bD, ->Decode from NPCOpD
  wire [1:0] NPCOpD;
  wire [4:0] ALUOpD;
  wire [1:0] alusrcaD;
  wire alusrcbD;
  wire memwriteD;

  wire [1:0] WDSelD;  //memtoregD
  wire       regwriteD;
  wire [2:0] DMTypeD;
  wire       rs2ZeroD;

  // to controller
  wire zeroE, ltE;


  wire jalrD = NPCOpD[1]&NPCOpD[0];
  wire jalD = NPCOpD[1]&~NPCOpD[0];
  wire bD = NPCOpD[0]&~NPCOpD[1];  //when SB_type, bD=1

  // Decode stage logic
//   wire [`RFIDX_BUS] rs1D = instrD[19:15];
//   wire  [`RFIDX_BUS] rs2D	= rs2ZeroD?5'b00000:instrD[24:20];//When don't use rs2, rs2=0 -> for forwarding and hazard detection;
//   wire [6:0] opD = instrD[6:0];
//   wire [4:0] rdD = instrD[11:7];
//   wire [2:0] funct3D = instrD[14:12];
//   wire [6:0] funct7D = instrD[31:25];
//   wire [11:0] immD = instrD[31:20];

  wire  [`RFIDX_BUS]  rs1D    = instrD[9:5];
	wire  [`RFIDX_BUS] rs2D	=(instrD[31:24]==8'b00000100)?instrD[4:0]:
                           ((instrD[31:24]==8'b00101001)? instrD[4:0]:
                           ((instrD[31:30]==2'b01)? instrD[4:0]:
                           (rs2ZeroD?5'b00000:instrD[14:10])));//When don't use rs2, rs2=0 -> for forwarding and hazard detection;
//	wire  [6:0]  opD 	= instrD[6:0];
	wire  [4:0]  rdD     =   (instrD[31:26]==6'b010101)?5'b00001:((instrD[31:24]==8'b00101001)?instrD[14:10]:instrD[4:0]);

  // immediate generate
  wire [`DATA_BUS] immoutD;
  wire [`DATA_BUS] rdata1D, rdata2D, wdataW;
  wire [`RFIDX_BUS] waddrW;



  //generate immediate processed 
  EXT U_EXT (
      .instrD(instrD),  // 产生立即数的指令
      .EXTOp (EXTOp),   // 立即数扩展操作码
      .immout(immoutD)  // 立即数扩展result
  );

  // register file (operates in decode and writeback)
  RF U_RF (
      //input 
      .clk(clk),
      .rst(reset),
      .RFWr(regwriteW),  //来自WB阶段的的写寄存器使能信号 
      .A1(rs1D),  //来自ID阶段的rs1地址
      .A2(rs2D),  //来自ID阶段的rs2地址
      .A3(waddrW),  //来自WB阶段的rd地址
      .WD(reg_data),  //来自WB阶段的写入寄存器的数 ??
      .pc(pcW),  //The pc address of ins which is write the RF.
      //output
      .RD1(rdata1D),  //读出寄存 ??1的数 ??
      .RD2(rdata2D)  //读出寄存 ??2的数 ??
      //.reg_sel(reg_sel),
      //.reg_data(reg_data)
  );

  wire [1:0] GPRSel;
  wire memreadD;
  wire Is_Branch_D;
  // instantiation of control unit


  wire [13:0] csr_num;
  wire rdcntidw_yes;

  LOONGARCH_CTRL U_ctrl (
      //input
      .INSTR(instrD), 
      .pcF(pcF),
      .Jump(~Is_Branch_D),  //是否是跳转instruction
      //output
      .MemRead(memreadD),
      .RegWrite(regwriteD),
      .MemWrite(memwriteD),
      .EXTOp(EXTOp),
      .ALUOp(ALUOpD),
      .NPCOp(NPCOpD),
      .ALUSrcA(alusrcaD),  //从rs1,0,pc中choose
      .ALUSrcB(alusrcbD),  //从rs2,imm中choose
      .GPRSel(GPRSel),
      .WDSel(WDSelD),  //写入寄存器数据来源choose
      .DMType(DMTypeD),  //数据内存字长类型选择
      .rs2Zero(rs2ZeroD),  //是否不需要rs2
	    .SignalsD(SignalsD),  //pass to下一个阶段的控制信号
      .plv(plv),  
      .csr_num(csr_num),
      .syscall(syscall),
      .break(break),
      .INE(INE),
      .rdcntidw_yes(rdcntidw_yes)
  );

    assign ds_valid=1'b1; //xuyao houxu xiugai   
    
	  wire  has_int;
    //下一个csr往下传的信号，后面打问号的要改一下名字

    assign ds_ex=fs_ex|ds_new_ex;
    assign ds_ecode=(ds_new_ex)?ds_new_ecode:fs_ecode;
    assign ds_esubcode=(ds_new_ex)?ds_new_esubcode:fs_esubcode;
    wire [153:0] csr_chuandi={ //154-32=122
        rdcntidw_yes,  //191
        plv[6]  ,  //158
        ds_esubcode    ,  //157  ？
        ds_ecode       ,  //156:151
        ds_ex ,  //150
        plv[1]      ,  //149
        csr_num     ,  //148:135
        compare_data2D  ,  //134:103
        (instrD[31:24]==8'b00000100)&(instrD[9:5]!=5'b00000)&(instrD[9:5]!=5'b00001)?compare_data1D  : 32'hffffffff ,  //33
        (plv[1]&plv[3])     ,  //32
        pcD             //31:0
    };
  

    // wire [31:0]csr_wdata_test_int;
    // assign csr_wdata_test_int=csr_chuandi[96:65];
    // wire [6:0] csr_chuandi_ecode_esubcode_test;
    // assign csr_chuandi_ecode_esubcode_test=csr_chuandi[150:1];


  wire [2:0] forwardC, forwardD;
  mux7 #(`DATA_BUS_WIDTH) compmux1D (
      // src input 
      .d0(rdata1D),  //regfile read data1 out 
      .d1(aluoutE),  //WB stage /*aluoutW*/
      .d2(aluoutM),  //MEM stage 
      .d3(readdataM),
      .d4(pcD+4),
      .d5(csr_rvalueE),
      .d6(csr_rvalueM),
      // forward control signs
      .s(forwardC),  //待定义[2:0]
      // output 
      .Out(compare_data1D)
  );  //

  mux7 #(`DATA_BUS_WIDTH) compmux2D (
      // src input 
      .d0(rdata2D),  //regfile read data2 out 
      .d1(aluoutE),  //WB stage /*aluoutW*/
      .d2(aluoutM),  //MEM stage 
      .d3(readdataM),
      .d4(pcD+4),
      .d5(csr_rvalueE),
      .d6(csr_rvalueM),

      // forward control signs
      .s(forwardD),  //待定义[2:0]
      // output 
      .Out(compare_data2D)
  );  


  Branch_judge U_Branch_judge (
      //input 
      .Branch_in1D(compare_data1D),
      .Branch_in2D(compare_data2D),
      .Branch_judge_ctrl(ALUOpD),
      //output
      .IsBranch_D(Is_Branch_D)
  );

wire [31:0]to_pcadder;
    mux2 #(`DATA_BUS_WIDTH) addrsource_select(
        .d0(pcD),
        .d1(rdata1D),
        .s(SignalsD[17]&SignalsD[16]),
        .Out(to_pcadder)
    );
    adder pcadder(
        .a(to_pcadder),
        .b(immoutD),
        .sum(pcbranchD)
    );

  assign pcsrc = SignalsD[17] | SignalsD[16];
  ///////////////////////////////////////////////////////////////////////////////////
  // ID/EX pipeline registers
  ///////////////////////////////////////////////////////////////////////////////////
 
  wire [`RFIDX_BUS] rdE, rs1E, rs2E;

  wire [17:0] SignalsE;
  //不允许写入pc或 ? pcsrc为brach或 ? jalr时进行flush
  wire flushE =  ~writenE;  //When writenE==0 ,flushE<-1
  wire [121:0] csr_chuandiE;

  //Install ID/EX
  IDEX IDEXReg (
      //system signs input
      .clk(clk),
      .rst(reset),
      .enable(writenE),
      .flushE(flushE),
      //input data1 in ID stage 
      //output data1 in EX stage 
      .rdata1D(rdata1D),
      .rdata1E(srca1E),
      //input data2 in ID stage
      //output data2 in EX stage
      .rdata2D(rdata2D),
      .rdata2E(srcb1E),
      //input imm in ID stage
      //output imm in EX stage
      .immoutD(immoutD),
      .immoutE(immoutE),
      //input rs1 in ID stage
      //output rs1 in EX stage
      .rs1D(rs1D),
      .rs1E(rs1E),
      //input rs2 in ID stage
      //output rs2 in EX stage
      .rs2D(rs2D),
      .rs2E(rs2E),
      //input rd in ID stage
      //output rd in EX stage
      .rdD(rdD),
      .rdE(rdE),
      //input pc in ID stage
      //output pc in EX stage
      .pcD(pcD),
      .pcE(pcE),
      .pcplus4D(pcplus4D),
      .pcplus4E(pcplus4E),
      //input control signals in ID stage
      //output control signals in EX stage
      .SignalsD(SignalsD),
      .SignalsE(SignalsE),
      .csr_in(csr_chuandi),
      .csr_out(csr_chuandiE),
      .plv(plv),
      .plvE(plvE)      
  );


  //decode signalsE    
    wire jalrE=SignalsE[17]&SignalsE[16];  
    wire jalE=SignalsE[17]&~SignalsE[16];
    wire bE=SignalsE[16]&~SignalsE[17];

    wire [1:0] alusrcaE=SignalsE[15:14];
    wire alusrcbE=SignalsE[13];
    wire [4:0] ALUOpE=SignalsE[12:8];
    wire regwriteE=SignalsE[7];
    wire memwriteE=SignalsE[6];
    wire memreadE=SignalsE[5];
    wire [2:0] DMTypeE=SignalsE[4:2];
    wire [1:0] WDSelE=SignalsE[1:0];
    



  //install forward unit 

  mux5 #(`DATA_BUS_WIDTH) Forward_mux_A (
      // src input 
      .d0(srca1E),  //EX stage 
      .d1(reg_data),  //WB stage /*aluoutW*/
      .d2(aluoutM),  //MEM stage 
      .d3(csr_rvalueM),
      // forward control signs
      .s(forwardA),
      // output 
      .Out(srca)
  );  //
  mux5 #(`DATA_BUS_WIDTH) Forward_mux_B (
      // src input 
      .d0(srcb1E),  //EX stage
      .d1(reg_data),  //WB stage /*aluoutW*/ 
      .d2(aluoutM),  //MEM stage 
      // forward control signs
      .d3(csr_rvalueM),
      .s(forwardB),
      // output
      .Out(srcb)
  );  //

  // ====================================================================
  //EX Stage
  // ====================================================================
  mux3 #(`DATA_BUS_WIDTH) srcamux (
      // src input 
      srca,  //前 ? 或者原始的rs
      0,  //lui这种加载常立即数，不使用rs
      pcE,  //jal这种 ??要用到PC的指 ??
      // forward control signs
      alusrcaE,
      // output
      srcaE
  );  // alu src a mux
  mux2 #(`DATA_BUS_WIDTH) srcbmux (
      // src input
      srcb,  //前 ? 或者原始的rs
      immoutE,  //立即 ??
      // forward control signs
      alusrcbE,
      // output 
      srcbE
  );  // alu src b mux
  wire [`ADDR_BUS_WIDTH-1:0] PCoutE;

  //need a more complex alu
  alu u_alu (
      //input 
      .A       (srcaE),
      .B       (srcbE),
      .ALUOp   (ALUOpE),
      .PC      (pcE),        // for debug use
      //output
      .C_out   (aluoutE),    //输出运算结果
      .Zero    (zeroE),      //Z
      .Overflow(overflowE),  //V
      .Lt      (ltE),        //L
      .Ge      (geE)         //G
  );


//   pc_imm U_pc_imm (
//       //input 
//       .pcE(pcD),
//       .immoutE(immoutE),
//       //output
//       .PCoutE(PCoutE)     //PC + imm
//   );  //adder



//   mux2 #(`DATA_BUS_WIDTH) brmux (
//       //input src
//       aluoutE,
//       PCoutE,
//       //input control signs
//       ~(SignalsE[17]&SignalsE[16]),
//       //output jump address(pc)
//       pcbranchD
//   );  // pcsrc mux	

    //assign es_ex=ds_ex|es_new_ex;
    wire es_inst_rdcntid=csr_chuandiE[121];
    wire es_ertn_flush=csr_chuandiE[120];
    assign es_esubcode=(es_new_ex)?es_new_esubcode:csr_chuandiE[119];
    assign es_ecode=(es_new_ex)?es_new_ecode:csr_chuandiE[118:113];
    wire csr_we=csr_chuandiE[70];
    wire es_csr_re=csr_chuandiE[111];
    assign es_csr_num=csr_chuandiE[110:97];
    wire [31:0] es_csr_wvalue=csr_chuandiE[96:65];
    wire [31:0] es_csr_wmask=csr_chuandiE[64:33];
    wire es_csr_we=csr_chuandiE[32];
    wire [31:0] es_pc=csr_chuandiE[31:0];

         //下面信号要适配
    wire [31:0] vaddr = aluoutE;
   
    wire [31:0] badvaddr = csr_chuandiE[112] ? es_pc : aluoutE;
    assign  es_ex = csr_chuandiE[112]|es_new_ex ;//还没tlb
                // |es_tlb_load_invalid_ex | es_tlb_store_invalid_ex | es_tlb_modify_ex |
                //  es_tlb_ppe_ex | es_tlb_refill_ex;




  hazard U_hazard (
      //input
      .clk    (clk),
      .WDSel  (WDSelE),   // ??测是否有ld
      .rdE    (rdE),      // ??测是否有
      .rs1D   (rs1D),
      .rs2D   (rs2D),
      .writenM(writenM),  //mem阶段的寄存器写使能信 ??
      //output 
      .writen (writenE)   //EX阶段的寄存器写使能信 ??
  );

  ///////////////////////////////////////////////////////////////////////////////////
  // EX/MEM pipeline registers
  ///////////////////////////////////////////////////////////////////////////////////
  // for control signals
  wire memreadM, regwriteM;
  wire [1:0] WDSelM;
  wire [2:0] DMTypeM;
  wire [`DATA_BUS] srcb1M;
  wire [`ADDR_BUS] PCoutM, pcM;
  // for data
  wire [`ADDR_BUS] pcplus4M;
  wire [`RFIDX_BUS] rdM;
  wire [7:0] CtrM;
  wire [153:0] csr_chuandiM;

wire [153:0] es_to_ms_bus = {badvaddr       ,  //210:179
                    //   es_mem_ex      ,  //178
                    //    es_tlb_refill_ex, //177
                    //    es_tlb_refetch ,  //176
                    //    inst_tlbsrch   ,  //175
                    //    inst_tlbrd     ,  //174
                    //    inst_tlbwr     ,  //173
                    //    inst_tlbfill   ,  //172
                    //    inst_invtlb    ,  //171
                    //    es_op_st_w     ,  //170
                       es_inst_rdcntid,  //169
                       es_ertn_flush  ,  //168
                       es_esubcode    ,  //167
                       es_ecode       ,  //166:161
                       es_ex ,  //160
                       es_csr_re      ,  //159
                       es_csr_num     ,  //158:145
                       es_csr_wvalue  ,  //144:113
                       es_csr_wmask   ,  //112:81
                       es_csr_we      ,  //80
                    //    data_sram_addr[1:0], //79:78
                    //    es_op_ld_w     ,  //77
                    //    es_op_ld_b     ,  //76
                    //    es_op_ld_bu    ,  //75
                    //    es_op_ld_h     ,  //74
                    //    es_op_ld_hu    ,  //73
                    //    es_op_st_b     ,  //72
                    //    es_op_st_h     ,  //71
//                       es_res_from_mem & ~ale_ex,  //70:70
                    //    es_gr_we       ,  //69:69
                    //    es_dest        ,  //68:64
                    //    es_final_result  ,  //63:32
                       es_pc             //31:0
                      };


  wire [31:0]writedataMt;
  //assign writedataM=srcb;

  EXMEM EXMEMReg (
      //system input signs
      clk,
      reset,
      flushM,
      //control signs from EX stage and output to MEM stage 
      SignalsE[7:0],
      CtrM,
      writenE,  //input
      writenM,  //output
      //data from EX stage and output to MEM stage 
      srcb1E,  //input  data_rs2 into EX/MEM 
      srcb1M,  //output data_rs2 out of EX/MEM
      aluoutE,  //input aluout from EX stage into EX/MEM
      aluoutM,  //output aluout from EX/MEM to MEM stage
      // chosen mem write data (from rs2(forward) or imm)
      srcb,  //input
      writedataMt,  //output 
      //indexx rd
      rdE,  //input 
      rdM,  //output 
      //pc from EX stage to MEM stage
      pcE,  //input  PC
      pcM,  //output PC
      pcplus4E,  //input  PC+4
      pcplus4M,  //output PC+4
      PCoutE,  //input  PC+IMM
      PCoutM,  //output PC+IMM
      es_to_ms_bus,  //input  csr_chuandi
      csr_chuandiM,  //output csr_chuandi
      plvE,
      plvM
  );

  //decode the signs in Mem stage

  assign regwriteM = CtrM[7];
  assign memwriteM = SignalsE[6];
  assign memreadM = CtrM[5];
  assign DMTypeM = CtrM[4:2];
  assign WDSelM = CtrM[1:0];

  dm_weaGen U_DM_WeaGen(
    //input
    .DMWr(memwriteE),
    .DMType(DMTypeE),
    .Addr_loc(aluoutE[1:0]),
    .DM_DataRaw(srcb),
    //output
    .DM_WriteData(writedataM),
    .wea(wea)
  );

  // ====================================================================
  //MEM Stage
  // ====================================================================
  wire [`DATA_BUS] dmoutM;
  dm_controller U_DM_CTRL( //根据DMTypeM生成WEA信号，以及对读出的数据进行符号扩 ??
        //input
        .DMType(DMTypeM),
        .dout_raw(readdataM),
        .Addr_loc(aluoutM[1:0]),
        //output
        .dout(dmoutM)
  );



    wire ms_inst_rdcntid=csr_chuandiM[121];
    wire [31:0] ms_badvaddr=csr_chuandiM[153:122];
    wire ms_ertn_flush=csr_chuandiM[120];
    assign ms_esubcode=(ms_new_ex)?ms_new_esubcode:csr_chuandiM[119];
    assign ms_ecode=(ms_new_ex)?ms_new_ecode:csr_chuandiM[118:113];
    assign  ms_ex=csr_chuandiM[112]|ms_new_ex ;
    wire ms_csr_re=csr_chuandiM[111];
    assign  ms_csr_num=csr_chuandiM[110:97];
    wire [31:0] ms_csr_wvalue=csr_chuandiM[96:65];
    wire [31:0] ms_csr_wmask=csr_chuandiM[64:33];
    wire ms_csr_we=csr_chuandiM[32];
    wire [31:0] ms_pc=csr_chuandiM[31:0];



  ///////////////////////////////////////////////////////////////////////////////////
  // MEM/WB pipeline registers
  ///////////////////////////////////////////////////////////////////////////////////
  wire [`RFIDX_BUS] rdW;
  wire [ `ADDR_BUS] pcplus4W;
  wire [ `ADDR_BUS] PCoutW;
  wire [`DATA_BUS] aluoutW, dmoutW;


  Forward U_Forward (
    //input 
    .regwriteM(regwriteM),
    .memreadM(memreadM),
    .memreadW(memreadW),
    .regwriteW(regwriteW),
    .memreadE(memreadE),
    .regwriteE(regwriteE),
    .rs1D(rs1D),
    .rs2D(rs2D),
    .rs1E(rs1E),
    .rs2E(rs2E),
    .rdE(rdE),
    .rdM(rdM),
    .rdW(rdW),
    .plvE(plvE),
    .plvM(plvM),

    .forwardA(forwardA),
    .forwardB(forwardB),
    .forwardC(forwardC),
    .forwardD(forwardD)


  );


assign ms_to_ws_bus = {//es_tlb_refill_ex, //198
                    //    ms_tlb_refetch ,  //197
                    //    inst_tlbsrch   ,  //196
                    //    inst_tlbrd     ,  //195
                    //    inst_tlbwr     ,  //194
                    //    inst_tlbfill   ,  //193
                    //    inst_invtlb    ,  //192
                       ms_inst_rdcntid,  //191
                       badvaddr       ,  //190:159
                       ms_ertn_flush  ,  //158
                       ms_esubcode    ,  //157
                       ms_ecode       ,  //156:151
                       ms_ex          ,  //150
                       ms_csr_re      ,  //149
                       ms_csr_num     ,  //148:135
                       ms_csr_wvalue  ,  //134:103
                       ms_csr_wmask   ,  //102:71
                       ms_csr_we      ,  //70
                    //    ms_gr_we       ,  //69:69
                    //    ms_dest        ,  //68:64
//                       ms_final_result,  //63:32
                       ms_pc             //31:0
                      };


  //encode the control signs in WB stage
  wire [3:0] CtrMW = {memreadM,regwriteM, WDSelM};
  wire [3:0] CtrWB;
  wire [153:0] csr_chuandiW;

  MEMWB MEMWBReg (
      //input 
      clk,
      reset,
      1'b0  /*flushW*/,  //default:not flush
      //control signs from MEM stage and output to WB stage
      CtrMW,  //input 
      CtrWB,  //output 
      //read data from memory waited for selection
      dmoutM,  //input
      dmoutW,  //output 
      aluoutM,  //input
      aluoutW,  //output
      //rd
      rdM,  //input 
      rdW,  //output 
      // all kinds of pc
      pcM,  //input 
      pcW,  //output
      pcplus4M,  //input 
      pcplus4W,  //output 
      PCoutM,  //input 
      PCoutW,  //output 
      csr_chuandiM,  //input
      csr_chuandiW,  //output
      plvM,
      plvW
  );

  wire [31:0] ws_csr_mvalue_test;
  assign ws_csr_mvalue_test=csr_chuandiM[96:65];
  //decode the signs in WB stage

  assign memreadW=CtrWB[3];
  assign regwriteW = CtrWB[2];
  wire [1:0] WDSelW = CtrWB[1:0];

  // ====================================================================
  //WB Stage
  // ====================================================================
    wire refill_ex;
    wire ws_inst_rdcntid;
    wire [31:0] ws_vaddr;
    wire ws_ertn_flush;


    wire ws_csr_re;

    wire [31:0] ws_csr_wvalue;
    wire [31:0] ws_csr_wmask;
    wire ws_csr_we;
    wire [31:0] ws_pc;
  assign {//refill_ex, //198
 //       s1_index       ,  //201:198
//        ws_tlb_refetch ,  //197
//        inst_tlbsrch   ,  //196
//        inst_tlbrd     ,  //195
//        inst_tlbwr     ,  //194
//        inst_tlbfill   ,  //193
//        inst_invtlb    ,  //192
        ws_inst_rdcntid,  //191
        ws_vaddr       ,  //190:159
        ws_ertn_flush  ,  //158
        ws_esubcode    ,  //157
        ws_ecode       ,  //156:151
        ws_ex          ,  //150
        ws_csr_re      ,  //149
        ws_csr_num     ,  //148:135
        ws_csr_wvalue  ,  //134:103 //
        ws_csr_wmask   ,  //102:71
        ws_csr_we      ,  //70
//        ws_gr_we       ,  //69:69
//        ws_dest        ,  //68:64
//        ws_final_result,  //63:32
        ws_pc             //31:0
  } = csr_chuandiW;

  wire [31:0] ws_csr_wvalue_test;
  assign ws_csr_wvalue_test=csr_chuandiW[96:65];
    wire  final_ex         = (ws_ex | ws_ertn_flush );//refill_ex



	wire [63:0] counter;
	wire [97:0] csr_tlb_in,csr_tlb_out;
	wire [31:0] csr_asid_rvalue,csr_tlbehi_rvalue,csr_crmd_rvalue,csr_dmw0_rvalue,csr_dmw1_rvalue,csr_tlbrentry_rvalue;
	
  
  
  
  U_CSR U_csr(
		.clk(clk), 
        .rst(reset),
        //input
		.csrWR(ws_csr_we), 
        .csr_num(ws_csr_num),
        .csr_numE(es_csr_num),
        .csr_numM(ms_csr_num),
        .csr_wmask(ws_csr_wmask ),
        .csr_wvalue(ws_csr_wvalue),
        //output
        .csr_rvalue(csr_rvalue),
        .csr_rvalueE(csr_rvalueE),
        .csr_rvalueM(csr_rvalueM),
        //input
		.ertn_flush(ws_ertn_flush), 
        .wb_ex(final_ex),
        .refill_ex(0),//refill_ex //空置还没tlb
        .wb_ecode(ws_ecode),
        .wb_esubcode(ws_esubcode),
		.wb_pc(pcW),
        .wb_vaddr(ws_vaddr),
        //output
        .ex_entry(ex_entry),
        .ex_era(ex_era),
        .has_int(has_int),
		.counter(counter),
                //tlb 暂时空置
		.csr_tlb_in(csr_tlb_in),
        .csr_tlb_out(csr_tlb_out),
        .csr_asid_rvalue(csr_asid_rvalue),
        .csr_tlbehi_rvalue(csr_tlbehi_rvalue),
		.csr_crmd_rvalue(csr_crmd_rvalue),
        .csr_dmw0_rvalue(csr_dmw0_rvalue),
        .csr_dmw1_rvalue(csr_dmw1_rvalue),
        .csr_tlbrentry_rvalue(csr_tlbrentry_rvalue),
    .TI(TI),
    .SWI(SWI)
	);
    
 

  mux4 #(`DATA_BUS_WIDTH) wdatamux (
      //src data input 
      .d0(aluoutW),
      .d1(dmoutW),
      .d2(pcplus4W),

      // register write data select control
      .s(WDSelW),
	  //output 
      .Out(wdataW)
  );


    exception U_exc(
      .clk(clk),
      .rst(reset),
      .IPI(1'b0),
      .TI(TI),
      .HWI(3'b0),
      .SWI(SWI),
      .PIL(1'b0),
      .PIS(1'b0),
      .PIF(1'b0),
      .PME(1'b0),
      .PPI(1'b0),
      .syscall(syscall),
      .break(break),
      .INE(INE),
      .IPE(1'b0),
      .TLBR(1'b0),

      .Fs_ecode(fs_ecode),
      .Fs_esubcode(fs_esubcode),
      .Fs_new_ex(fs_ex),

      .Ds_new_ecode(ds_new_ecode),
      .Ds_new_esubcode(ds_new_esubcode),
      .Ds_new_ex(ds_new_ex),

      .Es_new_ecode(es_new_ecode),
      .Es_new_esubcode(es_new_esubcode),
      .Es_new_ex(es_new_ex),

      .Ms_new_ecode(ms_new_ecode),
      .Ms_new_esubcode(ms_new_esubcode),
      .Ms_new_ex(ms_new_ex),

      .Ws_new_ecode(ws_new_ecode),
      .Ws_new_esubcode(ws_new_esubcode),
      .Ws_new_ex(ws_new_ex),

      

       .PC(final_nextpc),
       .addr(aluoutE),
       .load_or_store(memreadE|memwriteE),
       .es_adem_ex(es_adem_ex),
       .ale_ex(ale_ex)
    );


  assign pcW_out = pcW;
  assign reg_sel = rdW;
  assign reg_data = {32{ws_csr_re}} & csr_rvalue |
                    {32{~ws_csr_re}} & wdataW;
  assign waddrW = rdW;  //register destination [4:0]
endmodule

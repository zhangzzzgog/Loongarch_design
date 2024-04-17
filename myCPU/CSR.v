module U_CSR(clk,rst,csrWR,csr_num,csr_numE,csr_numM,csr_wmask,csr_wvalue,csr_rvalue,csr_rvalueE,csr_rvalueM,
            ,ertn_flush,wb_ex,refill_ex,wb_ecode,wb_esubcode,wb_pc,wb_vaddr
            ,ex_entry,ex_era,has_int,counter
            ,csr_tlb_in,csr_tlb_out,csr_asid_rvalue,csr_tlbehi_rvalue,csr_crmd_rvalue,csr_dmw0_rvalue,csr_dmw1_rvalue,csr_tlbrentry_rvalue
            ,TI,SWI
    );
    input clk,rst,csrWR;      //写信号
    input [13:0] csr_num;  //CSR地址
    input [13:0] csr_numE;
    input [13:0] csr_numM;
//    input [31:0] data;
    input  [31:0] csr_wmask ; //写掩�??
    input  [31:0] csr_wvalue; //写数�??
    output [31:0] csr_rvalue; //读返回�??
    output [31:0] csr_rvalueE;
    output [31:0] csr_rvalueM;
    wire ws_valid=1;
    input         ertn_flush;//ertn执行有效信号
    input         wb_ex     ;//写回流水级??
    input         refill_ex ;//重填异常
    input  [ 5:0] wb_ecode  ; //异常类型
    input         wb_esubcode;//
    input  [31:0] wb_pc     ; //!!!!

    input  [31:0] wb_vaddr  ; //访存虚地�??

    output [31:0] ex_entry  ; //to pre-IF, 异常处理入口地址
    output [31:0] ex_era    ;
    output        has_int   ; //to id, 中断有效信号
    // counter
    output [63:0] counter   ;
    // tlb
    input  [97:0] csr_tlb_in;
    output [97:0] csr_tlb_out;
    output [31:0] csr_asid_rvalue;
    output [31:0] csr_tlbehi_rvalue;
    output [31:0] csr_crmd_rvalue;
    output [31:0] csr_dmw0_rvalue;
    output [31:0] csr_dmw1_rvalue;
    output [31:0] csr_tlbrentry_rvalue;
    output reg TI;
    output [1:0] SWI;
    //0x0 CRMD
    //0x1 PRMD
    //0x2 EUEN
    //0x4 ECFG
    //0x5 ESTAT
    //0x6 ERA
    //0x7 BADV  //TLB
    //0xc EENTRY
    //0x10 TLBIDX  //TLB
    //0x11 TLBEHI  //TLB
    //0x12 TLBELO0  //TLB
    //0x13 TLBELO1  //TLB
    //0x18 ASID  //TLB
    //0x19 PGDL  //TLB
    //0x1a PGDH  //TLB
    //0x1b PGD  //TLB
    //0x20 CPUID
    //0x30-0x33 SAVE0-SAVE3
    //0x40 TID
    //0x41 TCFG
    //0x42 TVAL
    //0x44 TICLR
    //0x60 LLBCTL
    //0x88 TLBRENTRY  //TLB
    //0x98 CTAG
    //0x180-0x181 DMW0-DMW1

    reg [31:0] csr[385:0];
    wire [8:0] CPUID = 8'b0;//CPUID  处理器核编号9位;
    wire [31:0] init_counter_val = 32'hfffffffe;  //n=31
    integer i;


wire        inst_tlbfill;
wire        inst_tlbsrch;
wire        inst_tlbrd;
wire        inst_tlbwr;
wire        s1_found;
wire [ 3:0] s1_index;
wire        we;
wire [ 3:0] w_index;
wire        w_e;
wire [18:0] w_vppn;
wire [ 5:0] w_ps;
wire [ 9:0] w_asid;
wire        w_g;
wire [19:0] w_ppn0;
wire [ 1:0] w_plv0;
wire [ 1:0] w_mat0;
wire        w_d0;
wire        w_v0;
wire [19:0] w_ppn1;
wire [ 1:0] w_plv1;
wire [ 1:0] w_mat1;
wire        w_d1;
wire        w_v1;
wire [ 3:0] r_index;
wire        r_e;
wire [18:0] r_vppn;
wire [ 5:0] r_ps;
wire [ 9:0] r_asid;
wire        r_g;
wire [19:0] r_ppn0;
wire [ 1:0] r_plv0;
wire [ 1:0] r_mat0;
wire        r_d0;
wire        r_v0;
wire [19:0] r_ppn1;
wire [ 1:0] r_plv1;
wire [ 1:0] r_mat1;
wire        r_d1;
wire        r_v1;

assign  {//es_valid,    //96
          inst_tlbwr,//97
          inst_tlbfill,//96
          inst_tlbsrch,//95
          inst_tlbrd,  //94
          s1_found,    //93
          s1_index,    //92:89
          r_e,         //88
          r_vppn,      //87:69
          r_ps,        //68:63
          r_asid,      //62:53
          r_g,         //52
          r_ppn0,      //51:32
          r_plv0,      //31:30
          r_mat0,      //29:28
          r_d0,        //27
          r_v0,        //26
          r_ppn1,      //25:6
          r_plv1,      //5:4
          r_mat1,      //3:2
          r_d1,        //1
          r_v1         //0
        } = csr_tlb_in ;

assign csr_tlb_out   = {we,     //97
                        w_index,//96:93
                        w_e,    //92
                        w_vppn, //91:73
                        w_ps,   //72:67
                        w_asid, //66:57
                        w_g,    //56
                        w_ppn0, //55:36
                        w_plv0, //35:34
                        w_mat0, //33:32
                        w_d0,   //31
                        w_v0,   //30
                        w_ppn1, //29:10
                        w_plv1, //9:8
                        w_mat1, //7:6
                        w_d1,   //5
                        w_v1,   //4
                        r_index //3:0
                       };



    //初始化
    always @(posedge clk, posedge rst)begin
        if (rst) begin    //  reset
            for (i=0; i<386; i=i+1)
                csr[i] <= 0; 
            csr[0] <= {23'b0,2'b0,2'b0,2'b01,1'b0,2'b00};//CRMD  0位,DATM,DATF,PG,DA,IE,PLV
//            csr[1] <= {29b'0,1b'0,2b'0};//PRMD   0位,PIE,PPLV
//            csr[2] //EUEN 浮点数使能
//            csr[4] //ECFG  0位,LIE[12:11],0位,LIE[9:0]每一位控制一个中断源
//            csr[5] //ESTAT
//            csr[6] //ERA  PC返回地址
//            csr[7] //BADV  VAddr出错虚地址
            csr[12]<={8'h1c,8'h0,4'h8,12'h0}; //EENTRY  例外和中断入口地址，低6位为0，置为0位
//            csr[16]  //TLBIDX   NE,0,PS,0,index  //TLB index域位宽不大于16(n<16) 
//            csr[17] //TLBEHI  VPPN,13位0
//            csr[18] //TLBELO0
//            csr[19] //TLBELO1
            csr[24] <= {8'b0,8'b1010,16'b0}; //ASID  0位，asid位宽值（10），0位，asid值
            // csr[25] //PGDL
            // csr[26] //PGDH
            // csr[27] //PGD
            csr[32] <= CPUID;//CPUID  处理器核编号9位
            //定时器
            csr[64] <= CPUID;//TID  硬件可以将其复位成与 CSR.CPUID中 CoreID 相同的值
            csr[65] <= {1'b0,init_counter_val[30:2],2'b10};//TCFG    定时器配置  0位,初始值，控制位，使能位
            csr[66] <= init_counter_val;//TVAL  定时器数值  0位，计数值
//            csr[68] //TICLR   定时中断清除   0位，CLR clockreset
//            csr[96] //LLBCTL  LLbit  KLO,WCLLB,ROLLB
            // csr[136] //TLBRENTRY
            // csr[152] //CTAG 该寄存器用于 CACOP 指令直接访问 Cache 时，存放从 Cache Tag 中读出的内容或是将要写入 Cache Tag 的内容
            // csr[384] //DMW0
            // csr[385] //DMW1
            TI=0;
        end
     if(wb_ex | refill_ex) begin 
        //crmd   
        if(refill_ex) begin
            csr[0][4:3] <= 2'h01;
        end
        csr[0][1:0] <= 2'b0; //when exception happens, set highest priority level
        csr[0][2] <= 1'b0; //when exception happens, set interrupt disenble, to mask interrupt
        //prmd
        if(csr[0][2:0]!=3'b000) begin
            csr[1][1:0] <= csr[0][1:0]; //存储crmd状态
            csr[1][2] <= csr[0][2]; //
        end
        //estat,era
        if(~ertn_flush) begin
            //estat
            csr[5][21:16]    <= wb_ecode;
            csr[5][30:22] <= wb_esubcode; 
            //era
            csr[6] <= wb_pc;
        end
        //badv
        if(wb_ecode == 8 || wb_ecode == 9 || wb_ecode == 63 || wb_ecode == 3 || wb_ecode == 2 || wb_ecode == 1 || wb_ecode == 4 || wb_ecode == 7)begin
            csr[7] <= (wb_ecode == 8 && wb_esubcode == 0) ? wb_pc : wb_vaddr;
        end
        end
    if(ertn_flush) begin
        //crmd
        if(csr[5][21:16] == 6) begin  //代替estate_encode
            csr[0][4] <= 1'h1;
            csr[0][3] <= 1'h0;
        end
        csr[0][1:0] <= csr[1][1:0]; //when ERTN, recover pplv
        csr[0][2]  <= csr[1][2];  //when ERTN, recover pie
        end
    if (wb_ex & ~ertn_flush & (wb_ecode == 63 || 
                                    wb_ecode == 1  || 
                                    wb_ecode == 2  || 
                                    wb_ecode == 3  || 
                                    wb_ecode == 4  || 
                                    wb_ecode == 7)) begin
        csr[17][31:13] <= wb_vaddr[31:13];
        end
    if(csrWR) begin //csrwr, csrxchg
        //crmd
        if(csr_num==0) begin
        csr[0][1:0] <= csr_wmask[1:0] & csr_wvalue[1:0]
                     | ~csr_wmask[1:0] & csr[0][1:0];
        csr[0][2]  <= csr_wmask[2] & csr_wvalue[2]
                     | ~csr_wmask[2] & csr[0][2];
        csr[0][3]  <= csr_wmask[3] & csr_wvalue[3]
                     | ~csr_wmask[3] & csr[0][3];
        csr[0][4]  <= csr_wmask[4] & csr_wvalue[4]
                     | ~csr_wmask[4] & csr[0][4];
        csr[0][6:5] <= csr_wmask[6:5] & csr_wvalue[6:5]
                     | ~csr_wmask[6:5] & csr[0][6:5];
        csr[0][8:7] <= csr_wmask[8:7] & csr_wvalue[8:7]
                     | ~csr_wmask[8:7] & csr[0][8:7];
        end  
        //prmd
        else if(csr_num==1) begin
        csr[1][1:0] <= csr_wmask[1:0] & csr_wvalue[1:0]
                      | ~csr_wmask[1:0] & csr[1][1:0];
        csr[1][2]  <= csr_wmask[2]  & csr_wvalue[2]
                      | ~csr_wmask[2]  & csr[1][2];
        end
        //ecfg
        else if(csr_num==4) begin
        csr[4][31:13]<=19'b0;
        csr[4][12:11]<= csr_wmask[12:11] & csr_wvalue[12:11]
                     | ~csr_wmask[12:11] & csr[4][12:11];
        csr[4][10]<=1'b0;
        csr[4][9:0] <= csr_wmask[9:0] & csr_wvalue[9:0]
                     | ~csr_wmask[9:0] & csr[4][9:0];
        end

        //estat
        else if(csr_num==5) begin
        csr[5][31]=1'b0;
        csr[5][30:16] <= csr_wmask[30:16] & csr_wvalue[30:16]
                     | ~csr_wmask[30:16] & csr[5][30:16]; 
        csr[5][15:13]<=3'b0;
        csr[5][12:11]<= csr_wmask[12:11] & csr_wvalue[12:11]
                     | ~csr_wmask[12:11] & csr[5][12:11];
        csr[5][10]<=1'b0;
        csr[5][9:0] <= csr_wmask[9:0] & csr_wvalue[9:0]
                     | ~csr_wmask[9:0] & csr[5][9:0];      
        end
        //era
        else if(csr_num==6) begin
        csr[6] <= csr_wmask & csr_wvalue
                   | ~csr_wmask & csr[6];
        end
        //eentry
        else if(csr_num==12) begin
        csr[12][31:12] <= csr_wmask[31:12] & csr_wvalue[31:12]
                      | ~csr_wmask[31:12] & csr[12][31:12]; //31:6?
        end
        //save
        else if(csr_num==48) begin
        csr[48] <= csr_wmask & csr_wvalue
                       | ~csr_wmask & csr[48];
        end
        else if(csr_num==49) begin
        csr[49] <= csr_wmask & csr_wvalue
                       | ~csr_wmask & csr[49];
        end
        else if(csr_num==50)
        csr[50] <= csr_wmask & csr_wvalue
                       | ~csr_wmask & csr[50];
        else if(csr_num==51) begin
        csr[51] <= csr_wmask & csr_wvalue
                       | ~csr_wmask & csr[51];
        end
        //tid
        else if(csr_num==64) begin
        csr[64] <= csr_wmask & csr_wvalue 
                    | ~csr_wmask & csr[64];
        end 
        //tcfg
        else if(csr_num==65) begin  
        csr[65][0] <= csr_wmask[0] & csr_wvalue[0] |
                      ~csr_wmask[0] & csr[65][0];   
        csr[65][1] <= csr_wmask[1] & csr_wvalue[1] |
                            ~csr_wmask[1] & csr[65][1];
        csr[65][31:2]  <= csr_wmask[31:2]  & csr_wvalue[31:2]  |
                            ~csr_wmask[31:2]  & csr[65][31:2];
            if(csr_wvalue[0])begin
            csr[66] <= {csr_wmask[31:2]  & csr_wvalue[31:2]  |
                            ~csr_wmask[31:2]  & csr[65][31:2], 2'b0};
            if(csr_wvalue[31:2]==0)
            begin
                TI=1;
                csr[5][11]=1;
            end
            end
        end 
        //ticlr
        else if(csr_num==68) begin
            csr[5][11]= ~(csr_wmask[0] & csr_wvalue[0]);
        end
        //tlbidx
        else if(csr_num==16) begin
        csr[16][31]    <= csr_wmask[31] & csr_wvalue[31]
                         | ~csr_wmask[31] & csr[16][31]; 
        csr[16][29:24]    <= csr_wmask[29:24] & csr_wvalue[29:24]
                         | ~csr_wmask[29:24] & csr[16][29:24];
        csr[16][4:0] <= csr_wmask[4:0] & csr_wvalue[4:0]
                         | ~csr_wmask[4:0] & csr[16][4:0];        
        end   
        //tlbehi 
        else if(csr_num==17) begin
        csr[17][31:13] <= csr_wmask[31:13] & csr_wvalue[31:13]
                        | ~csr_wmask[31:13] & csr[17][31:13];               
        end
        //tlbelo0
        else if(csr_num==18) begin
        csr[18][0]   <= csr_wmask[0] & csr_wvalue[0]
                        | ~csr_wmask[0] & csr[18][0]; 
        csr[18][1]      <= csr_wmask[1] & csr_wvalue[1]
                        | ~csr_wmask[1] & csr[18][1];
        csr[18][3:2]    <= csr_wmask[3:2] & csr_wvalue[3:2]
                        | ~csr_wmask[3:2] & csr[18][3:2];
        csr[18][5:4]    <= csr_wmask[5:4] & csr_wvalue[5:4]
                        | ~csr_wmask[5:4] & csr[18][5:4];
        csr[18][6]    <= csr_wmask[6] & csr_wvalue[6]
                        | ~csr_wmask[6] & csr[18][6];
        csr[18][31:8]      <= csr_wmask[31:8] & csr_wvalue[31:8]
                        | ~csr_wmask[31:8] & csr[18][31:8];    
        end
        //tlbelo1
        else if(csr_num==19) begin
        csr[19][0]   <= csr_wmask[0] & csr_wvalue[0]
                        | ~csr_wmask[0] & csr[19][0]; 
        csr[19][1]   <= csr_wmask[1] & csr_wvalue[1]
                        | ~csr_wmask[1] & csr[19][1];
        csr[19][3:2] <= csr_wmask[3:2] & csr_wvalue[3:2]
                        | ~csr_wmask[3:2] & csr[19][3:2];
        csr[19][5:4] <= csr_wmask[5:4] & csr_wvalue[5:4]
                        | ~csr_wmask[5:4] & csr[19][5:4];
        csr[19][6] <= csr_wmask[6] & csr_wvalue[6]
                        | ~csr_wmask[6] & csr[19][6];
        csr[19][31:8]   <= csr_wmask[31:8] & csr_wvalue[31:8]
                        | ~csr_wmask[31:8] & csr[19][31:8];    
        end
        //asid
        else if(csr_num==24) begin
        csr[24][9:0]  <= csr_wmask[9:0] & csr_wvalue[9:0]
                       | ~csr_wmask[9:0] & csr[24][9:0];             
        end
        //tlbentry
        else if(csr_num == 136) begin
        csr[136][31:6] <= csr_wmask[31:6] & csr_wvalue[31:6]
                         | ~csr_wmask[31:6] & csr[136][31:6]; 
        end
        //dmw0
        else if (csr_num == 384) begin
        csr[384][31:29] <= csr_wmask[31:29] & csr_wvalue[31:29]
                      | ~csr_wmask[31:29] & csr[384][31:29];
        csr[384][27:25] <= csr_wmask[27:25] & csr_wvalue[27:25]
                      | ~csr_wmask[27:25] & csr[384][27:25];
        csr[384][5:4]  <= csr_wmask[5:4]  & csr_wvalue[5:4]
                      | ~csr_wmask[5:4]  & csr[384][5:4];
        csr[384][3] <= csr_wmask[3] & csr_wvalue[3]
                      | ~csr_wmask[3] & csr[384][3];
        csr[384][0] <= csr_wmask[0] & csr_wvalue[0]
                      | ~csr_wmask[0] & csr[384][0];
        end
        //dmw1
        else if (csr_num == 385) begin
        csr[385][31:29] <= csr_wmask[31:29] & csr_wvalue[31:29]
                      | ~csr_wmask[31:29] & csr[385][31:29];
        csr[385][27:25] <= csr_wmask[27:25] & csr_wvalue[27:25]
                      | ~csr_wmask[27:25] & csr[385][27:25];
        csr[385][5:4]  <= csr_wmask[5:4]  & csr_wvalue[5:4]
                      | ~csr_wmask[5:4]  & csr[385][5:4];
        csr[385][3] <= csr_wmask[3] & csr_wvalue[3]
                      | ~csr_wmask[3] & csr[385][3];
        csr[385][0] <= csr_wmask[0] & csr_wvalue[0]
                      | ~csr_wmask[0] & csr[385][0];
        end        
        end

    if (csr[4][11]&&csr[65][0] && csr[66] != 32'hffffffff) begin
        if (csr[66][31:0] == 32'b0 )
        begin
             TI=0;
            if(csr[65][1])
            begin
            csr[66] <= {csr[65][31:2], 2'b0};
            end
        end
        else if(csr[66]!=32'h1)
        begin
            csr[66] <= csr[66] - 1'b1;  
        end 
        else if((csr[66]==32'b1))
        begin 
            csr[66] <= csr[66] - 1'b1;
            TI=1;
            csr[5][11]=1;
        end 

    end
  
    //tlbidx
    if(inst_tlbsrch) begin
        if(s1_found) begin
            csr[16][31]    <= 1'b0;
            csr[16][4:0] <= s1_index;
        end
        else 
            csr[16][31]    <= 1'b1;
        end
    if(ws_valid && inst_tlbrd) begin
        csr[16][31]    <= ~r_e;
        end
    if(inst_tlbrd && r_e) begin
        csr[16][29:24]    <= r_ps;
        //tlbehi
        csr[17][31:13] <= r_vppn;
        //tlbelo0
        csr[18][0]   <= r_v0;
        csr[18][1]   <= r_d0;
        csr[18][3:2] <= r_plv0;
        csr[18][5:4] <= r_mat0;
        csr[18][6]   <= r_g;
        csr[18][31:8] <= r_ppn0;
        //tlbelo1
        csr[19][0]   <= r_v1;
        csr[19][1]   <= r_d1;
        csr[19][3:2] <= r_plv1;
        csr[19][5:4] <= r_mat1;
        csr[19][6]   <= r_g;
        csr[19][31:8] <= r_ppn1;
        //asid
        csr[24][9:0] <= r_asid;
        end 
    end
reg [ 3:0] tlbfill_index;
always @(posedge clk)begin
    if(rst)begin
        tlbfill_index <= 4'b0;
    end
    else if(inst_tlbfill & ws_valid) begin
        if(tlbfill_index == 4'd15) begin
            tlbfill_index <= 4'b0;
        end
        else begin
            tlbfill_index <= tlbfill_index + 4'b1;
        end
    end
end

assign we      = ws_valid && inst_tlbwr || inst_tlbfill;
assign w_index = ws_valid && inst_tlbfill ?tlbfill_index : csr[16][4:0];
assign w_e     = csr[5][21:16] == 6'h3f ? 1'b1         : ~csr[16][31];
assign w_vppn  = csr[17][31:13];
assign w_ps    = csr[16][29:24];
assign w_asid  = csr[24][9:0];
assign w_g     = csr[18][6] && csr[19][6];
assign w_ppn0  = csr[18][31:8];
assign w_plv0  = csr[18][3:2];
assign w_mat0  = csr[18][5:4];
assign w_d0    = csr[18][1];
assign w_v0    = csr[18][0];
assign w_ppn1  = csr[19][31:8];
assign w_plv1  = csr[19][3:2];
assign w_mat1  = csr[19][5:4];
assign w_d1    = csr[19][1];
assign w_v1    = csr[19][0];
assign r_index = csr[16][4:0];


assign csr_rvalueE= {32{csr_numE==0    }} & csr[0]
                  | {32{csr_numE==1     }} & csr[1]
                  | {32{csr_numE==5    }} & csr[5]
                  | {32{csr_numE==6      }} & csr[6]
                  | {32{csr_numE==12   }} & csr[12]
                  | {32{csr_numE==4     }} & csr[4]
                  | {32{csr_numE==48    }} & csr[48]
                  | {32{csr_numE==49    }} & csr[49]
                  | {32{csr_numE==50    }} & csr[50]
                  | {32{csr_numE==51    }} & csr[51]
                  | {32{csr_numE==7     }} & csr[7]
                  | {32{csr_numE==64      }} & csr[64]
                  | {32{csr_numE==65     }} & csr[65]
                  | {32{csr_numE==66     }} & csr[66]
                  | {32{csr_numE==68    }} & csr[68]
                  | {32{csr_numE==16   }} & csr[16]
                  | {32{csr_numE==17   }} & csr[17]
                  | {32{csr_numE==18  }} & csr[18]
                  | {32{csr_numE==19  }} & csr[19]
                  | {32{csr_numE==24     }} & csr[24]
                  | {32{csr_numE==136}} & csr[136]
                  | {32{csr_numE==384     }} & csr[384]
                  | {32{csr_numE==385     }} & csr[385]; 

assign csr_rvalueM= {32{csr_numM==0    }} & csr[0]
                  | {32{csr_numM==1     }} & csr[1]
                  | {32{csr_numM==5    }} & csr[5]
                  | {32{csr_numM==6      }} & csr[6]
                  | {32{csr_numM==12   }} & csr[12]
                  | {32{csr_numM==4     }} & csr[4]
                  | {32{csr_numM==48    }} & csr[48]
                  | {32{csr_numM==49    }} & csr[49]
                  | {32{csr_numM==50    }} & csr[50]
                  | {32{csr_numM==51    }} & csr[51]
                  | {32{csr_numM==7     }} & csr[7]
                  | {32{csr_numM==64      }} & csr[64]
                  | {32{csr_numM==65     }} & csr[65]
                  | {32{csr_numM==66     }} & csr[66]
                  | {32{csr_numM==68    }} & csr[68]
                  | {32{csr_numM==16   }} & csr[16]
                  | {32{csr_numM==17   }} & csr[17]
                  | {32{csr_numM==18  }} & csr[18]
                  | {32{csr_numM==19  }} & csr[19]
                  | {32{csr_numM==24     }} & csr[24]
                  | {32{csr_numM==136}} & csr[136]
                  | {32{csr_numM==384     }} & csr[384]
                  | {32{csr_numM==385     }} & csr[385]; 



assign csr_rvalue = {32{csr_num==0    }} & csr[0]
                  | {32{csr_num==1     }} & csr[1]
                  | {32{csr_num==5    }} & csr[5]
                  | {32{csr_num==6      }} & csr[6]
                  | {32{csr_num==12   }} & csr[12]
                  | {32{csr_num==4     }} & csr[4]
                  | {32{csr_num==48    }} & csr[48]
                  | {32{csr_num==49    }} & csr[49]
                  | {32{csr_num==50    }} & csr[50]
                  | {32{csr_num==51    }} & csr[51]
                  | {32{csr_num==7     }} & csr[7]
                  | {32{csr_num==64      }} & csr[64]
                  | {32{csr_num==65     }} & csr[65]
                  | {32{csr_num==66     }} & csr[66]
                  | {32{csr_num==68    }} & csr[68]
                  | {32{csr_num==16   }} & csr[16]
                  | {32{csr_num==17   }} & csr[17]
                  | {32{csr_num==18  }} & csr[18]
                  | {32{csr_num==19  }} & csr[19]
                  | {32{csr_num==24     }} & csr[24]
                  | {32{csr_num==136}} & csr[136]
                  | {32{csr_num==384     }} & csr[384]
                  | {32{csr_num==385     }} & csr[385]; 

assign has_int = ((csr[5][11:0] & csr[4][11:0]) != 12'b0)
                && (csr[0][2] == 1'b1);

reg [63:0] cnt;
always @(posedge clk) begin
    if (rst)
        cnt <= 64'b0;
    else 
        cnt <= cnt + 1'b1;
end

assign counter = cnt;



assign ex_era=csr[6];   
assign ex_entry = csr[12];

assign csr_crmd_rvalue = csr[0];
assign csr_dmw0_rvalue = csr[384];
assign csr_dmw1_rvalue = csr[385];
assign csr_asid_rvalue = csr[24];
assign csr_tlbehi_rvalue = csr[17];
assign csr_tlbrentry_rvalue = csr[136]; 

assign SWI = (csr_num==32'h5)&(csr_wmask[1:0]&csr_wvalue[1:0]);
 

endmodule
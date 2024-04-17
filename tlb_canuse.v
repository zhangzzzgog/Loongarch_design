/*
//Adapted from https://github.com/JosephQiu21/LoongArchCPU/tlb.v @Louise4one & JosephQiu21 
//According to their degisn, we edit it to fit our CPU.(Page size, two-way associated.)
//To run Linux,page size should be set to 4KB. [Support 4KB or 4MB]
//Please install it in  CPU.v(CPUCore).
*/
module tlb#(parameter TLBNUM = 16)(
input clk,
// search port 0 (For Instruction Memory access)
input  [              18:0] s0_vppn,        //virtual even page number
input                       s0_va_bit12,    //index bits
input  [               9:0] s0_asid,        //AddrSpaceID
output                      s0_found,       //found?
output [$clog2(TLBNUM)-1:0] s0_index,       //PathNum(Index in set)
output [              19:0] s0_ppn,         //physical page number
output [               5:0] s0_ps,          //page size
output [               1:0] s0_plv,         //Privilege Level
output [               1:0] s0_mat,         //Memory Access Type
output                      s0_d,           //?
output                      s0_v,           //Valid bit?
// search port 1 (For Data Memory access)
input  [              18:0] s1_vppn,
input                       s1_va_bit12,
input  [               9:0] s1_asid,
output                      s1_found,
output [$clog2(TLBNUM)-1:0] s1_index,
output [              19:0] s1_ppn,
output [               5:0] s1_ps,
output [               1:0] s1_plv,
output [               1:0] s1_mat,
output                      s1_d,
output                      s1_v,
// invtlb opcode
input                       invtlb_valid,   //valid signal
input  [               4:0] invtlb_op,      //invtlb_opcode
// write port:write or edit an entry
input                       we,             //write enable
input  [$clog2(TLBNUM)-1:0] w_index,
input                       w_e,            //Exist bit
input  [              18:0] w_vppn,
input  [               5:0] w_ps,
input  [               9:0] w_asid,
input                       w_g,            //Global sign bit
input  [              19:0] w_ppn0,
input  [               1:0] w_plv0,
input  [               1:0] w_mat0,
input                       w_d0,
input                       w_v0,
input  [              19:0] w_ppn1,
input  [               1:0] w_plv1,
input  [               1:0] w_mat1,
input                       w_d1,
input                       w_v1,
// read port：read an entry
input  [$clog2(TLBNUM)-1:0] r_index,
output                      r_e,
output [              18:0] r_vppn,
output [               5:0] r_ps,
output [               9:0] r_asid,
output                      r_g,
output [              19:0] r_ppn0,
output [               1:0] r_plv0,
output [               1:0] r_mat0,
output                      r_d0,
output                      r_v0,
output [              19:0] r_ppn1,
output [               1:0] r_plv1,
output [               1:0] r_mat1,
output                      r_d1,
output                      r_v1
);

//Actually Storaged Information
//×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
reg [TLBNUM-1:0] tlb_exist;
reg [TLBNUM-1:0] tlb_PageSize4MB; //pagesize 1:4MB, 0:4KB
reg [ 18     :0] tlb_VPPN           [TLBNUM-1:0];
reg [ 9      :0] tlb_AddrSpaceID    [TLBNUM-1:0];
reg [TLBNUM-1:0] tlb_global;
reg [ 19     :0] tlb_PPN0           [TLBNUM-1:0]; //set 0(index 0)
reg [ 1      :0] tlb_PriLevel0      [TLBNUM-1:0];
reg [ 1      :0] tlb_AccessType0    [TLBNUM-1:0];
reg [TLBNUM-1:0] tlb_d0;
reg [TLBNUM-1:0] tlb_valid0; //valid bit
reg [ 19     :0] tlb_PPN1           [TLBNUM-1:0];//set 1(index 1)
reg [ 1      :0] tlb_PriLevel1      [TLBNUM-1:0];
reg [ 1      :0] tlb_AccessType1    [TLBNUM-1:0];
reg [TLBNUM-1:0] tlb_d1;
reg [TLBNUM-1:0] tlb_valid1; //valid bit
//××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××

//*********************************************************************************************
//                                     INPUT & JUDGE
//
//×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
wire      [15:0] P0Match;    //Match the query of port0 or not
wire      [15:0] P1Match;    
wire             Local           [TLBNUM-1:0];
wire             Global          [TLBNUM-1:0];
wire             SpaceMatch      [TLBNUM-1:0];
wire             PageMatch       [TLBNUM-1:0];
wire             inv_match       [TLBNUM-1:0];  

genvar i;
generate 
    for (i = 0; i < TLBNUM; i = i + 1)                                      //Assign a batch of qi jian at the same time
    begin
        //IM Search Port********************************************************************
        assign P0Match[i] = (s0_vppn[18:0] == tlb_VPPN[i][18:0])          //port0，if(match) match0[i]=1,denotes that it matched the i_st entry
            && (s0_vppn[9:0] == tlb_VPPN[i][9:0])                            /*Same Page*/
            && (s0_asid == tlb_AddrSpaceID[i] || tlb_global[i]);             /*same address space or is global entry*/

        //DM Search Port********************************************************************
        assign Local      [i]  =~tlb_global[i];                                   //Case1：Entry is global
        assign Global     [i]  = tlb_global[i];                                   //Case2：Entry is local
        assign SpaceMatch [i]  = (s1_asid == tlb_AddrSpaceID[i]);                 //Case3：In the same address space
        assign PageMatch  [i]  = (s1_vppn[18:10] == tlb_VPPN[i][18:10])           //Case4：Same address
                        && (tlb_PageSize4MB[i] || s1_vppn[9:0]==tlb_VPPN[i][9:0]);
        
        assign P1Match[i] = PageMatch[i]&&(SpaceMatch[i] || Global[i]);           //port1,if(match)match1[i]=1,denotes that it matched the i_st entry
                    

            //清除的不同Case（对应不同的清除操作）
        assign inv_match[i] =   (invtlb_op <= 1)                           | //ALL
                                (invtlb_op == 2)&& Global[i]               | //ALL GLOBAL
                                (invtlb_op == 3)&& Local[i]                | //ALL LOCAL
                                (invtlb_op == 4)&& (Local[i] && SpaceMatch[i]) |//A SPACE
                                (invtlb_op == 5)&& (Local[i] && SpaceMatch[i] && PageMatch[i]) |//A PAGE
                                (invtlb_op == 6)&& P1Match[i];//AN ADDRESS
        
        always @(posedge clk )begin
            //Write or update an entry
            if (we && w_index == i) begin                           
                tlb_exist        [i] <= w_e;
                tlb_PageSize4MB  [i] <= (w_ps == 6'h16);
                tlb_VPPN         [i] <= w_vppn;
                tlb_AddrSpaceID  [i] <= w_asid;
                tlb_global       [i] <= w_g;
                tlb_PPN0         [i] <= w_ppn0;
                tlb_PriLevel0    [i] <= w_plv0;
                tlb_AccessType0  [i] <= w_mat0;
                tlb_d0           [i] <= w_d0;
                tlb_valid0       [i] <= w_v0;
                tlb_PPN1         [i] <= w_ppn1;
                tlb_PriLevel1    [i] <= w_plv1;
                tlb_AccessType1  [i] <= w_mat1;
                tlb_d1           [i] <= w_d1;
                tlb_valid1       [i] <= w_v1;
            end
            //表项的失效「有效位清除」
            else if (inv_match[i] & invtlb_valid) begin  
                tlb_exist    [i] <= 1'b0;
            end
        end
    end
endgenerate

//*********************************************************************************************
//                                      OUTPUT
//
/*×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
×××××××××         IM Search Port               ××××××××××××××××××××××××××××××××××××××××××××××××
×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××*/
assign s0_found = (P0Match != 16'd0);
//get s0_index(4 bits)
assign s0_index[3]=P0Match[8]|P0Match[9]|P0Match[10]|P0Match[11]|P0Match[12]|P0Match[13]|P0Match[14]|P0Match[15];
assign s0_index[2]=P0Match[4]|P0Match[5]|P0Match[6] |P0Match[7] |P0Match[12]|P0Match[13]|P0Match[14]|P0Match[15];
assign s0_index[1]=P0Match[2]|P0Match[3]|P0Match[6] |P0Match[7] |P0Match[10]|P0Match[11]|P0Match[14]|P0Match[15];
assign s0_index[0]=P0Match[1]|P0Match[3]|P0Match[5] |P0Match[7] |P0Match[9] |P0Match[11]|P0Match[13]|P0Match[15];
assign s0_ps       = tlb_PageSize4MB[s0_index]? 6'h16:6'h0c; //4MB or 4KB
wire   s0_CPath1   = tlb_PageSize4MB[s0_index] ? s0_vppn[9] : s0_va_bit12;  //Choose which path
assign s0_ppn      = s0_CPath1 ? tlb_PPN1[s0_index] : tlb_PPN0[s0_index];
assign s0_plv      = s0_CPath1?tlb_PriLevel1[s0_index]  :tlb_PriLevel0[s0_index];
assign s0_mat      = s0_CPath1?tlb_AccessType1[s0_index]:tlb_AccessType0[s0_index];
assign s0_d        = s0_CPath1?tlb_d1[s0_index]:tlb_d0[s0_index];
assign s0_v        = s0_CPath1?tlb_valid1[s0_index]:tlb_valid0[s0_index];
/*×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
×××××××××         DM Search Port               ××××××××××××××××××××××××××××××××××××××××××××××××
×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××*/
assign s1_found = (P1Match != 16'b0);
//get s1_index(4 bits)
assign s1_index[3]=P1Match[8]|P1Match[9]|P1Match[10]|P1Match[11]|P1Match[12]|P1Match[13]|P1Match[14]|P1Match[15];
assign s1_index[2]=P1Match[4]|P1Match[5]|P1Match[6] |P1Match[7] |P1Match[12]|P1Match[13]|P1Match[14]|P1Match[15];
assign s1_index[1]=P1Match[2]|P1Match[3]|P1Match[6] |P1Match[7] |P1Match[10]|P1Match[11]|P1Match[14]|P1Match[15];
assign s1_index[0]=P1Match[1]|P1Match[3]|P1Match[5] |P1Match[7] |P1Match[9] |P1Match[11]|P1Match[13]|P1Match[15];
assign s1_ps    = tlb_PageSize4MB[s1_index]? 6'h16 : 6'h0c;
wire   s1_CPath1= tlb_PageSize4MB[s1_index] ? s1_vppn[9] : s1_va_bit12;
assign s1_ppn   = s1_CPath1 ? tlb_PPN1[s1_index] : tlb_PPN0[s1_index];
assign s1_plv   = s1_CPath1 ? tlb_PriLevel1[s1_index]   : tlb_PriLevel0[s1_index];
assign s1_mat   = s1_CPath1 ? tlb_AccessType1[s1_index] : tlb_AccessType0[s1_index];
assign s1_d     = s1_CPath1 ? tlb_d1[s1_index] : tlb_d0[s1_index];
assign s1_v     = s1_CPath1 ? tlb_valid1[s1_index] : tlb_valid0[s1_index];
/*×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
×××××××××××××××××××        Read port                   ××××××××××××××××××××××××××××××××××××××××
×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××*/
assign r_e        = tlb_exist[r_index]; 
assign r_vppn     = tlb_VPPN[r_index];
assign r_ps       = tlb_PageSize4MB[r_index] ? 6'h16 : 6'h0c;
assign r_asid     = tlb_AddrSpaceID[r_index];
assign r_g        = tlb_global[r_index];
assign r_ppn0     = tlb_PPN0[r_index];
assign r_plv0     = tlb_PriLevel0[r_index];
assign r_mat0     = tlb_AccessType0[r_index];
assign r_d0       = tlb_d0[r_index];
assign r_v0       = tlb_valid0[r_index];
assign r_ppn1     = tlb_PPN1[r_index];
assign r_plv1     = tlb_PriLevel1[r_index];
assign r_mat1     = tlb_AccessType1[r_index];
assign r_d1       = tlb_d1[r_index];
assign r_v1       = tlb_valid1[r_index];

endmodule
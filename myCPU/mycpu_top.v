/*说明：该文件是为了是CPU.v适合soc_lite_top.v中所要求的定义�?�构建的转接模块。本文件中只允许出现组合逻辑�?*/
//cpu /*发布要求的接�?*/
module mycpu_top(
    input  wire        clk,
    input  wire        resetn, //low active
    // inst sram interface
    output wire        inst_sram_en,
    output wire [ 3:0] inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_en,
    output wire [ 3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
assign inst_sram_we=4'b0;//指令存储器写使能 不写
assign inst_sram_wdata=32'h0;//指令存储器写入数�? 不写

assign data_sram_en=1'b1;//数据存储器使�?
wire [31:0] pcW; // no use
wire memwriteM; // no use


    CPU U_CPU_Core(
         .clk(clk),                
         .reset(~resetn),           //High active?
         .TLBR(1'b0),               //TLB read
         //input            
         .instr(inst_sram_rdata),   // input: 输入指令
         .readdataM(data_sram_rdata),       // input:  从datamem中读出的数据输入到CPU
         //output
         .wea(data_sram_we),              // output: DM write enable
         .memwriteM(memwriteM),           // output: 写内存使能信�??
         .pcF(inst_sram_addr),            // output: 当前PC，被其他模块使用
         .inst_sram_en(inst_sram_en),     //output:指令存储器使�?
        //.this_pc(pcW),                   // output: 当前PC，调试方便打印相关信�??
         .Addr_out(data_sram_addr),       // output: datamem地址
         .writedataM(data_sram_wdata),    // output: 写入datamem的数�??
        //debug
         .pcW_out(debug_wb_pc),       // output: 当前写回阶段指令的PC
         .wb_rf_we(debug_wb_rf_we),    //RF的wea信号
         .reg_sel(debug_wb_rf_wnum),        
         .reg_data(debug_wb_rf_wdata)
         );
endmodule

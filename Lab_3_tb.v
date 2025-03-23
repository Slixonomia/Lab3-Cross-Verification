`timescale 1ns/1ps
module tb_Lab_3();

// 参数定义
parameter pADDR_WIDTH = 12;
parameter pDATA_WIDTH = 32;
parameter Tape_Num = 11;
parameter MAX_DATA = 1024;

// DUT接口信号声明
reg         awvalid;
wire        awready;
reg  [11:0] awaddr;
reg         wvalid;
wire        wready;
reg  [31:0] wdata;
reg         arvalid;
wire        arready;
reg  [11:0] araddr;
wire        rvalid;
reg         rready;
wire [31:0] rdata;

reg         ss_tvalid;
reg  [31:0] ss_tdata;
reg         ss_tlast;
wire        ss_tready;

wire        sm_tvalid;
wire [31:0] sm_tdata;
wire        sm_tlast;
reg         sm_tready;

wire [3:0]  tap_WE;
wire        tap_EN;
wire [31:0] tap_Di;
wire [11:0] tap_A;
reg  [31:0] tap_Do;

wire [3:0]  data_WE;
wire        data_EN;
wire [31:0] data_Di;
wire [11:0] data_A;
reg  [31:0] data_Do;

reg clk;
reg rst_n;

// 测试数据存储
reg [31:0] xn_data [0:MAX_DATA-1];
reg [31:0] golden_data [0:MAX_DATA-1];
reg [31:0] yn_buffer [0:MAX_DATA-1];
integer data_length;
integer latency_time;
integer test_round;
integer error_count;

// 实例化DUT
Lab_3 #(
    .pADDR_WIDTH(pADDR_WIDTH),
    .pDATA_WIDTH(pDATA_WIDTH),
    .Tape_Num(Tape_Num)
) dut (
    .awready(awready),
    .wready(wready),
    .awvalid(awvalid),
    .awaddr(awaddr),
    .wvalid(wvalid),
    .wdata(wdata),
    .arready(arready),
    .rready(rready),
    .arvalid(arvalid),
    .araddr(araddr),
    .rvalid(rvalid),
    .rdata(rdata),
    .ss_tvalid(ss_tvalid),
    .ss_tdata(ss_tdata),
    .ss_tlast(ss_tlast),
    .ss_tready(ss_tready),
    .sm_tready(sm_tready),
    .sm_tvalid(sm_tvalid),
    .sm_tdata(sm_tdata),
    .sm_tlast(sm_tlast),
    .tap_WE(tap_WE),
    .tap_EN(tap_EN),
    .tap_Di(tap_Di),
    .tap_A(tap_A),
    .tap_Do(tap_Do),
    .data_WE(data_WE),
    .data_EN(data_EN),
    .data_Di(data_Di),
    .data_A(data_A),
    .data_Do(data_Do),
    .axis_clk(clk),
    .axis_rst_n(rst_n)
);

// 时钟生成
always #5 clk = ~clk;

// 主测试流程
initial begin
    initialize;
    for(test_round=0; test_round<3; test_round=test_round+1) begin
        $display("=== Test Round %0d ===", test_round+1);
        setup_phase;
        execution_phase;
        checking_phase;
    end
    $display("All tests completed with %0d errors", error_count);
    $finish;
end

// 初始化任务
task initialize;
begin
    clk = 0;
    rst_n = 0;
    error_count = 0;
    awvalid = 0;
    wvalid = 0;
    arvalid = 0;
    ss_tvalid = 0;
    sm_tready = 1;
    
    // 复位操作
    #20 rst_n = 1;
    
    // 加载测试数据
    $readmemh("xn_data.hex", xn_data);
    $readmemh("golden_data.hex", golden_data);
    
    // 计算数据长度
    data_length = 0;
    while(xn_data[data_length] !== 32'bx && data_length < MAX_DATA) 
        data_length = data_length + 1;
end
endtask

// Setup Phase
task setup_phase;
begin
    // 配置抽头参数
    program_tap_parameters;
    
    // 配置数据长度
    axilite_write(12'h10, data_length);
    
    // 检查控制信号
    check_ctrl_signals(0, 1, 0);
end
endtask

// Execution Phase
task execution_phase;
begin
    // 启动ap_start
    axilite_write(0, 1);
    
    // 启动计时器
    latency_time = $time;
    
    // 顺序执行并发任务
    begin
        stream_in_xn;
        stream_out_yn;
        axilite_polling;
    end
end
endtask

// Checking Phase
task checking_phase;
begin
    latency_time = $time - latency_time;
    $display("Latency: %0d ns", latency_time);
    compare_data;
end
endtask

//--------------------- 底层任务 ---------------------
task program_tap_parameters;
integer i;
begin
    for(i=0; i<Tape_Num; i=i+1) begin
        axilite_write(12'h40 + i*4, $random);
    end
end
endtask

task check_ctrl_signals;
input start_exp, idle_exp, done_exp;
begin
    axilite_read(0);
    if(rdata[0] !== start_exp || rdata[2] !== idle_exp) begin
        $display("Error: Control signals mismatch");
        error_count = error_count + 1;
    end
end
endtask

task stream_in_xn;
integer i;
begin
    for(i=0; i<data_length; i=i+1) begin
        @(posedge clk);
        ss_tvalid = 1;
        ss_tdata = xn_data[i];
        ss_tlast = (i == data_length-1);
        while(!ss_tready) @(posedge clk);
    end
    ss_tvalid = 0;
end
endtask

task stream_out_yn;
integer idx;
begin
    idx = 0;
    while(1) begin
        @(posedge clk);
        if(sm_tvalid && sm_tready) begin
            yn_buffer[idx] = sm_tdata;
            idx = idx + 1;
            if(sm_tlast) break;
        end
    end
end
endtask

task axilite_polling;
begin
    while(1) begin
        axilite_read(0);
        if(rdata[1]) begin
            disable stream_in_xn;
            disable stream_out_yn;
            disable axilite_polling;
            break;
        end
        #100; // 轮询间隔
    end
end
endtask

task compare_data;
integer i;
begin
    for(i=0; i<data_length; i=i+1) begin
        if(yn_buffer[i] !== golden_data[i]) begin
            $display("Error at index %0d: Exp %h, Got %h", 
                    i, golden_data[i], yn_buffer[i]);
            error_count = error_count + 1;
        end
    end
end
endtask

//--------------------- AXI-Lite驱动 ---------------------
task axilite_write;
input [11:0] addr;
input [31:0] data;
begin
    @(posedge clk);
    awvalid = 1;
    awaddr = addr;
    wvalid = 1;
    wdata = data;
    while(!(awready && wready)) @(posedge clk);
    awvalid = 0;
    wvalid = 0;
end
endtask

task axilite_read;
input [11:0] addr;
begin
    @(posedge clk);
    arvalid = 1;
    araddr = addr;
    while(!arready) @(posedge clk);
    arvalid = 0;
    while(!rvalid) @(posedge clk);
end
endtask

endmodule
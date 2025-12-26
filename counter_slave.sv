`timescale 1ns / 1ps

module counter_slave(
    //external
    input  logic clk,
    input  logic reset,   // active high

    // internal
    output logic [31:0] counter, // from timer module
    input  logic run,     
    input  logic updn,     
    input  logic clr_cmd,  
    input  logic load_cmd, 
    input  logic [31:0] set_tim_num,

    //external board
    output [7:0]  fnd_data,
    output [3:0]  fnd_com,
    output [15:0] led
);

    // ---------------------------------------------------------
    // 1) Counter 동작부
    // ---------------------------------------------------------
    counter_dp U_counter_DP(
        .clk(clk),
        .reset(reset),
        .run(run),
        .updn(updn),
        .clr_cmd(clr_cmd),
        .load_cmd(load_cmd),
        .set_tim_num(set_tim_num),
        .counter(counter)
    );

    // ---------------------------------------------------------
    // 2) counter → FND + LED 변환 로직
    // ---------------------------------------------------------

    logic [13:0] cnt_fnd;   // 0~9999
    logic [15:0] cnt_led;   // 0~65535

    // FND: 하위 4자리만 사용
    assign cnt_fnd = counter % 10000;    

    // LED: 10000단위로 나눈 몫
    assign cnt_led = counter / 10000;

    assign led = cnt_led;   // LED 바로 출력

    // ---------------------------------------------------------
    // 3) FND 컨트롤러
    // ---------------------------------------------------------
    fnd_controller U_FND_CONTROLLER(
        .clk(clk),
        .rst(reset),
        .cnt10000(cnt_fnd),
        .fnd_data(fnd_data),
        .fnd_com(fnd_com)
    );

    

endmodule

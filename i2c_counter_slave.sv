`timescale 1ns / 1ps

module    i2c_counter_slave #(
    parameter [6:0] I2C_ADDR = 7'b111_1111  // 7-bit 슬레이브 주소 
) (
    input  logic clk,
    input  logic reset,   // active high

    input  logic SCL,
    inout  wire  SDA,
    
    output [7:0]  fnd_data,
    output [3:0]  fnd_com,
    output [15:0] led
);


    // internal
    logic [31:0] counter; // from timer module
    logic run;     
    logic updn;    
    logic clr_cmd;  
    logic load_cmd; 
    logic [31:0] set_tim_num;

    counter_slave U_CNT(.*);
    i2c_counter_slave_interface U_I2C_CNT_INF(.*);

endmodule

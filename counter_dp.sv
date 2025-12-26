`timescale 1ns / 1ps
module counter_dp (
    input clk,
    input reset,
    
    input  logic run,     //1:run
    input  logic updn,    // 1:up,0:down 
    input  logic clr_cmd,  //1tick
    input  logic load_cmd, //1tick
    input  logic [31:0]set_tim_num,

    output logic [31:0] counter //max = 655,359,999
);
    
    wire tick; // 10hz

    tick_10hz U_TICK_10HZ (
    .clk(clk),
    .reset(reset),
    .tick(tick)
);

cnt_max U_CNT_MAX (
    .clk(clk),
    .reset(reset),
    .run(run),
    .updn(updn),
    .clr_cmd(clr_cmd),
    .load_cmd(load_cmd),
    .set_tim_num(set_tim_num),
    .tick(tick),
    .counter(counter)
);



endmodule


//---------------------------///


module cnt_max(
    input  logic clk,
    input  logic reset,

    input  logic run,     //1:run
    input  logic updn,    // 1:up,0:down 
    input  logic clr_cmd,  //1tick
    input  logic load_cmd, //1tick
    input  logic [31:0]set_tim_num,

    input  logic tick, //1ms
    output logic [31:0] counter
);
// fnd(4자리), led(16개) 표현가능
parameter int MAX_NUM = 65535 * 10000 + 9999;  // = 655,359,999



always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            counter   <= 32'd0;
        end else begin
            // 우선순위: CLR > LOAD > RUN
            if (clr_cmd) begin
                counter <= 32'd0;
            end else if (load_cmd) begin
                if( set_tim_num < MAX_NUM)
                    counter <= set_tim_num; 
                else begin // MAX 넘는값들어와도 block
                    counter <= MAX_NUM;
                end
            end else if (run) begin
                if(tick) begin
                    if (updn == 1) begin
                        // UP count
                        if (counter == MAX_NUM) counter <= 0;
                        else counter <= counter + 1;
                    end else begin
                        // DOWN count
                        if (counter == 0) counter <= MAX_NUM;
                        else counter <= counter - 1;
                    end
                end
            end
        end
    end

endmodule


module tick_10hz (
    input clk,
    input reset,
    output reg tick
);



reg [$clog2(10_000_000)-1:0]cnt_reg ;
// reg [$clog2(1000)-1:0]cnt_reg ; //tb용(10000배 가속)
 
 always @(posedge clk or posedge reset) begin
    if(reset)begin
        tick <=0;
        cnt_reg <=0;
    end else begin
        if (cnt_reg ==10_000_000 -1 ) begin
            cnt_reg <=0;
            tick <=1'b1;
        end else begin
            cnt_reg <= cnt_reg +1;
            tick <=0;
        end
    end 

 end

    
endmodule
`timescale 1ns / 1ps

module fnd_controller (
    input clk,
    input rst,
    input  [13:0]cnt10000,
    output [7:0] fnd_data,
    output [3:0] fnd_com

);

wire tick_1khz;
reg [3:0] fnd_data_4pick;
wire [3:0]digit1,digit10,digit100,digit1000;
wire [1:0]w_4_cnt;


tick_1khz U_1khz(
    .clk(clk),
    .rst(rst),
    .tick_1khz(tick_1khz)
);

    digit_spliter U_digit_splite(
    .cnt(cnt10000),
    .digit1(digit1),
    .digit10(digit10),
    .digit100(digit100),
    .digit1000(digit1000)
);

//seg data
//4X1 mux
    always @(*) begin
        case (w_4_cnt)
          2'b00  : fnd_data_4pick = digit1;
          2'b01  : fnd_data_4pick = digit10;
          2'b10  : fnd_data_4pick = digit100;
          2'b11  : fnd_data_4pick = digit1000;
            default: fnd_data_4pick = digit1;
        endcase
    end



    bcd_docoder U_BCD_DECODER(
    .bcd(fnd_data_4pick),
    .fnd_data(fnd_data)
    );

// seg com'

    seg_com_sel U_SEGCOM(
    .cnt(w_4_cnt),
    .seg_com(fnd_com)
    );


    cnt_4 U_CNT_4(
    .clk(clk),
    .rst(rst),
    .tick(tick_1khz), // 1khz
    .cnt(w_4_cnt)
    );

endmodule



//-------------------------------------------------------//



module cnt_4(
    input clk,
    input rst,
    input tick,
    output reg [1:0]cnt 
    );


    always @(posedge clk, posedge rst) begin
        if(rst) begin
            cnt <= 0;
        end else if(tick)begin
            if(cnt == 2'd3) begin
                cnt <= 0;
            end else begin
                cnt <= cnt +1;
            end
        end
    end
endmodule

module seg_com_sel (
    input [1:0]     cnt,
    output reg [3:0]seg_com
);


always @(*) begin
    case (cnt)
        2'b00: seg_com = 4'b1110;
        2'b01: seg_com = 4'b1101;
        2'b10: seg_com = 4'b1011;
        2'b11: seg_com = 4'b0110;
        default: seg_com = 4'b1110;
    endcase
end


    
endmodule


module tick_1khz (
    input clk,
    input rst,
    output reg tick_1khz
);



reg [$clog2(100_000)-1:0]cnt_reg ;
// reg [$clog2(10)-1:0]cnt_reg ; //tb용 (10000배가속)
 
 always @(posedge clk, posedge rst) begin
    if(rst)begin
        tick_1khz <=0;
        cnt_reg <=0;
    end else begin
        if (cnt_reg ==100_000-1) begin
            cnt_reg <=0;
            tick_1khz <=1'b1;
        end else begin
            cnt_reg <= cnt_reg +1;
            tick_1khz <=0;

        end
    end
 end

    
endmodule


module digit_spliter (
    input  [13:0]cnt,
    output [3:0] digit1,
    output [3:0] digit10,
    output [3:0] digit100,
    output [3:0] digit1000
);

    assign digit1 = cnt%10 ;
    assign digit10 = (cnt/10)%10 ;
    assign digit100 = (cnt/100)%10 ;
    assign digit1000 = (cnt/1000)%10 ;


    
    
endmodule


module  bcd_docoder (
    input [3:0]bcd,
    output reg [7:0]fnd_data
);

    always @(*) begin
        case (bcd)
            4'd0: fnd_data = 8'hc0;
            4'd1: fnd_data = 8'hF9;
            4'd2: fnd_data = 8'hA4;
            4'd3: fnd_data = 8'hB0;
            4'd4: fnd_data = 8'h99;
            4'd5: fnd_data = 8'h92;
            4'd6: fnd_data = 8'h82;
            4'd7: fnd_data = 8'hF8;
            4'd8: fnd_data = 8'h80;
            4'd9: fnd_data = 8'h90;
        
            default: fnd_data = 8'hc0;
        endcase
    end
    
endmodule
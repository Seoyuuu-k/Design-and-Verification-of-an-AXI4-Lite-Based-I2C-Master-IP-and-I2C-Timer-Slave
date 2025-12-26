`timescale 1ns/1ps

module I2C_Slave(
    input clk,
    input reset,
    input SCL,
    inout SDA,
    //output [7:0] LED
    output [15:0] LED,
    output [7:0] slv_reg0,
    output [7:0] slv_reg1,
    output [7:0] slv_reg2,
    output [7:0] slv_reg3

    
);
    parameter IDLE=0, ADDR=1, ACK=2, READ=3, DATA=4, READ_ACK=5, READ_CNT=6, DATA_ACK = 7, DATA_NACK=8, STOP=9;

    reg [3:0] state, state_next;
    reg [7:0] temp_rx_data_reg, temp_rx_data_next;
    reg [7:0] temp_tx_data_reg, temp_tx_data_next;
    reg [7:0] temp_addr_reg, temp_addr_next;
    reg [3:0] bit_counter_reg, bit_counter_next;
    reg [1:0] slv_count_reg, slv_count_next;
    reg en;
    reg o_data;
    reg read_ack_reg, read_ack_next;



    reg sclk_sync0, sclk_sync1;
    wire sclk_rising, sclk_falling;

    reg sda_sync0, sda_sync1;
    wire sda_rising, sda_falling;

    reg [7:0] slv_reg0_reg, slv_reg0_next;
    reg [7:0] slv_reg1_reg, slv_reg1_next;
    reg [7:0] slv_reg2_reg, slv_reg2_next;
    reg [7:0] slv_reg3_reg, slv_reg3_next;

    reg [15:0] led_reg, led_next;
    assign SDA= en? o_data: 1'bz;
    assign LED=led_reg;
    
    assign slv_reg0 = slv_reg0_reg;
    assign slv_reg1 = slv_reg1_reg;
    assign slv_reg2 = slv_reg2_reg;
    assign slv_reg3 = slv_reg3_reg;
    
    //always @(posedge clk or posedge reset) begin
    //   if(reset) begin
    //       state <= IDLE;
    //       sclk_sync0 <=0;
    //       sclk_sync1 <=0;
    //       temp_rx_data_reg <=0;
    //       temp_tx_data_reg <=0;
    //       bit_counter_reg <=0;
    //       temp_addr_reg <=0;
    //       led_reg <=0;
    //       slv_reg0_reg <=0;
    //       read_ack_reg <=1'bz;
    //   end else begin
    //       state <= state_next;
    //       sclk_sync0 <= SCL;
    //       sclk_sync1 <= sclk_sync0;
    //       temp_rx_data_reg <= temp_rx_data_next;
    //       temp_tx_data_reg <= temp_tx_data_next;
    //       bit_counter_reg <= bit_counter_next;
    //       temp_addr_reg <= temp_addr_next;
    //       led_reg <= led_next;
    //       slv_reg0_reg <= slv_reg0_next;
    //       read_ack_reg <= read_ack_next;
    //   end
    //end

     always @(posedge clk or posedge reset) begin
         if(reset) begin
             state <= IDLE;
             sclk_sync0 <=1;
             sclk_sync1 <=1;
             sda_sync0 <=1;
             sda_sync1 <=1;
             temp_rx_data_reg <=0;
             temp_tx_data_reg <=0;
             bit_counter_reg <=0;
             temp_addr_reg <=0;
             led_reg <=0;
             read_ack_reg <=1'bz;
         end else begin
             state <= state_next;
             sclk_sync0 <= SCL;
             sclk_sync1 <= sclk_sync0;
             sda_sync0 <= SDA;
             sda_sync1  <= sda_sync0;
             temp_rx_data_reg <= temp_rx_data_next;
             temp_tx_data_reg <= temp_tx_data_next;
             bit_counter_reg <= bit_counter_next;
             temp_addr_reg <= temp_addr_next;
             led_reg <= led_next;
             read_ack_reg <= read_ack_next;
         end
     end

     always @(posedge clk or posedge reset) begin
        if(reset) begin
            slv_reg0_reg <=0;
            slv_reg1_reg <=0;
            slv_reg2_reg <=0;
            slv_reg3_reg <=0;
            slv_count_reg <=0;
        end else begin
            slv_reg0_reg <= slv_reg0_next;
            slv_reg1_reg <= slv_reg1_next;
            slv_reg2_reg <= slv_reg2_next;
            slv_reg3_reg <= slv_reg3_next;
            slv_count_reg <= slv_count_next;
        end
     end

    assign sclk_rising = sclk_sync0 & ~sclk_sync1;
    assign sclk_falling = ~sclk_sync0 & sclk_sync1;

    assign sda_rising = sda_sync0 & ~sda_sync1;
    assign sda_falling = ~sda_sync0 & sda_sync1;

    always @(*) begin
        state_next = state;
        en = 1'b0;
        o_data = 1'b0;
        temp_rx_data_next = temp_rx_data_reg;
        temp_tx_data_next = temp_tx_data_reg;
        bit_counter_next = bit_counter_reg;
        temp_addr_next = temp_addr_reg;
        read_ack_next=read_ack_reg;
        led_next = led_reg;
        slv_count_next = slv_count_reg;
        slv_reg0_next = slv_reg0_reg;
        slv_reg1_next = slv_reg1_reg;
        slv_reg2_next = slv_reg2_reg;
        slv_reg3_next = slv_reg3_reg;
        case (state)
            IDLE: begin
                led_next[15:8] = 8'b1000_0000;
                if(sclk_falling && ~SDA) begin
                    state_next = ADDR;
                    bit_counter_next = 0;
                    slv_count_next =0;
                end
            end
            ADDR: begin
                led_next[15:8] = 8'b0100_0000;
                if(sclk_rising) begin
                    temp_addr_next = {temp_addr_reg[6:0], SDA};
                end
                if(sclk_falling) begin
                    if (bit_counter_reg == 8-1) begin
                        bit_counter_next = 0;
                        state_next = ACK;
                    end else begin
                        bit_counter_next = bit_counter_reg + 1;
                    end
                end
            end
            ACK: begin
                led_next[15:8] = 8'b0010_0000;
                if (temp_addr_reg[7:1] == 7'b1010101) begin
                    en = 1'b1;
                    o_data =1'b0;
                    if(sclk_falling) begin
                        if(temp_addr_reg[0]) begin
                            state_next= READ;
                            temp_tx_data_next = slv_reg0_reg;
                        end else begin
                            state_next=DATA;
                        end
                    end
                end else begin
                    state_next= IDLE;
                end
            end
            READ: begin
                led_next[15:8] = 8'b001_0000;
                en=1'b1;
                o_data = temp_tx_data_reg[7];
                if(sclk_falling) begin
                    if (bit_counter_reg == 8-1) begin
                        bit_counter_next = 0;
                        state_next = READ_ACK;
                    end else begin
                        temp_tx_data_next = {temp_tx_data_reg[6:0], 1'b0};
                        bit_counter_next = bit_counter_reg + 1;
                    end
                end
            end
            READ_ACK: begin
                led_next[15:8] = 8'b000_1000;
                en=1'b0;
                if(sclk_rising) begin
                    read_ack_next= SDA;
                end
                if(sclk_falling) begin
                    if(read_ack_reg ==1'b1) begin
                        state_next = STOP;
                        read_ack_next= 1'bz;
                    end else if (read_ack_reg == 1'b0) begin
                        state_next= READ_CNT;
                        slv_count_next = slv_count_reg +1;
                        read_ack_next = 1'bz;
                    end
                end
                if(slv_count_reg == 3) begin
                    state_next = STOP;
                end
            end
            READ_CNT: begin
                led_next[15:8] = 8'b000_0100;
                state_next= READ;
                case(slv_count_reg)
                    2'd0: begin
                        temp_tx_data_next = slv_reg0_reg;
                    end
                    2'd1: begin
                        temp_tx_data_next = slv_reg1_reg;
                    end
                    2'd2: begin
                        temp_tx_data_next = slv_reg2_reg;
                    end
                    2'd3: begin
                        temp_tx_data_next = slv_reg3_reg;
                    end
                endcase
            end
            DATA: begin
                led_next[15:8] = 8'b000_0010;
                if(sclk_rising) begin
                    temp_rx_data_next = {temp_rx_data_reg[6:0], SDA};
                end
                if (sclk_falling) begin
                    if (bit_counter_reg == 8-1) begin
                        bit_counter_next = 0;
                        state_next = DATA_ACK;
                        slv_count_next= slv_count_reg + 1;
                        case(slv_count_reg)
                            2'd0: begin
                                slv_reg0_next = temp_rx_data_reg;
                            end
                            2'd1: begin
                                slv_reg1_next = temp_rx_data_reg;
                            end
                            2'd2: begin
                                slv_reg2_next = temp_rx_data_reg;
                            end
                            2'd3: begin
                                slv_reg3_next = temp_rx_data_reg;
                            end
                        endcase
                    end else begin
                        bit_counter_next = bit_counter_reg + 1;
                    end
                end
                if(SCL && sda_rising) begin
                    state_next = STOP;
                end
            end
            DATA_ACK: begin
                led_next[15:8] = 8'b000_0001;
                en=1'b1;
                o_data =1'b0;
                if(sclk_falling) begin
                    state_next= DATA;
                end
            end
            STOP: begin
                led_next[15:8] = 8'b000_1111;
                if(SDA && SCL) begin
                    state_next = IDLE;
                end
            end
        endcase
    end

endmodule
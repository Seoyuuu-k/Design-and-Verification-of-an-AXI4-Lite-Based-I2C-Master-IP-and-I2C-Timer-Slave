`timescale 1ns / 1ps

//최종
// pactive high reset(center btn)
// slave 따로 비트스트림만들어서
// 다른 fpga board에 연결

module i2c_counter_slave_interface #(
    parameter [6:0] I2C_ADDR = 7'b111_1111  // 7-bit 슬레이브 주소 
) (
    //external
    input  logic clk,
    input  logic reset,   // active high

    input  logic SCL,
    inout  wire  SDA,
    //internal

    input  logic [31:0]counter, // from timer module
    output logic run,     //cr
    output logic updn,     
    output logic clr_cmd,  
    output logic load_cmd,
    output logic [31:0]set_tim_num
);

    // --------------------------------------------------
    // 내부 레지스터/와이어 선언
    // --------------------------------------------------
    logic  [7:0] temp_rx_data_reg, temp_rx_data_next;
    logic  [7:0] temp_tx_data_reg, temp_tx_data_next;
    logic  [7:0] temp_addr_reg,    temp_addr_next;
    logic  [7:0] reg_addr_reg,     reg_addr_next;

    logic  [3:0] bit_counter_reg,  bit_counter_next;
    logic  [1:0] byte_cnt_reg,     byte_cnt_next;

    logic        read_ack_reg,     read_ack_next;
    logic        o_data;

    logic [31:0] temp_reg_0_next,temp_reg_0_reg; //read위해서 미리 저장
    logic [31:0] temp_reg_1_next,temp_reg_1_reg;
    logic [31:0] temp_reg_2_next,temp_reg_2_reg;

    // --------------------------------------------------
    // I2C 라인 동기화 (모두 clk 도메인에서 처리) + start/stop condition
    // --------------------------------------------------
    /*
    logic scl_sync, scl_prev;
    logic sda_sync, sda_prev;

    always_ff @(posedge clk or negedge reset) begin
        if (!reset) begin
            scl_sync <= 1'b1;
            scl_prev <= 1'b1;
            sda_sync <= 1'b1;
            sda_prev <= 1'b1;
        end else begin
            scl_prev <= scl_sync;
            sda_prev <= sda_sync;
            scl_sync <= SCL;
            sda_sync <= SDA;
        end
    end

    wire scl_rise = (scl_sync == 1'b1 && scl_prev == 1'b0);
    wire scl_fall = (scl_sync == 1'b0 && scl_prev == 1'b1);

    // START / STOP 조건 (SCL high에서 SDA 변화)
    wire start_cond = (scl_sync == 1'b1 && sda_prev == 1'b1 && sda_sync == 1'b0);
    wire stop_cond  = (scl_sync == 1'b1 && sda_prev == 1'b0 && sda_sync == 1'b1);

*/

    // --------------------------------------------------
// I2C 라인 동기화 + 글리치 필터 + 엣지 검출 (매우 안전 버전)
//  - SCL/SDA는 외부 비동기 신호
//  - clk 도메인으로 3단 파이프 + majority 필터
// --------------------------------------------------

logic [2:0] scl_pipe, sda_pipe;   // 비동기 입력을 clk 도메인으로 파이프
logic       scl_sync, sda_sync;   // 필터링/동기화된 값
logic       scl_prev, sda_prev;   // 이전 클럭 값 (엣지 검출용)

// 3-탭 majority 함수 (2개 이상 1이면 1로 판단)
function automatic logic majority3(input logic [2:0] v);
    majority3 = (v[0] & v[1]) | (v[1] & v[2]) | (v[0] & v[2]);
endfunction

always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        // I2C idle 상태가 high 이므로 전부 1로 초기화
        scl_pipe <= 3'b111;
        sda_pipe <= 3'b111;

        scl_sync <= 1'b1;
        sda_sync <= 1'b1;
        scl_prev <= 1'b1;
        sda_prev <= 1'b1;
    end else begin
        // 1) 비동기 입력을 3단 플립플롭 체인에 태움 (메타 안정성 위험 크게 감소)
        scl_pipe <= {scl_pipe[1:0], SCL};
        sda_pipe <= {sda_pipe[1:0], SDA};

        // 2) majority 필터 (짧은 글리치는 무시)
        scl_sync <= majority3(scl_pipe);
        sda_sync <= majority3(sda_pipe);

        // 3) 이전 값 저장 (엣지 검출용)
        scl_prev <= scl_sync;
        sda_prev <= sda_sync;
    end
end

// 동기화 SCL/SDA 기준으로 엣지/스타트/스톱 검출
wire scl_rise = (scl_sync == 1'b1 && scl_prev == 1'b0);
wire scl_fall = (scl_sync == 1'b0 && scl_prev == 1'b1);

// START / STOP 조건 (SCL high + SDA 에지)
wire start_cond = (scl_sync == 1'b1 && sda_prev == 1'b1 && sda_sync == 1'b0);
wire stop_cond  = (scl_sync == 1'b1 && sda_prev == 1'b0 && sda_sync == 1'b1);

    // --------------------------------------------------
    // SDA 구동  
    // --------------------------------------------------
    logic sda_en;
    assign SDA = sda_en ? o_data : 1'bz;

    // --------------------------------------------------
    // 32-bit 레지스터들
    // --------------------------------------------------
    logic [31:0] reg0_cr_reg, reg0_cr_next;   // Control
    logic [31:0] reg1_set_reg, reg1_set_next; // 초기값
    logic [31:0] reg2_time;                   // 현재값 (Read Only)
    logic [31:0] reg0_cr, reg1_set;
    assign reg0_cr = reg0_cr_reg;
    assign reg1_set = reg1_set_reg;

    // REG0_CR 
    assign run      = reg0_cr[0];
    assign updn     = reg0_cr[1];
    assign clr_cmd  = reg0_cr[2];
    assign load_cmd = reg0_cr[3];
    //REG1_SET_NUM
    assign set_tim_num = reg1_set;

    // --------------------------------------------------
    // I2C 슬레이브 FSM
    // --------------------------------------------------
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_ADDR,      // 주소 + RW 수신
        ST_ACK_ADDR,  // 주소 ACK
        ST_REG,       // 레지스터 주소 수신
        ST_ACK_REG,   // 레지스터 주소 ACK
        ST_RX,        // 데이터 수신 (32bit write)
        ST_ACK_RX,    // 데이터 바이트 ACK
        ST_TX,        // 데이터 전송 (32bit read)
        ST_MACK       // 마스터 ACK/NACK 체크
    } i2c_state_t;

    i2c_state_t state, state_next;

    // --------------------------------------------------
    // slave module의 timer값 reg(reg2_time)
    // 매 clk마다 갱신중
    // --------------------------------------------------
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            reg2_time <= 32'd0;
        end else begin
            reg2_time <= counter; // from timer module
        end
    end

    // --------------------------------------------------
    // 상태/레지스터 플립플롭 (단일 sequential 블록)
    // --------------------------------------------------
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state           <= ST_IDLE;
            temp_rx_data_reg <= 8'd0;
            temp_tx_data_reg <= 8'd0;
            temp_addr_reg    <= 8'd0;
            reg_addr_reg     <= 8'd0;
            bit_counter_reg  <= 4'd0;
            byte_cnt_reg     <= 2'd0;
            read_ack_reg     <= 1'b0;

            reg0_cr_reg      <= 32'd0;
            reg1_set_reg     <= 32'd0;

            temp_reg_0_reg <=32'd0;
            temp_reg_1_reg <=32'd0;
            temp_reg_2_reg <=32'd0;
        end else begin
            state           <= state_next;
            temp_rx_data_reg <= temp_rx_data_next;
            temp_tx_data_reg <= temp_tx_data_next;
            temp_addr_reg    <= temp_addr_next;
            reg_addr_reg     <= reg_addr_next;
            bit_counter_reg  <= bit_counter_next;
            byte_cnt_reg     <= byte_cnt_next;
            read_ack_reg     <= read_ack_next;

            reg0_cr_reg      <= reg0_cr_next;
            reg1_set_reg     <= reg1_set_next;

            temp_reg_0_reg <= temp_reg_0_next;
            temp_reg_1_reg <= temp_reg_1_next;
            temp_reg_2_reg <= temp_reg_2_next;
        end
    end

    // --------------------------------------------------
    // I2C FSM + 레지스터 write 로직 (조합)
    // --------------------------------------------------
    always_comb begin
        // 기본값 (hold)
        state_next         = state;
        sda_en             = 1'b0;
        o_data             = 1'b0;

        temp_rx_data_next  = temp_rx_data_reg;
        temp_tx_data_next  = temp_tx_data_reg;
        temp_addr_next     = temp_addr_reg;
        reg_addr_next      = reg_addr_reg;

        bit_counter_next   = bit_counter_reg;
        byte_cnt_next      = byte_cnt_reg;
        read_ack_next      = read_ack_reg;

        reg0_cr_next       = reg0_cr_reg;
        reg1_set_next      = reg1_set_reg;

        temp_reg_0_next = temp_reg_0_reg;
        temp_reg_1_next = temp_reg_1_reg;
        temp_reg_2_next = temp_reg_2_reg;

        // ★ 1순위: Repeated START (어디서든 새 트랜잭션 시작)
        if (start_cond) begin
            state_next        = ST_ADDR;
            bit_counter_next  = 4'd0;
            byte_cnt_next     = 2'd0;
            temp_addr_next    = 8'd0;
   //         reg_addr_next     = 8'd0; //repeat start할떄 reg정보 날라가면 안됨!!!
            temp_rx_data_next = 8'd0;
            temp_tx_data_next = 8'd0;
            sda_en            = 1'b0;
            // read(S->M) 를 위해 당시 data 캡쳐 (중간에 바뀔 수도 있으니)
            temp_reg_0_next = reg0_cr_reg;
            temp_reg_1_next = reg1_set_reg;
            temp_reg_2_next = reg2_time;

        end
        // ★ 2순위: STOP
        else if (stop_cond) begin
            state_next        = ST_IDLE;
            sda_en            = 1'b0;
        end
        // ★ 3순위: 상태 머신 동작
        else begin
            case (state)
                //--------------------------------------------------
                ST_IDLE: begin
                    sda_en  = 1'b0;
                    // start_cond는 위에서 이미 처리
                end

                //--------------------------------------------------
                // 7bit 주소 + R/W 비트 수신 (M -> S)
                ST_ADDR: begin
                    sda_en = 1'b0; // 마스터가 SDA 구동

                    if (scl_rise) begin
                        temp_addr_next = {temp_addr_reg[6:0], SDA};
                    end

                    if (scl_fall) begin
                        if (bit_counter_reg == 4'd8) begin  
                            bit_counter_next = 4'd0;
                            state_next       = ST_ACK_ADDR;
                        end else begin
                            bit_counter_next = bit_counter_reg + 4'd1;
                        end
                    end
                end

                // 주소 ACK
                ST_ACK_ADDR: begin
                    if (temp_addr_reg[7:1] == I2C_ADDR) begin
                        sda_en = 1'b1;
                        o_data = 1'b0;   // ACK(0)

                        if (scl_fall) begin
                            byte_cnt_next = 2'd0;
                            if (temp_addr_reg[0] == 1'b1) begin //read
                                // Read 동작 (M<=S)
                                state_next       = ST_TX;
                                bit_counter_next = 4'd0;

                                // Read: 기존 reg_addr_reg 기준으로 첫 바이트 준비
                                // 당시 data 캡쳐 (중간에 바뀔 수도 있으니)
                                // start 할때 캡쳐해놓은 데이터

                                case (reg_addr_reg[1:0])
                                    2'd0: temp_tx_data_next = temp_reg_0_reg[31:24];
                                    2'd1: temp_tx_data_next = temp_reg_1_reg[31:24];
                                    2'd2: temp_tx_data_next = temp_reg_2_reg[31:24];
                                    default: temp_tx_data_next = 8'h00;
                                endcase

                                byte_cnt_next = byte_cnt_reg + 2'd1;
                            end else begin
                                // Write 동작 → 레지스터 주소 수신
                                state_next = ST_REG;
                            end
                        end
                    end else begin
                        // 주소 잘못 접근 → IDLE
                        state_next = ST_IDLE;
                    end
                end

                //--------------------------------------------------
                // 레지스터 주소 수신 (8bit, M->S)
                ST_REG: begin
                    sda_en = 1'b0;

                    if (scl_rise) begin
                        reg_addr_next = {reg_addr_reg[6:0], SDA};
                    end

                    if (scl_fall) begin
                        if (bit_counter_reg == 4'd7) begin
                            bit_counter_next = 4'd0;
                            state_next       = ST_ACK_REG;
                        end else begin
                            bit_counter_next = bit_counter_reg + 4'd1;
                        end
                    end
                end

                // 레지스터 주소 ACK
                ST_ACK_REG: begin
                    sda_en = 1'b1;
                    o_data = 1'b0;  // ACK

                    if (scl_fall) begin
                        state_next       = ST_RX; // Write (M->S)
                        byte_cnt_next    = 2'd0;
                        bit_counter_next = 4'd0;
                    end
                end

                //--------------------------------------------------
                // 데이터 수신 (32bit Write, 최대 4바이트)
                ST_RX: begin
                    sda_en = 1'b0;

                    // 비트 쉬프트 (M -> S)
                    if (scl_rise) begin
                        temp_rx_data_next = {temp_rx_data_reg[6:0], SDA};
                    end

                    if (scl_fall) begin
                        if (bit_counter_reg == 4'd7) begin
                            // 마지막 비트까지 포함된 새 바이트
                           // logic [7:0] new_byte;
                           // new_byte = {temp_rx_data_reg[6:0], SDA};

                            bit_counter_next = 4'd0;
                            state_next       = ST_ACK_RX;

                            // reg_addr_reg[1:0]에 따라 32bit 분배
                            case (reg_addr_reg[1:0])
                                2'd0: begin
                                    case (byte_cnt_reg)
                                        2'd0: reg0_cr_next[31:24] = temp_rx_data_reg;
                                        2'd1: reg0_cr_next[23:16] = temp_rx_data_reg;
                                        2'd2: reg0_cr_next[15:8 ] = temp_rx_data_reg;
                                        2'd3: reg0_cr_next[7:0  ] = temp_rx_data_reg;
                                    endcase
                                end
                                2'd1: begin
                                    case (byte_cnt_reg)
                                        2'd0: reg1_set_next[31:24] = temp_rx_data_reg;
                                        2'd1: reg1_set_next[23:16] = temp_rx_data_reg;
                                        2'd2: reg1_set_next[15:8 ] = temp_rx_data_reg;
                                        2'd3: reg1_set_next[7:0  ] = temp_rx_data_reg;
                                    endcase
                                end
                                2'd2: begin
                                    // REG2_TIME는 Read Only → 무시
                                end
                            endcase

                            byte_cnt_next = byte_cnt_reg + 2'd1;
                        end else begin
                            bit_counter_next = bit_counter_reg + 4'd1;
                        end
                    end
                end

                // 수신 데이터에 대한 ACK
                ST_ACK_RX: begin
                    sda_en = 1'b1;
                    o_data = 1'b0; // ACK

                    if (scl_fall) begin
                        state_next = ST_RX; // 마스터가 STOP/RESTART 걸면 위의 start/stop_cond 처리됨
                    end
                end

                //--------------------------------------------------
                // 데이터 전송 (32bit Read, S->M)
                ST_TX: begin
                    sda_en = 1'b1;
                    o_data = temp_tx_data_reg[7];

                    if (scl_fall) begin
                        if (bit_counter_reg == 4'd7) begin
                            bit_counter_next = 4'd0;
                            state_next       = ST_MACK; // 마스터 ACK/NACK 받을 차례
                        end else begin
                            temp_tx_data_next = {temp_tx_data_reg[6:0], 1'b0};
                            bit_counter_next  = bit_counter_reg + 4'd1;
                        end
                    end
                end

                // 마스터 ACK/NACK 샘플 (M->S)
                ST_MACK: begin
                    sda_en = 1'b0;

                    if (scl_rise) begin
                        read_ack_next = SDA; // 0: ACK, 1: NACK
                    end

                    if (scl_fall) begin
                        if (read_ack_reg == 1'b1) begin
                            // NACK → 전송 종료
                            read_ack_next = 1'b0;
                            state_next    = ST_IDLE;
                        end else if (read_ack_reg == 1'b0) begin
                            // ACK → 다음 바이트 준비
                            read_ack_next  = 1'b0;
                            state_next     = ST_TX;
                            bit_counter_next = 4'd0;

                            // 다음 바이트 선택 (32bit 전체 4바이트 지원)
                            case (reg_addr_reg[1:0])
                                2'd0: begin
                                    case (byte_cnt_reg)
                                        2'd0: temp_tx_data_next = temp_reg_0_reg[31:24];
                                        2'd1: temp_tx_data_next = temp_reg_0_reg[23:16];
                                        2'd2: temp_tx_data_next = temp_reg_0_reg[15:8 ];
                                        2'd3: temp_tx_data_next = temp_reg_0_reg[7:0  ];
                                        default: temp_tx_data_next = 8'h00;
                                    endcase
                                end
                                2'd1: begin
                                    case (byte_cnt_reg)
                                        2'd0: temp_tx_data_next = temp_reg_1_reg[31:24];
                                        2'd1: temp_tx_data_next = temp_reg_1_reg[23:16];
                                        2'd2: temp_tx_data_next = temp_reg_1_reg[15:8 ];
                                        2'd3: temp_tx_data_next = temp_reg_1_reg[7:0  ];
                                        default: temp_tx_data_next = 8'h00;
                                    endcase
                                end
                                2'd2: begin
                                    case (byte_cnt_reg)
                                        2'd0: temp_tx_data_next = temp_reg_2_reg[31:24];
                                        2'd1: temp_tx_data_next = temp_reg_2_reg[23:16];
                                        2'd2: temp_tx_data_next = temp_reg_2_reg[15:8 ];
                                        2'd3: temp_tx_data_next = temp_reg_2_reg[7:0  ];
                                        default: temp_tx_data_next = 8'h00;
                                    endcase
                                end
                                default: temp_tx_data_next = 8'h00;
                            endcase

                            byte_cnt_next = byte_cnt_reg + 2'd1;
                        end
                    end
                end

                default: begin
                    state_next = ST_IDLE;
                end
            endcase
        end

        // ------------------------------------------------------
        // reg0_cr의 one-shot 비트 자동 클리어
        // ------------------------------------------------------
        if (reg0_cr[2])  reg0_cr_next[2] = 1'b0; //clr_cmd
        if (reg0_cr[3]) reg0_cr_next[3] = 1'b0; //load_cmd
    end

endmodule

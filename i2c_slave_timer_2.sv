`timescale 1ns / 1ps

module i2c_timer_slave_2 #(
    parameter [6:0] I2C_ADDR = 7'h30  // 7-bit 슬레이브 주소
) (
    input logic clk,
    input logic reset,

    input logic SCL,
    inout wire  SDA
);


  // --------------------------------------------------
  // I2C 라인 동기화 
  // --------------------------------------------------
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

  wire  scl_rise = (scl_sync == 1'b1 && scl_prev == 1'b0);
  wire  scl_fall = (scl_sync == 1'b0 && scl_prev == 1'b1);

  // START / STOP 조건 (SCL high에서 SDA 변화)
  wire  start_cond = (scl_sync == 1'b1 && sda_prev == 1'b1 && sda_sync == 1'b0); // high일떄, rising
  wire  stop_cond = (scl_sync == 1'b1 && sda_prev == 1'b0 && sda_sync == 1'b1);// high일떄 폴링

  // --------------------------------------------------
  // SDA 구동  (open-drain : 0만 직접 드라이브, 1은 풀업에 맡김)
  // --------------------------------------------------
  logic sda_en;
  assign SDA = sda_en ? 1'b0 : 1'bz;

  // --------------------------------------------------
  // 32-bit 레지스터들
  // --------------------------------------------------
  logic [31:0] reg0_cr;  // Control
  logic [31:0] reg1_set;  // 초기값
  logic [31:0] reg2_time;  // 현재값 (Read Only)

  // 타이머 카운터
  logic [31:0] counter;

  // REG0_CR 비트 alias
  wire         run = reg0_cr[0];
  wire         updn = reg0_cr[1];
  wire         clr_cmd = reg0_cr[2];
  wire         load_cmd = reg0_cr[3];

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

  i2c_state_t       st;

  logic       [2:0] bit_cnt;  // 0..7
  logic       [1:0] byte_cnt;  // 0..3 (32bit)
  logic       [7:0] sh_in;
  logic       [7:0] sh_out;

  logic             rw;  // 0:Write, 1:Read
  logic             addr_ok;
  logic       [7:0] reg_ptr;  // 0x00, 0x01, 0x02

  // 쓰기용 임시 바이트
  logic       [7:0] data_byte;

  // --------------------------------------------------
  // I2C FSM + 타이머 로직 (clk 도메인, SCL 이벤트 기반)
  //  - Repeated START 지원: 어떤 상태에서든 start_cond 발생 시 ST_ADDR로 점프
  //  - 타이머도 같은 always_ff에서 처리 (reg0_cr에 대한 다중 드라이버 방지)
  // --------------------------------------------------
  always_ff @(posedge clk or negedge reset) begin
    if (!reset) begin
      st <= ST_IDLE;
      bit_cnt <= 3'd7;
      byte_cnt <= 2'd0;
      sh_in <= 8'h00;
      sh_out <= 8'h00;
      sda_en <= 1'b0;
      rw <= 1'b0;
      addr_ok <= 1'b0;
      reg_ptr <= 8'h00;
      reg0_cr <= 32'd0;
      reg1_set <= 32'd0;
      reg2_time <= 32'd0;  // reg2_time, counter는 위 타이머 블록에서 초기화 (이제 이 블록에서 같이 초기화)
      counter <= 32'd0;
    end else begin
      // --------------------------------------------------
      // 타이머 로직 (clk 도메인)
      // --------------------------------------------------
      // 우선순위: CLR > LOAD > RUN
      if (clr_cmd) begin
        counter <= 32'd0;
      end else if (load_cmd) begin
        counter <= reg1_set;
      end else if (run) begin
        if (updn) begin
          // UP 카운트
          counter <= counter + 1;
        end else begin
          // DOWN 카운트
          counter <= counter - 1;
        end
      end

      // 현재 값 미러링
      reg2_time <= counter;

      // 한 사이클짜리 제어 비트 자동 클리어
      if (clr_cmd) reg0_cr[2] <= 1'b0;
      if (load_cmd) reg0_cr[3] <= 1'b0;

      // --------------------------------------------------
      // I2C FSM
      // --------------------------------------------------

      // ★ 1순위: Repeated START (어디서든 새 트랜잭션 시작)
      if (start_cond) begin
        st       <= ST_ADDR;
        bit_cnt  <= 3'd7;
        byte_cnt <= 2'd0;
        sda_en   <= 1'b0;
      end  // ★ 2순위: STOP
      else if (stop_cond) begin
        st     <= ST_IDLE;
        sda_en <= 1'b0;
      end  // ★ 3순위: 상태 머신 동작
      else begin
        case (st)
          //--------------------------------------------------
          ST_IDLE: begin
            sda_en <= 1'b0;  // z
            // start_cond는 위에서 이미 처리
          end

          //--------------------------------------------------
          // 7bit 주소 + R/W 비트 수신
          ST_ADDR: begin
            sda_en <= 1'b0;  // 마스터가 SDA 구동
            if (scl_rise) begin // 라이징엣지에서 data read
              sh_in[bit_cnt] <= sda_sync;  // MSB first
              if (bit_cnt == 0) begin
                // [7:1] = 주소, [0] = R/W
                rw      <= sda_sync;
                addr_ok <= (sh_in[7:1] == I2C_ADDR);
                st      <= ST_ACK_ADDR;
                bit_cnt <= 3'd7;
              end else begin
                bit_cnt <= bit_cnt - 3'd1;
              end
            end
          end

          // 주소 ACK
          ST_ACK_ADDR: begin
            // OK면 ACK(0), 아니면 NACK(1) → open-drain
            // ACK : sda_en=1 (0 당김)
            // NACK: sda_en=0 (release → 1)
            sda_en <= addr_ok ? 1'b1 : 1'b0;
            if (scl_rise) begin
              sda_en <= 1'b0;  // 다음 비트에서 릴리즈
              if (!addr_ok) begin
                st <= ST_IDLE;
              end else begin
                byte_cnt <= 2'd0;
                if (rw == 1'b0) begin
                  // Write: 다음 바이트는 레지스터 주소
                  st <= ST_REG;
                end else begin
                  // Read: 기존 reg_ptr 기준으로 첫 바이트 준비 후 TX
                  st      <= ST_TX;
                  bit_cnt <= 3'd7;
                  case (reg_ptr[1:0])
                    2'd0: sh_out <= reg0_cr[31:24];
                    2'd1: sh_out <= reg1_set[31:24];
                    2'd2: sh_out <= reg2_time[31:24];
                    default: sh_out <= 8'h00;
                  endcase
                end
              end
            end
          end

          //--------------------------------------------------
          // 레지스터 주소 수신 (8bit)
          ST_REG: begin
            sda_en <= 1'b0;
            if (scl_rise) begin
              sh_in[bit_cnt] <= sda_sync;
              if (bit_cnt == 0) begin
                reg_ptr <= {sh_in[6:0], sda_sync};  // 0x00,0x01,0x02 예상
                st      <= ST_ACK_REG;
                bit_cnt <= 3'd7;
              end else begin
                bit_cnt <= bit_cnt - 3'd1;
              end
            end
          end

          // 레지스터 주소 ACK
          ST_ACK_REG: begin
            sda_en <= 1'b1;  // ACK(0)
            if (scl_rise) begin
              sda_en   <= 1'b0;
              st       <= ST_RX;
              byte_cnt <= 2'd0;
              bit_cnt  <= 3'd7;
            end
          end

          //--------------------------------------------------
          // 데이터 수신 (32bit Write, 최대 4바이트)
          ST_RX: begin
            sda_en <= 1'b0;
            if (scl_rise) begin
              // 매 비트마다 쉬프트
              sh_in <= {sh_in[6:0], sda_sync};  // MSB first 수신

              if (bit_cnt == 0) begin
                // 이 순간의 sh_in은 [b7..b1], sda_sync는 b0
                logic [7:0] new_byte;
                new_byte = {sh_in[6:0], sda_sync};  // 로컬 변수 (blocking)

                // reg_ptr에 따라 저장
                case (reg_ptr[1:0])
                  2'd0: begin
                    case (byte_cnt)
                      2'd0: reg0_cr[31:24] <= new_byte;
                      2'd1: reg0_cr[23:16] <= new_byte;
                      2'd2: reg0_cr[15:8] <= new_byte;
                      2'd3: reg0_cr[7:0] <= new_byte;
                    endcase
                  end
                  2'd1: begin
                    case (byte_cnt)
                      2'd0: reg1_set[31:24] <= new_byte;
                      2'd1: reg1_set[23:16] <= new_byte;
                      2'd2: reg1_set[15:8] <= new_byte;
                      2'd3: reg1_set[7:0] <= new_byte;
                    endcase
                  end
                  2'd2: begin
                    // REG2_TIME는 Read Only → 무시
                  end
                endcase

                byte_cnt <= byte_cnt + 2'd1;
                bit_cnt  <= 3'd7;
                st       <= ST_ACK_RX;
              end else begin
                bit_cnt <= bit_cnt - 3'd1;
              end
            end
          end

          // 수신 데이터에 대한 ACK
          ST_ACK_RX: begin
            sda_en <= 1'b1;  // ACK(0)
            if (scl_rise) begin
              sda_en <= 1'b0;
              st     <= ST_RX;  // 마스터가 STOP/RESTART 걸면 위의 start/stop_cond 처리됨
            end
          end

          //--------------------------------------------------
          // 데이터 전송 (32bit Read)
          ST_TX: begin
            // 슬레이브가 SDA 구동
            // open-drain: 보낼 비트가 0일 때만 당김
            sda_en <= (sh_out[bit_cnt] == 1'b0);

            if (scl_rise) begin
              if (bit_cnt == 0) begin
                bit_cnt <= 3'd7;
                st      <= ST_MACK;  // 마스터 ACK/NACK 받을 차례
              end else begin
                bit_cnt <= bit_cnt - 3'd1;
              end
            end
          end

          // 마스터 ACK/NACK 샘플
          ST_MACK: begin
            sda_en <= 1'b0;  // 릴리즈, 마스터가 구동
            if (scl_rise) begin
              if (sda_sync == 1'b0) begin
                // ACK → 다음 바이트 준비
                byte_cnt <= byte_cnt + 2'd1;

                // 다음 바이트 선택 (32bit 전체 4바이트 지원)
                case (reg_ptr[1:0])
                  2'd0: begin
                    case (byte_cnt)
                      2'd0: sh_out <= reg0_cr[23:16];
                      2'd1: sh_out <= reg0_cr[15:8];
                      2'd2: sh_out <= reg0_cr[7:0];
                      2'd3: sh_out <= 8'h00;     // 5번째 이후는 0
                      default: sh_out <= 8'h00;
                    endcase
                  end
                  2'd1: begin
                    case (byte_cnt)
                      2'd0: sh_out <= reg1_set[23:16];
                      2'd1: sh_out <= reg1_set[15:8];
                      2'd2: sh_out <= reg1_set[7:0];
                      2'd3: sh_out <= 8'h00;
                      default: sh_out <= 8'h00;
                    endcase
                  end
                  2'd2: begin
                    case (byte_cnt)
                      2'd0: sh_out <= reg2_time[23:16];
                      2'd1: sh_out <= reg2_time[15:8];
                      2'd2: sh_out <= reg2_time[7:0];
                      2'd3: sh_out <= 8'h00;
                      default: sh_out <= 8'h00;
                    endcase
                  end
                  default: sh_out <= 8'h00;
                endcase
                st <= ST_TX;
              end else begin
                // NACK → 전송 종료
                st <= ST_IDLE;
              end
            end
          end

          default: st <= ST_IDLE;
        endcase
      end
    end
  end

endmodule

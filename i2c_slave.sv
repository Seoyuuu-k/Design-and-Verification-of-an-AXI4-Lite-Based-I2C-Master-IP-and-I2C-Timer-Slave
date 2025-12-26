module i2c_slave_simple_nopull #(
  parameter [6:0] I2C_ADDR = 7'h48
)(
  input  wire SCL,
  inout  wire SDA
);

  // SDA 드라이브
  reg sda_drive;
  reg sda_val;
  assign SDA = sda_drive ? sda_val : 1'bz;

  typedef enum logic [2:0] {
    ST_IDLE, ST_ADDR, ST_ACK_ADDR,
    ST_RX, ST_ACK_RX,
    ST_TX, ST_MACK
  } st_t;

  st_t st = ST_IDLE;

  // 비트/쉬프트
  reg [2:0] bit_cnt = 3'd7;
  reg [7:0] sh_in  = 8'h00;  // 수신 시프트
  reg [7:0] sh_out = 8'h00;  // 송신 시프트

  // 상태 변수
  reg rw;           // 0=Write, 1=Read
  reg addr_ok;
  reg [7:0] ptr = 8'h00;
  reg ptr_loaded;

  // 간단 메모리
  reg [7:0] mem [0:255];
  integer i;
  initial begin
    for (i=0; i<256; i=i+1) mem[i] = i[7:0];
  end

  // START/STOP 검출
  reg sda_q = 1, scl_q = 1;
  always @(posedge SCL or negedge SCL or posedge SDA or negedge SDA) begin
    scl_q <= SCL;
    sda_q <= SDA;
  end

  wire start_cond = (SCL == 1'b1 && sda_q == 1'b1 && SDA == 1'b0);  // 1->0
  wire stop_cond  = (SCL == 1'b1 && sda_q == 1'b0 && SDA == 1'b1);  // 0->1

  // 메인 FSM
  always @(posedge SCL or posedge start_cond or posedge stop_cond) begin
    if (start_cond) begin
      // 새 트랜잭션 시작 (Repeated START 포함)
      st         <= ST_ADDR;
      bit_cnt    <= 3'd7;
      sda_drive  <= 1'b0;    // SDA 릴리즈
      // 포인터는 유지 → REG_WRITE 후 REG_READ 에서 재사용
      ptr_loaded <= 1'b0;    // 이번 트랜잭션에서 첫 바이트를 포인터인지 다시 판단
    end
    else if (stop_cond) begin
      st        <= ST_IDLE;
      sda_drive <= 1'b0;
    end
    else begin
      case (st)
        // --------------------------
        // 주소 + R/W 수신
        // --------------------------
        ST_IDLE: begin
          sda_drive <= 1'b0;
        end

        ST_ADDR: begin
          sda_drive       <= 1'b0;       // 항상 릴리즈
          sh_in[bit_cnt]  <= SDA;
          if (bit_cnt == 0) begin
            rw      <= SDA;              // 마지막 비트 = R/W
            addr_ok <= (sh_in[7:1] == I2C_ADDR);  // 상위 7비트 비교
            bit_cnt <= 3'd7;
            st      <= ST_ACK_ADDR;
          end
          else bit_cnt <= bit_cnt - 3'd1;
        end

        ST_ACK_ADDR: begin
          if (addr_ok) begin
            // 주소 일치 → ACK
            sda_drive <= 1'b1;
            sda_val   <= 1'b0;           // ACK = 0
            if (rw == 1'b0) begin
              // Write 트랜잭션: 다음 바이트 수신
              st <= ST_RX;
            end
            else begin
              // Read 트랜잭션: 현재 ptr에서 읽기 시작
              sh_out   <= mem[ptr];
              st       <= ST_TX;
            end
          end
          else begin
            // 주소 불일치 → 릴리즈
            sda_drive <= 1'b0;
            st        <= ST_IDLE;
          end
        end

        // --------------------------
        // 바이트 수신 (Write)
        // --------------------------
        ST_RX: begin
          sda_drive      <= 1'b0;        // 슬레이브는 버스 릴리즈
          sh_in[bit_cnt] <= SDA;
          if (bit_cnt == 0) begin
            bit_cnt <= 3'd7;
            // 첫 바이트면 포인터로 사용
            if (!ptr_loaded) begin
              ptr        <= {sh_in[6:0], SDA};
              ptr_loaded <= 1'b1;
            end
            else begin
              mem[ptr] <= {sh_in[6:0], SDA};
              ptr      <= ptr + 8'd1;
            end
            st <= ST_ACK_RX;
          end
          else bit_cnt <= bit_cnt - 3'd1;
        end

        ST_ACK_RX: begin
          sda_drive <= 1'b1;
          sda_val   <= 1'b0;   // ACK
          st        <= ST_RX;  // 마스터가 STOP/RS 해 줄 때까지 계속
        end

        // --------------------------
        // 바이트 송신 (Read)
        // --------------------------
        ST_TX: begin
          sda_drive <= 1'b1;
          sda_val   <= sh_out[bit_cnt];
          if (bit_cnt == 0) begin
            bit_cnt <= 3'd7;
            st      <= ST_MACK;   // 마스터의 ACK/NACK 대기
          end
          else bit_cnt <= bit_cnt - 3'd1;
        end

        // 마스터 ACK/NACK 샘플
        ST_MACK: begin
          sda_drive <= 1'b0;   // 릴리즈, 마스터가 구동
          if (SDA == 1'b0) begin
            // 마스터 ACK → 다음 바이트
            ptr    <= ptr + 8'd1;
            sh_out <= mem[ptr + 8'd1];
            st     <= ST_TX;
          end
          else begin
            // 마스터 NACK → 끝
            st <= ST_IDLE;
          end
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

endmodule

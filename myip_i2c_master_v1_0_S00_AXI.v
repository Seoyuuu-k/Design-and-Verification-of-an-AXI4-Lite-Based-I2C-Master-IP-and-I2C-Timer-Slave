`timescale 1 ns / 1 ps

module myip_i2c_master_v1_0_S00_AXI #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 4
) (
    // User-side ports
    output              i2c_en,
    output              start,
    input               ready,
    input               done,

    output      [1:0]   mode,        // 00:Write, 01:Read, 10:RegWrite, 11:RegRead
    output      [6:0]   slave_addr,
    output      [7:0]   reg_addr,
    output      [1:0]   burst_len,   // 00:1B, 01:2B, 10:3B, 11:4B

    output      [31:0]  tx_data,     // up to 4 bytes
    input       [31:0]  rx_data,
    input               tx_done,
    input               rx_done,
    input               rx_done_final, // read 동작 다끝나면 ,out_rx_data 갱신!

    output reg [31:0] out_rx_data,

    // AXI4-Lite slave ports
    input  wire                       S_AXI_ACLK,
    input  wire                       S_AXI_ARESETN,
    input  wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_AWADDR,
    input  wire [2 : 0]               S_AXI_AWPROT,
    input  wire                       S_AXI_AWVALID,     
    output wire                       S_AXI_AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
    input  wire                       S_AXI_WVALID,
    output wire                       S_AXI_WREADY,
    output wire [1 : 0]               S_AXI_BRESP,
    output wire                       S_AXI_BVALID,
    input  wire                       S_AXI_BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_ARADDR,
    input  wire [2 : 0]               S_AXI_ARPROT,
    input  wire                       S_AXI_ARVALID,
    output wire                       S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_RDATA,
    output wire [1 : 0]               S_AXI_RRESP,
    output wire                       S_AXI_RVALID,
    input  wire                       S_AXI_RREADY
);

    // --------------------------------------------------------
    // AXI infra signals
    // --------------------------------------------------------
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_awaddr;
    reg axi_awready;
    reg axi_wready;
    reg [1 : 0] axi_bresp;
    reg axi_bvalid;
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_araddr;
    reg axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1 : 0] axi_rdata;
    reg [1 : 0] axi_rresp;
    reg axi_rvalid;

    localparam integer ADDR_LSB          = (C_S_AXI_DATA_WIDTH/32) + 1;
    localparam integer OPT_MEM_ADDR_BITS = 1;

    // --------------------------------------------------------
    // Slave registers
    // --------------------------------------------------------
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg0; // status + control
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg1; // addr
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg2; // tx_data
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg3; // rx_data

    wire slv_reg_rden;
    wire slv_reg_wren;
    reg  [C_S_AXI_DATA_WIDTH-1:0] reg_data_out;
    integer byte_index;
    reg aw_en;


    // ---------- reg0 bit 위치 ----------
    localparam integer I2C_EN_BIT   = 0; // RW
    localparam integer START_BIT    = 1; // RW (done==1이면 자동 0)
    localparam integer MODE_LSB     = 2; // [3:2] RW
    localparam integer BURST_LSB    = 4; // [5:4] RW
    localparam integer READY_BIT    = 6; // RO
    localparam integer RX_DONE_BIT  = 7; // RO
    localparam integer TX_DONE_BIT  = 8; // RO
    localparam integer DONE_BIT     = 9; // RO

    // ---------- reg1 위치 ----------
    localparam integer SLAVE_LSB    = 0; // [6:0] RW
    localparam integer REGADDR_LSB  = 7; // [14:7] RW

    // --------------------------------------------------------
    // AXI outputs
    // --------------------------------------------------------
    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;

    // --------------------------------------------------------
    // AWREADY
    // --------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_awready <= 1'b0;
            aw_en       <= 1'b1;
        end else begin
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
                axi_awready <= 1'b1;
                aw_en       <= 1'b0;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                aw_en       <= 1'b1;
                axi_awready <= 1'b0;
            end else begin
                axi_awready <= 1'b0;
            end
        end
    end

    // --------------------------------------------------------
    // AWADDR latch
    // --------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN)
            axi_awaddr <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        else if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en)
            axi_awaddr <= S_AXI_AWADDR;
    end

    // --------------------------------------------------------
    // WREADY
    // --------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN)
            axi_wready <= 1'b0;
        else if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID && aw_en)
            axi_wready <= 1'b1;
        else
            axi_wready <= 1'b0;
    end

    // --------------------------------------------------------
    // Register write enable
    // --------------------------------------------------------
    assign slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

    // --------------------------------------------------------
    // Register write + status + start auto-clear
    // --------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            slv_reg0 <= {C_S_AXI_DATA_WIDTH{1'b0}};
            slv_reg1 <= {C_S_AXI_DATA_WIDTH{1'b0}};
            slv_reg2 <= {C_S_AXI_DATA_WIDTH{1'b0}};
            slv_reg3 <= {C_S_AXI_DATA_WIDTH{1'b0}};
        end else begin
            // ------------ SW write ------------
            if (slv_reg_wren) begin
                case (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
                    // reg0 : control + status (status bit들은 아래 HW가 override)
                    2'h0: begin
                        for (byte_index = 0;
                             byte_index <= (C_S_AXI_DATA_WIDTH/8)-1;
                             byte_index = byte_index + 1) begin
                            if (S_AXI_WSTRB[byte_index]) begin
                                slv_reg0[(byte_index*8)+:8] <=
                                    S_AXI_WDATA[(byte_index*8)+:8];
                            end
                        end
                    end

                    // reg1 : addr
                    2'h1: begin
                        for (byte_index = 0;
                             byte_index <= (C_S_AXI_DATA_WIDTH/8)-1;
                             byte_index = byte_index + 1) begin
                            if (S_AXI_WSTRB[byte_index]) begin
                                slv_reg1[(byte_index*8)+:8] <=
                                    S_AXI_WDATA[(byte_index*8)+:8];
                            end
                        end
                    end

                    // reg2 : tx_data
                    2'h2: begin
                        for (byte_index = 0;
                             byte_index <= (C_S_AXI_DATA_WIDTH/8)-1;
                             byte_index = byte_index + 1) begin
                            if (S_AXI_WSTRB[byte_index]) begin
                                slv_reg2[(byte_index*8)+:8] <=
                                    S_AXI_WDATA[(byte_index*8)+:8];
                            end
                        end
                    end

                    // reg3 : rx_data (RO → SW write 무시)
                    2'h3: begin
                        // do nothing
                    end

                    default: ;
                endcase
            end

            // ------------ HW status (read-only) 갱신 ------------
            slv_reg0[DONE_BIT]    <= done;
            slv_reg0[TX_DONE_BIT] <= tx_done;
            slv_reg0[RX_DONE_BIT] <= rx_done;
            slv_reg0[READY_BIT]   <= ready;

            // rx_data도 항상 reg3에 반영 (RO)
            slv_reg3 <= rx_data;

            // ------------ start 자동 클리어 ------------
            // 알아서 자동으로 내려가도록! 
            // 안그러면  IDLE상태로 다시 back했을떄
            // i2c_en상태에서
            // start가 또 걸리면서 또 시작해버림
            if (done)
                slv_reg0[START_BIT] <= 1'b0;
        end
    end

    // --------------------------------------------------------
    // Write response
    // --------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_bvalid <= 1'b0;
            axi_bresp  <= 2'b0;
        end else begin
            if (axi_awready && S_AXI_AWVALID &&
                ~axi_bvalid && axi_wready && S_AXI_WVALID) begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b0; // OKAY
            end else if (S_AXI_BREADY && axi_bvalid) begin
                axi_bvalid <= 1'b0;
            end
        end
    end

    // --------------------------------------------------------
    // ARREADY + ARADDR
    // --------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_arready <= 1'b0;
            axi_araddr  <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        end else begin
            if (~axi_arready && S_AXI_ARVALID) begin
                axi_arready <= 1'b1;
                axi_araddr  <= S_AXI_ARADDR;
            end else begin
                axi_arready <= 1'b0;
            end
        end
    end

    // --------------------------------------------------------
    // RVALID / RRESP
    // --------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_rvalid <= 1'b0;
            axi_rresp  <= 2'b0;
        end else begin
            if (axi_arready && S_AXI_ARVALID && ~axi_rvalid) begin
                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b0; // OKAY
            end else if (axi_rvalid && S_AXI_RREADY) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

    // --------------------------------------------------------
    // Read mux
    // --------------------------------------------------------
    assign slv_reg_rden = axi_arready & S_AXI_ARVALID & ~axi_rvalid;

    always @(*) begin
        case (axi_araddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
            2'h0   : reg_data_out = slv_reg0;
            2'h1   : reg_data_out = slv_reg1;
            2'h2   : reg_data_out = slv_reg2;
            2'h3   : reg_data_out = slv_reg3;
            default: reg_data_out = {C_S_AXI_DATA_WIDTH{1'b0}};
        endcase
    end

    // --------------------------------------------------------
    // Read data output
    // --------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN)
            axi_rdata <= {C_S_AXI_DATA_WIDTH{1'b0}};
        else if (slv_reg_rden)
            axi_rdata <= reg_data_out;
    end

    // ========================================================
    // User-side outputs 연결
    // ========================================================
    assign i2c_en     = slv_reg0[I2C_EN_BIT];
    assign start      = slv_reg0[START_BIT];
    assign mode       = slv_reg0[MODE_LSB+1  : MODE_LSB];
    assign burst_len  = slv_reg0[BURST_LSB+1 : BURST_LSB];

    assign slave_addr = slv_reg1[SLAVE_LSB+6  : SLAVE_LSB];
    assign reg_addr   = slv_reg1[REGADDR_LSB+7: REGADDR_LSB];

    assign tx_data    = slv_reg2;

    always @(posedge S_AXI_ACLK) begin
         if (!S_AXI_ARESETN)
            out_rx_data <= 32'b0;
        else if (rx_done_final )out_rx_data <= slv_reg3;
        // rx_done_final =1일때 갱신!!
        
    end

endmodule

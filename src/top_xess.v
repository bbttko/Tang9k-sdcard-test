/*
 * testing sdcard
 *
 * hardware:
 *      TangNano9k w/ Gowin GW1NR-LV9QN88P
 *      uart 115200,N81 (connected through TangNano9K )
 *      sdcard
 * rtl:
 *      SDCard.vhd from https://github.com/xesscorp/VHDL_Lib/blob/master/SDCard.vhd
 * 
 * to read: type 'r' in uart to read from sdcard lba.  for lba, see 'addr'
 * to write: not implemented.. 
 * 
 */
module top_xess  (
        input wire clk, rstn, 

        output wire [6:1] LEDS,

        // uart
        input wire uart_rx,
        output wire uart_tx,

        //SPI pinouts
        input wire SD_MISO,
        output wire SD_MOSI,
        output wire SD_DCLK,
        output wire SD_nCS
    );


    reg [31:0] addr;
    reg [7:0] wdata, next_wdata;
    reg wrena, next_wrena;
    wire [7:0] rdata;
    wire ready;
    wire wready, busy;
    reg rdena;
    wire data_rdy;
    reg data_ack;
    
    SdCardCtrl #(
        .FREQ_G          (27),
        .INIT_SPI_FREQ_G (0.4),
        .SPI_FREQ_G      (0.4),
        .BLOCK_SIZE_G    (512),
        .CARD_TYPE_G     (0)
    ) sdc (
        .clk_i      (clk),
        .reset_i    (~rstn),
        .rd_i       (rdena),
        .wr_i       (wrena),
//        .continue_i (), use default = 0, do not continue using address
        .addr_i     (addr),
        .data_i     (wdata),
        .data_o     (rdata),
        .busy_o     (busy),
        .hndShk_i   (data_ack), // : in  std_logic;  -- High when host has data to give or has taken data.
        .hndShk_o   (data_rdy), // : out std_logic;  -- High when controller has taken data or has data to give.
//        .error_o    : out std_logic_vector(15 downto 0) := (others => NO);

        .cs_bo      (SD_nCS),
        .sclk_o     (SD_DCLK),
        .mosi_o     (SD_MOSI),
        .miso_i     (SD_MISO)
    );

    reg [6:1] ledreg;
    assign LEDS = ledreg;

    localparam IDLE = 0,
                CHECK = 1,
                WRITE = 2,
                CHECK2 = 3,
                DONE = 4,
                START_READ = 5,
                DO_READ = 6,
                WAIT_TX_DONE = 7,
                START_WRITE = 8,
                DO_WRITE = 9;

    reg [3:0] state, next_state;
    reg [7:0] mydata, next_mydata;
    reg [$clog2(512):0] counter, next_counter;
    wire do_read, do_write;

    // register operations
    always @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            state <= IDLE;
            wdata <= 0;
            counter <= 0;
            wrena <= 0;
        end else begin
            state <= next_state;
            wdata <= next_wdata;
            counter <= next_counter;
            wrena <= next_wrena;
        end
    end


    // fsm
    always @* begin
        next_state = state;
        next_wrena = 0;
        next_wdata = wdata;
        next_counter = counter;
        rdena = 0;
        data_ack = 0;
        uart_tx_dv = 0;
        uart_tx_byte = 0;

        case (state)
            IDLE:   // #0
                    begin
                        if (do_read) next_state = START_READ;
                        if (do_write) next_state = START_WRITE;
                        ledreg[6]=1'b0;
                    end

            CHECK:  begin   // #1
                        next_wrena = 0;
                        if (wready) next_state = WRITE;
                        else next_state = CHECK;
                    end

            WRITE:          // #2
                if (wready) begin
                    if (counter == 0) 
                        next_state = DONE;
                    else begin
                        next_wdata = wdata + 1;
                        next_counter = counter - 1;
                        next_wrena = 1;
                        next_state = CHECK2;
                    end
                end else begin
                    next_state = WRITE;
                end

            CHECK2: if (!wready) next_state = CHECK;     // #3
                    else next_state = CHECK2;

            DONE: next_state = DONE;    // #4

            START_READ:    // #5     - start reading from address 1
                begin
                    ledreg[4]=1'b1;
                    rdena = 1;
                    addr = 0;
                    if (busy) next_state = DO_READ;     // busy='1' during rdena, the read has started..
                end

            DO_READ: 
                begin
                    ledreg[5]=1'b1;
                    if (busy == 0) begin            // completed reading
                        next_state = IDLE;
                    end else if (data_rdy) begin    // write to uart when data is available
                        uart_tx_byte = rdata;
                        uart_tx_dv = 1;
                        next_state = WAIT_TX_DONE;
                    end
                end

            START_WRITE:
                begin
                    ledreg[4]=1'b1;
                    next_state = IDLE;
                end

//            DO_WRITE:
//                begin
//                end


            WAIT_TX_DONE:                   // wait for uart tx to complete
                if (uart_tx_done) begin
                    next_state = DO_READ;
                    data_ack = 1;
                end
                
        endcase
    end


    // -------- uart --------------
    wire uart_rx_valid;
    wire [7:0] uart_rx_byte;

    reg uart_tx_dv;
    reg [7:0] uart_tx_byte;
    wire uart_tx_active;
    wire uart_tx_done;

    UART_RX #(.g_CLKS_PER_BIT (234)) uartrx (
        .i_Clk       (clk),
        .i_RX_Serial (uart_rx),
        .o_RX_DV     (uart_rx_valid),
        .o_RX_Byte   (uart_rx_byte)
    );
    UART_TX #(.g_CLKS_PER_BIT(234)) uarttx (
        .i_Clk       (clk),
        .i_TX_DV     (uart_tx_dv),
        .i_TX_Byte   (uart_tx_byte),
        .o_TX_Active (uart_tx_active),
        .o_TX_Serial (uart_tx),
        .o_TX_Done   (uart_tx_done)
    );


    assign do_read = ((uart_rx_valid) && (uart_rx_byte == 8'h72) && (~uart_tx_active)); // 0x72 = 'r'
    assign do_write = ((uart_rx_valid) && (uart_rx_byte == 8'h77) && (~uart_tx_active));    // 0x77 = 'w'

endmodule

module top (
        input wire clk, rstn, button,

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


    reg [31:0] waddr;
    reg [7:0] wdata, next_wdata;
    reg wrena, next_wrena;
    wire [7:0] rdata;
    wire ready, button_db;
    wire wready;


    sdcard_interface sdcard (
        .clk    (clk),
        .rstn   (rstn),     // initialize system and sdcard 
        
        // useless debug stuff 
        .led0_r (LEDS[1]),
        .led0_g (LEDS[2]),
        .led0_b (LEDS[3]),

        // wr interface
        .waddr  (waddr),
        .wdata  (wdata),
        .wrena  (wrena),
        .wready (wready),
        .rdata  (rdata),
        .ready  (ready),

        // sdcard interface
        .SD_MISO    (SD_MISO),
        .SD_MOSI    (SD_MOSI),
        .SD_DCLK    (SD_DCLK),
        .SD_nCS     (SD_nCS)
    );


    localparam IDLE = 0,
                CHECK = 1,
                WRITE = 2,
                CHECK2 = 3,
                DONE = 4,
                DO_READ=5;


    reg [2:0] state, next_state;
    reg [7:0] mydata, next_mydata;
    reg [$clog2(512):0] counter, next_counter;
    reg do_read;

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
        waddr = 2052;      // <==============
        next_wdata = wdata;
        next_counter = counter;
        case (state)
            IDLE:   // #0
                    if (button_db) begin
                        next_state = CHECK;
                        next_wrena = 1;
                        waddr = 2053;
                        next_wdata = 0;
                        next_counter = 516;
                    end else begin
                        if (do_read) begin      // internal uart read
                            next_state = DO_READ;
                        end
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

            DO_READ:    // #5
                begin
                    
                end


        endcase
    end

	debounce_explicit (
		.clk    (clk),
		.rst_n  (rstn),
		.sw     (button),
		.db_tick(button_db)
    );


    // -------- uart --------------
    wire uart_rx_valid;
    wire [7:0] uart_rx_byte;
    reg [7:0] uart_byte;

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


    localparam UART_IDLE = 1,
                UART_R_CHECK=2,
                UART_TX_WAIT=3;

    reg [3:0] uart_state, n_uart_state;

    // reg ops
    always @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            uart_state <= UART_IDLE;
        end else begin
            uart_state <= n_uart_state;
        end
    end


    // uart fsm 
    always @* begin
        n_uart_state = uart_state;
        uart_tx_dv = 0;
        uart_tx_byte = 0;
        do_read = 0;
        case (uart_state)
            UART_IDLE:
                    if (uart_rx_valid) begin
                        n_uart_state = UART_R_CHECK;
                        uart_byte = uart_rx_byte;
                    end
            UART_R_CHECK: 
                    begin
                        if ((uart_byte == 8'h72) && (~uart_tx_active))begin       // 'r'
                            do_read = 1;
                            uart_tx_byte = 8'h78;                                // 'x'
                            uart_tx_dv = 1;
                            n_uart_state = UART_TX_WAIT;
                        end else begin
                            n_uart_state = UART_IDLE;
                        end
                    end
                    
            UART_TX_WAIT: if (uart_tx_done) n_uart_state = UART_IDLE;
            default: n_uart_state = UART_IDLE;

        endcase
    end

endmodule

//====================================================================================================================
//====================================================================================================================
//====================================================================================================================

    // register operations
//    always @(posedge clk, negedge rstn) begin
//        if (~rstn) begin
//            state <= IDLE;
//            wdata <= 0;
//            counter <= 0;
//            wrena <= 0;
//        end else begin
//            state <= next_state;
//            wdata <= next_wdata;
//            counter <= next_counter;
//            wrena <= next_wrena;
//        end
//    end

    // fsm
//    always @* begin
//        next_state = state;
//        next_wrena = 0;
//        waddr = 0;
//        next_wdata = wdata;
//        next_counter = counter;
//        case (state)
//            IDLE:   // #0
//                begin
//                    if (button_db) begin
//                        next_state = WRITE;
//                        next_wrena = 1;
//                        waddr = 2052;
//                        next_wdata = 0;
//                        next_counter = 512;
//                    end
//                end

//            WRITE:  // #1
//                if (ready) begin
//                    if (counter == 0) begin
//                        next_state = DONE;
//                    end else begin
//                        next_wdata = wdata + 1;
//                        next_counter = counter - 1;
//                        next_wrena = 1;
//                    end
//                end

//            DONE: next_state = DONE;    // #2
//        endcase
//    end

/*
module top (
        input wire clk, rstn, button,

        output wire [6:1] LEDS,

        //SPI pinouts
        input wire SD_MISO,
        output wire SD_MOSI,
        output wire SD_DCLK,
        output wire SD_nCS
    );


    reg [31:0] waddr;
    reg [7:0] wdata, next_wdata;
    reg wrena, next_wrena;
    wire [7:0] rdata;
    wire ready, button_db;

    sdcard_interface sdcard (
        .clk    (clk),
        .rstn   (rstn),     // initialize system and sdcard 
        
        // useless debug stuff 
        .led0_r (LEDS[1]),
        .led0_g (LEDS[2]),
        .led0_b (LEDS[3]),

        // wr interface
        .waddr  (waddr),
        .wdata  (wdata),
        .wrena  (wrena),
        .rdata  (rdata),
        .ready  (ready),

        // sdcard interface
        .SD_MISO    (SD_MISO),
        .SD_MOSI    (SD_MOSI),
        .SD_DCLK    (SD_DCLK),
        .SD_nCS     (SD_nCS)
    );


    localparam IDLE = 0,
                WRITE = 1,
                DONE = 2;


    reg [2:0] state, next_state;
    reg [7:0] mydata, next_mydata;
    reg [$clog2(512):0] counter, next_counter;

    // register operations
    always @(posedge clk, negedge rstn) begin
        if (~rstn) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end


    // fsm
    always @* begin
        next_state = state;
        wrena = 0;
        case (state)
            IDLE:   // #0
                begin
                    if (button_db) begin
                        next_state = DONE;
                        wrena = 1;
                        waddr = 2051;
                        wdata = 0;
                    end
                end

            WRITE:  // #1
                if (ready) begin
                    if (counter == 0) begin
                        next_state = DONE;
                    end else begin
                        next_wdata = wdata + 1;
                        next_counter = counter - 1;
                        next_wrena = 1;
                    end
                end

            DONE: next_state = DONE;    // #2
        endcase
    end

	debounce_explicit (
		.clk    (clk),
		.rst_n  (rstn),
		.sw     (button),
		.db_tick(button_db)
    );
endmodule
*/
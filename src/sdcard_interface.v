`timescale 1ns / 1ps

module sdcard_interface(
        input wire clk, rstn,
        output wire led0_r,led0_g,led0_b, //{red,green,blue} red if SDCARD initialization is stuck at CMD0, blue if stuck somewhere else, green if initialization complete

        //SPI pinouts
        input wire SD_MISO,
        output wire SD_MOSI,
        output wire SD_DCLK,
        output wire SD_nCS,


        input [31:0] waddr,
        input [7:0] wdata,
        input wrena,
        output wready,

        output reg [7:0] rdata,
        output ready
    );
    wire rst = ~rstn;
//    wire btn = ~btnn;

	 //FSM states
    localparam POWER_ON=0, 
               COMMANDS=1,
               SEND_COMMAND=2,
               RECEIVE_RESPONSE=3,
               END_CMD=4,
			   IDLE=5,
			   DELAY=6,
                WRITE_X=10,
                WRITE_DATA=11,
			   WRITE_1=7,
			   WRITE_2=8,
			   BUSY=9;

					 
    reg[3:0] state_q=0,state_d;
    reg[9:0] counter_q=0,counter_d; //counter for the 74 clk cycles needed for power-on
	reg[3:0] cmd_counter_q=0,cmd_counter_d; //index for cmd_list
	reg[3:0] response_counter_q=0,response_counter_d; //number of bytes needed for a response (R1=1 byte , R7=R3=5 bytes)
    reg[55:0] wr_data_q=0,wr_data_d;
	reg[39:0] rd_data_q=0,rd_data_d;
    reg[2:0] led_q=0,led_d;
	reg stuck_q=0,stuck_d;
	reg[9:0] stuck_counter_q=0,stuck_counter_d;
	reg[15:0] addr_counter_q,addr_counter_d;
	wire key_tick;
	
    //SPI pinouts
    reg rd,wr,hold;
    reg[7:0] wr_data;
    reg clk_div_q=0,clk_div_d;
    wire[7:0] rd_data;
    wire done_tick;
    wire clk_div,cs_n_1;
    
    //uart PINOUTS
//	 reg wr_uart,rd_uart;
//	 reg[7:0] wr_data_uart;
//	 wire[7:0] rd_data_uart;
//	 wire rx_empty;
	 
	 //list of commands for SDCARD initialization
	 localparam INIT_LAST_INDEX=6;
	 reg[55:0] cmd_list[10:0];
	 initial begin
        cmd_list[0]=48'h40_00_00_00_00_95;  //CMD0: GO_IDLE_STATE (R1) //resets SDCARD for SPI mode
        cmd_list[1]=48'h48_00_00_01_AA_87;  //CMD8: SEND_IF_COND (R7)  //chceck host voltage supply and if ver2.00 above
        cmd_list[2]=48'h7B_00_00_00_00_83;  //CMD59 CRC_ON_OFF (R1) //turn off CRC checking
        cmd_list[3]=48'h77_00_00_00_00_00;  //CMD55: prefix for every ACMD (application commad) (R1)
        cmd_list[4]=48'h69_40_00_00_00_00;  //ACMD41:SD_SEND_OP_COND ((R1) //iniitialize SDCARD
        cmd_list[5]=48'h7A_00_00_00_00_00;  //CMD58:READ_OCR (R3) //check if SDSC(standard) or SDHC/SDXC
        cmd_list[6]=48'h50_00_00_02_00_00; //CMD16: set block length to 512
        cmd_list[7]=48'h58_00_00_00_00_00;  //CMD24: Single WRITE TO ADDRESS 0
        cmd_list[8]=48'h4D_00_00_00_00_00;  //Status Reg
	 end
	 				 

wire out;
		reg [31:0] current_address;
	 
    //register operations
    always @(posedge clk,posedge rst) begin
        if(rst) begin
            state_q<=0;
            counter_q<=0;
            wr_data_q<=0;
			rd_data_q<=0;
            led_q<=0;
			cmd_counter_q<=0;
			response_counter_q<=0;
			stuck_q<=0;
			stuck_counter_q<=0;
			clk_div_q<=0;
			addr_counter_q<=0;
        end else begin
            state_q<=state_d;
            counter_q<=counter_d;
            wr_data_q<=wr_data_d;
			rd_data_q<=rd_data_d;
            led_q<=led_d;
		    cmd_counter_q<=cmd_counter_d;
		    response_counter_q<=response_counter_d;
		    stuck_q<=stuck_d;
		    stuck_counter_q<=stuck_counter_d;
		    clk_div_q<=clk_div_d;
		    addr_counter_q<=addr_counter_d;
        end
    end
    
    //FSM logic 
    always @* begin
        state_d     = state_q;
        counter_d   = counter_q;
        wr_data_d   = wr_data_q;
		rd_data_d   = rd_data_q;
        led_d       = led_q;
		cmd_counter_d = cmd_counter_q;
	    response_counter_d = response_counter_q;	  
	    stuck_d     = stuck_q;
	    stuck_counter_d = stuck_counter_q;
	    clk_div_d   = clk_div_q;
	    addr_counter_d = addr_counter_q;
        rd = 0;
        wr = 0;
        hold = 0;
//		wr_data = 8'hff;
 
        case(state_q)
      ///////////////////////////////////START SDCARD INITIALIZATION////////////////////////////////////////////
            POWER_ON: if(out) begin //send at least 74 clk cycles with cs_n and d_out line high
                        rd=1;
                        led_d=3'b100;
					    cmd_counter_d=0;
					    clk_div_d=0;
					    addr_counter_d=0;
                        if(done_tick) begin
                            counter_d=counter_q+1'b1;
                            if(counter_q==15) begin//8*10=80 clk cycles had passed
                                rd=0;
                                state_d=COMMANDS;
                            end
                        end
                      end
            COMMANDS: if(ready) begin //commands to be sent to SDCARD
                        wr_data_d=cmd_list[cmd_counter_q]; 
//                        if(cmd_counter_q==7) wr_data_d[39:8]=2049+addr_counter_q; //start address(2050): Real start: 2049-2048+1=2
                        if (cmd_counter_q==7)
//                            wr_data_d[39:8] = current_address + addr_counter_q; //start address(2050): Real start: 2049-2048+1=2
                            wr_data_d[39:8] = current_address; //start address(2050): Real start: 2049-2048+1=2
                        state_d=SEND_COMMAND;
						response_counter_d=0;
						counter_d=0;
		                stuck_counter_d=0;
		                stuck_d=0;
                      end
            SEND_COMMAND: if(ready || done_tick) begin 
                        wr_data_d={wr_data_q[47:0],8'hff}; //shift by 1 byte 
                        wr_data=wr_data_d[55:48];
                        wr=1;
                        counter_d=counter_q+1'b1;
                        if(counter_q==7) begin //6 bytes had been sent to SPI, another 1 byte for the 8 clk cycles needed by sdcard before responding
							wr=0;
                            rd=1; //response always starts at logic 0 so hold the clock until then
                            counter_d=0;
                            state_d=RECEIVE_RESPONSE;
                        end
                      end 
            RECEIVE_RESPONSE: if(done_tick) begin
                         response_counter_d=response_counter_q+1; //counter for some responses that has multiple bytes (R3 and R7)
                         //rules for types of response for every command
                         case(cmd_counter_q) 
                            0: begin
                                    cmd_counter_d=(rd_data==8'h01)? cmd_counter_q+1:cmd_counter_q; //CMD0: resets SDCARD for SPI mode
                                    state_d=END_CMD;
                                end
                            1: begin
                                    rd_data_d={rd_data_q[31:0],rd_data};
                                    rd=1;
                                    led_d=3'b001;
                                    if(response_counter_d==5) begin//5 bytes had been received
                                        cmd_counter_d=(rd_data_d==40'h01_00_00_01_AA)? cmd_counter_q+1:cmd_counter_q; //CMD8: Host Voltage Supply is correct and SDCARD is ver2.00 or later
                                        rd=0;
                                        state_d=END_CMD;
                                    end
                                end
                            2: begin
                                    cmd_counter_d=(rd_data==8'h01)? cmd_counter_q+1:cmd_counter_q; //CMD59: turns off CRC checking
                                    state_d=END_CMD;
                                end
                            3: begin
                                    cmd_counter_d=(rd_data==8'h01)? cmd_counter_q+1:cmd_counter_q; //CMD55: Now ready for next command which is an Application Commmand(ACMD)
                                    state_d=END_CMD;
                                end
                            4: begin
                                    stuck_counter_d=stuck_counter_q+1;
                                    if(stuck_counter_q==1000) stuck_d=1; //if ACMD41 stuck 1000x , go back to power-on
                                    cmd_counter_d=(rd_data==8'h00)? cmd_counter_q+1:cmd_counter_q-1; //ACMD41: Initialization of SDCARD is complete 
                                    state_d=END_CMD;
                                end
                            5: begin
                                    rd_data_d={rd_data_q[31:0],rd_data};
                                    rd=1;
                                    if(response_counter_d==5) begin//5 bytes had been received
                                            rd=0;
                                            cmd_counter_d=(rd_data_d==40'h00_C0_FF_80_00)? cmd_counter_q+1:cmd_counter_q; //CMD58: CCS bit is 1 and SDCARD is classified as SDHC(High Capacity) or SDXC(eXtended Capacity)
                                            state_d=END_CMD;
                                    end
                                end  
                            6: begin
                                    cmd_counter_d=(rd_data==8'h00)? cmd_counter_q+1:cmd_counter_q; //CMD55: Now ready for next command which is an Application Commmand(ACMD)
                                    state_d=END_CMD;
                                end
                            7: begin //acknowledge for write 
                                    state_d=WRITE_X;

                                end
                            8: begin
                                    rd_data_d={rd_data_q[31:0],rd_data};
                                    rd=1;
                                    if(response_counter_d==2) begin//5 bytes had been received
                                        cmd_counter_d=(rd_data_d==16'h0000)? cmd_counter_q+1:cmd_counter_q; //CMD8: Host Voltage Supply is correct and SDCARD is ver2.00 or later
                                        rd=1;
                                        state_d=IDLE;
                                        led_d=3'b010;
                                    end
                                end
                          endcase			
                         end
            	
            END_CMD:if(ready) begin //must provide 8 clks before shutting down sclk or starting new command
                        wr=1;
                        state_d=stuck_q? POWER_ON:COMMANDS;
                        if(cmd_counter_q==INIT_LAST_INDEX+1 || clk_div_q) state_d=DELAY;
                    end
            DELAY: if(ready) begin  //delay before switching to high frequency for writing in SDCARD 
                        stuck_counter_d=stuck_counter_d+1;
                        clk_div_d=1;
                        if(stuck_counter_d==1000) begin 
                            led_d=3'b010;
                            state_d=IDLE;
                        end
                    end
   ///////////////////////////////////////INITIALIZATION COMPLETE///////////////////////////////////////////////
   
   /////////////////////////////SDCARD READ/WRITE OPERATION//////////////////////////////////////////////////
            IDLE:
                if (wrena) begin
//                    current_address = waddr;
                    cmd_counter_d = 7; // write
                    state_d = COMMANDS;
                    addr_counter_d = addr_counter_q+1;
                end


`ifdef working
            WRITE_X:
                if (wrena) begin
                    if(counter_q==0 || counter_q==513 || counter_q==514) 
                        wr_data = 8'hFE; //start_token
                    else 
                        wr_data = wdata;

                    wr=1;
                    counter_d = counter_q+1'b1;
                    if(counter_q == 515) begin //515 bytes had been sent to SPI,
                        wr = 0;
                        rd = 1; //Data response immediately
                        counter_d = 0;
                        state_d = WRITE_2;
                    end
                end
`else
            WRITE_DATA: begin
//                            if (counter_q==0 || counter_q==513 || counter_q==514 || counter_q==515) state_d = WRITE_X;
                            if (counter_q==0 || counter_q >=513) state_d = WRITE_X;
                            if (wrena) begin
//                                wr_data = wdata;
                                wr_data = 8'h32;
                                wr=1;
                                counter_d = counter_q+1'b1;
                            end
                        end


            WRITE_X: 
                    if (ready || done_tick) begin
                        if (wrena) begin
                            wr_data = wdata;
                            wr=1;
                            counter_d = counter_q+1'b1;
                        end else begin

                            if (counter_q==0 || counter_q==513 || counter_q==514) begin
                                wr_data = 8'hFE; //start_token
                                wr=1;
                                counter_d = counter_q+1'b1;
    //                        end else begin
    //                            state_d = WRITE_DATA;
    //                            wr = 0;
                            end
                        end

//                        wr=1;
//                        counter_d = counter_q+1'b1;
                        if(counter_q == 515) begin //515 bytes had been sent to SPI,
                            wr = 0;
                            rd = 1; //Data response immediately
                            counter_d = 0;
                            state_d = WRITE_2;
                        end
                    end
`endif

            WRITE_1:        // command 24 : single block(sector) write
                if(ready || done_tick) begin 
                    if(counter_q==0 || counter_q==513 || counter_q==514) 
                        wr_data = 8'hFE; //start_token
                    else
                        wr_data = 8'h69; //8'h60 + addr_counter_q+2; //data transmitted are 512 a's, then 512 b's ......
                    wr=1;
                    counter_d = counter_q+1'b1;

                    if(counter_q == 515) begin //515 bytes had been sent to SPI,
                        wr = 0;
                        rd = 1; //Data response immediately
                        counter_d = 0;
                        state_d = WRITE_2;
                    end
                  end 

            WRITE_2: if(done_tick) state_d = BUSY;

            BUSY: begin
                    rd=1;
                    if(SD_MISO && done_tick) begin
                        cmd_counter_d=8;
                        state_d=COMMANDS;
                    end
                  end

            default: state_d=POWER_ON;
        endcase
    end
    

    always @(posedge wrena) begin
        if ( state_q == IDLE ) current_address = waddr;
    end


    assign wready = (( state_q == WRITE_X ) || (state_q == WRITE_DATA)) && (ready || done_tick);


    assign SD_nCS=(state_q==POWER_ON || state_q==COMMANDS || (state_q==END_CMD && ready) || state_q==IDLE)? 1'b1:cs_n_1; //at power_on, toggle clk 74 times WHILE cs_n is high

	assign led0_r=led_q[2]? clk:1'b1, 
	       led0_g=led_q[1]? clk:1'b1,
	       led0_b=led_q[0]? clk:1'b1; //PWM used is the clk itself 
    assign idle=state_q==IDLE;
	 
    //module instantiations
    //based on [27MHz]: High freq: /2= 13.5 MHz , Low freq: /30 = 900KHz
    spi #(.HI_FREQ_DIV(2), .LO_FREQ_DIV(30), .SPI_MODE(0)) m0 
    ( 
        .clk(clk),
        .rst(rst),                                
        //SPI control
        .clk_div(clk_div_q),
        .rd(rd),
        .wr(wr),
        .hold(hold),  //pins to start read or write operation, hold is for holding the clock for multibyte read/write 
        .wr_data(wr_data), //data to be sent to the slave
        .rd_data(rd_data), //data received from the slave
        .done_tick(done_tick), //ticks if either write or read operation is finished
        .ready(ready), //can perform read/write operation only if ready is "1" (except for multibyte read/write where state will never go to IDLE)
        //SPI pinouts
        .miso(SD_MISO),
        .mosi(SD_MOSI),
        .sclk(SD_DCLK),
        .cs_n(cs_n_1)
	);
	
    delay dly ( .clk(clk), .rstn(!rst), .out(out) );
endmodule


module delay (
        input clk, rstn,
        output out
    );

    reg [15:0] counter;
    assign out = &counter;

    always @(posedge clk, negedge rstn) begin
        if (~rstn) 
            counter = 0;
        else begin
            if (! (&counter) ) counter = counter + 1;
        end
    end
endmodule
/*
            WRITE_X:
//                if ((ready || done_tick) && wrena) begin
                if (wrena) begin
                    if(counter_q==0 || counter_q==513 || counter_q==514) 
                        wr_data = 8'hFE; //start_token
                    else 
//                        wr_data = 8'h5A;
                        wr_data = wdata;

                    wr=1;
                    counter_d = counter_q+1'b1;
                    if(counter_q == 515) begin //515 bytes had been sent to SPI,
                        wr = 0;
                        rd = 1; //Data response immediately
                        counter_d = 0;
                        state_d = WRITE_2;
                    end
                end
*/

/*
            IDLE: if (key_tick) begin
                       cmd_counter_d=7; //WRITE
                       state_d=COMMANDS;
                       addr_counter_d=addr_counter_q+1;
                       led_d=100; 
                    end

            WRITE_1: if(ready || done_tick) begin 
                            if(counter_q==0 || counter_q==513 || counter_q==514) 
                                wr_data=8'b1111_1110; //start_token
                            else
                                wr_data=8'h60 + addr_counter_q+2; //data transmitted are 512 a's, then 512 b's ......
                            wr=1;
                            counter_d=counter_q+1'b1;
                            if(counter_q==515) begin //515 bytes had been sent to SPI,
                                wr=0;
                                rd=1; //Data response immediately
                                counter_d=0;
                                state_d=WRITE_2;
                            end
                          end 
            WRITE_2: if(done_tick) state_d = BUSY;

            BUSY: begin
                    rd=1;
                    if(SD_MISO && done_tick) begin
                        cmd_counter_d=8;
                        state_d=COMMANDS;
                    end
                  end
*/

/*
//	 uart #(.DBIT(8),.SB_TICK(16),.DVSR(78),.DVSR_WIDTH(9),.FIFO_W(10)) m1 //9600 Baud UART FOR DEBUGGING
	 uart #(.DBIT(8),.SB_TICK(16),.DVSR(176), .FIFO_W(10)) m1 //9600 Baud UART FOR DEBUGGING, based on 27MHz
	(
		.clk(clk),
		.rst_n({!rst}),
		.rd_uart(rd_uart),
		.wr_uart(wr_uart),
		.wr_data(wr_data_uart),
		.rx(uart_rx),
		.tx(uart_tx),
		.rd_data(rd_data_uart),
		.rx_empty(rx_empty),
		.tx_full()
    );
    
    //UART for debugging SDCARD responses
    always @* begin
    wr_uart=0;
    rd_uart=0;
	wr_data_uart=0;
 		wr_uart= wr || (state_q==RECEIVE_RESPONSE && done_tick) || (state_q==WRITE_2 && done_tick); //write all commands passing thrugh MOSI and MISO
		wr_data_uart=wr? wr_data:rd_data;
    end
*/

//	debounce_explicit
//	(
//		.clk(clk),
//		.rst_n({!rst}),
//		.sw(btn),
//		.db_level(),
//		.db_tick(key_tick)
//    );

`timescale 1ns / 1ps

module baud_generator 
    #(
        parameter N = 163
    ) (
        input clk,
        input rst_n,
        output reg s_tick
    );

    reg[$clog2(N)-1:0] counter=0;

    always @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            counter <= 0;
        else begin
            s_tick = ( counter == N-1 ) ? 1 : 0;        // s_tick high when reached max count
            counter = ( counter == N-1 ) ? 0 : counter + 1; // roll count over.
		end
	end
endmodule

/*
module baud_generator_original
    #(
        parameter N = 163
    ) (
        input clk,
        input rst_n,
        output reg s_tick
    );

    reg[$clog2(N)-1:0] counter=0;

    always @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            counter <= 0;
        else begin
            s_tick=0;
			if( counter == N-1 ) begin
				s_tick  = 1;
				counter <= 0;
			end else begin
				counter <= counter + 1;
			end
			
		end
	end
endmodule
*/
`timescale 1ns / 1ps

/*
 * Slave simulation model for testing purpose.
 * Support clock stretching.
 * Slave should stretch the clock before the low->high SCL transition
 * Clock can only be stretched if it is low, So on a low level of SCL, slave is free to drive 0 on SCL
*/

module i2c_slave #(
	parameter DATA_WIDTH = 8,
	parameter CLK_CYCLES = 100_000_000,
	parameter SCL_CYCLES = 100_000,
	parameter SAMPLE_DATA = 8'hBE
)(
	input clk_target,
	input rstn_target,
	inout scl_target,
	inout sda_target,
	output slv_err,
	
	// The following ports are to help with the testbench
	output [DATA_WIDTH-1:0] target_addr_recv_o,
	output [DATA_WIDTH-1:0] reg_addr_recv_o,
	output [DATA_WIDTH-1:0] reg_data_recv_o,
	output stop_detect_o	
);

localparam integer clk_counter = CLK_CYCLES/(2*SCL_CYCLES) - 1'd1;
localparam [DATA_WIDTH-1:0] data_to_send = SAMPLE_DATA;

localparam [3:0] IDLE = 4'd0,
					CHECK_START = 4'd1,
					TARGET_ADDR = 4'd2,
					STRETCH_CLK = 4'd3,
					REG_ADDR = 4'd4,
					RD_DATA = 4'd5,
					ACK_CTRL = 4'd6,
					CHECK_STOP = 4'd7,
					WAIT_NEXT = 4'd8,
					CHECK_REPT_START = 4'd9,
					GIVE_ACK = 4'd10;

localparam integer five_us = 1000/2 - 1;

// min time after rising edge of SCL that SDA should go high.
localparam integer tsu_sto_cntr = five_us; 

wire scl_in_slv, sda_in_slv;

reg [DATA_WIDTH-1:0] target_addr_recv;
reg [DATA_WIDTH-1:0] reg_addr_recv;
reg [DATA_WIDTH-1:0] reg_data_recv;
reg [DATA_WIDTH-1:0] data_send;

reg read;
reg [3:0] state_reg;
reg [$clog2(clk_counter*2):0] scl_timer_counter;
reg [$clog2(DATA_WIDTH):0] num_bits;
reg [$clog2(clk_counter*2):0] scl_stretch_counter;
reg [$clog2(clk_counter*2):0] err_counter;

reg slv_err_reg;
reg sda_low, clk_stretch;

// To simualte some internal register read at target.
reg [DATA_WIDTH-1:0] written_data;

reg give_ack;
reg got_ack;

reg scl_prev;
reg sda_prev;

reg rpt_start;
reg [3:0] state_prev;

assign scl_target = clk_stretch ? 1'b0 : 1'bz;
assign sda_target = sda_low ? 1'b0 : 1'bz;
assign slv_err = slv_err_reg;
assign scl_in_slv = scl_target;
assign sda_in_slv = sda_target;

wire scl_rise = (scl_prev == 0) && (scl_in_slv == 1);
wire scl_fall = (scl_prev == 1) && (scl_in_slv == 0);

wire sda_rise = (sda_prev == 0) && (sda_in_slv == 1);
wire sda_fall = (sda_prev == 1) && (sda_in_slv == 0);

assign start_detect = scl_in_slv && sda_fall;
assign stop_detect = scl_in_slv && sda_rise;

assign target_addr_recv_o = target_addr_recv;
assign reg_addr_recv_o = reg_addr_recv;
assign reg_data_recv_o = reg_data_recv;
assign stop_detect_o = stop_detect;

always @(posedge clk_target) begin
	if (!rstn_target) begin
		state_reg <= IDLE;
		scl_timer_counter <= 'd0;
		num_bits <= 'd0;
		slv_err_reg <= 'd0;
		sda_low <= 'd0;
		clk_stretch <= 'd0;
		scl_stretch_counter <= 'd0;
		read <= 'd0;
		target_addr_recv <= 'd0;
		reg_addr_recv <= 'd0;
		reg_data_recv <= 'd0;
		written_data <= 'd0;
		give_ack <= 'd0;
		rpt_start <= 'd0;
		state_prev <= 'd0;
		data_send <= 'd0;
		err_counter <= 'd0;
		got_ack <= 'd0;
	end else begin
		scl_prev <= scl_in_slv;
		sda_prev <= sda_in_slv;

		if (scl_timer_counter)
			scl_timer_counter <= scl_timer_counter - 1'd1;

		if (scl_stretch_counter)
			scl_stretch_counter <= scl_stretch_counter - 1'd1;

		if (err_counter)
			scl_stretch_counter <= scl_stretch_counter - 1'd1;

		case (state_reg)
			IDLE:
				begin
					if (!sda_in_slv && scl_in_slv) begin
						state_reg <= CHECK_START;
					end else begin
						state_reg <= IDLE;
						scl_timer_counter <= 'd0;
						sda_low <= 'd0;
						clk_stretch <= 'd0;
						err_counter <= 'd0;
					end
				end

			CHECK_START:
				begin
					// Wait till the falling edge of SCL
					if (!scl_in_slv) begin
						state_reg <= TARGET_ADDR;
						num_bits <= DATA_WIDTH + 1'd1;
					end
				end

			TARGET_ADDR:
				begin
					if (scl_rise) begin
						num_bits <= num_bits - 1'd1;
						target_addr_recv <= {target_addr_recv[6:0], sda_in_slv};
						state_prev <= TARGET_ADDR;
						state_reg <= (num_bits == 2) ? GIVE_ACK : TARGET_ADDR;

						// Here we will not get the correct read/write bit. We can use the stretch_clk state
						// to get the read/write bit.
						// read <= (num_bits == 2) ? target_addr_recv[0] : 'd0;
					end else begin
						state_reg <= TARGET_ADDR;
					end
				end

			STRETCH_CLK:
				begin
					if (!scl_stretch_counter) begin
						clk_stretch <= 'd0;

						if (read) begin
							state_reg <= RD_DATA;
							sda_low <= !data_to_send[7];
							data_send <= {data_to_send[6:0], 1'b0};
							num_bits <= DATA_WIDTH;
						end else begin
							state_reg <= REG_ADDR;
							data_send <= 'd0;
							num_bits <= DATA_WIDTH + 1'd1;
						end						
					end
				end

			REG_ADDR:
				begin
					if (scl_rise) begin
						num_bits <= num_bits - 1'd1;	
						reg_addr_recv <= {reg_addr_recv[6:0], sda_in_slv};
						state_reg <= (num_bits == 2) ? GIVE_ACK : REG_ADDR;
						state_prev <= REG_ADDR;
					end else begin
						state_reg <= REG_ADDR;
					end
				end

			// Here we monitor the bus, to see if there is a repeated start or data byte phase
			WAIT_NEXT:
				begin
					if (!scl_timer_counter) begin
						state_prev <= WAIT_NEXT;
						if (start_detect) begin
							state_reg <= CHECK_REPT_START;
							num_bits <= DATA_WIDTH + 1'd1;
						end else if (stop_detect) begin
							state_reg <= IDLE;
						end else if (scl_rise) begin
						
							// Get the wdata byte sent from the controller
							num_bits <= num_bits - 1'd1;
							reg_data_recv <= {reg_data_recv[6:0], sda_in_slv};
							state_reg <= (num_bits == 2) ? GIVE_ACK : WAIT_NEXT;
						end else begin
							state_reg <= WAIT_NEXT;
						end
					end
				end
			
			GIVE_ACK:
				begin
					// Give ACKT to the master
					if (scl_fall) begin
						// We will end up here after the 8th falling edge and 9th falling edge
						num_bits <= num_bits - 1'd1;
						written_data <= reg_data_recv;				
						
						if (num_bits) begin
							/* 8th falling edge */

							state_reg <= GIVE_ACK;
							sda_low <= 1'd1; // Give ACK to controller
							give_ack <= 1'd1;
						end else begin
							/* 9th falling edge */

							if (state_prev == WAIT_NEXT) begin
								state_reg <= CHECK_STOP;
								err_counter <= 3*five_us;
							end else if (state_prev == TARGET_ADDR) begin
								read <= target_addr_recv[0];
								// Clock stretching logic
								scl_stretch_counter <= (2*CLK_CYCLES)/SCL_CYCLES;
								clk_stretch <= 1'd1;
								state_reg <= STRETCH_CLK;
								num_bits <= DATA_WIDTH + 1'd1;
							end else if (state_prev == REG_ADDR) begin
								num_bits <= DATA_WIDTH + 1'd1;
								state_reg <= WAIT_NEXT;
							end
							
							sda_low <= 1'd0; // Give ACK to controller
							give_ack <= 1'd0;
						end
					end
				end

			CHECK_REPT_START:
				begin
					// Then move to target_addr state
					rpt_start <= 1'd1;
					state_reg <= TARGET_ADDR;
				end

			RD_DATA:
				begin
					// send 'data_to_send' that is visible on posedge of SCL to the controller
					if (scl_fall) begin
						sda_low <= (num_bits == 1) ? 'd0 : !data_send[7];
						data_send <= {data_send[6:0], 1'b0};
						num_bits <= num_bits - 1'd1;
						state_reg <= (num_bits == 1) ? ACK_CTRL : RD_DATA;
					end
				end

			ACK_CTRL:
				begin
					if (scl_rise) begin
						// Check if master has raised the SDA
						if (sda_in_slv) begin
							state_reg <= CHECK_STOP;
							err_counter <= 3*five_us;
							got_ack <= 1'b1;							
						end else begin
							slv_err_reg <= 1'd1;
						end
					end
				end

			CHECK_STOP:
				begin
					if (stop_detect) begin
						state_reg <= IDLE;
					end else begin
						slv_err_reg <= !err_counter ? 1'd1 : 1'd0;
					end
				end
		endcase
	end
end

endmodule

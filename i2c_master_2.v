`timescale 1ns / 1ps

/* 
 * We should schedule SDA relative to the falling edge. We cannot do so relative to the rising edge
 * reason being that we cannot know when the SCL will go high (as the slave can stretch the clock)
 * 
 * generate 100 KHz clock
 * Min timing constraints: {tSU;STA = 4.7 us | tHD;STA = 4 us |  tSU;STO = 4 us | tBUF = 4.7 us}
 * 
*/


/*
 * The slave can pull the scl low if it wants to slow down the master (clock stretching)
 * The master must wait for it to go high again before continuing

 * scl_wait_high_reg/next is for checking if the slave has released the SCL line when master wants to 
   do a low to high transition. It is a way for "waiting" for the slave to release the scl line.
	
 * When reading data from the slave, the master will NACK (pull SDA high) after receiving 1 byte.
 * Reading multiple bytes successively from the slave is not supported yet.
*/

module i2c_master_2 #(
	parameter DATA_WIDTH = 8,
	parameter CLOCK_CYCLES = 100_000_000,
	parameter SCL_CYCLES = 100_000
)(
	input clk_i,
	input rstn_i,
	input start_i,
	input [DATA_WIDTH-1:0] target_addr_i,
	input [DATA_WIDTH-1:0] addr_i,
	input [DATA_WIDTH-1:0] data_i,
	input write_i,
	inout scl,
	inout sda,
	output [DATA_WIDTH-1:0] rdata_o,
	output busy_o,
	output err_o
);

localparam integer ten_us = 1000 - 1;
localparam integer five_us = 1000/2 - 1;

localparam integer scl_counter = CLOCK_CYCLES/(2*SCL_CYCLES) - 1'd1;
localparam integer tbuf_cntr = ten_us;
localparam integer tsu_sta_cntr = five_us; // Only taken into account for "repeated start" condtions
localparam integer thd_sta_cntr = five_us; // Time SDA must be kept low for "before" SCL goes low
localparam integer tsu_sto_cntr = five_us; // min time after rising edge of SCL that SDA should go high.

localparam [3:0] IDLE = 4'd0,
					START_SETUP = 4'd1,
					START_HD = 4'd2,
					TARGET_ADDR = 4'd3,
					REG_ADDR = 4'd4,
					WR_DATA = 4'd5,
					RD_DATA_CLKGEN = 4'd6,
					ACK_TARGET = 4'd7, // ack coming from the target
					ACK_CTRL = 4'd8, // ack coming from the controller
					PREPARE_STOP = 4'd9,
					STOP = 4'd10,
					FINISH_STOP = 4'd11,
					REPEATED_START_PREP = 4'd12,
					REPEATED_START_END = 4'd13;

reg [$clog2(scl_counter):0] scl_timer_counter_reg, scl_timer_counter_next;
reg [$clog2(scl_counter):0] state_machine_cntr_reg, state_machine_cntr_next;
reg scl_in_d;

wire scl_in;
wire sda_in;

wire scl_rise =  scl_in & ~scl_in_d;
wire scl_fall = ~scl_in &  scl_in_d;

reg scl_low_reg, scl_low_next;
reg sda_low_reg, sda_low_next;
reg [3:0] num_bits_remaining_reg, num_bits_remaining_next;
reg [3:0] state_reg, state_next, state_prev;

reg ack_t_received;

reg rpt_start_reg, rpt_start_next;
reg clk_on_reg, clk_on_next;
reg busy_reg, busy_next;
reg err_reg, err_next;
reg [DATA_WIDTH-1:0] target_addr_reg, target_addr_next;
reg [DATA_WIDTH-1:0] waddr_reg, waddr_next;
reg [DATA_WIDTH-1:0] wdata_reg, wdata_next;
reg [DATA_WIDTH-1:0] rdata_reg, rdata_next;

reg scl_wait_high_reg, scl_wait_high_next;
					
/* Master can only drive the pin low, else the pin should be floating
   The effect of keeping that pin floating is, when 'scl_low == 0' then we can have two cases.
	1) Either the pull up register can pull it high OR,
	2) The slave can keep it low ("clock stretching")
*/
assign scl = scl_low_reg ? 1'b0 : 1'bz;
assign sda = sda_low_reg ? 1'b0 : 1'bz;

// Readback signals for scl and sda
assign scl_in = scl;
assign sda_in = sda;

assign busy_o = busy_reg;
assign err_o = err_reg;
assign rdata_o = rdata_reg;

always @(*) begin
	state_next = state_reg;
	busy_next = busy_reg;
	num_bits_remaining_next = num_bits_remaining_reg;
	err_next = err_reg;
	waddr_next = waddr_reg;
	wdata_next = wdata_reg;
	rdata_next = rdata_reg;
	target_addr_next = target_addr_reg;
	scl_timer_counter_next = scl_timer_counter_reg;
	state_machine_cntr_next = state_machine_cntr_reg;
	clk_on_next = clk_on_reg;
	sda_low_next = sda_low_reg;
	scl_low_next = scl_low_reg;
	scl_wait_high_next = scl_wait_high_reg;
	rpt_start_next = rpt_start_reg;
	ack_t_received = 'd0;

	// State machine counter for keeping count of things like tSU;STA,tHD;STA etc.
	if (state_machine_cntr_reg) begin
		state_machine_cntr_next = state_machine_cntr_reg - 1'd1;
	end

// ---------------- **** Clocking logic **** --------------

	if (clk_on_reg) begin
		// Counter running
		if (scl_timer_counter_reg != 0) begin
			scl_timer_counter_next = scl_timer_counter_reg - 1'd1;
		end

		// Counter expired, state transition
		else begin
		  // if scl is low, release it to go high
		  // Set wait_high_next to wait for slave to release SCL (clock stretching case)
        if (scl_low_reg) begin
            scl_low_next       = 1'b0;
            scl_wait_high_next = 1'b1;
        end

        // if scl_in is high, Then slave has released the scl line. Load the counter
        else if (scl_wait_high_reg) begin
				// Keep spinning until the slave releases the SCL.
            if (scl_in) begin
               scl_wait_high_next = 1'b0;
               scl_timer_counter_next = scl_counter - 1'd1; // HIGH phase
            end
        end

        // High to low transition
        else begin
            scl_low_next = 1'b1;
            scl_timer_counter_next = scl_counter;
        end
		end
	end else begin
		scl_low_next = 1'b0;
		scl_timer_counter_next = 'd0;
		scl_wait_high_next = 1'b0;
	end

// ---------------- **** I2C control logic **** --------------

	case (state_reg)
		IDLE:
			begin
				if (start_i) begin
					busy_next = 1'd1;
					state_next = START_SETUP;
					state_machine_cntr_next = tbuf_cntr;
					num_bits_remaining_next = DATA_WIDTH + 1'd1;
				end else begin
					scl_low_next = 1'b0;
					sda_low_next = 1'b0;
					rpt_start_next = 1'b0;
				end
			end

		START_SETUP:
			begin
				if (!state_machine_cntr_reg) begin
					// tBUF is satisfied
					sda_low_next = 1'd1;
					target_addr_next = {target_addr_i[6:0],1'b0};
					state_machine_cntr_next = thd_sta_cntr;
					state_next = START_HD;
				end
			end

		START_HD:
			begin
				if (!state_machine_cntr_reg) begin
					// tSU_HD is satisfied, we can pull SCL low
					state_next = TARGET_ADDR;
					clk_on_next = 1'd1;
				end
			end

		TARGET_ADDR:
			begin
				// tHD_STA is satisfied
				if (scl_fall) begin
					num_bits_remaining_next = num_bits_remaining_reg - 1'b1;
					sda_low_next = num_bits_remaining_next ? ~target_addr_reg[7] : 1'b0;
					target_addr_next = {target_addr_reg[6:0], 1'b0};
					state_next = num_bits_remaining_next ? TARGET_ADDR : ACK_TARGET;
					state_machine_cntr_next = 'd0;
				end else begin
					state_next = TARGET_ADDR;
				end
			end

		REG_ADDR:
			begin
				if (scl_fall) begin
					num_bits_remaining_next = num_bits_remaining_reg - 1'b1;
					sda_low_next = num_bits_remaining_next? ~waddr_reg[7] : 1'b0;
					waddr_next = {waddr_reg[6:0], 1'b0};
					state_next = num_bits_remaining_next ? REG_ADDR : ACK_TARGET;

					/* We have sent the whole byte and now we should sample an ACK from the target */
					state_machine_cntr_next = 'd0;
				end
			end

		WR_DATA:
			begin
				if (!state_machine_cntr_reg) begin
					if (scl_fall) begin
						num_bits_remaining_next = num_bits_remaining_reg - 1'b1;
						sda_low_next = num_bits_remaining_next? ~wdata_reg[7] : 1'b0;
						wdata_next = {wdata_reg[6:0], 1'b0};
						state_next = num_bits_remaining_next ? WR_DATA : ACK_TARGET;
						state_machine_cntr_next = 'd0;
					end
				end
			end

		RD_DATA_CLKGEN:
			begin
				if (scl_rise) begin
					rdata_next = {rdata_reg[6:0] , sda_in};
					num_bits_remaining_next = num_bits_remaining_reg - 1'b1;
					if (num_bits_remaining_next == 1) begin
						state_next = ACK_CTRL;
					end
				end
			end

		ACK_CTRL:
			begin
				// Pull the SDA line low to send an ack to the target on the 9th clock pulse
				if (scl_fall) begin
					sda_low_next = 1'd0;
					state_next = PREPARE_STOP;
					state_machine_cntr_next = 'd0;
				end
			end

		ACK_TARGET:
			begin
				// See if we got an ACK back, else error out
				
				if (scl_rise) begin
					if (!sda_in) begin
						if (state_prev == TARGET_ADDR) begin
							if (rpt_start_reg) begin
								state_next = RD_DATA_CLKGEN;
								waddr_next = 'd0;
							end else begin
								state_next = REG_ADDR;
								waddr_next = addr_i;
							end

							rpt_start_next = 1'b0;
							ack_t_received = 'd1;
							num_bits_remaining_next = DATA_WIDTH + 1'd1;
							state_machine_cntr_next = 'd0;
						end else if (state_prev == REG_ADDR) begin
							// If we are reading from the target, then we should issue a repeated start
							state_next = write_i ? WR_DATA : REPEATED_START_PREP;
							num_bits_remaining_next = DATA_WIDTH + 1'd1;
							ack_t_received = 'd1;
							wdata_next = data_i;
							state_machine_cntr_next = 'd0;
						end else begin
							state_machine_cntr_next = 'd0;
							ack_t_received = 'd1;
							state_next = PREPARE_STOP;
							num_bits_remaining_next = 4'd0;
						end
					end else begin
						err_next = 1'd1;
					end
				end
			end

		REPEATED_START_PREP:
			begin
				/*
				 * tSU;STA is important for repeated start case.
				 * It is the minimum time the SDA line is required to remain high before initiating a repeated start
				 */
				if (scl_rise) begin
					clk_on_next = 1'd0;
					state_machine_cntr_next = tsu_sta_cntr;
					state_next = REPEATED_START_END;
				end
			end

		REPEATED_START_END:
			begin
				if (!state_machine_cntr_reg) begin
					rpt_start_next = 1'b1;
					sda_low_next = 1'b1;
					target_addr_next = {target_addr_i[6:0],1'b1};
					state_machine_cntr_next = thd_sta_cntr;
					state_next = START_HD;
				end
			end

		PREPARE_STOP:
			begin
				if (scl_fall) begin
						 // Issue a NACK from master, now at the 9th clock pulse, the slave will see that master
						 // does not want to read any more bytes.
						sda_low_next = 1'd1;
						state_next = STOP;
						state_machine_cntr_next = 'd0;
				end
			end

		STOP:
			begin
				// Make sure that SCL is high already at this point
				// That means the target should not have stretched the clock,
				// If it has, we should spin around in the STOP state until the clock is released.
				if (scl_in) begin
					state_machine_cntr_next = tsu_sto_cntr - 1'd1;
					clk_on_next = 'd0;
					state_next = FINISH_STOP;
				end else begin
					state_machine_cntr_next = 'd0;
					state_next = STOP;
				end
			end

		FINISH_STOP:
			begin
				if (!state_machine_cntr_reg) begin
					sda_low_next = 1'd0;
					busy_next = 1'b0;
					state_next = IDLE;
				end
			end
	endcase
end

always @(posedge clk_i) begin
	scl_in_d <= scl_in;

	if (!rstn_i) begin
		// SCL and SDA should be set in a high Z state by default
		scl_low_reg <= 1'd0;
		sda_low_reg <= 1'd0;
		state_reg <= IDLE;
		state_prev <= 1'd0;
		scl_timer_counter_reg <= 'd0;
		state_machine_cntr_reg <= 'd0;
		num_bits_remaining_reg <= 'd0;
		target_addr_reg <= 'd0;
		waddr_reg <= 'd0;
		wdata_reg <= 'd0;
		rdata_reg <= 'd0;
		err_reg <= 'd0;
		busy_reg <= 'd0;
		clk_on_reg <= 'd0;
		scl_wait_high_reg <= 'd0;

		rpt_start_reg <= 'd0;
		
		ack_t_received <= 'd0;
	end else begin
		scl_low_reg <= scl_low_next;
		sda_low_reg <= sda_low_next;
		state_reg <= state_next;
		
		/*
		 * state_prev should not update on all clock cycles
		 * Rather it should only update when state_reg is changing to a new state.
		 * At the point when state_reg is being updated, state_prev should latch the old value.
		*/
		if (state_next != state_reg)
			state_prev <= state_reg;

		scl_timer_counter_reg <= scl_timer_counter_next;
		state_machine_cntr_reg <= state_machine_cntr_next;
		num_bits_remaining_reg <= num_bits_remaining_next;
		target_addr_reg <= target_addr_next;
		waddr_reg <= waddr_next;
		wdata_reg <= wdata_next;
		rdata_reg <= rdata_next;
		err_reg <= err_next;
		busy_reg <= busy_next;
		clk_on_reg <= clk_on_next;
		scl_wait_high_reg <= scl_wait_high_next;
		rpt_start_reg <= rpt_start_next;
	end
end

endmodule

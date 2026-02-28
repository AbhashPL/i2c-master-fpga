`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:04:14 12/05/2025 
// Design Name: 
// Module Name:    uart_tx 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module uart_tx #(
	CLK_CYCLES = 100_000_000,
	BAUD_RATE  = 19200
)
(
	input clk_i,
	input [7:0] tx_byte_i,
	input transmit_i,
	output tx_o,
	output tx_err_o,
	output is_transmitting_o
);

`include "math_utils.vh"

localparam one_baud_cycles = (CLK_CYCLES / BAUD_RATE) - 1;
localparam [2:0] 
	TX_IDLE  = 3'd0,
	TX_SENDING = 3'd1,
	TX_STOP	= 3'd2,
	TX_ERR  = 3'd3;

reg tx_reg;
reg tx_err_reg;
reg is_transmitting_reg;
reg [7:0] tx_send_byte;
reg [log2(one_baud_cycles):0] tx_timer = 1'd0;
reg [2:0] tx_state = TX_IDLE;
reg [3:0] tx_bits_remaining;

assign tx_o = tx_reg;
assign tx_err_o = tx_err_reg;
assign is_transmitting_o = is_transmitting_reg;

always @(posedge clk_i) begin
	if (tx_timer) begin
		tx_timer <= tx_timer - 1'd1;
	end
	
	case (tx_state)
		TX_IDLE:
				if (transmit_i) begin
					tx_timer <= one_baud_cycles;
					tx_reg <= 0; // Start bit
					tx_state <= TX_SENDING;
					tx_send_byte <= tx_byte_i;
					tx_bits_remaining <= 4'd7;
					tx_err_reg <= 1'b0;
					is_transmitting_reg <= 1'b1;
				end else begin
					tx_timer <= 0;
					tx_reg <= 1'b1;
					tx_state <= TX_IDLE;
					tx_send_byte <= 8'd0;
					tx_bits_remaining <= 4'd0;
					tx_err_reg <= 1'b0;
					is_transmitting_reg <= 1'b0;
				end
		TX_SENDING:
			if (!tx_timer) begin
				// One baud expired
				tx_reg <= tx_send_byte[0];
				tx_bits_remaining <= tx_bits_remaining - 1'd1;
				tx_send_byte <= {1'd0,tx_send_byte[7:1]};
				
				// One bit sent, reload the timer for next bit or a stop bit
				tx_timer <= one_baud_cycles;
				tx_state <= (tx_bits_remaining) ? TX_SENDING : TX_STOP;
			end else begin
				tx_state <= TX_SENDING;
			end
		TX_STOP:
			if (!tx_timer) begin
				if (transmit_i) begin
					tx_state <= TX_ERR;
				end else begin
					tx_timer <= one_baud_cycles;
					tx_reg <= 1'b1;
					tx_state <= TX_IDLE;
				end
			end else begin
				tx_state <= TX_STOP;
			end
		TX_ERR:
			tx_err_reg <= 1'b1;
	endcase	
end

endmodule

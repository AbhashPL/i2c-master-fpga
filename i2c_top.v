`timescale 1ns / 1ps

/*
 * Read the who_am_i register and send it over uart every 2 seconds. 


 * The slave address for mpl3115a2 is 0x60
 
 * For read 0x60 <=> 11000000 (0xc0)
 * For write 0x60 <=> 11000001 (0xc1)

*/

module i2c_top(
	input CLK_100MHz,
	inout EXP_IO_N2, // scl
	inout EXP_IO_P2, // sda
	output EXP_IO_P4, // busy
	output TX
);

localparam [7:0] hold_reset_ns = 28'd200;
localparam [7:0] rst_count = (hold_reset_ns/10) - 1;

localparam integer DATA_WIDTH = 4'd8;
localparam [7:0] mpl3115_slave_addr = 'h60;
localparam [7:0] who_am_i_reg = 'h0c;

localparam [2:0] RESET = 3'd0,
					RESET_FINI = 3'd1,
					START_TRANSACTION = 3'd2,
					WAIT_FOR_FINISH = 3'd3,
					SEND_ON_UART = 3'd4,
					UART_TRANSMIT_FINISH = 3'd5;

reg [DATA_WIDTH-1:0] target_addr_reg;
reg [DATA_WIDTH-1:0] addr_reg;
reg [DATA_WIDTH-1:0] data_reg;

reg rstn_reg;
reg write_reg;
reg start_reg;

wire busy_out;
wire err_out;
wire [DATA_WIDTH-1:0] rdata_out;

reg [28:0] local_timer;
reg [2:0] state_reg;

reg uart_transmit = 1'b0;
reg [7:0] uart_tx_data;
wire tx_err;
wire is_transmitting;

i2c_master_2 #(
	.DATA_WIDTH(8),
	.CLOCK_CYCLES(100_000_000),
	.SCL_CYCLES(100_000)
) i2c_master_inst (
	.clk_i(CLK_100MHz),
	.rstn_i(rstn_reg),
	.start_i(start_reg),
	.target_addr_i(target_addr_reg),
	.addr_i(addr_reg),
	.data_i(data_reg),
	.write_i(write_reg),
	.rdata_o(rdata_out),
	.scl(EXP_IO_N2),
	.sda(EXP_IO_P2),
	.busy_o(EXP_IO_P4),
	.err_o(err_out)
);

uart_tx my_uart_tx(
	.clk_i(CLK_100MHz),
	.tx_byte_i(uart_tx_data),
	.transmit_i(uart_transmit),
	.tx_o(TX),
	.tx_err_o(tx_err),
	.is_transmitting_o(is_transmitting)
);

initial begin
    state_reg = RESET;
    local_timer = 0;
    rstn_reg = 0;
    start_reg = 0;
    write_reg = 0;
	 uart_transmit = 1'b0;
end

always @(posedge CLK_100MHz) begin

	if (local_timer) begin
		local_timer <= local_timer - 1'd1;
	end
	
	case (state_reg)
		RESET:
			begin
				local_timer <= rst_count;
				rstn_reg <= 1'd0;
				state_reg <= RESET_FINI;
			end

		RESET_FINI:
			begin
				if (!local_timer) begin
					rstn_reg	<= 1'd1;
					local_timer <= 999;
					state_reg <= START_TRANSACTION;
				end
			end
		
		START_TRANSACTION:
			begin
				if (!local_timer) begin
					start_reg <= 1'b1;
					write_reg <= 1'b0;
					target_addr_reg <= mpl3115_slave_addr;
					addr_reg <= who_am_i_reg;
					data_reg <= 'd0;
					state_reg <= WAIT_FOR_FINISH;
				end
			end
		
		WAIT_FOR_FINISH:
			begin
				if (!EXP_IO_P4) begin
					start_reg <= 1'b0;
					state_reg <= SEND_ON_UART;
					local_timer <= 'd0;
				end
			end

		SEND_ON_UART:
			begin
				if (!local_timer) begin
					uart_transmit <= 1'b1;
					uart_tx_data <= rdata_out;
					local_timer <= 999;
					state_reg <= UART_TRANSMIT_FINISH;
				end
			end

		UART_TRANSMIT_FINISH:
			begin
				if (!local_timer) begin
					uart_transmit <= 1'b0;
					local_timer <= 200_000_000 - 1;
					state_reg <= START_TRANSACTION;
				end
			end
	endcase

end

endmodule

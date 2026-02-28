`timescale 1ns / 1ps
	
module i2c_master_tb();

/*
 "4D" address = 'b0100_1101
 After adding write bit at end it becomes "'h9A" <=> 'b1001_1010
 
 I case of read;
 "'h9B" <=> 'b1001_1011
*/

// localparam integer sample_target_addr = 'h4D;
// localparam integer sample_target_addr = 'h29;
localparam integer sample_target_addr = 'h60;
localparam integer DATA_WIDTH = 4'd8;
localparam integer sample_data_for_slave = 8'hC4;
localparam integer NUM_WRITES = 4'd10;
localparam integer NUM_READS = 4'd10;

reg clk_tb;
reg rstn_tb;
reg start_tb;
reg [DATA_WIDTH-1:0] target_addr_tb;
reg [DATA_WIDTH-1:0] addr_tb;
reg [DATA_WIDTH-1:0] data_tb;
reg write_tb;

wire [DATA_WIDTH-1:0] rdata_tb;
wire err_tb;
wire busy_tb;

wire scl_wire;
wire sda_wire;
wire slv_err_tb;

wire [DATA_WIDTH-1:0] target_addr_recv_tb;
wire [DATA_WIDTH-1:0] reg_addr_recv_tb;
wire [DATA_WIDTH-1:0] reg_data_recv_tb;
wire stop_detect_tb;

reg scl_out;
reg sda_out;

pullup(scl_wire);
pullup(sda_wire);

i2c_master_2 #(
	.DATA_WIDTH(8),
	.CLOCK_CYCLES(100_000_000),
	.SCL_CYCLES(100_000)
) i2c_master_inst (
	.clk_i(clk_tb),
	.rstn_i(rstn_tb),
	.start_i(start_tb),
	.target_addr_i(target_addr_tb),
	.addr_i(addr_tb),
	.data_i(data_tb),
	.write_i(write_tb),
	.rdata_o(rdata_tb),
	.scl(scl_wire),
	.sda(sda_wire),
	.busy_o(busy_tb),
	.err_o(err_tb)
);

i2c_slave #(
	.DATA_WIDTH(8),
	.CLK_CYCLES(100_000_000),
	.SCL_CYCLES(100_000),
	.SAMPLE_DATA(sample_data_for_slave)
) i2c_slave_inst (
	.clk_target(clk_tb),
	.rstn_target(rstn_tb),
	.scl_target(scl_wire),
	.sda_target(sda_wire),
	.slv_err(slv_err_tb),

// Ports to help with the testbench
	.target_addr_recv_o(target_addr_recv_tb),
	.reg_addr_recv_o(reg_addr_recv_tb),
	.reg_data_recv_o(reg_data_recv_tb),
	.stop_detect_o(stop_detect_tb)
);

always begin
	#5;
	clk_tb = ~clk_tb;
end

task automatic write_val(reg [DATA_WIDTH-1:0] addr, reg [DATA_WIDTH-1:0] data);
begin
	$display("\n ----------------- I2C WRITE --------------------- \n");
	@(posedge clk_tb);
	start_tb = 1'b1;
	write_tb = 1'b1;
	target_addr_tb = sample_target_addr;
	addr_tb = addr;
	data_tb = data;
	#20;
	wait(stop_detect_tb);
	$display("** STOP received by slave **");
	$display("1) target_addr received by slave : %h", target_addr_recv_tb);
	$display("2) reg_addr received by slave : %h", reg_addr_recv_tb);
	$display("3) reg_data received by slave : %h", reg_data_recv_tb);
	
	if ((reg_addr_recv_tb ==  addr) &&
	    (reg_data_recv_tb == data) &&
		 (target_addr_recv_tb == {sample_target_addr, 1'b0}))
		$display("\n!!! SUCCESS: written data = %h at addr = %h\n", data, addr);
	else
		$display("\n??? FAILURE");

	wait(busy_tb == 0);
	start_tb = 1'b0;
	$display("\n\n");
end
endtask

task automatic read_val(reg [DATA_WIDTH-1:0] addr);
begin
	$display("\n --------------------- I2C READ ------------------------ \n");
	@(posedge clk_tb);
	start_tb = 1'b1;
	write_tb = 1'b0;
	target_addr_tb = sample_target_addr;
	addr_tb = addr;
	#20;
	wait(stop_detect_tb);
	$display("** STOP received by slave **");
	$display("1) target_addr received by slave : %h", target_addr_recv_tb);
	$display("2) reg_addr received by slave : %h", reg_addr_recv_tb);
	$display("3) reg_data received by master : %h", rdata_tb);
	
	if ((reg_addr_recv_tb ==  addr) && 
		 (rdata_tb == sample_data_for_slave) && 
		 (target_addr_recv_tb == {sample_target_addr, 1'b1}))
		$display("\n!!! SUCCESS: read val at addr = %h : %h\n", addr, rdata_tb);
	else
		$display("\n??? FAILURE");

	wait(busy_tb == 0);
	start_tb = 1'b0;	
end
endtask

integer i;

initial begin
	$display("Start simulation");
   // Initialization

   clk_tb = 0;
     
   // Assert reset
   rstn_tb = 0;
   #100;
   rstn_tb = 1;
   #100;
	
	/*
	 Test Plan:
	 * 10 writes in a row
	 * 10 reads in a row
	 * 5 reads + 5 writes
	 * R + W + R + W + R + W
	 * W + R + W + R + W + R
	*/

	read_val('h0C);
	#100;

	$display("\n\n $$$$$ Test case 1: 10 Writes \n");
	for (i = 0; i < NUM_WRITES; i=i+1) begin
		write_val(('h7C + i), ('h9B + i));
	end

	#100;
	$display("\n\n $$$$$ Test case 2: 10 Reads \n");
	for (i = 0; i < NUM_READS; i=i+1) begin
		read_val(('hA3 + i));
	end
	
	#100;
	$display("\n\n $$$$$ Test case 2: R + W + R + W \n");
	for (i = 0; i < NUM_READS; i=i+1) begin
		read_val(('h5B + i));
		#50;
		write_val(('h23 + i), ('h21 + i));
		#50;
	end
	
	$display("\n\n $$$$$ Test case 2: W + R + W + R \n");
	for (i = 0; i < NUM_READS; i=i+1) begin
		write_val(('h11 + i), ('hc2 + i));
		#50;
		read_val(('h1e + i));
		#50;
	end

	#100;
	$display("End simulation");
	$finish;
end

endmodule

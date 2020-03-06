// Create Date:    11:58:12 10/13/2018 
// Author: Laurentiu-Cristian Duca
// License: GNU GPL

module latency(clk, rst, SCK, SSEL, MOSI, MISO, leds, interrupt, interrupt_ack, btn2); //,
	//uart_REC_dataH, uart_XMIT_dataH);

//input uart_REC_dataH;
//output uart_XMIT_dataH;
	
input clk, rst, btn2;
input SCK, SSEL, MOSI;
output MISO;
output [7:0] leds;
output interrupt;
input interrupt_ack;

wire spi_output_valid, spi_output_valid_sing;
wire [7:0] byte_data_received;
reg [7:0] byte_data_send, next_byte_data_send;

reg [31:0] int_ack_latency_cnt, next_int_ack_latency_cnt,
	spi_latency_cnt, next_spi_latency_cnt;
reg [3:0] state, next_state;
reg interrupt, next_interrupt;
reg [31:0] cnt, next_cnt;
parameter delay=(5*100*1000); // 10ms

// Debouncer
`define DEB_LEN 10
// SCK
reg sck_deb=1'b0;
reg [`DEB_LEN-1:0] sck_pipe={`DEB_LEN{1'b0}};
always @(posedge clk) begin
    sck_pipe <= {sck_pipe[`DEB_LEN-2:0], SCK};
    if (&sck_pipe[`DEB_LEN-1:1] == 1'b1)
      sck_deb <= 1'b1;
    else if (|sck_pipe[`DEB_LEN-1:1] == 1'b0)
      sck_deb <= 1'b0;
end
// SSEL
reg ssel_deb=1'b1;
reg [`DEB_LEN-1:0] ssel_pipe={`DEB_LEN{1'b1}};
always @(posedge clk) begin
    ssel_pipe <= {ssel_pipe[`DEB_LEN-2:0], SSEL};
    if (&ssel_pipe[`DEB_LEN-1:1] == 1'b1)
      ssel_deb <= 1'b1;
    else if (|ssel_pipe[`DEB_LEN-1:1] == 1'b0)
      ssel_deb <= 1'b0;
end

`define DEB_INTACK_LEN	10
reg int_ack_deb=1'b1;
reg [`DEB_INTACK_LEN-1:0] int_ack_pipe={`DEB_INTACK_LEN{1'b1}};
always @(posedge clk) begin
    int_ack_pipe <= {int_ack_pipe[`DEB_INTACK_LEN-2:0], interrupt_ack};
    if (&int_ack_pipe[`DEB_INTACK_LEN-1:1] == 1'b1)
      int_ack_deb <= 1'b1;
    else if (|int_ack_pipe[`DEB_INTACK_LEN-1:1] == 1'b0)
      int_ack_deb <= 1'b0;
end

// VeriFLA
//top_of_verifla verifla (.clk(clk), .rst_l(!rst), .sys_run(1'b1),
//				.data_in({spi_latency_cnt[9:8], spi_output_valid_sing, interrupt_ack, SCK, SSEL, MOSI, MISO}),
//				// Transceiver
//				.uart_XMIT_dataH(uart_XMIT_dataH), .uart_REC_dataH(uart_REC_dataH));

spi_slave_lcd si (.clk(clk), .SCK(sck_deb), .SSEL(ssel_deb), .MOSI(MOSI), .MISO(MISO), 
	.output_valid(spi_output_valid), .byte_data_received(byte_data_received), .byte_data_send(byte_data_send));
single_pulse sp (.clk(clk), .rst_l(!rst), .ub(spi_output_valid), .ubsing(spi_output_valid_sing));

assign leds=
	(btn2 == 0)
		? {^byte_data_received, interrupt, int_ack_deb, ssel_deb, state[3], state[2], state[1], state[0]}
		: cnt[7:0];

always @(posedge clk or posedge rst)
	if(rst) begin
		state = 7;
		int_ack_latency_cnt = 0;
		spi_latency_cnt = 0;
		interrupt = 0;
		cnt = 0;
		byte_data_send = 0;
	end else begin
		state = next_state;
		int_ack_latency_cnt = next_int_ack_latency_cnt;
		spi_latency_cnt = next_spi_latency_cnt;
		interrupt = next_interrupt;
		cnt = next_cnt;
		byte_data_send = next_byte_data_send;
	end

always @(*) begin
	// implicit
	next_cnt = cnt;
	next_int_ack_latency_cnt = int_ack_latency_cnt;
	next_spi_latency_cnt = spi_latency_cnt;
	next_state = state;
	next_byte_data_send = byte_data_send;
	next_interrupt = interrupt;
	// explicit
	case(state)
	7: begin
		// at the begining, wait for a dummy spi transfer.
		if(spi_output_valid_sing)
			next_state = 0;
	end
	0: begin
		if(!int_ack_deb) begin
			if (cnt >= delay) begin
				next_interrupt = 1;
				next_state = 1;
				next_int_ack_latency_cnt = 0;
				next_cnt = 0;
			end else
				next_cnt = cnt + 1;
		end
	end
	1: begin
		if(!int_ack_deb) begin
			// Protocol: first send interrupt_ack, then spi transfer.
			next_int_ack_latency_cnt = int_ack_latency_cnt + 1;
		end else begin
			next_int_ack_latency_cnt = int_ack_latency_cnt - (`DEB_INTACK_LEN - 1);
			next_spi_latency_cnt = 0;
			next_interrupt = 0;
			next_state = 2;
		end
	end
	2: begin
		//if(ssel_deb) begin
		if(ssel_deb || !sck_deb) begin
			next_spi_latency_cnt = spi_latency_cnt + 1;
		end else begin
			// little endian (LSB first)
			next_byte_data_send = int_ack_latency_cnt[7:0];
			next_state = 3;
			next_cnt = 1;
		end
	end
	3: begin
		if(spi_output_valid_sing) begin
			if(cnt == 1)
				next_byte_data_send = int_ack_latency_cnt[15:8];
			else if(cnt == 2)
				next_byte_data_send = int_ack_latency_cnt[23:16];
			else if(cnt == 3)
				next_byte_data_send = int_ack_latency_cnt[31:24];
			else if(cnt == 4)
				next_byte_data_send = spi_latency_cnt[7:0];
			else if(cnt == 5)
				next_byte_data_send = spi_latency_cnt[15:8];
			else if(cnt == 6)
				next_byte_data_send = spi_latency_cnt[23:16];
			else if(cnt == 7)
				next_byte_data_send = spi_latency_cnt[31:24];
			next_cnt = cnt + 1;
		end else if((cnt >= 9)) begin //&& (ssel_deb == 1)) begin
			next_cnt = 0;
			next_state = 0;
		end
	end
	endcase
end

endmodule

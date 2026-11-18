// A memory memory_libmap put on brams.txt's block RAM -> one BRAM slice, by
// the pins of datasheet table 12 (the uarch takes each port as the pin of its
// name): write and read enables on nWEN and nREN (the read data holds while
// nREN is high, datasheet 9.4.2), the clock enables on, a falling-edge port
// by its clock inversion. On the Shrike a memory of 512 x 8 written and read
// at random every cycle reads back right at 50 MHz, and one read or written
// on the falling edge at 12.5 MHz (2026-11-18).
//
// RATIO 0 only. Below a byte the word's place in it rides on WDATA, as
// datasheet tables 13 and 14 have it (write port WDATA[7:5], read port
// WDATA[4:2]; the placer's ram_def has the two the other way round): every
// word reads back so with the address held for four cycles. But at 4096 x 1
// a read takes those bits at the clock edge only while 1024 x 4 reads wrong
// when nREN gates the reads, and a memory written every cycle at either ratio
// reads back wrong even at 12.5 MHz: what times the narrow ratios is not
// known yet.
module $__SLG47910_BRAM_ (...);
	parameter INIT = 0;
	parameter WIDTH = 8;
	parameter PORT_W_CLK_POL = 1;
	parameter PORT_R_CLK_POL = 1;

	input PORT_W_CLK;
	input PORT_W_WR_EN;
	input [8:0] PORT_W_ADDR;
	input [7:0] PORT_W_WR_DATA;
	input PORT_R_CLK;
	input PORT_R_RD_EN;
	input [8:0] PORT_R_ADDR;
	output [7:0] PORT_R_RD_DATA;

	SLG_BRAM _TECHMAP_REPLACE_ (
		.WCLK(PORT_W_CLK), .RCLK(PORT_R_CLK),
		.NWEN(!PORT_W_WR_EN), .NWCLKEN(1'b0), .NREN(!PORT_R_RD_EN), .NRCLKEN(1'b0),
		.WCLKINV(PORT_W_CLK_POL ? 1'b0 : 1'b1), .RCLKINV(PORT_R_CLK_POL ? 1'b0 : 1'b1),
		.PD(1'b0), .RATIO(2'b00),
		.WADDR(PORT_W_ADDR), .WDATA(PORT_W_WR_DATA), .RADDR(PORT_R_ADDR), .RDATA(PORT_R_RD_DATA));
endmodule

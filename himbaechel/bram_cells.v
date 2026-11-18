// One BRAM slice (datasheet 9.2, table 12), by its pins' names: the uarch
// puts each port on the slice's pin of that name. SLICE picks the slice (0-3
// north, 4-7 south); -1, the first one free.
(* blackbox *)
module SLG_BRAM #(parameter SLICE = -1) (
	input WCLK, input RCLK,
	input NWEN, input NWCLKEN, input NREN, input NRCLKEN,
	input WCLKINV, input RCLKINV, input PD, input [1:0] RATIO,
	input [8:0] WADDR, input [7:0] WDATA, input [8:0] RADDR, output [7:0] RDATA);
endmodule

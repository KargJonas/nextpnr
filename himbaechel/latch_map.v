// A latch the chip has no cell for, as the vendor's placer builds one ("not
// supported in EFLX hardware! Creating SW work-around to mimick the
// data-latch functionality in LUTs"): a LUT whose output feeds back into it,
// Q = R ? 0 : E ? D : Q. dfflegalize makes every latch this one, and a
// flip-flop with an asynchronous set and reset two flip-flops, this latch and
// a mux (Yosys's emulation, as synth_xilinx's). Mapped after abc, which takes
// no loop.
module \$_DLATCH_PP0_ (input E, input R, input D, output Q);
    \$lut #(.WIDTH(4), .LUT(16'h5140)) _TECHMAP_REPLACE_ (.A({Q, D, E, R}), .Y(Q));
endmodule

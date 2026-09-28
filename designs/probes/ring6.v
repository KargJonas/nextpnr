// Five-inverter LUT ring on the six nearby PMOD pins: no clock, no flip-flop.
// It exercises LUTs, switch-matrix routing and output IOBs only; on silicon it
// oscillates, or it settles in a state that names the branch that inverts.
module ring6(output wire [5:0] q);
    (* keep, dont_touch *) wire r0, r1, r2, r3, r4;
    (* keep, dont_touch *) LUT1 #(.INIT(2'h1)) inv0 (.I0(r4), .O(r0));
    (* keep, dont_touch *) LUT1 #(.INIT(2'h1)) inv1 (.I0(r0), .O(r1));
    (* keep, dont_touch *) LUT1 #(.INIT(2'h1)) inv2 (.I0(r1), .O(r2));
    (* keep, dont_touch *) LUT1 #(.INIT(2'h1)) inv3 (.I0(r2), .O(r3));
    (* keep, dont_touch *) LUT1 #(.INIT(2'h1)) inv4 (.I0(r3), .O(r4));
    assign q = {r0, r4, r3, r2, r1, r0};
endmodule

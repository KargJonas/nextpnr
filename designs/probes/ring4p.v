// ring4 with each inverter pinned to its own site, spaced apart: a tile can
// take only two distinct incoming nets (its two D1 down nodes), and the
// placer alone packs the ring into one tile.
module ring4p(output wire [3:0] q);
    (* keep, dont_touch *) wire r0, r1, r2, r3, r4;
    (* keep, dont_touch, BEL = "X27/Y26/Z0/LLUT" *) LUT1 #(.INIT(2'h1)) inv0 (.I0(r4), .O(r0));
    (* keep, dont_touch, BEL = "X23/Y26/Z0/LLUT" *) LUT1 #(.INIT(2'h1)) inv1 (.I0(r0), .O(r1));
    (* keep, dont_touch, BEL = "X27/Y22/Z0/LLUT" *) LUT1 #(.INIT(2'h1)) inv2 (.I0(r1), .O(r2));
    (* keep, dont_touch, BEL = "X23/Y22/Z0/LLUT" *) LUT1 #(.INIT(2'h1)) inv3 (.I0(r2), .O(r3));
    (* keep, dont_touch, BEL = "X25/Y20/Z0/LLUT" *) LUT1 #(.INIT(2'h1)) inv4 (.I0(r3), .O(r4));
    assign q = {r3, r2, r1, r0};
endmodule

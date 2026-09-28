// HITL diagnostic: explicit LUT1 inverter ring routed to GPIO8-15.
// This avoids clock-tree dependencies and keeps the ring from collapsing into
// constants during generic Verilog optimization.
module ring_lut_pmod(
    output wire [7:0] q
);
    (* keep, dont_touch *) wire r0;
    (* keep, dont_touch *) wire r1;
    (* keep, dont_touch *) wire r2;
    (* keep, dont_touch *) wire r3;
    (* keep, dont_touch *) wire r4;

    (* keep, dont_touch *) LUT1 #(.INIT(2'h1)) inv0 (.I0(r4), .O(r0));
    (* keep, dont_touch *) LUT1 #(.INIT(2'h1)) inv1 (.I0(r0), .O(r1));
    (* keep, dont_touch *) LUT1 #(.INIT(2'h1)) inv2 (.I0(r1), .O(r2));
    (* keep, dont_touch *) LUT1 #(.INIT(2'h1)) inv3 (.I0(r2), .O(r3));
    (* keep, dont_touch *) LUT1 #(.INIT(2'h1)) inv4 (.I0(r3), .O(r4));

    assign q = {r2, r1, r0, r4, r3, r2, r1, r0};
endmodule

// osc_free's toggle flip-flop pinned to slice 0 of site (10,9) (X23/Y28), the
// site nextpnr picked for it: tells a site or slice fault from a route fault.
module osc_site10(input wire clk, output wire q0, output wire clk_en);
    wire t, tn;
    (* keep, dont_touch, BEL = "X23/Y28/Z0/LLUT" *) LUT1 #(.INIT(2'h1)) inv (.I0(t), .O(tn));
    (* keep, dont_touch, BEL = "X23/Y28/Z4/LFF" *) FDRE ff (.C(clk), .CE(1'b1), .R(1'b0), .D(tn), .Q(t));
    assign q0 = t;
    assign clk_en = 1'b1;
endmodule

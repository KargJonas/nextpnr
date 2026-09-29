// The toggle flip-flop and its inverter pinned to slice D of RBB_28_26, where
// the vendor's osc_probe puts them, driving only GPIO10 (no constant outputs).
module osc_d2(input wire clk, output wire q0);
    wire t, tn;
    (* keep, dont_touch, BEL = "X27/Y28/Z3/LLUT" *) LUT1 #(.INIT(2'h1)) inv (.I0(t), .O(tn));
    (* keep, dont_touch, BEL = "X27/Y28/Z7/LFF" *) FDRE ff (.C(clk), .CE(1'b1), .R(1'b0), .D(tn), .Q(t));
    assign q0 = t;
endmodule

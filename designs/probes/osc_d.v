// osc_probe with the toggle flip-flop and its inverter pinned to slice D of
// RBB_28_26 -- exactly where the vendor's own osc_probe puts them -- so our
// bitstream can be diffed against the vendor's bit by bit.
module osc_d(input wire clk, output wire [7:0] q, output wire [7:0] q_en, output wire clk_en);
    wire t, tn;
    (* keep, dont_touch, BEL = "X27/Y28/Z3/LLUT" *) LUT1 #(.INIT(2'h1)) inv (.I0(t), .O(tn));
    (* keep, dont_touch, BEL = "X27/Y28/Z7/LFF" *) FDRE ff (.C(clk), .CE(1'b1), .R(1'b0), .D(tn), .Q(t));
    assign q[0] = t;
    assign q[7:1] = 7'b0;
    assign q_en = 8'hFF;
    assign clk_en = 1'b1;
endmodule

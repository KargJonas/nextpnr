// osc_d2 plus clk_en = 1 on the pad the vendor names EFLX_OUT_0_25: the
// oscillator's OSC_EN input (datasheet: OSC_EN is driven from a dedicated
// core IOB). Without it the 50 MHz oscillator does not run.
module osc_d3_GPIO13(input wire clk, output wire q0, output wire clk_en);
    wire t, tn;
    (* keep, dont_touch, BEL = "X27/Y28/Z3/LLUT" *) LUT1 #(.INIT(2'h1)) inv (.I0(t), .O(tn));
    (* keep, dont_touch, BEL = "X27/Y28/Z7/LFF" *) FDRE ff (.C(clk), .CE(1'b1), .R(1'b0), .D(tn), .Q(t));
    assign q0 = t;
    assign clk_en = 1'b1;
endmodule

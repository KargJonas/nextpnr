// A toggle flip-flop placed and routed freely by nextpnr, on GPIO10, with the
// oscillator enabled through its OSC_EN pad.
module osc_free(input wire clk, output wire q0, output wire clk_en);
    reg t = 1'b0;
    always @(posedge clk) t <= ~t;
    assign q0 = t;
    assign clk_en = 1'b1;
endmodule

// Open-flow equivalent of build/clock_debug/vendor_osc_probe/osc_probe.v.
// Single fabric FF that toggles on every Clock_0 edge; q[0] = FF.Q,
// q[7:1] = 0. Used to validate the clock-fanout overlay table
// (tools/clock_fanout.py) end-to-end on the open flow.
module osc_probe(
    input  wire        clk,
    output wire [7:0]  q,
    output wire [7:0]  q_en,
    output wire        clk_en
);
    (* keep, dont_touch *) reg toggle = 1'b0;
    always @(posedge clk) toggle <= ~toggle;

    assign q[0]   = toggle;
    assign q[7:1] = 7'b0;
    assign q_en   = 8'hFF;
    assign clk_en = 1'b1;
endmodule

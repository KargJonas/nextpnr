// Minimal clock witness: q[0] should rise to 1 after the first clock edge.
module ff_set_once(
    input  wire       clk,
    output wire [7:0] q,
    output wire       clk_en,
    output wire [7:0] q_en
);
    reg seen_clk = 0;
    always @(posedge clk)
        seen_clk <= 1'b1;

    assign q      = {7'b1010101, seen_clk};
    assign clk_en = 1'b1;
    assign q_en   = 8'hFF;
endmodule

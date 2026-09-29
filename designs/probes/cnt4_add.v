// The same 4-bit counter through an adder: yosys maps it to the carry chain.
module cnt4_add(input wire clk, output wire [3:0] q, output wire clk_en);
    reg [3:0] t = 0;
    always @(posedge clk) t <= t + 1'b1;
    assign q = t;
    assign clk_en = 1'b1;
endmodule

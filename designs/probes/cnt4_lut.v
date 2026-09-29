// A 4-bit counter as explicit LUT logic (no adder, so no carry chain):
// t[k] toggles at f/2^(k+1) on GPIO8, 9, 10, 12.
module cnt4_lut(input wire clk, output wire [3:0] q, output wire clk_en);
    reg [3:0] t = 0;
    always @(posedge clk) begin
        t[0] <= ~t[0];
        t[1] <= t[1] ^ t[0];
        t[2] <= t[2] ^ (t[0] & t[1]);
        t[3] <= t[3] ^ (t[0] & t[1] & t[2]);
    end
    assign q = t;
    assign clk_en = 1'b1;
endmodule

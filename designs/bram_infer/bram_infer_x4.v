// bram_infer as a 1024 x 4 memory: Yosys puts it on one BRAM slice at a
// narrower ratio, the word's place in a byte on WDATA (bram_map.v).
module bram_infer_x4(input clk, input we, input [9:0] wa, input [3:0] wd, input [9:0] ra,
                  output [3:0] q);
    reg [3:0] mem [0:1023];
    reg [10:0] fill = 0;
    reg [3:0] rd;
    reg [1:0] ok = 0;
    wire filling = !fill[10];
    always @(posedge clk) begin
        if (filling) fill <= fill + 1'b1;
        if (filling || we) mem[filling ? fill[9:0] : wa] <= filling ? fill[3:0] ^ fill[9:6] : wd;
        rd <= mem[ra];
        ok <= {ok[0], !filling};
    end
    assign q = ok[1] ? rd : 1'b0;
endmodule

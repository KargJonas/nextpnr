// bram_infer as a 4096 x 1 memory: Yosys puts it on one BRAM slice at a
// narrower ratio, the word's place in a byte on WDATA (bram_map.v).
module bram_infer_x1(input clk, input we, input [11:0] wa, input [0:0] wd, input [11:0] ra,
                  output [0:0] q);
    reg [0:0] mem [0:4095];
    reg [12:0] fill = 0;
    reg [0:0] rd;
    reg [1:0] ok = 0;
    wire filling = !fill[12];
    always @(posedge clk) begin
        if (filling) fill <= fill + 1'b1;
        if (filling || we) mem[filling ? fill[11:0] : wa] <= filling ? fill[0:0] ^ fill[11:11] : wd;
        rd <= mem[ra];
        ok <= {ok[0], !filling};
    end
    assign q = ok[1] ? rd : 1'b0;
endmodule

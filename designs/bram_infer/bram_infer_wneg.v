// bram_infer written on the falling edge (WCLKINV), half a cycle after its
// registered address and data: a memory Yosys maps onto a BRAM slice (flow.sh: brams.txt), for bist.py:
// from the start it writes all 512 words, then writes and reads at random (the
// LFSR's), the read registered, the old word read where both meet. q holds 0
// until every word is written, so that the simulation never reads one unwritten.
module bram_infer_wneg(input clk, input we, input [8:0] wa, input [7:0] wd, input [8:0] ra,
                  output [7:0] q);
    reg [7:0] mem [0:511];
    reg [9:0] fill = 0;
    reg [7:0] rd;
    reg [1:0] ok = 0;
    wire filling = !fill[9];
    always @(posedge clk) begin
        if (filling) fill <= fill + 1'b1;
        rd <= mem[ra];
        ok <= {ok[0], !filling};
    end
    reg wen = 0;
    reg [8:0] waddr;
    reg [7:0] wdata;
    always @(posedge clk) begin
        wen <= filling || we; waddr <= filling ? fill[8:0] : wa; wdata <= filling ? fill[7:0] ^ 8'h5a : wd;
    end
    always @(negedge clk) if (wen) mem[waddr] <= wdata;
    assign q = ok[1] ? rd : 8'h00;
endmodule

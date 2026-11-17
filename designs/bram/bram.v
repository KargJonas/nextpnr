// BRAM slice 0 written once (data bit k = address bit k+1), then read slowly,
// the address stepped every 4096 cycles. la[0] heartbeat, la[1] q[0] (3.05 kHz if right), la[2] q[1] (1.5 kHz).
module bram(input wire clk, output wire clk_en, output wire [2:0] la,
            output reg [7:0] d, output reg [8:0] wa, output reg [8:0] ra, output reg web, output reg wclken,
          output reg reb, output reg rclken, output wire [1:0] ratio, input wire [7:0] q);
    assign clk_en = 1'b1;
    assign ratio = 2'b00;
    reg [23:0] n = 0;
    always @(posedge clk) begin
        n <= n + 1'b1;
        wa <= n[8:0]; d <= {n[8:1], n[2], n[1]}; web <= 1'b0; wclken <= 1'b0;
        ra <= n[20:12]; reb <= 1'b0; rclken <= 1'b0;
    end
    assign la = {q[1], q[0], n[22]};
endmodule

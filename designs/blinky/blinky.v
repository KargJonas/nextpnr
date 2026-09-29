// Blink at ~1.5 Hz from the 50 MHz on-chip oscillator: count[24] on `led`,
// and three counter taps on GPIO8-10 (logic-analyzer channels D0-D2).
// clk_en drives the oscillator's OSC_EN input (the vendor's EFLX_OUT_0_25);
// without it the oscillator does not run.
module blinky(input wire clk, output wire led, output wire [2:0] la, output wire clk_en);
    reg [24:0] count = 0;
    always @(posedge clk) count <= count + 1'b1;
    assign led = count[24];
    assign la = {count[12], count[16], count[24]};
    assign clk_en = 1'b1;
endmodule

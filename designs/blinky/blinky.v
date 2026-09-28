// Blink the Shrike's FPGA LED (GPIO16) at ~1.5 Hz from the 50 MHz oscillator,
// and drive the same signal on GPIO8 (logic-analyzer channel D0) plus two
// faster counter taps on GPIO9/GPIO10 so a 2 s capture shows it running.
module blinky(input wire clk, output wire led, output wire [2:0] la);
    reg [24:0] count = 0;
    always @(posedge clk) count <= count + 1'b1;
    assign led = count[24];
    assign la = {count[12], count[16], count[24]};
endmodule

#include <Arduino.h>

// Pure MCU LED blinky — no FPGA, no serial wait. Blinks the Shrike board's
// LED pins at 1 Hz (500 ms on / 500 ms off) so "is the MCU alive?" can be
// confirmed by eye, independent of the FPGA or the HITL flash path.
static const uint8_t led_pins[] = {
    4, 5, 6, 7, 8, 9, 10, 11,
    14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29};
static const size_t NUM_LEDS = sizeof(led_pins) / sizeof(led_pins[0]);

void setup() {
    for (size_t i = 0; i < NUM_LEDS; i++) pinMode(led_pins[i], OUTPUT);
}

void loop() {
    for (size_t i = 0; i < NUM_LEDS; i++) digitalWrite(led_pins[i], HIGH);
    delay(500);
    for (size_t i = 0; i < NUM_LEDS; i++) digitalWrite(led_pins[i], LOW);
    delay(500);
}

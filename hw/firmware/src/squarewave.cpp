#include <Arduino.h>

// Toggle GPIO26 at 5 Hz (100 ms half-period) so a 2 s LA capture shows ~10 cycles.
#define SQWAVE_PIN     26
#define HALF_PERIOD_MS 100

void setup() {
    pinMode(SQWAVE_PIN, OUTPUT);
}

void loop() {
    digitalWrite(SQWAVE_PIN, HIGH);
    delay(HALF_PERIOD_MS);
    digitalWrite(SQWAVE_PIN, LOW);
    delay(HALF_PERIOD_MS);
}

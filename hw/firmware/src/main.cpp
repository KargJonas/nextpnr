#include <Arduino.h>
#include <LittleFS.h>
#include <Shrike.h>

ShrikeFlash fpga;

static const char*    HITL_PATH     = "/hitl.bin";
static const uint32_t MAX_BITSTREAM = 49152;  // 48 KB

// FPGA GPIO6 → RP2040 MISO (GPIO0). Goes HIGH when FPGA enters functional mode.
static const uint8_t CONFIG_PIN = 0;

// Receive `size` bytes from Serial and write them to LittleFS, then
// flash the FPGA.  Returns true on success.
static bool receive_and_flash(uint32_t size) {
    File f = LittleFS.open(HITL_PATH, "w");
    if (!f) {
        Serial.println("ERR: LittleFS open failed");
        return false;
    }

    uint8_t  buf[512];
    uint32_t remaining = size;
    while (remaining > 0) {
        uint32_t chunk = min(remaining, (uint32_t)sizeof(buf));
        int      n     = Serial.readBytes(buf, chunk);
        if (n <= 0) {
            f.close();
            Serial.println("ERR: serial timeout");
            return false;
        }
        f.write(buf, (size_t)n);
        remaining -= (uint32_t)n;
    }
    f.close();

    return fpga.flash(HITL_PATH);
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(30000);  // 30s should be enough for a 48 KB USB-CDC transfer

    // Wait for host connection (up to 5 s so pio upload can connect).
    while (!Serial && millis() < 5000)
        delay(10);

    if (!LittleFS.begin()) {
        Serial.println("ERR: LittleFS init failed");
        return;
    }

    if (!fpga.begin()) {
        Serial.println("ERR: FPGA init failed");
        return;
    }

    pinMode(CONFIG_PIN, INPUT);
    Serial.println("HITL READY");
}

void loop() {
    if (!Serial.available())
        return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("FLASH ")) {
        // FLASH <size>  ->  READY  ->  <size bytes>  ->  OK/ERR
        uint32_t size = (uint32_t)cmd.substring(6).toInt();
        if (size == 0 || size > MAX_BITSTREAM) {
            Serial.println("ERR: invalid size");
            return;
        }
        Serial.println("READY");
        bool ok = receive_and_flash(size);
        // Give FPGA time to enter functional mode then read CONFIG signal.
        delay(100);
        int cfg = digitalRead(CONFIG_PIN);
        Serial.print("CONFIG=");
        Serial.println(cfg);
        Serial.println(ok ? "OK" : "ERR: flash failed");
    } else if (cmd == "STATUS") {
        Serial.println("IDLE");
    } else if (cmd == "CONFIG") {
        // Read FPGA CONFIG pin without flashing.
        delay(10);
        Serial.print("CONFIG=");
        Serial.println(digitalRead(CONFIG_PIN));
    } else if (cmd == "PING") {
        Serial.println("PONG");
    }
}

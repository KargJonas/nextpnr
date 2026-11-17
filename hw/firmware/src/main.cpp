#include <Arduino.h>
#include <LittleFS.h>
#include <Shrike.h>

ShrikeFlash fpga;

static const char* HITL_PATH = "/hitl.bin";

// The MCU image as pack.py writes it: 1280 zeros, the sync word, then the
// header (9 words) and 1024 rows (11 words each), every block followed by its
// check word. The FPGA takes the image without the check words, and 32 zeros.
static const uint32_t PREAMBLE = 1280, ROWS = 1024, HEADER_WORDS = 9, ROW_WORDS = 11, PAD = 32;
static const uint32_t MCU_BYTES = PREAMBLE + 4 + (HEADER_WORDS + 1 + ROWS * (ROW_WORDS + 1)) * 4;
static const uint8_t  SYNC[4] = {0xAA, 0x22, 0xFF, 0x11};

// FPGA GPIO6 → RP2040 MISO (GPIO0). Goes HIGH when FPGA enters functional mode.
static const uint8_t CONFIG_PIN = 0;

static uint8_t image[MCU_BYTES];

// eda-placer's check word of a block of AXI words (EFLX_bitstream_AXI_CRC.log):
// CRC-16, polynomial 0xA2EB reflected, initial value 0, no final XOR, each
// word (stored little-endian) fed from bit 0 to bit 31.
static uint16_t crc16(const uint8_t* p, uint32_t words) {
    uint16_t r = 0;
    for (uint32_t i = 0; i < words * 4; i++) {
        r ^= p[i];
        for (int b = 0; b < 8; b++) r = (r & 1) ? (r >> 1) ^ 0xD745 : r >> 1;
    }
    return r;
}

// Check the received image and strip its check words in place: the length of
// the image the FPGA takes, or 0 having said REFUSED and where.
static uint32_t check() {
    for (uint32_t i = 0; i < PREAMBLE; i++)
        if (image[i]) {
            Serial.printf("REFUSED preamble byte %lu\n", i);
            return 0;
        }
    if (memcmp(image + PREAMBLE, SYNC, 4)) {
        Serial.println("REFUSED sync word");
        return 0;
    }
    uint32_t in = PREAMBLE + 4, out = in;
    for (uint32_t block = 0; block <= ROWS; block++) {
        uint32_t words = block ? ROW_WORDS : HEADER_WORDS;
        const uint8_t* c = image + in + words * 4;
        if ((c[0] | (c[1] << 8)) != crc16(image + in, words) || c[2] || c[3]) {
            if (block) Serial.printf("REFUSED row %lu\n", block - 1);
            else Serial.println("REFUSED header");
            return 0;
        }
        memmove(image + out, image + in, words * 4);
        in += (words + 1) * 4;
        out += words * 4;
    }
    memset(image + out, 0, PAD);
    return out + PAD;
}

// Receive the MCU image from Serial, check it, and flash the FPGA with it.
// False, the FPGA untouched, when the image is refused or the transfer fails.
static bool receive_and_flash(uint32_t size) {
    uint32_t got = Serial.readBytes(image, size);
    if (got < size) {
        Serial.printf("ERR: serial timeout after %lu of %lu bytes\n", got, size);
        return false;
    }
    uint32_t n = check();
    if (!n) return false;
    File f = LittleFS.open(HITL_PATH, "w");
    if (!f) {
        Serial.println("ERR: LittleFS open failed");
        return false;
    }
    f.write(image, n);
    f.close();
    if (!fpga.flash(HITL_PATH)) {
        Serial.println("ERR: flash failed");
        return false;
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(30000);  // 30s should be enough for a 50 KB USB-CDC transfer

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
        // FLASH <size>  ->  READY  ->  <size bytes>  ->  CONFIG=<pin> and OK, or REFUSED <where>, or ERR
        uint32_t size = (uint32_t)cmd.substring(6).toInt();
        if (size != MCU_BYTES) {
            Serial.printf("ERR: %lu bytes, not the %lu of an MCU image\n", size, MCU_BYTES);
            return;
        }
        Serial.println("READY");
        if (!receive_and_flash(size))
            return;
        // Give FPGA time to enter functional mode then read CONFIG signal.
        delay(100);
        int cfg = digitalRead(CONFIG_PIN);
        Serial.print("CONFIG=");
        Serial.println(cfg);
        Serial.println("OK");
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

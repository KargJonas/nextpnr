#include <Arduino.h>
#include <SPI.h>

// The Shrike's RP2040 to its FPGA (Vicharak's pin map): EN (nSLEEP), PWR
// (nRST), and the configuration SPI, which also talks to a loaded design.
static const uint8_t EN_PIN = 13, PWR_PIN = 12, SS_PIN = 1, SCK_PIN = 2, MOSI_PIN = 3, MISO_PIN = 0;

// The MCU image as pack.py writes it: 1280 zeros, the sync word, then the
// header (9 words) and 1024 rows (11 words each), every block followed by its
// check word. The FPGA takes the image without the check words, and 32 zeros.
static const uint32_t PREAMBLE = 1280, ROWS = 1024, HEADER_WORDS = 9, ROW_WORDS = 11, PAD = 32;
static const uint32_t MCU_BYTES = PREAMBLE + 4 + (HEADER_WORDS + 1 + ROWS * (ROW_WORDS + 1)) * 4;
static const uint8_t  SYNC[4] = {0xAA, 0x22, 0xFF, 0x11};

// FPGA GPIO6 → RP2040 MISO (GPIO0). Goes HIGH when FPGA enters functional mode.
static const uint8_t CONFIG_PIN = MISO_PIN;

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

// Load the FPGA as Vicharak's ShrikeFlash does (1.6 MHz SPI, the select
// pulsed, the image in one selected stream), but reset it by PWR alone. Taken
// low together with EN, as ShrikeFlash does, the reset leaves the BRAM reading
// 0 until the chip next powers up: on a second Shrike-Lite every load after the
// first read 0 with it and the right data with PWR alone (Vicharak's
// stack_processor, 2026-11-17).
static void load_fpga(const uint8_t* p, uint32_t n) {
    digitalWrite(SS_PIN, LOW);
    digitalWrite(PWR_PIN, LOW);
    delay(100);
    digitalWrite(PWR_PIN, HIGH);
    delay(600);
    digitalWrite(SS_PIN, HIGH);
    delayMicroseconds(2000);
    digitalWrite(SS_PIN, LOW);
    static uint8_t chunk[256];
    SPI.beginTransaction(SPISettings(1600000, MSBFIRST, SPI_MODE0));
    for (uint32_t i = 0; i < n; i += sizeof chunk) {
        uint32_t k = min((uint32_t)sizeof chunk, n - i);
        memcpy(chunk, p + i, k);  // transfer() overwrites its buffer with what comes back
        SPI.transfer(chunk, k);
    }
    SPI.endTransaction();
    digitalWrite(SS_PIN, HIGH);
    delay(100);
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(30000);  // 30s should be enough for a 50 KB USB-CDC transfer

    // Wait for host connection (up to 5 s so pio upload can connect).
    while (!Serial && millis() < 5000)
        delay(10);

    // The FPGA awake (EN) and held in reset (PWR) until the first load.
    pinMode(EN_PIN, OUTPUT);
    pinMode(PWR_PIN, OUTPUT);
    pinMode(SS_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);
    digitalWrite(PWR_PIN, LOW);
    digitalWrite(SS_PIN, HIGH);
    SPI.setSCK(SCK_PIN);
    SPI.setTX(MOSI_PIN);
    SPI.setRX(MISO_PIN);
    SPI.begin();
    // No pull-down on MISO (an RP2040 pin's default): GPIO6 reads its own
    // pull-up, as CONFIG and as a design's input.
    gpio_disable_pulls(MISO_PIN);
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
        uint32_t got = Serial.readBytes(image, size);
        if (got < size) {
            Serial.printf("ERR: serial timeout after %lu of %lu bytes\n", got, size);
            return;
        }
        uint32_t n = check();
        if (!n)
            return;
        load_fpga(image, n);
        // Give FPGA time to enter functional mode then read CONFIG signal.
        delay(100);
        Serial.print("CONFIG=");
        Serial.println(digitalRead(CONFIG_PIN));
        Serial.println("OK");
    } else if (cmd == "CONFIG") {
        // Read FPGA CONFIG pin without flashing.
        delay(10);
        Serial.print("CONFIG=");
        Serial.println(digitalRead(CONFIG_PIN));
    } else if (cmd == "PING") {
        Serial.println("PONG");
    } else if (cmd.startsWith("PIN ")) {
        // PIN <rp2040 gpio> <0|1>  ->  OK: drive an MCU pin (a design's reset, say)
        int sp = cmd.indexOf(' ', 4);
        int pin = cmd.substring(4, sp).toInt(), v = cmd.substring(sp + 1).toInt();
        pinMode(pin, OUTPUT);
        digitalWrite(pin, v ? HIGH : LOW);
        Serial.println("OK");
    } else if (cmd.startsWith("XFER ")) {
        // XFER <hex byte> ...  ->  RX <hex byte> ...: to the loaded design over
        // the configuration SPI (mode 0, 1 MHz, MSB first), each byte with a
        // select of its own and 10 ms after it, as Vicharak's examples talk to
        // theirs.
        Serial.print("RX");
        for (int i = 5; i + 1 < (int)cmd.length(); i += 3) {
            uint8_t b = (uint8_t)strtol(cmd.substring(i, i + 2).c_str(), nullptr, 16);
            digitalWrite(SS_PIN, LOW);
            SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
            uint8_t r = SPI.transfer(b);
            SPI.endTransaction();
            digitalWrite(SS_PIN, HIGH);
            Serial.printf(" %02X", r);
            delay(10);
        }
        Serial.println();
    }
}

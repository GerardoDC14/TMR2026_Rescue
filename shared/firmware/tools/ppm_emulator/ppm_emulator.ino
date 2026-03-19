/*
 * ppm_emulator.ino  —  FS-iA6B PPM emulator for a spare ESP32
 *
 * Generates a 6-channel PPM stream compatible with Rescue_Firmware's RC.cpp:
 *   - Rising-edge to rising-edge interval = channel value in µs
 *   - Sync gap > 3000 µs resets the channel index in the decoder ISR
 *   - 6 channels, 50 Hz frame rate, default 1500 µs per channel
 *
 * Wiring:
 *   PPM_OUT_PIN (GPIO 2 on this emulator) ──► GPIO 34 on the robot ESP32
 *   GND ──────────────────────────────────►  GND on the robot ESP32
 *
 * Serial commands (115200 baud, send with newline):
 *   ch<N> <us>          e.g.  ch1 1500   ch4 1200
 *   channel <N> <us>    e.g.  channel 3 1700
 *   <N> <us>            e.g.  1 1500<
 *   status              print all channel values
 *   reset               set all channels to 1500 µs
 *
 * Channel map (matches config.h):
 *   Ch1 = flipper target   Ch2 = forward/back
 *   Ch4 = left/right turn  Ch5 = mode switch (ARM vs MAIN)
 */

#define PPM_OUT_PIN     2       // Wire this to GPIO 34 on the robot ESP32
#define NUM_CHANNELS    6
#define PULSE_US        300     // High-pulse width per channel slot (µs)
#define FRAME_US        20000   // 20 ms → 50 Hz
#define CH_MIN_US       800
#define CH_MAX_US       2500
#define CH_DEFAULT_US   1500

// ─── Shared channel state ─────────────────────────────────────────────────────
static uint16_t       g_ch[NUM_CHANNELS];
static portMUX_TYPE   g_mux = portMUX_INITIALIZER_UNLOCKED;

// ─── PPM generation task (runs on Core 0) ────────────────────────────────────
static void ppmTask(void *) {
    pinMode(PPM_OUT_PIN, OUTPUT);
    digitalWrite(PPM_OUT_PIN, LOW);

    for (;;) {
        // Snapshot channel values
        uint16_t ch[NUM_CHANNELS];
        portENTER_CRITICAL(&g_mux);
        memcpy(ch, g_ch, sizeof(ch));
        portEXIT_CRITICAL(&g_mux);

        // Sync gap = remainder of 20 ms frame after all channel slots.
        // Must stay well above the 3000 µs threshold in RC.cpp.
        uint32_t used = 0;
        for (int i = 0; i < NUM_CHANNELS; i++) used += ch[i];
        uint32_t sync_us = (FRAME_US > used + 5000) ? (FRAME_US - used) : 5000;

        // Output NUM_CHANNELS channel pulses.
        // Each slot: pin HIGH for PULSE_US, then LOW for (ch[i] - PULSE_US).
        // Rising edge of next slot arrives exactly ch[i] µs after this one.
        for (int i = 0; i < NUM_CHANNELS; i++) {
            digitalWrite(PPM_OUT_PIN, HIGH);
            delayMicroseconds(PULSE_US);
            digitalWrite(PPM_OUT_PIN, LOW);
            delayMicroseconds(ch[i] - PULSE_US);
        }

        // Sync pulse: rising edge starts the gap the decoder recognises as sync.
        digitalWrite(PPM_OUT_PIN, HIGH);
        delayMicroseconds(PULSE_US);
        digitalWrite(PPM_OUT_PIN, LOW);
        delayMicroseconds(sync_us - PULSE_US);
    }
}

// ─── Serial command parser ────────────────────────────────────────────────────
static void setChannel(int ch_1indexed, int us) {
    if (ch_1indexed < 1 || ch_1indexed > NUM_CHANNELS) {
        Serial.printf("  ERR: channel must be 1–%d\n", NUM_CHANNELS);
        return;
    }
    if (us < CH_MIN_US || us > CH_MAX_US) {
        Serial.printf("  ERR: value must be %d–%d µs\n", CH_MIN_US, CH_MAX_US);
        return;
    }
    portENTER_CRITICAL(&g_mux);
    g_ch[ch_1indexed - 1] = (uint16_t)us;
    portEXIT_CRITICAL(&g_mux);
    Serial.printf("  ch%d = %d µs\n", ch_1indexed, us);
}

static void printStatus() {
    Serial.println("  --- Channel values ---");
    const char *labels[] = {"flipper", "forward", "ch3", "turn", "mode", "ch6"};
    for (int i = 0; i < NUM_CHANNELS; i++) {
        portENTER_CRITICAL(&g_mux);
        uint16_t v = g_ch[i];
        portEXIT_CRITICAL(&g_mux);
        Serial.printf("  ch%d %-8s %4d µs\n", i + 1, labels[i], v);
    }
}

static void parseCommand(const String &raw) {
    String line = raw;
    line.trim();
    line.toLowerCase();

    if (line == "status") { printStatus(); return; }

    if (line == "reset") {
        portENTER_CRITICAL(&g_mux);
        for (int i = 0; i < NUM_CHANNELS; i++) g_ch[i] = CH_DEFAULT_US;
        portEXIT_CRITICAL(&g_mux);
        Serial.println("  All channels reset to 1500 µs");
        return;
    }

    int ch_num = 0, us_val = 0;

    // "ch<N> <us>"  or  "ch<N>: <us>"
    if (line.startsWith("ch")) {
        if (sscanf(line.c_str(), "ch%d %d", &ch_num, &us_val) == 2 ||
            sscanf(line.c_str(), "ch%d: %d", &ch_num, &us_val) == 2) {
            setChannel(ch_num, us_val);
            return;
        }
    }

    // "channel <N> <us>"  or  "channel <N>: <us> us"
    if (line.startsWith("channel")) {
        if (sscanf(line.c_str(), "channel %d %d", &ch_num, &us_val) == 2 ||
            sscanf(line.c_str(), "channel %d: %d", &ch_num, &us_val) == 2) {
            setChannel(ch_num, us_val);
            return;
        }
    }

    // "<N> <us>"  bare numbers
    if (sscanf(line.c_str(), "%d %d", &ch_num, &us_val) == 2 &&
        ch_num >= 1 && ch_num <= NUM_CHANNELS) {
        setChannel(ch_num, us_val);
        return;
    }

    Serial.println("  ERR: unrecognised command. Try: ch1 1500 | channel 4: 1200 | status | reset");
}

// ─── Arduino entry points ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    for (int i = 0; i < NUM_CHANNELS; i++) g_ch[i] = CH_DEFAULT_US;

    // ppmTask busy-waits with delayMicroseconds, starving Core 0's idle task.
    // Remove IDLE0 from watchdog monitoring so it doesn't panic.
    disableCore0WDT();

    xTaskCreatePinnedToCore(ppmTask, "ppm", 2048, nullptr, 5, nullptr, 0);

    Serial.println("\n=== PPM Emulator ready ===");
    Serial.printf("Output pin: GPIO %d  |  %d channels  |  %d Hz\n",
                  PPM_OUT_PIN, NUM_CHANNELS, 1000000 / FRAME_US);
    Serial.println("Commands: ch<N> <us>  |  channel <N>: <us>  |  status  |  reset");
    Serial.println("Channels: 1=flipper  2=forward  4=turn  5=mode");
    printStatus();
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        parseCommand(cmd);
    }
}

#include "Arduino.h"
#include "camera.h"
#include "dot_detect.h"
#include "triangulation.h"

static TriangConfig g_triang;
static bool         g_calibrated = false;
static uint32_t     g_last_report_ms = 0;

// ── Serial command handler ──────────────────────────────────────────────────
// Commands (newline-terminated):
//   cal          — use current dot position as reference (height = 0 mm)
//   cal <mm>     — use current dot position as reference at known height
//   thr <0-255>  — set detection threshold at runtime
//   dbg          — dump next frame stats (peak, centroid, pixel count)
static uint8_t g_threshold = DETECT_THRESHOLD;
static bool    g_dbg_once  = false;

static void handle_serial() {
    static char buf[32];
    static int  pos = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            buf[pos] = '\0';
            pos = 0;

            if (strncmp(buf, "cal", 3) == 0) {
                g_calibrated = false;           // trigger re-cal on next frame
                g_triang.ref_height_mm = 0.0f;
                if (buf[3] == ' ') {
                    g_triang.ref_height_mm = atof(buf + 4);
                }
                Serial.printf("[cmd] will calibrate at next valid frame (ref=%.1f mm)\n",
                              g_triang.ref_height_mm);

            } else if (strncmp(buf, "thr ", 4) == 0) {
                int v = atoi(buf + 4);
                if (v >= 0 && v <= 255) {
                    g_threshold = (uint8_t)v;
                    Serial.printf("[cmd] threshold set to %d\n", g_threshold);
                }

            } else if (strcmp(buf, "dbg") == 0) {
                g_dbg_once = true;
                Serial.println("[cmd] will dump next frame");

            } else {
                Serial.printf("[cmd] unknown: %s\n", buf);
            }
        } else {
            if (pos < (int)sizeof(buf) - 1) buf[pos++] = c;
        }
    }
}

// ── setup / loop ────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[DoughCV] booting");

    g_triang = triang_default_config();

    if (!camera_init()) {
        Serial.println("[DoughCV] FATAL: camera init failed — halting");
        while (true) delay(1000);
    }
    Serial.println("[DoughCV] camera OK");
    Serial.printf("[DoughCV] laser=%.1f° baseline=%.1f mm focal=%.1f px threshold=%d\n",
                  g_triang.laser_angle_deg,
                  g_triang.baseline_mm,
                  g_triang.focal_length_px,
                  g_threshold);
    Serial.println("[DoughCV] serial commands: cal [mm] | thr <0-255> | dbg");
}

void loop() {
    handle_serial();

    uint32_t now = millis();
    if (now - g_last_report_ms < REPORT_INTERVAL_MS) return;
    g_last_report_ms = now;

    Frame f{};
    if (!camera_capture(&f)) {
        Serial.println("[dot] capture failed");
        return;
    }

    DotResult dot = dot_detect(f.buf, f.width, f.height, g_threshold);

    if (g_dbg_once) {
        g_dbg_once = false;
        Serial.printf("[dbg] peak=%.0f count=%d valid=%d cx=%.2f cy=%.2f thr=%d\n",
                      dot.peak, dot.count, dot.valid, dot.cx, dot.cy, g_threshold);
        // Print a tiny ASCII brightness map around centroid
        int ox = (int)dot.cx - 8, oy = (int)dot.cy - 8;
        for (int dy = 0; dy < 16; ++dy) {
            int py = oy + dy;
            if (py < 0 || py >= f.height) { Serial.println(); continue; }
            for (int dx = 0; dx < 16; ++dx) {
                int px = ox + dx;
                if (px < 0 || px >= f.width) { Serial.print(' '); continue; }
                uint8_t v = f.buf[py * f.width + px];
                const char* chars = " .:-=+*#@";
                Serial.print(chars[v * 8 / 256]);
            }
            Serial.println();
        }
    }

    if (!dot.valid) {
        Serial.printf("[dot] no dot (peak=%.0f < thr=%d)\n", dot.peak, g_threshold);
        camera_release(&f);
        return;
    }

    // First valid dot after a cal command → set calibration reference
    if (!g_calibrated) {
        triang_calibrate(&g_triang, dot.cy);
        g_calibrated = true;
        Serial.printf("[cal] reference set: cy=%.2f height=%.1f mm\n",
                      dot.cy, g_triang.ref_height_mm);
    }

    float rise_mm = triang_height(&g_triang, dot.cy);

    Serial.printf("[rise] %.2f mm  (cy=%.2f peak=%.0f n=%d)\n",
                  rise_mm, dot.cy, dot.peak, dot.count);

    camera_release(&f);
}

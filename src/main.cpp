#include "Arduino.h"
#include "camera.h"
#include "line_detect.h"
#include "triangulation.h"

static TriangConfig g_triang;
static bool         g_calibrated = false;
static uint32_t     g_last_report_ms = 0;

static uint8_t g_threshold = DETECT_THRESHOLD;
static bool    g_dbg_once  = false;

// Calibration: store per-column reference y
static float g_cal_y[LINE_COLS];
static bool  g_cal_valid[LINE_COLS];

// ── Serial command handler ────────────────────────────────────────────────────
// cal          — capture current line position as baseline (height=0)
// thr <0-255>  — set detection threshold live
// dbg          — print next frame's line profile as ASCII

static void handle_serial() {
    static char buf[32];
    static int  pos = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            buf[pos] = '\0'; pos = 0;

            if (strncmp(buf, "cal", 3) == 0) {
                g_calibrated = false;
                Serial.println("[cmd] will calibrate at next valid frame");
            } else if (strncmp(buf, "thr ", 4) == 0) {
                int v = atoi(buf + 4);
                if (v >= 0 && v <= 255) {
                    g_threshold = (uint8_t)v;
                    Serial.printf("[cmd] threshold -> %d\n", g_threshold);
                }
            } else if (strcmp(buf, "dbg") == 0) {
                g_dbg_once = true;
                Serial.println("[cmd] will dump next line profile");
            } else {
                Serial.printf("[cmd] unknown: %s\n", buf);
            }
        } else {
            if (pos < (int)sizeof(buf) - 1) buf[pos++] = c;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[DoughCV] booting (line-laser mode)");

    g_triang = triang_default_config();
    memset(g_cal_valid, 0, sizeof(g_cal_valid));

    if (!camera_init()) {
        Serial.println("[DoughCV] FATAL: camera init failed");
        while (true) delay(1000);
    }
    Serial.println("[DoughCV] camera OK");
    Serial.printf("[DoughCV] laser=%.1f° baseline=%.1fmm focal=%.1fpx thr=%d\n",
                  g_triang.laser_angle_deg, g_triang.baseline_mm,
                  g_triang.focal_length_px, g_threshold);
    Serial.println("[DoughCV] commands: cal | thr <0-255> | dbg");
}

void loop() {
    handle_serial();

    uint32_t now = millis();
    if (now - g_last_report_ms < REPORT_INTERVAL_MS) return;
    g_last_report_ms = now;

    Frame f{};
    if (!camera_capture(&f)) {
        Serial.println("[line] capture failed");
        return;
    }

    LineProfile prof = line_detect(f.buf, f.width, f.height, g_threshold);

    if (g_dbg_once) {
        g_dbg_once = false;
        Serial.printf("[dbg] n_valid=%d/%d peak=%.0f thr=%d\n",
                      prof.n_valid, LINE_COLS, prof.peak, g_threshold);
        // ASCII bar chart of line y position across columns
        for (int ci = 0; ci < LINE_COLS; ci++) {
            if (!prof.valid[ci]) { Serial.print('.'); continue; }
            // Map cy to 0-9
            int bar = (int)(prof.cy[ci] * 9.9f / f.height);
            Serial.print((char)('0' + bar));
        }
        Serial.println();
    }

    if (prof.n_valid < LINE_COLS / 4) {
        Serial.printf("[line] no line (peak=%.0f n=%d thr=%d)\n",
                      prof.peak, prof.n_valid, g_threshold);
        camera_release(&f);
        return;
    }

    // First valid profile after cal → set calibration baseline
    if (!g_calibrated) {
        memcpy(g_cal_y, prof.cy, sizeof(g_cal_y));
        memcpy(g_cal_valid, prof.valid, sizeof(g_cal_valid));
        g_calibrated = true;
        triang_calibrate(&g_triang, prof.cy[LINE_COLS / 2]); // centre column for legacy path
        Serial.printf("[cal] baseline captured (%d/%d cols)\n", prof.n_valid, LINE_COLS);
        camera_release(&f);
        return;
    }

    // Find maximum vertical displacement → rise height
    const float tanθ = tanf(g_triang.laser_angle_deg * (float)M_PI / 180.0f);
    float max_rise = 0.0f;
    int   first_col = -1, last_col = -1;

    for (int ci = 0; ci < LINE_COLS; ci++) {
        if (!prof.valid[ci] || !g_cal_valid[ci]) continue;
        float disp = g_cal_y[ci] - prof.cy[ci];
        if (disp < 0.5f) continue;
        float h = disp * g_triang.baseline_mm / (g_triang.focal_length_px * tanθ);
        if (h > max_rise) max_rise = h;
        if (first_col < 0) first_col = ci;
        last_col = ci;
    }

    float footprint_px = (first_col >= 0) ? (last_col - first_col) * (float)f.width / LINE_COLS : 0.0f;

    Serial.printf("[rise] %.2f mm  footprint=%.1f mm  line=%d/%d cols\n",
                  max_rise, footprint_px * g_triang.baseline_mm / g_triang.focal_length_px,
                  prof.n_valid, LINE_COLS);

    camera_release(&f);
}

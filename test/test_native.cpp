// Native unit tests — run with: pio test -e native_test
// Tests dot_detect and triangulation math without any hardware.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

// Stub defines normally injected by platformio.ini
#ifndef LASER_ANGLE_DEG
#define LASER_ANGLE_DEG    30.0f
#define CAMERA_BASELINE_MM 30.0f
#define FOCAL_LENGTH_PX    88.0f
#define DETECT_THRESHOLD   180
#endif

#include "../src/dot_detect.h"
#include "../src/dot_detect.cpp"
#include "../src/triangulation.h"

// ── helpers ─────────────────────────────────────────────────────────────────

static int g_pass = 0, g_fail = 0;

static void check(bool ok, const char* name) {
    if (ok) { printf("  PASS  %s\n", name); ++g_pass; }
    else     { printf("  FAIL  %s\n", name); ++g_fail; }
}

static bool near(float a, float b, float tol = 0.5f) {
    return fabsf(a - b) <= tol;
}

// Make a black W×H image with a Gaussian dot at (cx, cy), peak amplitude `amp`.
static uint8_t* make_dot_image(int W, int H, float cx, float cy,
                                float sigma, uint8_t amp) {
    uint8_t* img = (uint8_t*)calloc(W * H, 1);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            float d2 = ((float)(x)-cx)*((float)(x)-cx)
                     + ((float)(y)-cy)*((float)(y)-cy);
            float v = amp * expf(-d2 / (2.0f * sigma * sigma));
            img[y * W + x] = (uint8_t)(v < 255.0f ? v : 255.0f);
        }
    return img;
}

// ── dot_detect tests ─────────────────────────────────────────────────────────

static void test_dot_detect_center() {
    const int W = 96, H = 96;
    float true_cx = 48.0f, true_cy = 52.0f;
    uint8_t* img = make_dot_image(W, H, true_cx, true_cy, 3.0f, 220);

    DotResult r = dot_detect(img, W, H, 180);
    free(img);

    check(r.valid,                   "center_dot/valid");
    check(near(r.cx, true_cx, 0.5f), "center_dot/cx");
    check(near(r.cy, true_cy, 0.5f), "center_dot/cy");
    check(r.count > 1,               "center_dot/count>1");
}

static void test_dot_detect_no_dot() {
    const int W = 96, H = 96;
    uint8_t* img = (uint8_t*)calloc(W * H, 1);
    // uniform dim image — nothing above threshold
    memset(img, 50, W * H);

    DotResult r = dot_detect(img, W, H, 180);
    free(img);

    check(!r.valid, "no_dot/not_valid");
}

static void test_dot_detect_dim_dot() {
    // Dot exists but peak is just below threshold — should return invalid
    const int W = 96, H = 96;
    uint8_t* img = make_dot_image(W, H, 30.0f, 30.0f, 4.0f, 170);

    DotResult r = dot_detect(img, W, H, 180);
    free(img);

    check(!r.valid, "dim_dot/not_valid");
}

static void test_dot_detect_rejects_spurious() {
    // Two bright spots: main laser dot at (48,48) amp=220, spurious at (10,10) amp=200
    const int W = 96, H = 96;
    uint8_t* img = make_dot_image(W, H, 48.0f, 48.0f, 3.0f, 220);
    // Add spurious — lower amplitude so peak finder picks the main dot
    uint8_t* spurious = make_dot_image(W, H, 10.0f, 10.0f, 3.0f, 195);
    for (int i = 0; i < W * H; ++i)
        img[i] = img[i] > spurious[i] ? img[i] : spurious[i];
    free(spurious);

    DotResult r = dot_detect(img, W, H, 180);
    free(img);

    // Centroid should still be close to main dot (48,48), not dragged to (10,10)
    check(r.valid,                   "reject_spurious/valid");
    check(near(r.cx, 48.0f, 2.0f),  "reject_spurious/cx_near_main");
    check(near(r.cy, 48.0f, 2.0f),  "reject_spurious/cy_near_main");
}

// ── triangulation tests ──────────────────────────────────────────────────────

static void test_triang_zero_at_ref() {
    TriangConfig cfg = triang_default_config();
    cfg.ref_height_mm = 0.0f;
    triang_calibrate(&cfg, 50.0f);  // reference at cy=50

    float h = triang_height_linear(&cfg, 50.0f);
    check(near(h, 0.0f, 0.01f), "triang/zero_at_ref");
}

static void test_triang_linear_rise() {
    // With θ=30°, B=30mm, f=88px:
    //   scale = B / (f * tan θ) = 30 / (88 * 0.5774) ≈ 0.590 mm/px
    // A 10-pixel upward shift should give ≈ 5.9 mm rise.
    TriangConfig cfg = triang_default_config();
    cfg.ref_height_mm = 0.0f;
    triang_calibrate(&cfg, 60.0f);   // reference at cy=60

    float h = triang_height_linear(&cfg, 50.0f); // dot moved up 10 px
    float expected = 10.0f * 30.0f / (88.0f * tanf(30.0f * (float)M_PI / 180.0f));

    printf("  [info] triang_linear_rise: h=%.3f expected=%.3f\n", h, expected);
    check(near(h, expected, 0.05f), "triang/linear_rise");
    check(h > 0.0f,                 "triang/positive_for_rise");
}

static void test_triang_direction() {
    TriangConfig cfg = triang_default_config();
    triang_calibrate(&cfg, 60.0f);

    // Surface moves toward camera → dot moves up in frame (cy decreases)
    float h_up   = triang_height_linear(&cfg, 50.0f);  // cy went down from 60 → 50
    // Surface moves away → dot moves down (cy increases)
    float h_down = triang_height_linear(&cfg, 70.0f);

    check(h_up   > 0.0f, "triang/up_positive");
    check(h_down < 0.0f, "triang/down_negative");
}

static void test_triang_nonlinear_small_delta() {
    // For small Δz, linear and non-linear should agree within 1%
    TriangConfig cfg = triang_default_config();
    cfg.ref_height_mm = 100.0f;
    triang_calibrate(&cfg, 60.0f);

    float h_lin = triang_height_linear(&cfg, 59.0f);
    float h_nl  = triang_height(&cfg, 59.0f);
    float diff  = fabsf(h_lin - h_nl);
    float tol   = 0.01f * fabsf(h_lin);  // 1 %
    printf("  [info] nl vs linear: lin=%.4f nl=%.4f diff=%.5f tol=%.5f\n",
           h_lin, h_nl, diff, tol);
    check(diff <= tol + 1e-6f, "triang/nonlinear_matches_linear_small");
}

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
    printf("=== DoughCV native tests ===\n");

    printf("\n-- dot_detect --\n");
    test_dot_detect_center();
    test_dot_detect_no_dot();
    test_dot_detect_dim_dot();
    test_dot_detect_rejects_spurious();

    printf("\n-- triangulation --\n");
    test_triang_zero_at_ref();
    test_triang_linear_rise();
    test_triang_direction();
    test_triang_nonlinear_small_delta();

    printf("\n=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

// Native unit tests — run with: pio test -e native_test

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#ifndef LASER_ANGLE_DEG
#define LASER_ANGLE_DEG    30.0f
#define CAMERA_BASELINE_MM 30.0f
#define FOCAL_LENGTH_PX    88.0f
#define DETECT_THRESHOLD   180
#endif

#include "../src/line_detect.h"
#include "../src/line_detect.cpp"
#include "../src/triangulation.h"

static int g_pass = 0, g_fail = 0;

static void check(bool ok, const char *name) {
    if (ok) { printf("  PASS  %s\n", name); ++g_pass; }
    else     { printf("  FAIL  %s\n", name); ++g_fail; }
}
static bool near(float a, float b, float tol = 0.5f) { return fabsf(a - b) <= tol; }

// Make a W×H grayscale image with a horizontal laser line at y=line_y, sigma pixels wide.
static uint8_t *make_line_image(int W, int H, float line_y, float sigma, uint8_t amp) {
    uint8_t *img = (uint8_t *)calloc(W * H, 1);
    for (int y = 0; y < H; ++y) {
        float dy = (float)y - line_y;
        uint8_t v = (uint8_t)(amp * expf(-dy*dy / (2.0f*sigma*sigma)));
        for (int x = 0; x < W; ++x)
            img[y * W + x] = v;
    }
    return img;
}

// ── line_detect tests ──────────────────────────────────────────────────────────

static void test_line_center() {
    const int W = 160, H = 96;
    float true_y = 48.0f;
    uint8_t *img = make_line_image(W, H, true_y, 3.0f, 220);

    LineProfile p = line_detect(img, W, H, 180);
    free(img);

    check(p.n_valid > LINE_COLS / 2, "line_center/most_cols_valid");
    // Check a few interior columns
    bool ok_cy = true;
    for (int ci = 2; ci < LINE_COLS - 2; ci++)
        if (p.valid[ci] && fabsf(p.cy[ci] - true_y) > 0.5f) ok_cy = false;
    check(ok_cy, "line_center/cy_accurate");
}

static void test_line_no_line() {
    const int W = 160, H = 96;
    uint8_t *img = (uint8_t *)calloc(W * H, 1);
    memset(img, 50, W * H);
    LineProfile p = line_detect(img, W, H, 180);
    free(img);
    check(p.n_valid == 0, "no_line/zero_valid");
}

static void test_line_dim() {
    const int W = 160, H = 96;
    uint8_t *img = make_line_image(W, H, 40.0f, 3.0f, 170);
    LineProfile p = line_detect(img, W, H, 180);
    free(img);
    check(p.n_valid == 0, "dim_line/not_detected");
}

static void test_line_shifted() {
    // After dough rises, line moves up (lower y)
    const int W = 160, H = 96;
    float cal_y = 60.0f, rise_y = 45.0f;
    uint8_t *cal = make_line_image(W, H, cal_y,  3.0f, 220);
    uint8_t *cur = make_line_image(W, H, rise_y, 3.0f, 220);

    LineProfile cp = line_detect(cal, W, H, 180);
    LineProfile pp = line_detect(cur, W, H, 180);
    free(cal); free(cur);

    bool all_positive = true;
    for (int ci = 2; ci < LINE_COLS - 2; ci++) {
        if (!cp.valid[ci] || !pp.valid[ci]) continue;
        float disp = cp.cy[ci] - pp.cy[ci];
        if (disp < 0) all_positive = false;
    }
    check(all_positive, "line_shift/displacement_positive");

    // Check expected displacement at centre column
    int mid = LINE_COLS / 2;
    float disp = cp.cy[mid] - pp.cy[mid];
    check(near(disp, cal_y - rise_y, 1.0f), "line_shift/magnitude");
}

// ── triangulation tests (unchanged from dot version) ─────────────────────────

static void test_triang_zero_at_ref() {
    TriangConfig cfg = triang_default_config();
    triang_calibrate(&cfg, 50.0f);
    check(near(triang_height_linear(&cfg, 50.0f), 0.0f, 0.01f), "triang/zero_at_ref");
}

static void test_triang_linear_rise() {
    TriangConfig cfg = triang_default_config();
    triang_calibrate(&cfg, 60.0f);
    float h = triang_height_linear(&cfg, 50.0f);
    float expected = 10.0f * 30.0f / (88.0f * tanf(30.0f * (float)M_PI / 180.0f));
    printf("  [info] triang_linear_rise: h=%.3f expected=%.3f\n", h, expected);
    check(near(h, expected, 0.05f), "triang/linear_rise");
    check(h > 0.0f, "triang/positive_for_rise");
}

static void test_triang_direction() {
    TriangConfig cfg = triang_default_config();
    triang_calibrate(&cfg, 60.0f);
    check(triang_height_linear(&cfg, 50.0f) > 0.0f, "triang/up_positive");
    check(triang_height_linear(&cfg, 70.0f) < 0.0f, "triang/down_negative");
}

int main() {
    printf("=== DoughCV native tests (line-laser) ===\n");

    printf("\n-- line_detect --\n");
    test_line_center();
    test_line_no_line();
    test_line_dim();
    test_line_shifted();

    printf("\n-- triangulation --\n");
    test_triang_zero_at_ref();
    test_triang_linear_rise();
    test_triang_direction();

    printf("\n=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

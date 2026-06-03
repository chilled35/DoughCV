#pragma once
#include <stdint.h>
#include <stdbool.h>

// Number of column samples across the frame.
#define LINE_COLS 40

// Per-column line profile.
struct LineProfile {
    float cy[LINE_COLS];       // intensity-weighted centroid y per column
    bool  valid[LINE_COLS];    // true if line found in that column
    int   n_valid;             // number of valid columns
    float peak;                // brightest pixel found (for threshold tuning)
};

// Scan a grayscale image and return the per-column centroid of the horizontal
// laser line.  For each sampled column the centroid y of all pixels above
// `threshold` is computed.  Columns with no bright pixels are marked invalid.
//
// For a colour image (RGB565) pass the red channel extracted by the caller,
// or use a grayscale frame directly.
LineProfile line_detect(const uint8_t *buf, int width, int height, uint8_t threshold);

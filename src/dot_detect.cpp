#include "dot_detect.h"
#include <string.h>
#include <math.h>

// Two-pass algorithm:
//  1. Find the global peak pixel.
//  2. Intensity-weighted centroid of all pixels above `threshold` that are
//     within MAX_CLUSTER_RADIUS pixels of the peak.
//     This rejects stray bright pixels elsewhere in the frame (e.g. window
//     glare, LED reflections) while still capturing the full Gaussian spread
//     of the laser dot for sub-pixel centroid accuracy.

static constexpr float MAX_CLUSTER_RADIUS = 12.0f;  // pixels; tune if dot is larger

DotResult dot_detect(const uint8_t* buf, int width, int height, uint8_t threshold) {
    DotResult r{};

    // Pass 1: find peak
    uint8_t peak_val = 0;
    int peak_x = 0, peak_y = 0;
    for (int y = 0; y < height; ++y) {
        const uint8_t* row = buf + y * width;
        for (int x = 0; x < width; ++x) {
            if (row[x] > peak_val) {
                peak_val = row[x];
                peak_x   = x;
                peak_y   = y;
            }
        }
    }

    r.peak = peak_val;

    if (peak_val <= threshold) {
        r.valid = false;
        return r;
    }

    // Pass 2: weighted centroid around peak
    float sum_w = 0.0f, sum_wx = 0.0f, sum_wy = 0.0f;
    int   count = 0;
    float r2_max = MAX_CLUSTER_RADIUS * MAX_CLUSTER_RADIUS;

    for (int y = 0; y < height; ++y) {
        float dy = (float)(y - peak_y);
        if (dy * dy > r2_max) continue;  // quick row skip

        const uint8_t* row = buf + y * width;
        for (int x = 0; x < width; ++x) {
            if (row[x] <= threshold) continue;
            float dx = (float)(x - peak_x);
            if (dx * dx + dy * dy > r2_max) continue;

            float w = (float)row[x];
            sum_w  += w;
            sum_wx += w * (float)x;
            sum_wy += w * (float)y;
            ++count;
        }
    }

    if (sum_w <= 0.0f || count == 0) {
        r.valid = false;
        return r;
    }

    r.cx    = sum_wx / sum_w;
    r.cy    = sum_wy / sum_w;
    r.count = count;
    r.valid = true;
    return r;
}

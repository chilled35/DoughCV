#pragma once
#include <stdint.h>
#include <stddef.h>

// Result of one dot-detection pass.
struct DotResult {
    float cx;       // centroid x (pixels, fractional)
    float cy;       // centroid y (pixels, fractional)
    float peak;     // brightest pixel value found (0–255)
    int   count;    // number of pixels above threshold that contributed
    bool  valid;    // false if no dot found (all pixels below threshold)
};

// Scan a grayscale image and return the intensity-weighted centroid of all
// pixels whose value exceeds `threshold`.  Only the brightest connected
// cluster is returned to reject ambient hot spots.
//
// buf    : row-major grayscale bytes
// width  : image width in pixels
// height : image height in pixels
// threshold : 0–255, pixels at or below this value are ignored
DotResult dot_detect(const uint8_t* buf, int width, int height, uint8_t threshold);

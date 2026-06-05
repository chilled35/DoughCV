#include "line_detect.h"
#include <string.h>

LineProfile line_detect(const uint8_t *buf, int width, int height, uint8_t threshold) {
    LineProfile prof{};
    const int step = width / LINE_COLS;

    for (int ci = 0; ci < LINE_COLS; ci++) {
        int x = ci * step + step / 2;
        float sum_w = 0.0f, sum_wy = 0.0f;
        uint8_t col_peak = 0;

        for (int y = 1; y < height - 1; y++) {
            uint8_t v = buf[y * width + x];
            if (v > col_peak) col_peak = v;
            if (v >= threshold) {
                sum_w  += v;
                sum_wy += v * (float)y;
            }
        }

        if (col_peak > prof.peak) prof.peak = col_peak;

        if (sum_w > 0.0f) {
            prof.cy[ci]    = sum_wy / sum_w;
            prof.valid[ci] = true;
            prof.n_valid++;
        }
    }
    return prof;
}

#pragma once
#include <stdint.h>
#include <stddef.h>

// XIAO ESP32-S3 Sense OV2640 pin map
// (matches Seeed's board definition)
#define CAM_PIN_PWDN  -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK   10
#define CAM_PIN_SIOD   40
#define CAM_PIN_SIOC   39
#define CAM_PIN_D7     48
#define CAM_PIN_D6     11
#define CAM_PIN_D5     12
#define CAM_PIN_D4     14
#define CAM_PIN_D3     16
#define CAM_PIN_D2     18
#define CAM_PIN_D1     17
#define CAM_PIN_D0     15
#define CAM_PIN_VSYNC  38
#define CAM_PIN_HREF   47
#define CAM_PIN_PCLK   13

// Capture resolution — small enough to keep latency low, large enough for
// sub-pixel centroid accuracy.  96×96 grayscale @ ~10 fps on S3.
#define CAM_WIDTH  96
#define CAM_HEIGHT 96

struct Frame {
    const uint8_t* buf;
    size_t         len;
    int            width;
    int            height;
};

// Returns true on success.  Call once from setup().
bool camera_init();

// Capture a single grayscale frame.  The returned buf pointer is owned by
// the camera driver; call camera_release() before calling again.
bool camera_capture(Frame* out);

// Return the frame buffer to the driver.
void camera_release(const Frame* f);

#include "camera.h"
#ifndef NATIVE_TEST

#include "esp_camera.h"
#include "Arduino.h"

static camera_fb_t* s_fb = nullptr;

bool camera_init() {
    camera_config_t cfg = {};
    cfg.pin_pwdn   = CAM_PIN_PWDN;
    cfg.pin_reset  = CAM_PIN_RESET;
    cfg.pin_xclk   = CAM_PIN_XCLK;
    cfg.pin_sscb_sda = CAM_PIN_SIOD;
    cfg.pin_sscb_scl = CAM_PIN_SIOC;
    cfg.pin_d7     = CAM_PIN_D7;
    cfg.pin_d6     = CAM_PIN_D6;
    cfg.pin_d5     = CAM_PIN_D5;
    cfg.pin_d4     = CAM_PIN_D4;
    cfg.pin_d3     = CAM_PIN_D3;
    cfg.pin_d2     = CAM_PIN_D2;
    cfg.pin_d1     = CAM_PIN_D1;
    cfg.pin_d0     = CAM_PIN_D0;
    cfg.pin_vsync  = CAM_PIN_VSYNC;
    cfg.pin_href   = CAM_PIN_HREF;
    cfg.pin_pclk   = CAM_PIN_PCLK;

    cfg.xclk_freq_hz = 20000000;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.pixel_format = PIXFORMAT_GRAYSCALE;
    cfg.frame_size   = FRAMESIZE_96X96;
    cfg.fb_count     = 2;      // double-buffer; uses PSRAM
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[camera] init failed: 0x%x\n", err);
        return false;
    }

    // Lower brightness slightly so laser dot stands out more
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, -1);
        s->set_exposure_ctrl(s, 0);    // manual exposure
        s->set_aec_value(s, 100);      // short shutter — tune for your room light
        s->set_gain_ctrl(s, 0);        // manual gain
        s->set_agc_gain(s, 0);
        s->set_awb_gain(s, 0);
        s->set_whitebal(s, 0);
        s->set_lenc(s, 0);
        s->set_raw_gma(s, 0);
    }
    return true;
}

bool camera_capture(Frame* out) {
    if (s_fb) esp_camera_fb_return(s_fb);  // shouldn't happen, but be safe
    s_fb = esp_camera_fb_get();
    if (!s_fb) return false;
    out->buf    = s_fb->buf;
    out->len    = s_fb->len;
    out->width  = s_fb->width;
    out->height = s_fb->height;
    return true;
}

void camera_release(const Frame* /*f*/) {
    if (s_fb) {
        esp_camera_fb_return(s_fb);
        s_fb = nullptr;
    }
}

#endif // NATIVE_TEST

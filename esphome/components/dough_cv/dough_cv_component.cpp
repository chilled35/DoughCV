#include "dough_cv_component.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "img_converters.h"

#include <algorithm>

namespace esphome {
namespace dough_cv {

static const char *TAG = "dough_cv";

// ── Lifecycle ──────────────────────────────────────────────────────────────────

void DoughCVComponent::setup() {
  pref_ = global_preferences->make_preference<CalData>(fnv1_hash(CAL_NVS_KEY));
  load_cal_();

  if (camera_ == nullptr) {
    ESP_LOGE(TAG, "No camera set — check esp32_camera_id in dough_cv config");
    mark_failed(); return;
  }
  camera_->add_listener(this);

  decode_buf_len_ = 320 * 240 * 3;
  decode_buf_ = (uint8_t *)heap_caps_malloc(decode_buf_len_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!decode_buf_)
    decode_buf_ = (uint8_t *)malloc(decode_buf_len_);
  if (!decode_buf_) {
    ESP_LOGE(TAG, "No memory for decode buffer");
    mark_failed(); return;
  }

  ESP_LOGI(TAG, "DoughCV ready | angle=%.1f° height=%.0fmm threshold=%d calibrated=%s",
           laser_angle_deg_, mount_height_mm_, threshold_,
           calibrated_ ? "yes" : "no");
}

void DoughCVComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "DoughCV:");
  ESP_LOGCONFIG(TAG, "  Laser angle:      %.1f°",  laser_angle_deg_);
  ESP_LOGCONFIG(TAG, "  Mount height:     %.0f mm", mount_height_mm_);
  ESP_LOGCONFIG(TAG, "  Line threshold:   %d",      threshold_);
  ESP_LOGCONFIG(TAG, "  Process interval: %u ms",   interval_ms_);
  ESP_LOGCONFIG(TAG, "  Scale factor:     %.3f",    scale_factor_);
  ESP_LOGCONFIG(TAG, "  Calibrated:       %s",      calibrated_ ? "yes" : "no");
}

// ── Frame processing ───────────────────────────────────────────────────────────

void DoughCVComponent::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
  uint32_t now = millis();
  bool do_process   = (now - last_ms_) >= interval_ms_;
  bool do_calibrate = capture_next_;
  if (!do_process && !do_calibrate) return;

  auto *esp_img = static_cast<esp32_camera::ESP32CameraImage *>(image.get());
  camera_fb_t *fb = esp_img->get_raw_buffer();
  if (!fb || !decode_buf_) return;

  const uint8_t *pixel_buf = nullptr;
  Fmt fmt;

  if (fb->format == PIXFORMAT_RGB565) {
    pixel_buf = (const uint8_t *)fb->buf; fmt = Fmt::RGB565;
  } else if (fb->format == PIXFORMAT_GRAYSCALE) {
    pixel_buf = (const uint8_t *)fb->buf; fmt = Fmt::GRAYSCALE;
  } else if (fb->format == PIXFORMAT_JPEG) {
    if (!fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, decode_buf_)) {
      ESP_LOGW(TAG, "JPEG decode failed"); return;
    }
    pixel_buf = decode_buf_; fmt = Fmt::RGB888;
  } else {
    ESP_LOGW(TAG, "Unsupported frame format %d", fb->format); return;
  }

  LineProfile prof = find_line_(pixel_buf, fb->width, fb->height, fmt);

  ESP_LOGD(TAG, "Line: %d/%d cols valid", prof.n_valid, LINE_COLS);

  // ── Calibration ─────────────────────────────────────────────────────────────
  if (do_calibrate) {
    capture_next_ = false;
    if (prof.n_valid >= LINE_COLS / 4) {
      memcpy(cal_y_,     prof.y,     sizeof(cal_y_));
      memcpy(cal_valid_, prof.valid, sizeof(cal_valid_));
      calibrated_ = true;
      save_cal_();
      ESP_LOGI(TAG, "Calibration captured: %d/%d cols", prof.n_valid, LINE_COLS);
    } else {
      ESP_LOGW(TAG, "Calibration failed: only %d valid cols — check laser & threshold",
               prof.n_valid);
    }
  }

  // ── Measurement ─────────────────────────────────────────────────────────────
  if (do_process) {
    last_ms_ = now;

    float cov = 100.0f * prof.n_valid / LINE_COLS;
    if (line_cov_) line_cov_->publish_state(cov);

    if (calibrated_ && prof.n_valid >= LINE_COLS / 4) {
      float rise = rise_height_mm_(prof, fb->width);
      float foot = footprint_mm_  (prof, fb->width);
      if (rise_height_) rise_height_->publish_state(rise);
      if (footprint_)   footprint_->publish_state(foot);
      ESP_LOGD(TAG, "Rise=%.1fmm Footprint=%.1fmm Coverage=%.0f%%", rise, foot, cov);
    }
  }
}

// ── Brightness helper ─────────────────────────────────────────────────────────

uint8_t DoughCVComponent::brightness_(const uint8_t *buf, int x, int y, int w, Fmt fmt) {
  if (fmt == Fmt::RGB565) {
    int i = (y * w + x) * 2;
    uint16_t p = ((uint16_t)buf[i] << 8) | buf[i + 1];
    uint8_t r = (uint8_t)((p >> 11) << 3);
    uint8_t g = (uint8_t)(((p >> 5) & 0x3F) << 2);
    // Laser is red — return 0 if green dominates (ambient light rejection)
    return (r > (uint8_t)(g * 1.3f)) ? r : 0;
  } else if (fmt == Fmt::RGB888) {
    int i = (y * w + x) * 3;
    uint8_t r = buf[i], g = buf[i + 1];
    return (r > (uint8_t)(g * 1.3f)) ? r : 0;
  } else {
    return buf[y * w + x];
  }
}

// ── Line detection ────────────────────────────────────────────────────────────
//
// Scan each sampled column for the intensity-weighted centroid y of the
// horizontal laser line.  The vertical line of the crosshair shows up in
// one or two columns as a vertical streak — it doesn't contribute a
// consistent centroid across columns, so it's naturally ignored.

LineProfile DoughCVComponent::find_line_(const uint8_t *buf, int w, int h, Fmt fmt) {
  LineProfile prof{};
  const int step = w / LINE_COLS;

  for (int ci = 0; ci < LINE_COLS; ci++) {
    int x = ci * step + step / 2;
    float sum_w = 0.0f, sum_wy = 0.0f;

    for (int y = 1; y < h - 1; y++) {
      uint8_t v = brightness_(buf, x, y, w, fmt);
      if (v >= threshold_) {
        sum_w  += v;
        sum_wy += v * (float)y;
      }
    }

    if (sum_w > 0.0f) {
      prof.y[ci]     = sum_wy / sum_w;
      prof.valid[ci] = true;
      prof.n_valid++;
    }
  }
  return prof;
}

// ── Triangulation: rise height ────────────────────────────────────────────────
//
// For each column, the vertical displacement of the laser line from its
// calibration position gives the local surface height at that column.
// We report the maximum (dome peak).

float DoughCVComponent::rise_height_mm_(const LineProfile &prof, int frame_w) {
  const float hfov_half_rad = 29.0f * (float)M_PI / 180.0f;
  const float focal_px  = (frame_w * 0.5f) / tanf(hfov_half_rad);
  const float tan_theta = tanf(laser_angle_deg_ * (float)M_PI / 180.0f);

  float max_h = 0.0f;
  for (int ci = 0; ci < LINE_COLS; ci++) {
    if (!prof.valid[ci] || !cal_valid_[ci]) continue;
    // Upward displacement in image (smaller y = higher in frame = surface rose)
    float disp_px = cal_y_[ci] - prof.y[ci];
    if (disp_px < 0.5f) continue;
    float h = disp_px * mount_height_mm_ / (focal_px * tan_theta) * scale_factor_;
    if (h > max_h) max_h = h;
  }
  return max_h;
}

// ── Triangulation: footprint ──────────────────────────────────────────────────
//
// Width in mm of the region where the line has risen measurably from baseline.

float DoughCVComponent::footprint_mm_(const LineProfile &prof, int frame_w) {
  const float hfov_half_rad = 29.0f * (float)M_PI / 180.0f;
  const float mm_per_px = mount_height_mm_ * tanf(hfov_half_rad) / (frame_w * 0.5f);
  const float px_per_col = (float)frame_w / LINE_COLS;

  int first = -1, last = -1;
  for (int ci = 0; ci < LINE_COLS; ci++) {
    if (!prof.valid[ci] || !cal_valid_[ci]) continue;
    if (cal_y_[ci] - prof.y[ci] < 1.5f) continue;
    if (first < 0) first = ci;
    last = ci;
  }
  if (first < 0) return 0.0f;

  return (last - first) * px_per_col * mm_per_px * scale_factor_;
}

// ── Calibration persistence ───────────────────────────────────────────────────

void DoughCVComponent::clear_calibration() {
  calibrated_ = false;
  memset(cal_y_,     0, sizeof(cal_y_));
  memset(cal_valid_, 0, sizeof(cal_valid_));
  CalData empty{};
  pref_.save(&empty);
  ESP_LOGI(TAG, "Calibration cleared");
}

void DoughCVComponent::save_cal_() {
  CalData d{};
  memcpy(d.y,     cal_y_,     sizeof(d.y));
  memcpy(d.valid, cal_valid_, sizeof(d.valid));
  pref_.save(&d);
}

void DoughCVComponent::load_cal_() {
  CalData d{};
  if (pref_.load(&d)) {
    memcpy(cal_y_,     d.y,     sizeof(cal_y_));
    memcpy(cal_valid_, d.valid, sizeof(cal_valid_));
    int n = 0;
    for (int i = 0; i < LINE_COLS; i++) if (cal_valid_[i]) n++;
    if (n >= LINE_COLS / 4) {
      calibrated_ = true;
      ESP_LOGI(TAG, "Calibration loaded (%d/%d cols)", n, LINE_COLS);
    }
  }
}

}  // namespace dough_cv
}  // namespace esphome

#include "dough_cv_component.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"

#include <algorithm>

namespace esphome {
namespace dough_cv {

static const char *TAG = "dough_cv";

// ── Lifecycle ─────────────────────────────────────────────────────────────────

void DoughCVComponent::setup() {
  pref_ = global_preferences->make_preference<CalData>(fnv1_hash("dough_cv_v1"));
  load_cal_();

  if (camera_ == nullptr) {
    ESP_LOGE(TAG, "No camera set — add esp32_camera_id to dough_cv config");
    mark_failed();
    return;
  }

  camera_->add_listener(this);

  ESP_LOGI(TAG, "DoughCV ready | angle=%.1f° height=%.0fmm threshold=%d calibrated=%s",
           laser_angle_deg_, mount_height_mm_, dot_threshold_,
           calibrated_ ? "yes" : "no");
}

void DoughCVComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "DoughCV:");
  ESP_LOGCONFIG(TAG, "  Laser angle:     %.1f°",  laser_angle_deg_);
  ESP_LOGCONFIG(TAG, "  Mount height:    %.0f mm", mount_height_mm_);
  ESP_LOGCONFIG(TAG, "  Dot threshold:   %d",      dot_threshold_);
  ESP_LOGCONFIG(TAG, "  Process interval: %u ms",  interval_ms_);
  ESP_LOGCONFIG(TAG, "  Scale factor:    %.3f",    scale_factor_);
  ESP_LOGCONFIG(TAG, "  Calibrated:      %s (%d dots)",
                calibrated_ ? "yes" : "no", (int)cal_dots_.size());
}

// ── Frame processing ──────────────────────────────────────────────────────────

void DoughCVComponent::on_camera_image(const std::shared_ptr<camera::CameraImage> &image) {
  uint32_t now = millis();
  bool do_process   = (now - last_ms_) >= interval_ms_;
  bool do_calibrate = capture_next_;

  if (!do_process && !do_calibrate) return;

  // Downcast to get the raw ESP32 frame buffer
  auto *esp_img = static_cast<esp32_camera::ESP32CameraImage *>(image.get());
  camera_fb_t *fb = esp_img->get_raw_buffer();
  if (!fb) return;

  if (fb->format != PIXFORMAT_RGB565 && fb->format != PIXFORMAT_GRAYSCALE) {
    ESP_LOGW(TAG, "Frame format %d is not RGB565/GRAYSCALE — cannot process. "
             "Remove jpeg_quality from esp32_camera config.", fb->format);
    return;
  }

  auto dots = find_dots_((const uint8_t *)fb->buf, fb->width, fb->height, fb->format);
  ESP_LOGD(TAG, "%d dots found in %dx%d frame", (int)dots.size(), fb->width, fb->height);

  // ── Calibration capture ───────────────────────────────────────────────────
  if (do_calibrate) {
    capture_next_ = false;
    if ((int)dots.size() >= 3) {
      cal_dots_    = dots;
      calibrated_  = true;
      save_cal_();
      ESP_LOGI(TAG, "Calibration captured: %d dots", (int)dots.size());
    } else {
      ESP_LOGW(TAG, "Calibration failed: only %d dots (need ≥3) — check lighting & threshold",
               (int)dots.size());
    }
  }

  // ── Measurement ───────────────────────────────────────────────────────────
  if (do_process) {
    last_ms_ = now;

    if (dot_count_) dot_count_->publish_state((float)dots.size());

    if (calibrated_ && (int)dots.size() >= 3) {
      float rise = rise_height_mm_(dots, fb->width);
      float foot = footprint_mm_  (dots, fb->width);
      if (rise_height_) rise_height_->publish_state(rise);
      if (footprint_)   footprint_->publish_state(foot);
      ESP_LOGD(TAG, "Rise=%.1fmm Footprint=%.1fmm", rise, foot);
    }
  }
}

// ── Dot detection (red-channel threshold + BFS blob finder) ──────────────────

std::vector<DotPos> DoughCVComponent::find_dots_(const uint8_t *buf, int w, int h,
                                                   pixformat_t fmt) {
  std::vector<bool> vis(w * h, false);
  std::vector<DotPos> dots;
  dots.reserve(MAX_DOTS);

  const bool is_rgb565 = (fmt == PIXFORMAT_RGB565);

  // For RGB565: check red channel dominates green (laser dot colour rejection).
  // For grayscale: simple brightness threshold — no colour info available.
  auto is_dot = [&](int x, int y) -> bool {
    if (is_rgb565) {
      int i = (y * w + x) * 2;
      uint16_t p = ((uint16_t)buf[i] << 8) | buf[i + 1];
      uint8_t r = (uint8_t)((p >> 11) << 3);
      uint8_t g = (uint8_t)(((p >> 5) & 0x3F) << 2);
      return r >= dot_threshold_ && r > (uint8_t)(g * 1.5f);
    } else {
      // Grayscale: assume ambient is dark and laser is bright
      return buf[y * w + x] >= dot_threshold_;
    }
  };

  // 1-pixel border excluded to avoid edge artefacts
  for (int y = 1; y < h - 1 && (int)dots.size() < MAX_DOTS; y++) {
    for (int x = 1; x < w - 1; x++) {
      if (vis[y * w + x] || !is_dot(x, y)) continue;

      // BFS flood fill
      std::vector<std::pair<int,int>> blob;
      std::vector<std::pair<int,int>> queue;
      queue.push_back({x, y});
      vis[y * w + x] = true;

      while (!queue.empty() && (int)blob.size() < MAX_BLOB_PX) {
        auto [cx, cy] = queue.back(); queue.pop_back();
        blob.push_back({cx, cy});
        const int dx[] = {1, -1, 0,  0};
        const int dy[] = {0,  0, 1, -1};
        for (int d = 0; d < 4; d++) {
          int nx = cx + dx[d], ny = cy + dy[d];
          if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
          int ni = ny * w + nx;
          if (!vis[ni] && is_dot(nx, ny)) {
            vis[ni] = true;
            queue.push_back({nx, ny});
          }
        }
      }

      if ((int)blob.size() >= MIN_BLOB_PX && (int)blob.size() < MAX_BLOB_PX) {
        float cx = 0, cy = 0;
        for (auto &[bx, by] : blob) { cx += bx; cy += by; }
        cx /= blob.size(); cy /= blob.size();
        dots.push_back({cx, cy});
      }
    }
  }

  return dots;
}

// ── Triangulation: rise height ────────────────────────────────────────────────
//
// Geometry (laser offset in the Y direction of the image):
//   Camera at height H looking straight down.
//   Laser at height H, angled θ from vertical toward the scene centre.
//   When surface rises by h at a point, the dot shifts toward the laser
//   by Δ_world = h * tan(θ) in the ground plane.
//   In the camera image: Δ_px = Δ_world * focal_px / H   (small-angle approx)
//   → h = Δ_px * H / (focal_px * tan(θ)) * scale_factor
//
// focal_px estimated from HFOV: focal_px = (frame_w/2) / tan(HFOV/2)
// OV2640 QQVGA: HFOV ≈ 58°

float DoughCVComponent::rise_height_mm_(const std::vector<DotPos> &dots, int frame_w) {
  const float hfov_half_rad = 29.0f * M_PI / 180.0f;
  const float focal_px  = (frame_w * 0.5f) / tanf(hfov_half_rad);
  const float tan_theta = tanf(laser_angle_deg_ * M_PI / 180.0f);

  float max_h = 0.0f;

  for (const auto &cal : cal_dots_) {
    // Find nearest current dot
    float best_d = MAX_MATCH_PX;
    const DotPos *match = nullptr;
    for (const auto &dot : dots) {
      float d = hypotf(dot.x - cal.x, dot.y - cal.y);
      if (d < best_d) { best_d = d; match = &dot; }
    }
    if (!match) continue;

    // Positive displacement = dot moved toward laser = surface rose
    float disp_px = cal.y - match->y;
    if (disp_px < 0.5f) continue;

    float h = disp_px * mount_height_mm_ / (focal_px * tan_theta) * scale_factor_;
    if (h > max_h) max_h = h;
  }

  return max_h;
}

// ── Triangulation: footprint ──────────────────────────────────────────────────
//
// Count calibration dots that have measurably risen (disp > 1.5px) and compute
// the bounding-box diagonal converted to mm.  This gives an estimate of the
// dough footprint diameter along the laser crosshair axes.

float DoughCVComponent::footprint_mm_(const std::vector<DotPos> &dots, int frame_w) {
  const float hfov_half_rad = 29.0f * M_PI / 180.0f;
  const float mm_per_px = mount_height_mm_ * tanf(hfov_half_rad) / (frame_w * 0.5f);

  float min_x = 1e9f, max_x = -1e9f;
  float min_y = 1e9f, max_y = -1e9f;
  bool any = false;

  for (const auto &cal : cal_dots_) {
    float best_d = MAX_MATCH_PX;
    const DotPos *match = nullptr;
    for (const auto &dot : dots) {
      float d = hypotf(dot.x - cal.x, dot.y - cal.y);
      if (d < best_d) { best_d = d; match = &dot; }
    }
    if (!match) continue;

    float disp = cal.y - match->y;
    if (disp < 1.5f) continue;  // Not meaningfully risen

    if (cal.x < min_x) min_x = cal.x;
    if (cal.x > max_x) max_x = cal.x;
    if (cal.y < min_y) min_y = cal.y;
    if (cal.y > max_y) max_y = cal.y;
    any = true;
  }

  if (!any) return 0.0f;

  float span_px = hypotf(max_x - min_x, max_y - min_y);
  return span_px * mm_per_px * scale_factor_;
}

// ── Calibration persistence (ESPHome NVS preferences) ────────────────────────

void DoughCVComponent::clear_calibration() {
  calibrated_ = false;
  cal_dots_.clear();
  CalData empty{};
  pref_.save(&empty);
  ESP_LOGI(TAG, "Calibration cleared");
}

void DoughCVComponent::save_cal_() {
  CalData d{};
  d.n = (uint8_t)std::min((int)cal_dots_.size(), MAX_DOTS);
  for (int i = 0; i < d.n; i++) { d.x[i] = cal_dots_[i].x; d.y[i] = cal_dots_[i].y; }
  pref_.save(&d);
  ESP_LOGD(TAG, "Calibration saved (%d dots)", d.n);
}

void DoughCVComponent::load_cal_() {
  CalData d{};
  if (pref_.load(&d) && d.n >= 3 && d.n <= MAX_DOTS) {
    cal_dots_.resize(d.n);
    for (int i = 0; i < d.n; i++) cal_dots_[i] = {d.x[i], d.y[i]};
    calibrated_ = true;
    ESP_LOGI(TAG, "Calibration loaded (%d dots)", d.n);
  }
}

}  // namespace dough_cv
}  // namespace esphome

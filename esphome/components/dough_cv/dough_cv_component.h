#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_camera/esp32_camera.h"

#include <vector>
#include <cmath>

namespace esphome {
namespace dough_cv {

static const int MAX_DOTS     = 32;   // maximum laser dots we track
static const int MIN_BLOB_PX  = 3;    // smallest valid dot blob (pixels)
static const int MAX_BLOB_PX  = 150;  // largest valid dot blob (pixels)
static const float MAX_MATCH_PX = 40.0f;  // max px distance to match a dot to its calibration ref

struct DotPos { float x, y; };

struct CalData {
  uint8_t n;
  float   x[MAX_DOTS];
  float   y[MAX_DOTS];
};

class DoughCVComponent : public Component {
 public:
  // ── Component lifecycle ────────────────────────────────────────────────────
  void setup()       override;
  void loop()        override {}
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  // ── Sensor registration ────────────────────────────────────────────────────
  void set_rise_height_sensor(sensor::Sensor *s) { rise_height_ = s; }
  void set_footprint_sensor(sensor::Sensor *s)   { footprint_  = s; }
  void set_dot_count_sensor(sensor::Sensor *s)   { dot_count_  = s; }

  // ── Configuration setters (called from generated code) ────────────────────
  void set_laser_angle_deg(float v)     { laser_angle_deg_ = v; }
  void set_mount_height_mm(float v)     { mount_height_mm_ = v; }
  void set_dot_threshold(uint8_t v)     { dot_threshold_   = v; }
  void set_process_interval_ms(uint32_t v) { interval_ms_  = v; }
  void set_scale_factor(float v)        { scale_factor_    = v; }

  // ── Public API (called from YAML lambdas) ──────────────────────────────────
  void capture_calibration()  { capture_next_ = true; }
  void clear_calibration();
  bool is_calibrated() const  { return calibrated_; }

 private:
  void on_frame_(std::shared_ptr<esp32_camera::CameraImage> img);
  std::vector<DotPos> find_dots_(const uint8_t *buf, int w, int h, pixformat_t fmt);
  float rise_height_mm_(const std::vector<DotPos> &dots, int frame_w);
  float footprint_mm_  (const std::vector<DotPos> &dots, int frame_w);
  void  save_cal_();
  void  load_cal_();

  sensor::Sensor *rise_height_{nullptr};
  sensor::Sensor *footprint_  {nullptr};
  sensor::Sensor *dot_count_  {nullptr};

  float    laser_angle_deg_{30.0f};
  float    mount_height_mm_{200.0f};
  uint8_t  dot_threshold_  {180};
  uint32_t interval_ms_    {2000};
  float    scale_factor_   {1.0f};

  std::vector<DotPos> cal_dots_;
  bool     calibrated_   {false};
  bool     capture_next_ {false};
  uint32_t last_ms_      {0};

  ESPPreferenceObject pref_;
};

}  // namespace dough_cv
}  // namespace esphome

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/esp32_camera/esp32_camera.h"

#include <vector>
#include <cmath>

namespace esphome {
namespace dough_cv {

// Number of column samples across the frame (every 4 px at 160 px wide).
// Stored in NVS — changing this requires bumping CAL_NVS_KEY.
static const int LINE_COLS = 40;

// Per-column line profile: intensity-weighted centroid y of the horizontal
// laser line, sampled at LINE_COLS evenly-spaced columns across the frame.
struct LineProfile {
  float y[LINE_COLS];
  bool  valid[LINE_COLS];
  int   n_valid{0};
};

struct CalData {
  float y[LINE_COLS];
  bool  valid[LINE_COLS];
};

static const char *CAL_NVS_KEY = "dough_cv_v2";  // bump if CalData layout changes

class DoughCVComponent : public Component, public camera::CameraListener {
 public:
  void setup()       override;
  void loop()        override {}
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_rise_height_sensor(sensor::Sensor *s) { rise_height_ = s; }
  void set_footprint_sensor(sensor::Sensor *s)   { footprint_   = s; }
  void set_dot_count_sensor(sensor::Sensor *s)   { line_cov_    = s; }  // repurposed: line coverage %

  void set_laser_angle_deg(float v)        { laser_angle_deg_ = v; }
  void set_mount_height_mm(float v)        { mount_height_mm_ = v; }
  void set_dot_threshold(uint8_t v)        { threshold_       = v; }
  void set_process_interval_ms(uint32_t v) { interval_ms_     = v; }
  void set_scale_factor(float v)           { scale_factor_    = v; }
  void set_camera(esp32_camera::ESP32Camera *cam) { camera_    = cam; }

  void capture_calibration()  { capture_next_ = true; }
  void clear_calibration();
  bool is_calibrated() const  { return calibrated_; }

  // CameraListener interface
  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;

 private:
  enum class Fmt { RGB565, GRAYSCALE, RGB888 };

  LineProfile find_line_(const uint8_t *buf, int w, int h, Fmt fmt);
  uint8_t     brightness_(const uint8_t *buf, int x, int y, int w, Fmt fmt);
  float rise_height_mm_(const LineProfile &prof, int frame_w);
  float footprint_mm_  (const LineProfile &prof, int frame_w);
  void  save_cal_();
  void  load_cal_();

  sensor::Sensor *rise_height_{nullptr};
  sensor::Sensor *footprint_  {nullptr};
  sensor::Sensor *line_cov_   {nullptr};

  float    laser_angle_deg_{30.0f};
  float    mount_height_mm_{200.0f};
  uint8_t  threshold_      {180};
  uint32_t interval_ms_    {2000};
  float    scale_factor_   {1.0f};

  esp32_camera::ESP32Camera *camera_{nullptr};
  uint8_t *decode_buf_{nullptr};
  size_t   decode_buf_len_{0};

  float cal_y_    [LINE_COLS]{};
  bool  cal_valid_[LINE_COLS]{};
  bool     calibrated_   {false};
  bool     capture_next_ {false};
  uint32_t last_ms_      {0};

  ESPPreferenceObject pref_;
};

}  // namespace dough_cv
}  // namespace esphome

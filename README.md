# DoughCV

Real-time dough-rise monitoring using structured-light computer vision on a **Seeed XIAO ESP32-S3 Sense**.

---

## Why not the VL53L8CX?

This project is a direct pivot from [TimeOfFlightDough](https://github.com/chilled35/TimeOfFlightDough), which used the STMicroelectronics VL53L8CX 8×8 Time-of-Flight sensor grid. That approach failed for three compounding reasons:

| Problem | Detail |
|---|---|
| **Multipath interference** | The rising dome reflects IR light along multiple paths simultaneously. The sensor blends them into a single noisy reading — ~19% per-pixel error on curved surfaces without correction algorithms. |
| **Specular/variable reflectance** | Dough changes surface texture during proof: dry flour → sticky → thin crust. The VL53L8CX depends on consistent Lambertian reflectance; dough doesn't provide it. |
| **Sparse resolution** | At 200mm mount height the 63° FoV gives ~20.5mm per zone. A 150mm boule spans only ~7 zones across the dome peak — far too coarse to profile a dome shape accurately. |

Peer-reviewed food-science literature confirms this: a direct comparison of ToF vs. structured light for food volume measurement (Blasco et al., *Food Control* 2013) found structured light significantly outperformed ToF on curved irregular objects, including foods with dome and sphere geometry.

---

## How DoughCV works

A **laser diffraction grating** (crosshair dot pattern, salvaged from a Bosch cross-line level) projects a regular grid of dots onto the dough surface from an oblique angle. An overhead **OV2640 camera** (built into the XIAO ESP32-S3 Sense) observes the dot positions.

```
      Laser (angled ~30° from vertical)
         \  ·  ·  ·  ·  ·   ← dot row projected across dough
          \
           \   Flat surface: dots at calibration positions
            \  Risen dome:   dots displaced toward laser
                             displacement ∝ rise height
```

When the dough rises, each dot shifts laterally in the camera frame. The shift magnitude at a known laser angle and mount height gives the surface height at that point via triangulation:

```
height_mm = displacement_px × mount_height_mm / (focal_px × tan(laser_angle))
```

This is immune to surface colour, texture, and reflectance — the laser wavelength is simply thresholded in the red channel. Two axes of dots (the crosshair) give height profiles along both X and Y, from which dome height and footprint diameter are extracted.

---

## Hardware

| Part | Notes |
|---|---|
| Seeed XIAO ESP32-S3 Sense | Built-in OV2640, 8MB PSRAM, 8MB Flash |
| Bosch cross-line laser level (disassembled) | 650nm red, Class 2 (<1mW after grating), crosshair diffraction grating |
| 3D-printed or cardboard mount | Camera overhead, laser at ~30° offset, ~200mm above surface |
| Proofing container with matte base | Dark or contrasting background improves dot detection |

---

## Software architecture

```
ESP32-S3 (ESPHome)
  └── esp32_camera  →  OV2640 @ QQVGA RGB565 @ ~1fps
  └── dough_cv      →  Dot detection → triangulation → publish sensors
        ├── sensor: rise_height_mm
        ├── sensor: footprint_mm
        └── sensor: dot_count

ESPHome web_server (port 80)
  ├── /events          SSE stream of sensor state changes
  ├── /camera/stream   MJPEG live view
  └── /camera/snapshot Single JPEG

Dashboard (browser → ESP32 direct)
  ├── Live MJPEG camera feed
  ├── Rise height time-series chart
  └── Footprint time-series chart
```

---

## Setup

### 1. Driver files (no external driver needed)

Unlike the VL53L8CX project, this component uses only the ESP32 camera driver built into ESPHome. No external SDK required.

### 2. ESPHome secrets

Create `esphome/secrets.yaml` (gitignored):

```yaml
wifi_ssid: "YourSSID"
wifi_password: "YourPassword"
ap_fallback_password: "doughcv-fallback"
api_encryption_key: "base64-key-here"  # generate with: openssl rand -base64 32
ota_password: "your-ota-password"
```

### 3. Flash

```bash
cd esphome
esphome run dough_cv.yaml
```

### 4. Calibration

1. Place the empty proofing container under the sensor with no dough.
2. Open the dashboard and confirm laser dots are visible in the camera feed.
3. Press **Capture Calibration** — the flat-surface dot positions are saved to NVS.
4. Place your dough and start monitoring.

### 5. Dashboard

Copy `dashboard/` to your HA `/config/www/dough_cv/` and visit:
`http://your-ha-ip/local/dough_cv/dough_cv_dashboard.html`

Or open `dashboard/dough_cv_dashboard.html` directly in a browser pointed at the ESP32.

---

## Geometry calibration

The default `scale_factor: 1.0` uses an estimated OV2640 focal length (~58° HFOV at QQVGA). For accurate mm readings, calibrate with a known-height object:

1. Place a 50mm block on the flat calibration surface.
2. Note the `rise_height_mm` reading.
3. Set `scale_factor: 50.0 / <reported_value>` in the YAML and reflash.

---

## Licence

MIT

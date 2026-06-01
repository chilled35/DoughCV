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

A **Bosch laser + crosshair diffraction grating** projects a grid of red dots onto the dough surface from an oblique angle. An overhead **OV2640 camera** (built into the XIAO ESP32-S3 Sense) observes the dot positions.

```
      Laser (angled ~30° from vertical)
         \  ·  ·  ·  ·  ·   ← crosshair dot row across dough
          \  ·  ·  ·  ·  ·
           \   Flat surface: dots at calibration positions
            \  Risen dome:   dots displaced toward laser
                             displacement ∝ rise height
```

When the dough rises, each dot shifts toward the laser in the camera frame. The shift magnitude gives surface height via triangulation (approx. for small rises):

```
height_mm ≈ displacement_px × mount_height_mm / (focal_px × tan(laser_angle))
```

This is immune to surface colour, texture, and reflectance — the laser wavelength is simply thresholded in the red channel. The crosshair gives dot rows along both X and Y axes, from which dome height and footprint diameter are extracted.

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

There are two independent firmware paths — use whichever suits your setup:

| Path | Use when |
|---|---|
| **ESPHome** (`esphome/`) | You want Home Assistant integration, OTA updates, and the web dashboard |
| **PlatformIO standalone** (`src/`) | You want to develop or test the CV pipeline directly, without ESPHome overhead |

---

## ESPHome setup

### 1. Create secrets file

Create `esphome/secrets.yaml` (gitignored):

```yaml
wifi_ssid: "YourSSID"
wifi_password: "YourPassword"
ap_fallback_password: "doughcv-fallback"
api_encryption_key: "base64-key-here"  # generate with: openssl rand -base64 32
ota_password: "your-ota-password"
```

### 2. Flash

```bash
cd esphome
esphome run dough_cv.yaml
```

### 3. Calibrate

1. Place the empty proofing container under the sensor with no dough.
2. Open the dashboard and confirm laser dots are visible in the camera feed.
3. Press **Capture Calibration** — the flat-surface dot positions are saved to NVS.
4. Place your dough and start monitoring.

### 4. Dashboard

Copy `dashboard/` to your HA `/config/www/dough_cv/` and visit:
`http://your-ha-ip/local/dough_cv/dough_cv_dashboard.html`

Or open `dashboard/dough_cv_dashboard.html` directly in a browser pointed at the ESP32 IP.

### Scale factor calibration (ESPHome)

The default `scale_factor: 1.0` uses an estimated OV2640 focal length (~58° HFOV at QQVGA). For accurate mm readings, calibrate with a known-height object:

1. Place a 50mm block on the calibration surface.
2. Note the `rise_height_mm` reading.
3. Set `scale_factor: 50.0 / <reported_value>` in `dough_cv.yaml` and reflash.

---

## PlatformIO standalone firmware

*For direct development and testing of the CV pipeline — no ESPHome required.*

### Build & flash

```bash
pip install platformio
pio run -e xiao_esp32s3_sense --target upload
pio device monitor
```

### Calibrate

With the dough at its starting height, type `cal` in the serial monitor.
Optionally provide the known height: `cal 0` (baseline) or `cal 50` (50 mm above reference).

### Watch it rise

Output every 500 ms:
```
[rise] 12.34 mm  (cy=44.21 peak=223 n=17)
```

### Tuning dot detection

Increase `DETECT_THRESHOLD` (default 180) if ambient light causes false detections; decrease if the dot appears dim. Live adjustment without reflashing: `thr 160`

Use `dbg` in the serial monitor to get a 16×16 ASCII brightness map around the detected centroid — useful for setting the threshold:
```
[dbg] peak=224 count=14 valid=1 cx=47.83 cy=52.11 thr=180
```

### Geometry constants (requires reflash)

Edit the build flags in `platformio.ini`:

| Flag | Meaning | Default |
|------|---------|---------|
| `LASER_ANGLE_DEG` | Laser tilt from vertical | 30° |
| `CAMERA_BASELINE_MM` | Horiz. dist. laser exit → camera | 30 mm |
| `FOCAL_LENGTH_PX` | Camera focal length at 96×96 | 88 px |

### Running native unit tests (no hardware)

```bash
pio test -e native_test
```

Tests cover dot centroid accuracy (< 0.5 px error on synthetic Gaussian dot), threshold behaviour, spurious-dot rejection, and triangulation math.

---

## Licence

MIT

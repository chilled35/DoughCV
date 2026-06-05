#pragma once
#include <math.h>

// Laser triangulation model
// ──────────────────────────
//
//   Laser shines at angle θ (LASER_ANGLE_DEG) from vertical.
//   Camera is at horizontal distance B (CAMERA_BASELINE_MM) from the laser
//   exit point, looking straight down (or perpendicular to the surface).
//   Camera focal length in pixels: f (FOCAL_LENGTH_PX).
//
//   When the surface rises by Δz mm, the reflected dot moves Δy pixels in
//   the image.  The relationship is derived from similar triangles:
//
//       Δz = (Δy_px * pixel_pitch_mm) / tan(θ)
//
//   where  pixel_pitch_mm = B / (f * tan(θ) + ...)
//
//   For the simplified "thin-lens, camera looks straight down" model used here:
//
//       Δz = Δy_px * (B / f) / (tan(θ) + Δy_px * (B / f) / z_ref)
//
//   For small Δz relative to z_ref the linearised form is accurate enough:
//
//       Δz ≈ Δy_px * B / (f * tan(θ))                   [linear]
//
//   We implement both; callers choose.

struct TriangConfig {
    float laser_angle_deg;    // θ
    float baseline_mm;        // B
    float focal_length_px;    // f (pixels at capture resolution)
    float ref_height_mm;      // working distance camera→surface at calibration (z_ref)

    // Pixel row of the dot when the surface is at ref_height_mm.
    // Set by calling calibrate() with a known reference.
    float ref_dot_cy;
};

// Returns a default config populated from compile-time flags.
inline TriangConfig triang_default_config() {
    return {
        LASER_ANGLE_DEG,
        CAMERA_BASELINE_MM,
        FOCAL_LENGTH_PX,
        /* ref_height_mm */ 0.0f,
        /* ref_dot_cy    */ 0.0f,
    };
}

// Call once with the dot position at the known reference surface height.
inline void triang_calibrate(TriangConfig* cfg, float dot_cy_at_ref) {
    cfg->ref_dot_cy = dot_cy_at_ref;
}

// Linear approximation — accurate to ~1 % for Δz < 0.1 * z_ref.
// Returns rise in mm (positive = surface moved toward camera).
inline float triang_height_linear(const TriangConfig* cfg, float dot_cy) {
    float dy   = cfg->ref_dot_cy - dot_cy;   // pixel shift (↑ in image = smaller cy)
    float tanθ = tanf(cfg->laser_angle_deg * (float)M_PI / 180.0f);
    return dy * cfg->baseline_mm / (cfg->focal_length_px * tanθ);
}

// Non-linear form — valid over a wider range when z_ref is set.
// Falls back to linear if z_ref == 0.
inline float triang_height(const TriangConfig* cfg, float dot_cy) {
    if (cfg->ref_height_mm == 0.0f) return triang_height_linear(cfg, dot_cy);

    float dy      = cfg->ref_dot_cy - dot_cy;
    float tanθ    = tanf(cfg->laser_angle_deg * (float)M_PI / 180.0f);
    float scale   = cfg->baseline_mm / (cfg->focal_length_px * tanθ);
    // Δz = scale * dy / (1 + scale * dy / z_ref)
    float dz_lin  = scale * dy;
    return dz_lin / (1.0f + dz_lin / cfg->ref_height_mm);
}

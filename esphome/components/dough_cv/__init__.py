import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, esp32_camera
from esphome.const import (
    CONF_ID,
    UNIT_MILLIMETER,
    ICON_RULER,
    DEVICE_CLASS_DISTANCE,
    STATE_CLASS_MEASUREMENT,
)

CONF_ESP32_CAMERA_ID = "esp32_camera_id"

DEPENDENCIES = ["esp32_camera", "preferences"]
AUTO_LOAD = ["sensor"]

dough_cv_ns = cg.esphome_ns.namespace("dough_cv")
DoughCVComponent = dough_cv_ns.class_("DoughCVComponent", cg.Component)

CONF_RISE_HEIGHT      = "rise_height"
CONF_FOOTPRINT        = "footprint"
CONF_DOT_COUNT        = "dot_count"
CONF_LASER_ANGLE      = "laser_angle_deg"
CONF_MOUNT_HEIGHT     = "mount_height_mm"
CONF_DOT_THRESHOLD    = "dot_threshold"
CONF_PROCESS_INTERVAL = "process_interval_ms"
CONF_SCALE_FACTOR     = "scale_factor"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DoughCVComponent),
        cv.Optional(CONF_RISE_HEIGHT): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            icon=ICON_RULER,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_FOOTPRINT): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            icon="mdi:circle-expand",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DOT_COUNT): sensor.sensor_schema(
            icon="mdi:dots-grid",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.GenerateID(CONF_ESP32_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
        cv.Optional(CONF_LASER_ANGLE, default=30.0): cv.float_range(min=10.0, max=60.0),
        cv.Optional(CONF_MOUNT_HEIGHT, default=200.0): cv.positive_float,
        cv.Optional(CONF_DOT_THRESHOLD, default=180): cv.int_range(min=50, max=255),
        cv.Optional(CONF_PROCESS_INTERVAL, default=2000): cv.positive_int,
        cv.Optional(CONF_SCALE_FACTOR, default=1.0): cv.positive_float,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cam = await cg.get_variable(config[CONF_ESP32_CAMERA_ID])
    cg.add(var.set_camera(cam))

    # Expose a typed global pointer so YAML lambdas can call
    # dough_cv_component->capture_calibration() etc.
    cg.add_global(cg.RawExpression(
        f"::esphome::dough_cv::DoughCVComponent *dough_cv_component"
    ))
    cg.add(cg.RawExpression(
        f"dough_cv_component = {var}"
    ))

    if CONF_RISE_HEIGHT in config:
        sens = await sensor.new_sensor(config[CONF_RISE_HEIGHT])
        cg.add(var.set_rise_height_sensor(sens))

    if CONF_FOOTPRINT in config:
        sens = await sensor.new_sensor(config[CONF_FOOTPRINT])
        cg.add(var.set_footprint_sensor(sens))

    if CONF_DOT_COUNT in config:
        sens = await sensor.new_sensor(config[CONF_DOT_COUNT])
        cg.add(var.set_dot_count_sensor(sens))

    cg.add(var.set_laser_angle_deg(config[CONF_LASER_ANGLE]))
    cg.add(var.set_mount_height_mm(config[CONF_MOUNT_HEIGHT]))
    cg.add(var.set_dot_threshold(config[CONF_DOT_THRESHOLD]))
    cg.add(var.set_process_interval_ms(config[CONF_PROCESS_INTERVAL]))
    cg.add(var.set_scale_factor(config[CONF_SCALE_FACTOR]))

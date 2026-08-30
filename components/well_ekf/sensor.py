import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

well_ekf_ns = cg.esphome_ns.namespace("well_ekf")
WellEKF = well_ekf_ns.class_("WellEKF", cg.PollingComponent, sensor.Sensor)

CONF_AREA = "area"
CONF_LEVEL_SENSOR = "level_sensor_id"
CONF_PUMP_FLOW_SENSOR = "pump_flow_sensor_id"
CONF_INIT_H2_SENSOR = "init_h2_sensor_id"
CONF_INIT_K_SENSOR = "init_k_sensor_id"
CONF_PERMEABILITY_SENSOR = "permeability_sensor"
CONF_FILTERED_H1_SENSOR = "filtered_h1_sensor"
CONF_INNOVATION_SENSOR = "innovation_sensor"
CONF_P_H1_SENSOR = "p_h1_sensor"
CONF_P_H2_SENSOR = "p_h2_sensor"
CONF_P_K_SENSOR = "p_k_sensor"

CONFIG_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="m",
    accuracy_decimals=3,
).extend({
    cv.GenerateID(): cv.declare_id(WellEKF),
    cv.Required(CONF_AREA): cv.float_,
    cv.Required(CONF_LEVEL_SENSOR): cv.use_id(sensor.Sensor),
    cv.Required(CONF_PUMP_FLOW_SENSOR): cv.use_id(sensor.Sensor),
    cv.Required(CONF_INIT_H2_SENSOR): cv.use_id(sensor.Sensor),
    cv.Required(CONF_INIT_K_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_PERMEABILITY_SENSOR): sensor.sensor_schema(
        unit_of_measurement="1/s",
        accuracy_decimals=6,
    ),
    cv.Optional(CONF_FILTERED_H1_SENSOR): sensor.sensor_schema(
        accuracy_decimals=3,
    ),
    cv.Optional(CONF_INNOVATION_SENSOR): sensor.sensor_schema(
        accuracy_decimals=4,
    ),
    cv.Optional(CONF_P_H1_SENSOR): sensor.sensor_schema(
        accuracy_decimals=6,
    ),
    cv.Optional(CONF_P_H2_SENSOR): sensor.sensor_schema(
        accuracy_decimals=6,
    ),
    cv.Optional(CONF_P_K_SENSOR): sensor.sensor_schema(
        accuracy_decimals=8,
    ),
}).extend(cv.polling_component_schema("30s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)

    cg.add(var.set_area(config[CONF_AREA]))

    level_sens = await cg.get_variable(config[CONF_LEVEL_SENSOR])
    cg.add(var.set_level_sensor(level_sens))
    
    flow_sens = await cg.get_variable(config[CONF_PUMP_FLOW_SENSOR])
    cg.add(var.set_pump_flow_sensor(flow_sens))

    h2_sens = await cg.get_variable(config[CONF_INIT_H2_SENSOR])
    cg.add(var.set_init_h2_sensor(h2_sens))

    k_sens = await cg.get_variable(config[CONF_INIT_K_SENSOR])
    cg.add(var.set_init_k_sensor(k_sens))

    if CONF_PERMEABILITY_SENSOR in config:
        perm_sens = await sensor.new_sensor(config[CONF_PERMEABILITY_SENSOR])
        cg.add(var.set_permeability_sensor(perm_sens))
    if CONF_FILTERED_H1_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_FILTERED_H1_SENSOR])
        cg.add(var.set_filtered_h1_sensor(sens))
    if CONF_INNOVATION_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_INNOVATION_SENSOR])
        cg.add(var.set_innovation_sensor(sens))
    if CONF_P_H1_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_P_H1_SENSOR])
        cg.add(var.set_p_h1_sensor(sens))
    if CONF_P_H2_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_P_H2_SENSOR])
        cg.add(var.set_p_h2_sensor(sens))
    if CONF_P_K_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_P_K_SENSOR])
        cg.add(var.set_p_k_sensor(sens))
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

# Vytvoření namespace pro C++ kód
well_ekf_ns = cg.esphome_ns.namespace('well_ekf')
WellEKF = well_ekf_ns.class_('WellEKF', cg.PollingComponent, sensor.Sensor)

# Definice konfiguračních klíčů
CONF_LEVEL_SENSOR_ID = "level_sensor_id"
CONF_PUMP_FLOW_SENSOR_ID = "pump_flow_sensor_id"
CONF_AREA = "area"

# Diagnostické a výstupní senzory
CONF_PERMEABILITY_SENSOR = "permeability_sensor"
CONF_FILTERED_H1_SENSOR = "filtered_h1_sensor"
CONF_INNOVATION_SENSOR = "innovation_sensor"
CONF_P_H1_SENSOR = "p_h1_sensor"
CONF_P_H2_SENSOR = "p_h2_sensor"
CONF_P_K_SENSOR = "p_k_sensor"

# Definice YAML schématu
CONFIG_SCHEMA = sensor.sensor_schema(WellEKF).extend({
    cv.Required(CONF_LEVEL_SENSOR_ID): cv.use_id(sensor.Sensor),
    cv.Required(CONF_PUMP_FLOW_SENSOR_ID): cv.use_id(sensor.Sensor),
    cv.Required(CONF_AREA): cv.float_,
    
    cv.Optional(CONF_PERMEABILITY_SENSOR): sensor.sensor_schema(
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
}).extend(cv.polling_component_schema("5s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)

    # Propojení povinných vstupních senzorů
    level_sens = await cg.get_variable(config[CONF_LEVEL_SENSOR_ID])
    cg.add(var.set_level_sensor(level_sens))
    
    pump_flow_sens = await cg.get_variable(config[CONF_PUMP_FLOW_SENSOR_ID])
    cg.add(var.set_pump_flow_sensor(pump_flow_sens))

    # Nastavení konstant
    cg.add(var.set_area(config[CONF_AREA]))

    # Propojení volitelných výstupních senzorů
    if CONF_PERMEABILITY_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_PERMEABILITY_SENSOR])
        cg.add(var.set_permeability_sensor(sens))
        
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
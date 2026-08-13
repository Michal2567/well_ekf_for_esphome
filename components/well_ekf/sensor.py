import os
import shutil
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID
from esphome.core import CORE # Import pro přístup k cestám buildu

well_ekf_ns = cg.esphome_ns.namespace("well_ekf")
WellEKF = well_ekf_ns.class_("WellEKF", cg.PollingComponent, sensor.Sensor)

CONF_AREA = "area"
CONF_LEVEL_SENSOR = "level_sensor_id"
CONF_PUMP_FLOW_SENSOR = "pump_flow_sensor_id"
CONF_HA_H2_ENTITY = "ha_h2_entity"
CONF_HA_K_ENTITY = "ha_k_entity"
CONF_PERMEABILITY_SENSOR = "permeability_sensor"

CONFIG_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="m",
    accuracy_decimals=3,
).extend({
    cv.GenerateID(): cv.declare_id(WellEKF),
    cv.Required(CONF_AREA): cv.float_,
    cv.Required(CONF_LEVEL_SENSOR): cv.use_id(sensor.Sensor),
    cv.Required(CONF_PUMP_FLOW_SENSOR): cv.use_id(sensor.Sensor),
    cv.Required(CONF_HA_H2_ENTITY): cv.string,
    cv.Required(CONF_HA_K_ENTITY): cv.string,
    cv.Optional(CONF_PERMEABILITY_SENSOR): sensor.sensor_schema(
        unit_of_measurement="1/s",
        accuracy_decimals=6,
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

    cg.add(var.set_ha_h2_entity(config[CONF_HA_H2_ENTITY]))
    cg.add(var.set_ha_k_entity(config[CONF_HA_K_ENTITY]))

    if CONF_PERMEABILITY_SENSOR in config:
        perm_sens = await sensor.new_sensor(config[CONF_PERMEABILITY_SENSOR])
        cg.add(var.set_permeability_sensor(perm_sens))

# 1. Zdrojová cesta k fyzickým souborům Eigen z tvého Gitu
component_dir = os.path.dirname(__file__)
source_eigen = os.path.join(component_dir, "Eigen")

# 2. Cílová cesta v kořeni buildu (src/Eigen), na kterou vidí celý ESP-IDF
dest_eigen = CORE.relative_build_path("src", "Eigen")

# 3. Hrubé zkopírování celé složky včetně souborů bez přípon
if os.path.exists(source_eigen):
    if os.path.exists(dest_eigen):
        shutil.rmtree(dest_eigen)
    shutil.copytree(source_eigen, dest_eigen)
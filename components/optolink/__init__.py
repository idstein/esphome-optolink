import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_ID, CONF_SENSORS, CONF_LAMBDA

CODEOWNERS = ["@idstein"]

DEPENDENCIES = ["uart"]

optolink_ns = cg.esphome_ns.namespace("optolink")
OptoLinkBridge = optolink_ns.class_("OptoLinkBridge", cg.Component, uart.UARTDevice)
OptoLinkSensor = optolink_ns.class_("OptoLinkSensor", sensor.Sensor, cg.Component)
MULTI_CONF = True

CONF_OPTOLINK_ID = "optolink_id"
CONF_OPTOLINK_ADDRESS = "address"
CONF_UART_VITOCONNECT_ID = "uart_vitoconnect_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OptoLinkBridge),
        cv.GenerateID(CONF_UART_VITOCONNECT_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_SENSORS): cv.ensure_list(sensor.sensor_schema().extend(
            {
                cv.GenerateID(): cv.declare_id(OptoLinkSensor),
                cv.GenerateID(CONF_OPTOLINK_ID): cv.use_id(OptoLinkBridge),
                cv.Required(CONF_OPTOLINK_ADDRESS): cv.positive_int,
                cv.Required(CONF_LAMBDA): cv.returning_lambda,
                #cv.Optional(CONF_VALUE_REGEX, default=".*"): cv.string,
                #cv.Optional(CONF_FORMAT, default="float"): cv.enum(VALUE_FORMAT_TYPES, lower=True),
                #cv.Optional(CONF_TIMEOUT, default="5s"): cv.time_period,
            }
        )),
    }
).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    opto_link_bridge = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(opto_link_bridge, config)
    await uart.register_uart_device(opto_link_bridge, config)
    uart_vito_connect = await cg.get_variable(config[CONF_UART_VITOCONNECT_ID])
    cg.add(opto_link_bridge.set_uart_vitoconnect(uart_vito_connect))
    for conf in config[CONF_SENSORS]:
        s = await sensor.new_sensor(conf)
        cg.add(s.set_address(conf[CONF_OPTOLINK_ADDRESS]))
        lambda_ = await cg.process_lambda(
            conf[CONF_LAMBDA],
            [(cg.std_vector.template(cg.uint8), "x")],
            return_type=cg.float_,
        )
        cg.add(s.set_value_parser(lambda_))
        cg.add(opto_link_bridge.register_sensor(s))


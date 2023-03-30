#include <cmath>
#include <cstdlib>
#include <string>

#include "esphome/core/log.h"

#include "OptoLinkSensor.h"

#define TAG "optolink"

namespace esphome
{
    namespace optolink
    {
        void OptoLinkSensor::set_value_parser(value_parser_t parser) {
            this->value_parser_ = std::move(parser);
        }
        void OptoLinkSensor::parse_value(std::vector<uint8_t> &value) {
            auto state = this->value_parser_(value);
            ESP_LOGI(TAG, "Sensor 0x%04X parsed %f from %s", address_, state, to_hex(value).c_str());
            this->publish_state(state);
        }

        void OptoLinkSensor::set_address(uint16_t address) {
            this->address_ = address;
        }

        const uint16_t OptoLinkSensor::get_address() const {
            return this->address_;
        }

        const std::string to_hex(std::vector<uint8_t> v) {
            std::ostringstream ss;

            ss << std::hex << std::uppercase << std::setfill( '0' );
            for( int c : v ) {
                ss << std::setw( 2 ) << c;
            }

            return ss.str();
        }
    } // namespace obis_d0
} // namespace esphome
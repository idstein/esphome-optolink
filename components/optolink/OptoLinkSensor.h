#pragma once

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#include "esphome/components/sensor/sensor.h"

namespace esphome
{
    namespace optolink
    {
        using value_parser_t = std::function<float(std::vector<uint8_t> &)>;
        class OptoLinkSensor : public sensor::Sensor
        {
        public:
            OptoLinkSensor() = default;
            const uint16_t get_address() const;
            void set_address(uint16_t address);
            void set_value_parser(value_parser_t parser);
            void parse_value(std::vector<uint8_t> &value);
        protected:
            uint16_t address_;
            value_parser_t value_parser_;
        };

        const std::string to_hex(std::vector<uint8_t> v);
    } // namespace optolink
} // namespace esphome

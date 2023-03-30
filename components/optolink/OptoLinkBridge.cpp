#include "OptoLinkBridge.h"

#include <byteswap.h>

#define TAG "optolink"

namespace esphome {
    namespace optolink {

        void OptoLinkBridge::loop() {
            if (_state == RESET) {
                _resetHandler();
            }
            if (_state == RESET_ACK) {
                _resetAckHandler();
            }
            if (_state == INIT) {
                _initHandler();
            }
            if (_state == INIT_ACK) {
                _initAckHandler();
            }
            if (_state == IDLE) {
                _idleHandler();
            }
            if (_state == SEND) {
                _sendHandler();
            }
            if (_state == SEND_ACK) {
                _sendAckHandler();
            }
            if (_state == RECEIVE) {
                _receiveHandler();
            }
            if (_state == RECEIVE_ACK) {
                _receiveAckHandler();
            }
            if ((_action == PROCESS || _action == PROCESS_VITOCONNECT) &&
                (esphome::millis() - _lastMillis > 5 * 1000UL)) {  // general timeout when reading or writing
                ESP_LOGW(TAG, "read/write timeout");
                _errorCode = 1;
                _setAction(RETURN_ERROR);
                _setState(RESET);
            }
        }

// Set communication with Vitotronic to defined state = reset to KW protocol
        void OptoLinkBridge::_resetHandler() {
            uint8_t byte;
            if (vitoconnect_ != nullptr) {
                if (vitoconnect_->available() <= 0 || !vitoconnect_->read_byte(&byte) || byte != EOT) {
                    ESP_LOGD(TAG, "vitoconnect await EOT");
                    return;
                }
            }
            ESP_LOGI(TAG, "send EOT");
            _clearInputBuffer();
            write_byte(EOT);
            _lastMillis = esphome::millis();
            _setState(RESET_ACK);
        }

        void OptoLinkBridge::_resetAckHandler() {
            uint8_t signal;
            if (available() <= 0 || !read_byte(&signal)) {
                ESP_LOGD(TAG, "await ENQ");
                return;
            }
            if (signal == ENQ) {  // use peek so connection can be made immediately in next state
                if (vitoconnect_ != nullptr) {
                    ESP_LOGI(TAG, "vitoconnect forward ENQ");
                    // received 0x05/enquiry: optolink has been reset
                    vitoconnect_->write_byte(ENQ);
                }
                _setState(INIT);
                return;
            }
            if (esphome::millis() - _lastMillis > 500) {  // try again every 0,5sec
                ESP_LOGW(TAG, "Timed out awaiting ENQ");
                _setState(RESET);
            }
        }

// send initiator to Vitotronic to establish connection
        void OptoLinkBridge::_initHandler() {
            std::array<uint8_t, 3> sync_packet = {SYNC, 0x00, 0x00};
            if (vitoconnect_ != nullptr) {
                uint8_t byte;
                while (vitoconnect_->available() > 0 && vitoconnect_->peek_byte(&byte) && byte != SYNC) {
                    ESP_LOGW(TAG, "vitoconnect expected SYNC but received %#02x, skipping", byte);
                    vitoconnect_->read_byte(&byte);
                }
                if (vitoconnect_->available() < 3) {
                    ESP_LOGD(TAG, "vitoconnect await SYNC");
                    return;
                }

                if (!vitoconnect_->read_array(sync_packet.data(), 3) || sync_packet != std::array < uint8_t, 3 > {{SYNC,
                                                                                                                  0x00,
                                                                                                                  0x00}}) {
                    ESP_LOGW(TAG, "vitoconnect unable to read SYNC, received %#02x %#02x %#02x", sync_packet[0],
                             sync_packet[1], sync_packet[2]);
                    _setState(RESET);
                    return;
                }
                ESP_LOGD(TAG, "vitoconnect forward SYNC");
            }
            ESP_LOGI(TAG, "send SYNC");
            write_array(sync_packet);
            _lastMillis = esphome::millis();
            _setState(INIT_ACK);
        }

        void OptoLinkBridge::_initAckHandler() {
            uint8_t signal;
            if (available() <= 0 || !read_byte(&signal)) {
                ESP_LOGD(TAG, "await SYNC ACK");
                return;
            }
            if (signal == ACK) {
                if (vitoconnect_ != nullptr) {
                    ESP_LOGD(TAG, "forward SYNC ACK");
                    vitoconnect_->write_byte(ACK);
                }
                // ACK received, moving to next state
                _setState(IDLE);
                return;
            }
            if (esphome::millis() - _lastMillis > 1000UL) {  // if no ACK is coming, reset connection
                ESP_LOGW(TAG, "Timed out awaiting SYNC ACK");
                _setState(RESET);
            }
        }

// idle state, waiting for user action
        void OptoLinkBridge::_idleHandler() {
            if (vitoconnect_ == nullptr) {
                // TODO custom handling
                return;
            }
            if (vitoconnect_->available() <= 0) {
                return;
            }
            uint8_t byte;
            if (vitoconnect_->peek_byte(&byte) && byte == SYNC) { // vitoconnect wants to send a sync
                ESP_LOGD(TAG, "vitoconnect requests re-sync");
                _txBufferLen = 0;
                _setState(INIT);
                return;
            } else if (vitoconnect_->peek_byte(&byte) && byte == ACK) { // vitoconnect acknowledges previous message
                ESP_LOGD(TAG, "vitoconnect send ACK");
                if (vitoconnect_->read_byte(&byte)) {
                    ESP_LOGD(TAG, "send ACK");
                    write_byte(byte);
                }
                return;
            } else {
                while (vitoconnect_->available() > 0) {
                    /*if (esphome::millis() - _lastMillis > 15 * 1000UL) {  // send INIT every 15 seconds to keep communication alive
                      _setState(INIT);
                      return;
                    }*/
                    if (vitoconnect_->read_byte(&_txBuffer[_txBufferLen]) && _txBuffer[0] == VS2_DAP_STANDARD) {
                        _txBufferLen++;
                    }
                    if (_txBufferLen > 2 && _txBufferLen == _txBuffer[1] + 3) { // start byte + packet length + CRC byte
                        uint8_t *package = _txBuffer + 1;
                        uint8_t packageLength = _txBuffer[1];
                        if (!_checkChecksum(package, packageLength)) {  // checksum is wrong
                            _txBufferLen = 0;
                            _errorCode = 2;
                            vitoconnect_->write_byte(NACK); // tell vitoconnect nack
                            ESP_LOGE(TAG, "vitoconnect, checksum failed");
                            return;
                        }

                        _protocolIdentifier = static_cast<OptoLinkBridge::OptolinkProtocolIdentifier>(
                                (package[1] >> 4) & 0xF);
                        _messageIdentifier = static_cast<OptoLinkBridge::OptolinkMessageIdentifier>(package[1] & 0xF);
                        _messageSequenceNumber = (package[2] >> 4) & 0xF;
                        _functionCode = static_cast<OptoLinkBridge::OptolinkFunctionCode>(package[2] & 0xF);
                        _address = __bswap_16(reinterpret_cast<uint16_t *>(package + 3)[0]);
                        _blockLength = package[5];
                        ESP_LOGD(TAG, "Vitoconnect send %#02x %#02x %#02x %#02x %#02x %#02x", packageLength, package[1],
                                 package[2], package[3], package[4], package[5]);
                        ESP_LOGD(TAG, "Vitoconnect length %u protocol %u msgId %u msgSeq %u func %u addr %u block %u",
                                 packageLength, _protocolIdentifier, _messageIdentifier, _messageSequenceNumber,
                                 _functionCode, _address, _blockLength);
                        if (_functionCode == OptolinkFunctionCode::VIRTUAL_WRITE ||
                            _functionCode == OptolinkFunctionCode::PHYSICAL_WRITE ||
                            _functionCode == OptolinkFunctionCode::EEPROM_WRITE ||
                            _functionCode ==
                            OptolinkFunctionCode::RPC) { // Block length additional bytes only for XXX_WRITE, RPC
                            memcpy(_value, &package[6], _blockLength);
                        }
                        // flag as vitoconnect request
                        _txBufferLen = 0;
                        _setAction(PROCESS_VITOCONNECT);
                        _lastMillis = esphome::millis();
                        _setState(SEND);
                        return;
                    }
                }
            }


            /*if (_action == PROCESS) { // if no vitoconnect request; send our requests
              _setState(SEND);
            }*/
        }

        void OptoLinkBridge::_sendHandler() {
            ESP_LOGD(TAG, "send request");
            uint8_t buff[255];
            uint8_t packageLength = 5;
            if (_functionCode == OptolinkFunctionCode::VIRTUAL_WRITE ||
                _functionCode == OptolinkFunctionCode::PHYSICAL_WRITE ||
                _functionCode == OptolinkFunctionCode::EEPROM_WRITE ||
                _functionCode == OptolinkFunctionCode::RPC) {
                packageLength += _blockLength;
            }
            buff[0] = VS2_DAP_STANDARD;
            uint8_t *package = &buff[1];
            package[0] = packageLength; // Package length for the CRC
            package[1] =
                    ((_protocolIdentifier & 0xF) << 4) | (_messageIdentifier & 0xF); // Protocol + MessageIdentifier
            package[2] = ((_messageSequenceNumber & 0xF) << 4) | (_functionCode & 0xF); // SequenceNumber FunctionCode
            package[3] = (_address >> 8) & 0xFF; // Address High Byte
            package[4] = _address & 0xFF; // Address Low Byte
            package[5] = _blockLength; // Block Length
            if (_functionCode == OptolinkFunctionCode::VIRTUAL_WRITE ||
                _functionCode == OptolinkFunctionCode::PHYSICAL_WRITE ||
                _functionCode == OptolinkFunctionCode::EEPROM_WRITE ||
                _functionCode == OptolinkFunctionCode::RPC) { // Block length additional bytes only for XXX_WRITE, RPC
                memcpy(&package[6], _value, _blockLength);
                std::vector<uint8_t> block(&package[6], &package[6] + _blockLength);
                ESP_LOGI(TAG, "Request 0x%02x to address 0x%04X with block %u bytes %s", _functionCode, _address,
                         _blockLength, to_hex(block).c_str());
            } else {
                ESP_LOGI(TAG, "Request 0x%02x to address 0x%04X with block %u bytes", _functionCode, _address,
                         _blockLength);
            }
            package[packageLength + 1] = _calcChecksum(package, packageLength);
            _clearInputBuffer();  // keep input clean
            _rcvBufferLen = 0;
            ESP_LOGD(TAG, "Request send %#02x %#02x %#02x %#02x %#02x %#02x", package[0], package[1], package[2], package[3], package[4], package[5]);
            write_array(buff, packageLength + 3);
            _lastMillis = esphome::millis();
            _setState(SEND_ACK);
        }

        void OptoLinkBridge::_sendAckHandler() {
            if (esphome::millis() - _lastMillis > 1000UL) {  // if no ACK is coming, return to RESET
                ESP_LOGW(TAG, "ack timeout");
                _errorCode = 1;
                _setAction(RETURN_ERROR);
                _setState(RESET);
                return;
            }
            if (available() <= 0) {
                return;
            }
            uint8_t signal;
            read_byte(&signal);
            if (signal == ACK) {  // transmit succesful, moving to next state
                ESP_LOGD(TAG, "ack");
                if (_action == PROCESS_VITOCONNECT && vitoconnect_ != nullptr) {
                    vitoconnect_->write_byte(ACK);
                }
                _rcvBufferLen = 0;
                _setState(RECEIVE);
            } else if (signal == NACK) {  // transmit negatively acknowledged, return to IDLE
                ESP_LOGD(TAG, "nack");
                if (_action == PROCESS_VITOCONNECT && vitoconnect_ != nullptr) {
                    vitoconnect_->write_byte(NACK);
                }
                _errorCode = 1;
                _setAction(RETURN_ERROR);
                _setState(IDLE);
            }
        }

        void OptoLinkBridge::_receiveHandler() {
            ESP_LOGD(TAG, "receive response");
            while (available() > 0) {  // while instead of if: read complete RX buffer
                if (read_byte(&_rcvBuffer[_rcvBufferLen]) && _rcvBuffer[0] == VS2_DAP_STANDARD) {
                    _rcvBufferLen++;
                }
                if (_rcvBuffer[0] != VS2_DAP_STANDARD) {
                    if (_action == PROCESS_VITOCONNECT && vitoconnect_ != nullptr) {
                        vitoconnect_->write_array(_rcvBuffer, _rcvBufferLen);
                    }
                    _rcvBufferLen = 0;
                    _errorCode = 5;
                    _setState(RECEIVE_ACK);
                    ESP_LOGE(TAG, "nack, wrong message format expected 0x41 but received %#02x", _rcvBuffer[0]);
                    return;
                };
                if (_rcvBufferLen > 3 && _rcvBufferLen == _rcvBuffer[1] + 3) { // message complete, check message
                    uint8_t *package = _rcvBuffer + 1;
                    uint8_t packageLength = package[0];
                    // _printHex(_printer, _rcvBuffer, _rcvBufferLen);
                    /*if (package[0] != (_rcvLen - 3)) {  // check for message length
                      _errorCode = 4;
                      _setState(RECEIVE_ACK);
                      ESP_LOGE(TAG, "nack, length");
                      return;
                    }*/
                    auto protocolIdentifier = static_cast<OptoLinkBridge::OptolinkProtocolIdentifier>(
                            (package[1] >> 4) & 0xF);
                    auto messageIdentifier = static_cast<OptoLinkBridge::OptolinkMessageIdentifier>(package[1] & 0xF);
                    auto messageSequenceNumber = (package[2] >> 4) & 0xF;
                    auto functionCode = static_cast<OptoLinkBridge::OptolinkFunctionCode>(package[2] & 0xF);
                    auto address = __bswap_16(reinterpret_cast<uint16_t *>(package + 3)[0]);
                    auto blockLength = package[5];
                    if (messageIdentifier !=
                        OptoLinkBridge::OptolinkMessageIdentifier::RESPONSE) {  // Vitotronic returns an error message
                        if (_action == PROCESS_VITOCONNECT && vitoconnect_ != nullptr) {
                            vitoconnect_->write_array(_rcvBuffer, _rcvBufferLen);
                        }
                        _rcvBufferLen = 0;
                        _errorCode = 3;  // Vitotronic error
                        _setState(IDLE);
                        ESP_LOGE(TAG, "nack, comm error msgId %#02x", messageIdentifier);
                        return;
                    }
                    if (!_checkChecksum(package, packageLength)) {  // checksum is wrong
                        if (_action == PROCESS_VITOCONNECT && vitoconnect_ != nullptr) {
                            vitoconnect_->write_array(_rcvBuffer, _rcvBufferLen);
                        }
                        _rcvBufferLen = 0;
                        _errorCode = 2;
                        _setState(IDLE);  // should we return NACK?
                        ESP_LOGE(TAG, "nack, checksum");
                        return;
                    }
                    if (functionCode == OptoLinkBridge::OptolinkFunctionCode::VIRTUAL_READ) {

                        auto it = sensors_.find(address);
                        if (it != sensors_.end())
                        {
                            std::vector<uint8_t> data(&package[6], &package[6] + blockLength);
                            it->second->parse_value(data);
                        } else {
                            std::vector<uint8_t> data(&package[6], &package[6] + blockLength);
                            ESP_LOGI(TAG, "Vitocal 0x%04X read %u bytes: %s ", address, blockLength, to_hex(data).c_str());
                        }
                        // message is from READ command, so returning read value
                    } else if (functionCode == OptoLinkBridge::OptolinkFunctionCode::RPC) {
                        std::vector<uint8_t> data(&package[6], &package[6] + blockLength);
                        ESP_LOGD(TAG, "Vitocal 0x%04X RPC %u bytes: %s ", address, blockLength, to_hex(data).c_str());
                    } else {
                        ESP_LOGD(TAG, "Vitocal func 0x%02X bytes from address 0x%04X", functionCode, address);
                    }
                    if (_action == PROCESS_VITOCONNECT && vitoconnect_ != nullptr) {
                        vitoconnect_->write_array(_rcvBuffer, _rcvBufferLen);
                    }
                    _rcvBufferLen = 0;
                    _setState(IDLE);
                    _errorCode = 0;
                    ESP_LOGD(TAG, "ack");
                    return;
                }
            }
        }

// send Ack on message receive success
        void OptoLinkBridge::_receiveAckHandler() {
            if (esphome::millis() - _lastMillis > 1000UL) {  // if no ACK is coming, return to RESET
                ESP_LOGW(TAG, "receive ack timeout");
                _errorCode = 1;
                _setAction(RETURN_ERROR);
                _setState(RESET);
                return;
            }
            if(available() <= 0) {
                return;
            }
            if (_action == PROCESS_VITOCONNECT && vitoconnect_ != nullptr) {
                uint8_t byte;
                if (!(vitoconnect_->available() > 0 && vitoconnect_->read_byte(&byte) && byte == ACK)) {
                    return;
                }
            }
            write_byte(ACK);
            _lastMillis = esphome::millis();
            _setState(IDLE);
            if (!_errorCode) {
                _setAction(RETURN);
            } else {
                _setAction(RETURN_ERROR);
            }
        }

// set properties for datapoint and move state to SEND
        bool OptoLinkBridge::readFromDP(uint16_t address, uint8_t length) {
            return _transmit(address, length, OptolinkFunctionCode::VIRTUAL_READ, nullptr);
        }

// set properties datapoint and move state to SEND
        bool OptoLinkBridge::writeToDP(uint16_t address, uint8_t length, uint8_t value[]) {
            return _transmit(address, length, OptolinkFunctionCode::VIRTUAL_WRITE, value);
        }

        bool
        OptoLinkBridge::_transmit(uint16_t address, uint8_t length, OptoLinkBridge::OptolinkFunctionCode functionCode,
                                  uint8_t value[]) {
            if (_action != WAIT) {
                return false;
            }
            _protocolIdentifier = OptolinkProtocolIdentifier::LDAP;
            _messageSequenceNumber = 0;
            _messageIdentifier = OptolinkMessageIdentifier::REQUEST;
            _functionCode = functionCode;
            _address = address;
            _blockLength = length;
            if (_functionCode == OptolinkFunctionCode::VIRTUAL_WRITE ||
                _functionCode == OptolinkFunctionCode::PHYSICAL_WRITE ||
                _functionCode == OptolinkFunctionCode::EEPROM_WRITE ||
                _functionCode == OptolinkFunctionCode::RPC) {
                memcpy(_value, value, _blockLength);
            }
            _setAction(PROCESS);
            _lastMillis = esphome::millis();
            return true;
        }
    }
}
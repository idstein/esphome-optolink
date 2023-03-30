#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/optolink/OptoLinkSensor.h"

#include <unordered_map>

#define EOT 0x04
#define ENQ 0x05
#define ACK 0x06
#define NACK 0x15
#define SYNC 0x16
#define VS2_DAP_STANDARD 0x41

namespace esphome
{
    namespace optolink
    {
class OptoLinkBridge : public Component,
                           public uart::UARTDevice {
 public:
  OptoLinkBridge() = default;

  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  void loop() override;
  bool readFromDP(uint16_t address, uint8_t length);
  bool writeToDP(uint16_t address, uint8_t length, uint8_t value[]);
  void set_uart_vitoconnect(uart::UARTComponent *vitoconnect) { this->vitoconnect_ = vitoconnect; }
  void register_sensor(OptoLinkSensor* sensor) { this->sensors_.emplace(sensor->get_address(), sensor); }
 private:
  uart::UARTComponent* vitoconnect_{nullptr};
  enum OptolinkState : uint8_t { RESET = 0, RESET_ACK, INIT, INIT_ACK, IDLE, SEND, SEND_ACK, RECEIVE, RECEIVE_ACK } _state{RESET};
  enum OptolinkAction : uint8_t { WAIT = 0, PROCESS, PROCESS_VITOCONNECT, RETURN, RETURN_ERROR } _action{WAIT};
  enum OptolinkProtocolIdentifier : uint8_t { LDAP = 0x00, RDAP = 0x10 } _protocolIdentifier{LDAP};
  uint8_t _messageSequenceNumber{0};
  enum OptolinkMessageIdentifier : uint8_t { REQUEST = 0x00, RESPONSE = 0x01, UNACKD = 0x02, ERROR = 0x03 } _messageIdentifier{REQUEST};
  enum OptolinkFunctionCode: uint8_t { 
    UNDEFINED = 0x00,
    VIRTUAL_READ = 0x01,
    VIRTUAL_WRITE = 0x02,
    PHYSICAL_READ = 0x03,
    PHYSICAL_WRITE = 0x04,
    EEPROM_READ = 0x05,
    EEPROM_WRITE = 0x06,
    RPC = 0x07,

    Virtual_MBUS = 0x21,
    Virtual_MarktManager_READ = 0x22,
    Virtual_MarktManager_WRITE = 0x23,
    Virtual_WILO_READ = 0x24,
    Virtual_WILO_WRITE = 0x25,

    XRAM_READ = 0x31,
    XRAM_WRITE = 0x32,
    PORT_READ = 0x33,
    PORT_WRITE = 0x34,
    BE_READ = 0x35,
    BE_WRITE = 0x36,

    KMBUS_RAM_READ = 0x41,
    //KMBUS_RAM_WRITE = 0x42,
    KMBUS_EEPROM_READ = 0x43,
    //KMBUS_EEPROM_WRITE = 0x44,

    KBUS_DATAELEMENT_READ = 0x51,
    KBUS_DATAELEMENT_WRITE = 0x52,
    KBUS_DATABLOCK_READ = 0x53,
    KBUS_DATABLOCK_WRITE = 0x54,
    KBUS_TRANSPARENT_READ = 0x55,
    KBUS_TRANSPARENT_WRITE = 0x56,
    KBUS_INITIALISATION_READ = 0x57,
    KBUS_INITIALISATION_WRITE = 0x58,
    KBUS_EEPROM_LT_READ = 0x59,
    KBUS_EEPROM_LT_WRITE = 0x5A,
    //KBUS_CONTROL_READ = 0x5B,
    KBUS_CONTROL_WRITE = 0x5C,
    KBUS_MEMBERLIST_READ = 0x5D,
    KBUS_MEMBERLIST_WRITE = 0x5E,
    KBUS_VIRTUAL_READ = 0x5F,
    KBUS_VIRTUAL_WRITE = 0x60,
    KBUS_DIRECT_READ = 0x61,
    KBUS_DIRECT_WRITE = 0x62,
    KBUS_INDIRECT_READ = 0x63,
    KBUS_INDIRECT_WRITE = 0x64,
    KBUS_GATEWAY_READ = 0x65,
    KBUS_GATEWAY_WRITE = 0x66,

    PROZESS_WRITE = 0x78,
    PROZESS_READ = 0x79,

    OT_PHYSICAL_READ = 0xB4,
    OT_VIRTUAL_READ = 0xB5,
    OT_PHYSICAL_WRITE = 0xB6,
    OT_VIRTUAL_WRITE = 0xB7,

    GFA_READ = 0xC9,
    GFA_WRITE = 0xCA
   } _functionCode{UNDEFINED};
  uint16_t _address{0};
  uint8_t _blockLength{0};
  uint8_t _value[250];
  uint8_t _rcvBuffer[255];
  uint8_t _rcvBufferLen{0};
  uint8_t _txBuffer[255];
  uint8_t _txBufferLen{0};
  uint32_t _lastMillis{0};
  uint8_t _errorCode{0};
  std::unordered_map<uint16_t, OptoLinkSensor*> sensors_;
  void _resetHandler();
  void _resetAckHandler();
  void _initHandler();
  void _initAckHandler();
  void _idleHandler();
  void _sendHandler();
  void _sendAckHandler();
  void _receiveHandler();
  void _receiveAckHandler();
  void _returnHandler();
  bool _transmit(uint16_t address, uint8_t length, OptoLinkBridge::OptolinkFunctionCode functionCode, uint8_t value[]);

  // start with second byte (= package length) and end before checksum
  inline uint8_t _calcChecksum(uint8_t array[], uint8_t packageLength) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i <= packageLength; ++i) {  
      sum += array[i];
    }
    return sum;
  }
  // start with second byte and end before checksum
  inline bool _checkChecksum(uint8_t array[], uint8_t packageLength) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i <= packageLength; ++i) {  // start with second byte and end before checksum
      sum += array[i];
    }
    return (array[packageLength + 1] == sum);
  }

  inline void _clearInputBuffer() {
    uint8_t data;
    while (available() > 0) {
      read_byte(&data);
    }
  }

  inline void _setState(OptolinkState state) {
    _state = state;
  }
  inline void _setAction(OptolinkAction action) {
    _action = action;
  }
};
}
}
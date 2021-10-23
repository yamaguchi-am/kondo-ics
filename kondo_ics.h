#ifndef KONDO_ICS_H_
#define KONDO_ICS_H_

#include <stdint.h>
#include <stdlib.h>
#include <termios.h>
#include <string>

class KondoIcs {
 public:
  enum CommunicationResult {
    SUCCESS,
    TX_FAIL,
    RX_TIMEOUT_ERROR,
    RX_LENGTH_ERROR,
    RX_VERIFY_ERROR
  };

  KondoIcs(std::string deviceFileName, int baudrate);
  KondoIcs(const char* deviceFileName, int baudrate);
  ~KondoIcs();

  CommunicationResult SetPos(int16_t id, int16_t value,
                             uint16_t* capture) const;
  CommunicationResult SetSpeed(int16_t id, int16_t value) const;

  // Sets/gets the stretch value. Stretch is similar to P gain.
  CommunicationResult SetStretch(int16_t id, int16_t value) const;
  CommunicationResult GetStretch(int16_t id, int16_t* out_value) const;

  CommunicationResult WriteId(uint16_t id) const;

  // Gets current position of a servo and turn on servo to hold there.
  // This function is meant to be used for soft-start at the startup of a
  // program, when the program doesn't know the last position command sent to
  // the servo before its start.
  // This will cause a drift of position due to residual offset of the servo's
  // feedback control. Therefore programs using this function should move
  // servos to desired positions smoothly after calling this function, starting
  // from the captured positions.
  CommunicationResult PseudoCaptureAndHold(int16_t id, uint16_t* capture) const;

 private:
  static const uint8_t kServoId = 0b11100000;  // 111xxxxx (xxxxx = new ID)
  static const uint8_t kServoIdSubcommandRead = 0x00;
  static const uint8_t kServoIdSubcommandWrite = 0x01;
  static const uint8_t kParameterRead =
      0b10100000;  // 101xxxxx (xxxxx = servo ID)
  static const uint8_t kParameterWrite =
      0b11000000;  // 110xxxxx (xxxxx = servo ID)
  static const uint8_t kSubcommandEeprom = 0x00;
  static const uint8_t kSubcommandStretch = 0x01;
  static const uint8_t kSubcommandSpeed = 0x02;
  static const uint8_t kSubcommandCurrentLimit = 0x03;
  static const uint8_t kSubcommandTemperture = 0x04;

  int comm_fd;
  struct termios existingTermios;
  void Open(const char* deviceFileName, int baudrate);
  CommunicationResult ReadBytes(char* result, size_t expectBytes,
                                int timeout) const;
  CommunicationResult ExecuteCommand(size_t sendSize, const char* sendData,
                                     size_t receiveBytes, char* receiveData,
                                     int timeout) const;
  CommunicationResult ChangeCharacteristic(int16_t id, int16_t subCommand,
                                           int16_t value) const;
  CommunicationResult ReadCharacteristic(int16_t id, int16_t subCommand,
                                         int16_t* value) const;

  static const int kCommTimeout = 1000;
  bool IsError(CommunicationResult result) const { return result != SUCCESS; }
};

#endif /* KONDO_ICS_H_ */

#include "kondo_ics.h"

#include <assert.h>
#include <fcntl.h>
#include <gflags/gflags.h>
#include <linux/serial.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <string>

using std::cerr;
using std::endl;

DEFINE_int32(char_delay_usec, 0, "delay between characters in microseconds");
DEFINE_bool(enable_ics_debug_output, false, "enable debug output");

KondoIcs::KondoIcs(std::string deviceFileName, int baudrate) {
  Open(deviceFileName.c_str(), baudrate);
}

KondoIcs::KondoIcs(const char* deviceFileName, int baudrate) {
  Open(deviceFileName, baudrate);
}

void KondoIcs::Open(const char* deviceFileName, int baudrate) {
  comm_fd = open(deviceFileName, O_RDWR | O_NOCTTY);
  if (comm_fd < 0) {
    perror(deviceFileName);
    exit(-1);
  }

  tcgetattr(comm_fd, &existingTermios);
  struct termios newTermios;
  bzero(&newTermios, sizeof(newTermios));
  // Actual baud rate is configured by custom_divisor below.
  newTermios.c_cflag = B38400 | PARENB | CS8 | CLOCAL | CREAD;
  newTermios.c_iflag = PARENB | ICRNL;
  newTermios.c_oflag = 0;

  struct serial_struct serialSettings;
  ioctl(comm_fd, TIOCGSERIAL, &serialSettings);
  serialSettings.flags &= ~ASYNC_SPD_MASK;
  serialSettings.flags |= ASYNC_SPD_CUST;
  if (FLAGS_enable_ics_debug_output) {
    cerr << "base baud=" << serialSettings.baud_base << endl;
  }
  serialSettings.custom_divisor = serialSettings.baud_base / baudrate;
  ioctl(comm_fd, TIOCSSERIAL, &serialSettings);

  newTermios.c_lflag = 0;  // non-canonical mode
  newTermios.c_cc[VTIME] = 0;
  newTermios.c_cc[VMIN] = 0;

  tcflush(comm_fd, TCIFLUSH);
  tcsetattr(comm_fd, TCSANOW, &newTermios);
}

KondoIcs::~KondoIcs() {
  tcsetattr(comm_fd, TCSANOW, &existingTermios);
  close(comm_fd);
}

KondoIcs::CommunicationResult KondoIcs::ReadBytes(char* result,
                                                  size_t expectBytes,
                                                  int timeout) const {
  size_t recvSize = 0;
  for (int retryCount = 0; retryCount < timeout && recvSize < expectBytes;
       retryCount++) {
    int res = read(comm_fd, &result[recvSize], expectBytes - recvSize);
    if (res > 0) {
      if ((size_t)res <= expectBytes) {
        recvSize += res;
      } else {
        // Excessive data
        return RX_LENGTH_ERROR;
      }
    }
    usleep(1);
  }
  if (recvSize != expectBytes) {
    // Timeout
    return RX_TIMEOUT_ERROR;
  }
  return SUCCESS;
}

KondoIcs::CommunicationResult KondoIcs::SetPos(int16_t id, int16_t value,
                                               uint16_t* capture) const {
  char cmd[3];
  cmd[0] = 0x80 | (id & 0x1f);
  cmd[1] = (value & 0x3f80) >> 7;
  cmd[2] = value & 0x7f;
  char recv[6];
  CommunicationResult result =
      ExecuteCommand(3, cmd, 6, recv, KondoIcs::kCommTimeout);
  // TODO: verify R_CMD of return packet
  if (result != SUCCESS) {
    if (FLAGS_enable_ics_debug_output) {
      cerr << "failed to write command" << endl;
    }
    return result;
  }
  *capture = (((uint16_t)(unsigned char)recv[4]) << 7) | (uint16_t)recv[5];
  return SUCCESS;
}

KondoIcs::CommunicationResult KondoIcs::ExecuteCommand(size_t sendSize,
                                                       const char* writeData,
                                                       size_t recvSize,
                                                       char* receiveData,
                                                       int timeout) const {
  if (write(comm_fd, writeData, sendSize) != (ssize_t)sendSize) {
    return TX_FAIL;
  }
  CommunicationResult result = ReadBytes(receiveData, recvSize, timeout);
  if (FLAGS_enable_ics_debug_output) {
    cerr << "write: ";
    for (size_t i = 0; i < sendSize; i++) {
      cerr << std::hex << (int)((unsigned char)writeData[i]) << " ";
    }
    cerr << endl << "receive " << result << ": ";
    for (size_t i = 0; i < recvSize; i++) {
      cerr << std::hex << (int)((unsigned char)receiveData[i]) << " ";
    }
    cerr << endl;
  }
  if (IsError(result)) {
    return RX_LENGTH_ERROR;
  }
  // verify first 3 bytes of return packet (echo back)
  for (size_t i = 0; i < 2; i++) {
    if (receiveData[i] != writeData[i]) {
      return RX_VERIFY_ERROR;
    }
  }
  return SUCCESS;
}

KondoIcs::CommunicationResult KondoIcs::SetSpeed(int16_t id,
                                                 int16_t value) const {
  return ChangeCharacteristic(id, kSubcommandSpeed, value);
}

KondoIcs::CommunicationResult KondoIcs::ChangeCharacteristic(
    int16_t id, int16_t subCommand, int16_t value) const {
  char cmd[3];
  cmd[0] = kParameterWrite | (id & 0x1f);
  cmd[1] = subCommand;
  cmd[2] = value;
  char recv[6];
  return ExecuteCommand(3, cmd, 6, recv, kCommTimeout);
  // TODO: verify return packet
}

KondoIcs::CommunicationResult KondoIcs::ReadCharacteristic(
    int16_t id, int16_t subCommand, int16_t* resultValue) const {
  char cmd[2];
  cmd[0] = kParameterRead | (id & 0x1f);
  cmd[1] = subCommand;
  char recv[5];
  KondoIcs::CommunicationResult commResult =
      ExecuteCommand(2, cmd, 5, recv, kCommTimeout);
  // TODO: verify return packet
  *resultValue = (uint16_t)recv[4];
  return commResult;
}

KondoIcs::CommunicationResult KondoIcs::PseudoCaptureAndHold(
    int16_t id, uint16_t* capture) const {
  // Capture position and hold servo there.
  //
  // KONDO ICS doesn't have a single command to capture current target position
  // nor current position without modifying the servo's target position.
  // Instead, this function attempts to send torque off command to capture the
  // current position, and then sending it as a new target position right after
  // that to prevent move by external force.
  // (e.g. prevent articulated robot arm suddenly fall down)
  // This is a bad and dirty hack. Do not rely on this function too much.
  // If the second command fails, the function will quit leaving the servo
  // powered off.

  CommunicationResult result;
  uint16_t capturedPosition;

  if (IsError(result = SetPos(id, 0, &capturedPosition))) {
    return result;
  }
  if (IsError(result = SetPos(id, capturedPosition, &capturedPosition))) {
    return result;
  }
  *capture = capturedPosition;
  return result;
}

KondoIcs::CommunicationResult KondoIcs::WriteId(uint16_t id) const {
  char cmd[4];
  char result[5];
  cmd[0] = kServoId | (id & 0x1f);
  cmd[1] = kServoIdSubcommandWrite;
  cmd[2] = kServoIdSubcommandWrite;
  cmd[3] = kServoIdSubcommandWrite;
  KondoIcs::CommunicationResult resultCode =
      ExecuteCommand(4, cmd, 5, result, kCommTimeout);
  if (resultCode != SUCCESS) {
    return resultCode;
  }
  return result[4] == cmd[0] ? SUCCESS : RX_VERIFY_ERROR;
}

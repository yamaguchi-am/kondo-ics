// Commandline tool to send position to a single servo.

#include <gflags/gflags.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <iostream>

#include "kondo_ics.h"

DEFINE_string(port, "/dev/ttyUSB0", "serial port of USB ICS Adapter");
DEFINE_int32(baudrate, 1250000, "baud rate of serial servo");
DEFINE_double(id, 0, "ID of servo");
DEFINE_double(value, 0, "Position value to send");

using std::cout;
using std::endl;

void WritePos(const KondoIcs& k, int id, int value) {
  uint16_t capture;
  cout << k.SetPos(id, value, &capture) << " ";
  cout << capture << endl;
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  KondoIcs ics(FLAGS_port, FLAGS_baudrate);
  WritePos(ics, FLAGS_id, FLAGS_value);
  return 0;
}

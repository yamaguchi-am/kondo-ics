// Commandline tool to change an ID of a servo.
// Only 1 servo can be connected to the host when running this program.
// Otherwise, multiple servos will respond to the command and cause signal
// collision.

#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <gflags/gflags.h>

#include "kondo_ics.h"

DEFINE_string(port, "/dev/ttyUSB0", "serial port of USB ICS Adapter");
DEFINE_int32(baudrate, 1250000, "baud rate of serial servo");
DEFINE_int32(newid, -1, "New ID of servo");

using std::cout;
using std::endl;

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  KondoIcs ics(FLAGS_port, FLAGS_baudrate);
  int id = FLAGS_newid;
  if (id < 0 || id > 31) {
    fprintf(stderr, "Specify an ID betwen 0 and 31.\n");
    return -1;
  }
  KondoIcs::CommunicationResult result = ics.WriteId(id);
  printf("Result=%d\n", result);
  return 0;
}

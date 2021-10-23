// Soft-start and swings a servo by a sinusoidal wave pattern.

#include <gflags/gflags.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include <iostream>

#include "kondo_ics.h"

DEFINE_string(port, "/dev/ttyUSB0", "serial port of USB ICS Adapter");
DEFINE_double(id, 0, "ID of servo");
DEFINE_int32(baudrate, 1250000, "baud rate of serial servo");

using std::cout;
using std::endl;

const int kCenter = 7500;

double gettimeofdayInSeconds() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return (double)t.tv_sec + (double)t.tv_usec * 1e-6;
}

void MoveToCenter(const KondoIcs& k, int id) {
  uint16_t initPosition;
  const float kMoveDuration = 1.0;
  uint16_t dummy;
  if (k.PseudoCaptureAndHold(id, &initPosition) != KondoIcs::SUCCESS) {
    cout << "Failed to communicate with servo." << endl;
    exit(1);
  }
  double startTime = gettimeofdayInSeconds();
  while (true) {
    double elapsed = gettimeofdayInSeconds() - startTime;
    if (elapsed > kMoveDuration) {
      break;
    }
    double progress = elapsed / kMoveDuration;
    uint16_t newPosition = (1 - progress) * initPosition + progress * kCenter;
    k.SetPos(id, newPosition, &dummy);
  }
  k.SetPos(id, kCenter, &dummy);
}

void SwingDemo(const KondoIcs& k, int id) {
  int kMagnitude = 2000;
  int kCenter = 7500;
  float kFrequency = 0.3;
  double startTime = gettimeofdayInSeconds();
  while (gettimeofdayInSeconds() < startTime + 10) {
    double elapsed = gettimeofdayInSeconds() - startTime;
    int value = kCenter + sin(M_PI * 2 * elapsed * kFrequency) * kMagnitude;
    uint16_t capture;
    k.SetPos(id, value, &capture);
  }
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  KondoIcs ics(FLAGS_port, FLAGS_baudrate);
  MoveToCenter(ics, FLAGS_id);
  SwingDemo(ics, FLAGS_id);
  return 0;
}

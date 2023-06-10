// Multi-Kangaroo Sample for Kangaroo
// Copyright (c) 2015 Dimension Engineering Inc.
// See license.txt for license details.

#include <SoftwareSerial.h>
#include <Kangaroo.h>

// If two Kangaroos share the same S2 line, as in this example,
// you must first configure the Kangaroo with DEScribe.
//
// Download Settings, check "Enable multi-Kangaroo mode." on the Serial tab,
// set the Packet Serial Address, and then Upload Settings.
//
// Confirm using DEScribe that you are able to communicate with both Kangaroos
// over the tied-together S1, S2, and 0V lines.

// This example controls:
//       one Kangaroo's channel '1' on Packet Serial Address 128, and
//   another Kangaroo's channel '2' on Packet Serial Address 129.
// For mixed-mode, use 'D' and 'T' instead of '1' and '2', or whatever channel name you have set.

// Arduino TX (pin 11) goes to both Kangaroos' S1
// Arduino RX (pin 10) goes to both Kangaroos' S2
// Arduino GND         goes to both Kangaroos' 0V
// Arduino 5V          goes to one  Kangaroo's 5V (OPTIONAL, if you want Kangaroo to power the Arduino)
#define TX_PIN 11
#define RX_PIN 10

SoftwareSerial  SerialPort(RX_PIN, TX_PIN);
KangarooSerial  K(SerialPort);
KangarooChannel K1_128(K, '1', 128);
KangarooChannel K2_129(K, '2', 129);

void setup()
{
  SerialPort.begin(9600);
  SerialPort.listen();

  // Tell both Kangaroo channels to start.
  K1_128.start();
  K2_129.start();
  
  // Tell both to home at the same time. Then, wait for both to be done.
  KangarooMonitor  monitor128, monitor129;
  KangarooMonitor* monitorList[2] = { &monitor128, &monitor129 };
  monitor128 = K1_128.home();
  monitor129 = K2_129.home();
  waitAll(2, monitorList);
}

void loop()
{
  // Go to the minimum side at whatever speed limit is set on the potentiometers.
  // Tell both to go at once, then wait until they are both in position.
  long minimums[2];
  minimums[0] = K1_128.getMin().value();
  minimums[1] = K2_129.getMin().value();
  
  KangarooMonitor  monitor128, monitor129;
  KangarooMonitor* monitorList[2] = { &monitor128, &monitor129 };
  monitor128 = K1_128.p(minimums[0]);  
  monitor129 = K2_129.p(minimums[1]);
  waitAll(2, monitorList);
  
  delay(2000);
  
  // Going to the maximum side, limit speed to 1/10th of the range per second
  // (at least 10 seconds of travel time). Let's have them go one at a time.
  // Here I show two ways to wait on the motions.
  long maximums[2]; long speedLimits[2];
  maximums[0] = K1_128.getMax().value();
  maximums[1] = K2_129.getMax().value();
  
  speedLimits[0] = (maximums[0] - minimums[0]) / 10;
  K1_128.p(maximums[0], speedLimits[0]).wait();
    
  speedLimits[1] = (maximums[1] - minimums[1]) / 10;
  monitor129 = K2_129.p(maximums[1], speedLimits[1]);
  monitor129.wait();

  delay(2000);
}

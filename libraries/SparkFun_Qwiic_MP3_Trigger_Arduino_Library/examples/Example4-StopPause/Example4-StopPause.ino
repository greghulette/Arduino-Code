/*
  Controlling the Qwiic MP3 Trigger with I2C Commands
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 23rd, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  This example demonstrates how to pause or stop an MP3. If a track is playing then pausing
  will pause at that spot. Pressing pause again will continuing playing from that spot. If
  no track is playing and pause is pressed then the last played track will be started from
  the beginning.

  Note: For this example you will need to load an MP3 onto the device. You can
  use a USB C cable to load a file directly onto the SD card.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15165

  Hardware Connections:
  Plug in headphones
  Make sure the SD card is in the socket
  Don't have a USB cable connected right now
  If needed, attach a Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the Qwiic device onto an available Qwiic port
  Open the serial monitor at 115200 baud
*/

#include <Wire.h>

#include "SparkFun_Qwiic_MP3_Trigger_Arduino_Library.h" //http://librarymanager/All#SparkFun_MP3_Trigger
MP3TRIGGER mp3;

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  //Check to see if Qwiic MP3 is present on the bus
  if (mp3.begin() == false)
  {
    Serial.println("Qwiic MP3 failed to respond. Please check wiring and possibly the I2C address. Freezing...");
    while (1)
      ;
  }

  mp3.setVolume(10); //Volume can be 0 (off) to 31 (max)

  Serial.print("Song count: ");
  Serial.println(mp3.getSongCount());

  Serial.println("Press S to stop or P to pause.");

  mp3.playTrack(1); //Play the first track on the SD card
}

void loop()
{
  while (Serial.available() == false)
    delay(10); //Wait for user to send a character

  byte incoming = Serial.read();

  switch (incoming)
  {
  case 'S':
  case 's':
    mp3.stop();
    break;
  case 'P':
  case 'p':
    mp3.pause(); //Pause, or play from pause, the current track
    break;
  default:
    Serial.print("Unknown character: ");
    Serial.write(incoming);
    Serial.println();
    break;
  }
}

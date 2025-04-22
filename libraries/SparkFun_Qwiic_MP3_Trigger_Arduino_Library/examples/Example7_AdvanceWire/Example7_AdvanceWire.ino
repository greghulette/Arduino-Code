/*
  Controlling the Qwiic MP3 Trigger with I2C Commands
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 12th, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  This example shows how start the MP3 Trigger library using a different wire port (ie, Wire1, etc).
  This is handy if you need to connect *a lot* of Triggers and have multiple Wire ports on your dev platform.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15165

  Hardware Connections:
  Plug in headphones
  Make sure the SD card is in the socket and has some MP3s in the root directory
  Don't have a USB cable connected right now
  If needed, attach a Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the Qwiic device onto an available Qwiic port
  Open the serial monitor at 115200 baud
*/

#include <Wire.h> //Needed for I2C to Qwiic MP3 Trigger

#include "SparkFun_Qwiic_MP3_Trigger_Arduino_Library.h" //http://librarymanager/All#SparkFun_MP3_Trigger
MP3TRIGGER mp3;

void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic MP3 Trigger Example");

  Wire1.begin();          //Compilation will fail here if your platform doesn't have multiple I2C ports
  Wire1.setClock(400000); //MP3 Trigger supports higher I2C rates

  if (mp3.begin(Wire1) == false)
  {
    Serial.println("MP3 Trigger does not appear to be connected. Please check wiring. Freezing...");
    while (1)
      ;
  }

  mp3.playTrack(1); //Begin playing the first track on the SD card

  Serial.println("All done!");
}

void loop()
{
}

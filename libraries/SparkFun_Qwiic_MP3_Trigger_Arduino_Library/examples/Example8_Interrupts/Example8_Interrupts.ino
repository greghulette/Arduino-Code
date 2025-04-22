/*
  Controlling the Qwiic MP3 Trigger with I2C Commands
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 12th, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  This example shows how read the interrupt pin on the Trigger. Once a track has completed playing the INT pin
  will go low. Once it is low, the master must clear the interrupt before it will go high.

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

const byte mp3InterruptPin = 2; //Any GPIO will work. Connect to the INT pin on Qwiic MP3 Trigger

void setup()
{
  pinMode(mp3InterruptPin, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("Qwiic MP3 Example");

  Wire.begin();

  //Check to see if Qwiic MP3 is present on the bus
  if (mp3.begin() == false)
  {
    Serial.println("Qwiic MP3 failed to respond. Please check wiring and possibly the I2C address. Freezing...");
    while (1)
      ;
  }

  mp3.playTrack(1); //Begin playing a track. Once complete the INT pin will go low
}

void loop()
{
  Serial.print(".");
  delay(100);

  if (digitalRead(mp3InterruptPin) == LOW)
  {
    Serial.println("Interrupt! Song is complete.");

    mp3.clearInterrupts(); //Clear any interrupt bits before starting a new song
    mp3.playTrack(1);      //Being playing a new song - this can take ~250ms for
  }
}

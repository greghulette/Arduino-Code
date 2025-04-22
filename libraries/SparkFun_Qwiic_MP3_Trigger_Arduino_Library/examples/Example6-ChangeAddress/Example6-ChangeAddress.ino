/*
  Controlling the Qwiic MP3 Trigger with I2C Commands
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 12th, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  This example shows how to change the I2C address of the Qwiic MP3 Trigger.
  Valid addresses are 0x08 to 0x77 - 111 possible addresses!
  Device's I2C address is stored to memory and loaded on each power-on

  Note: If you accidentally set the device into an unknown you can either use the I2C scanner sketch found online
  or you can close the solder jumper on the Qwiic MP3 Trigger. Closing the jumper will force the device address to 0x36.

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

#include <Wire.h>

#include "SparkFun_Qwiic_MP3_Trigger_Arduino_Library.h" //http://librarymanager/All#SparkFun_MP3_Trigger
MP3TRIGGER mp3;

void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic MP3 Trigger Example");

  Wire.begin();

  //Scan bus looking for an MP3 Trigger
  //The .begin() function checks the device ID to verify the device at a given address is a Trigger
  byte currentAddress;
  for (currentAddress = 1; currentAddress < 127; currentAddress++)
  {
    currentAddress = findI2CDevice(currentAddress); //Start scanning at last address
    if (currentAddress == 0)
      break; //No device found!
    if (mp3.begin(Wire, currentAddress) == true)
      break; //Device found!
  }

  if (currentAddress == 0 || currentAddress == 127)
  {
    Serial.println("No MP3 Triggers found on the I2C bus. Freezing...");
    while (1)
      ;
  }

  //Begin communication with MP3 Trigger at current address
  if (mp3.begin(Wire, currentAddress) == true)
  {
    Serial.print("MP3 Trigger found at address 0x");
    Serial.print(currentAddress, HEX);
    Serial.print(" / ");
    Serial.print(currentAddress); //Print decimal
    Serial.println("(decimal)");

    byte newAddress = 0;
    while (1)
    {
      while (Serial.available())
        Serial.read(); //Trash any incoming chars
      Serial.println("Enter the address you'd like to change to in decimal. Valid is 8 to 119.");
      while (Serial.available() == false)
        ; //Wait for user to send character

      newAddress = Serial.parseInt(); //Get decimal address from user
      if (newAddress >= 8 && newAddress <= 119)
        break; //Address is valid
      Serial.println("Invalid address. Please try again.");
    }

    mp3.setAddress(newAddress); //Change I2C address of this device to 0x56.
    //Valid addresses are 0x08 to 0x77 - 111 possible addresses!
    //Device's I2C address is stored to memory and loaded on each power-on

    delay(10); //Time required for device to record address to EEPROM and re-init its I2C

    if (mp3.begin(Wire, newAddress) == true)
    {
      Serial.print("Address successfully changed to 0x");
      Serial.print(newAddress, HEX);
      Serial.print(" / ");
      Serial.print(newAddress); //Print decimal
      Serial.println("(decimal)");
      Serial.print("Now load another example sketch using .begin(Wire, 0x");
      Serial.print(newAddress, HEX);
      Serial.println(") to use this Qwiic MP3 Trigger");
      Serial.println("Freezing...");
      while (1)
        ;
    }
  }

  //Something went wrong, begin scanning I2C bus for valid addresses
  Serial.println("Address change failed. Beginning an I2C scan.");
}

void loop()
{
  Serial.println("Scanning...");

  byte found = 0;
  for (byte address = 1; address < 127; address++)
  {
    address = findI2CDevice(address); //Scans bus starting from given address. Returns address of discovered device.

    if (address > 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 0x0F)
        Serial.print("0"); //Pretty print
      Serial.print(address, HEX);
      Serial.print(" / ");
      Serial.print(address); //Print decimal
      Serial.println("(decimal)");

      found++;
    }
    else
    {
      if (found == 0)
        Serial.println("No I2C devices found\n");
      break; //Done searching
    }
  }

  delay(5000);
}

//Scans the ICC bus looking for devices
//Start scanning from a given address
byte findI2CDevice(byte startingAddress)
{
  if (startingAddress == 0)
    startingAddress = 1; //Error check

  for (byte address = startingAddress; address < 127; address++)
  {
    Wire.beginTransmission(address);
    byte response = Wire.endTransmission();

    if (response == 0) //A device acknowledged us at this address!
      return (address);
  }

  return (0); //No device found
}

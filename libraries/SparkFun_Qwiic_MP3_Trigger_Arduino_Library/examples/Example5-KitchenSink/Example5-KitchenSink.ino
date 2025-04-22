/*
  Controlling the Qwiic MP3 Trigger with I2C Commands
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 12th, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  This example uses a serial menu system to demonstrate all the various functions of the
  Qwiic MP3 Trigger.

  Open the serial terminal at 115200 to get the available menu of commands.

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

  Wire.begin();

  //Check to see if Qwiic MP3 is present on the bus
  if (mp3.begin() == false)
  {
    Serial.println("Qwiic MP3 failed to respond. Please check wiring and possibly the I2C address. Freezing...");
    while (1)
      ;
  }

  if (mp3.hasCard() == false)
  {
    Serial.println("Qwiic MP3 is missing its SD card. Freezing...");
    while (1)
      ;
  }

  mp3.setVolume(10); //Volume can be 0 (off) to 31 (max)

  Serial.print("Song count: ");
  Serial.println(mp3.getSongCount());

  Serial.print("Volume level: ");
  Serial.println(mp3.getVolume());

  Serial.print("EQ Setting: ");
  byte eqSetting = mp3.getEQ();
  if (eqSetting == 0)
    Serial.print("Normal");
  else if (eqSetting == 1)
    Serial.print("Pop");
  else if (eqSetting == 2)
    Serial.print("Rock");
  else if (eqSetting == 3)
    Serial.print("Jazz");
  else if (eqSetting == 4)
    Serial.print("Classic");
  else if (eqSetting == 5)
    Serial.print("Bass");
  Serial.println();

  Serial.print("Firmware version: ");
  Serial.println(mp3.getVersion());
}

void loop()
{
  Serial.println();
  Serial.println("Qwiic MP3 Trigger");
  Serial.println("1) Play track");
  Serial.println("2) Play file");
  Serial.println("3) Play next");
  Serial.println("4) Play previous");
  Serial.println("5) Set volume");
  Serial.println("6) Set EQ");
  Serial.println("7) Get Song Name");
  Serial.println("P) Pause/Play from Pause");
  Serial.println("S) Stop");

  while (Serial.available())
    Serial.read(); //Throw away incoming characters

  while (Serial.available() == false)
    delay(10); //Wait for user to send character

  byte incoming = Serial.read();

  switch (incoming)
  {
  case '1':
  {
    Serial.print("What track number do you want to play (1 to ");
    Serial.print(mp3.getSongCount());
    Serial.print(")?");

    while (Serial.available())
      Serial.read(); //Throw away incoming characters
    while (Serial.available() == false)
      delay(1); //Wait for user to send character
    int trackNumber = Serial.parseInt();

    mp3.playTrack(trackNumber);
    break;
  }
  case '2':
  {
    Serial.print("What specific file number would you like (ex: 6 will play F006.mp3)?");

    while (Serial.available())
      Serial.read(); //Throw away incoming characters
    while (Serial.available() == false)
      delay(1); //Wait for user to send character
    int fileNumber = Serial.parseInt();

    mp3.playFile(fileNumber);
    break;
  }
  case '3':
    mp3.playNext();
    delay(100); //Give the main loop a bit of time before we check to see if a track is playing (allow song to start)
    break;
  case '4':
    mp3.playPrevious();
    break;
  case '5':
  {
    Serial.print("What volume would you like(0 to 31)?");

    while (Serial.available())
      Serial.read(); //Throw away incoming characters
    while (Serial.available() == false)
      delay(1); //Wait for user to send character
    int volumeLevel = Serial.parseInt();

    mp3.setVolume(volumeLevel); //Volume can be 0 (off) to 31 (max)
    break;
  }
  case '6':
  {
    Serial.print("What EQ setting would you like(0 to 5)?");

    while (Serial.available())
      Serial.read(); //Throw away incoming characters
    while (Serial.available() == false)
      delay(1); //Wait for user to send character
    int eqLevel = Serial.parseInt();

    mp3.setEQ(eqLevel); //EQ is 0-normal, 1-pop, 2-rock, 3-jazz, 4-classical, 5-bass
    break;
  }

  case '7':
  {
    if (mp3.isPlaying() == true)
      Serial.print("Now playing: ");
    else
      Serial.print("Last played: ");

    String songName = mp3.getSongName();
    Serial.println(songName);

    break;
  }

  case 'S':
  case 's':
    Serial.println("Stopping");
    mp3.stop();
    break;

  case 'P':
  case 'p':
    if (mp3.isPlaying() == true)
      Serial.println("Pausing");
    else
      Serial.println("Playing");

    mp3.pause(); //Pause, or play from pause, the current track
    break;

  case '\n':
  case '\r':
    //Ignore new line and return characters
    break;

  default:
    Serial.print("Unknown: ");
    Serial.write(incoming);
    Serial.println();
    break;
  }
}

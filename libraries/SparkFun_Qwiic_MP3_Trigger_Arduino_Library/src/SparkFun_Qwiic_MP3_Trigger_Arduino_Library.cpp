/*
  This is a library written for the SparkFun Qwiic MP3 Trigger
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15165

  Written by Nathan Seidle @ SparkFun Electronics, January 12th, 2019

  The Qwiic MP3 Trigger is a I2C controlled MP3 player

  https://github.com/sparkfun/SparkFun_Qwiic_MP3_Trigger_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.7

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_Qwiic_MP3_Trigger_Arduino_Library.h"
#include "Arduino.h"

//Constructor
MP3TRIGGER::MP3TRIGGER()
{
}

//Initializes the device with basic settings
//Returns false if device is not detected
boolean MP3TRIGGER::begin(TwoWire &wirePort, uint8_t deviceAddress)
{
  _i2cPort = &wirePort; //Get setting from user

  _deviceAddress = deviceAddress; //Get setting from user

  if (isConnected() == false)
  {
    delay(2000); //Device may take up to 1500ms to mount the SD card

    //Try again
    if (isConnected() == false)
      return (false); //No device detected
  }

  return (true); //We're all setup!
}

//Returns true if I2C device has the correct ID
boolean MP3TRIGGER::isConnected()
{
  if (getID() == 0x39)
    return (true); //MP3 Trigger ID should be 0x39
  return (false);
}

//Returns the device ID
uint8_t MP3TRIGGER::getID()
{
  sendCommand(COMMAND_GET_ID);

  return (getResponse()); //Should return 0x39
}

//Checks the status of the player to see if MP3 is playing
//Returns true if song is playing
boolean MP3TRIGGER::isPlaying()
{
  sendCommand(COMMAND_GET_PLAY_STATUS);

  //0:stop, 1: play
  byte playStatus = getResponse();
  if (playStatus == 0x01)
    return (true);
  return (false);
}

//Change the I2C address of this address to newAddress
//If you forget what address you've set the QMP3 to then close the
//ADR jumper. This will force the I2C address to 0x36
void MP3TRIGGER::setAddress(byte newAddress)
{
  sendCommand(COMMAND_SET_ADDRESS, newAddress);
  _deviceAddress = newAddress; //Change the global variable to match the new address
}

//Clear any interrupts. Currently, the only interrupt is 'track is finished playing'.
void MP3TRIGGER::clearInterrupts()
{
  sendCommand(COMMAND_CLEAR_INTERRUPTS); //Clear the moved, clicked, and pressed bits
}

//Plays a given track number
//Think of this like a CD. The user can arrange the order of MP3s
//however. playTrack(4) will play whatever is in the 4th file.
void MP3TRIGGER::playTrack(byte trackNumber)
{
  sendCommand(COMMAND_PLAY_TRACK, trackNumber); //Play track
}

//Plays a file that has been named specifically.
//For example: passing in 6 will play F006xxx.mp3
void MP3TRIGGER::playFile(byte fileNumber)
{
  sendCommand(COMMAND_PLAY_FILENUMBER, fileNumber); //Play file number
}

//Play the next track
//Think of this like a CD. The audio files can be in any order. The user
//sets the file order. This plays the next one.
void MP3TRIGGER::playNext()
{
  sendCommand(COMMAND_PLAY_NEXT);
}

//Play the previous track
//Think of this like a CD. The audio files can be in any order. The user
//sets the file order. This plays the previous one.
void MP3TRIGGER::playPrevious()
{
  sendCommand(COMMAND_PLAY_PREVIOUS);
}

//Pause a currently playing song, or begin playing if current track is paused
void MP3TRIGGER::pause()
{
  sendCommand(COMMAND_PAUSE);
}

//Stop playing the current track
void MP3TRIGGER::stop()
{
  sendCommand(COMMAND_STOP);
}

//Change the equalizer to one of 6 types
void MP3TRIGGER::setEQ(byte eqType)
{
  //0-normal, 1-pop, 2-rock, 3-jazz, 4-classical, 5-bass
  sendCommand(COMMAND_SET_EQ, eqType); //Change equalizer to bass
}

//Get current EQ setting: (0=normal, pop, rock, jazz, classic, 5=bass)
uint8_t MP3TRIGGER::getEQ()
{
  sendCommand(COMMAND_GET_EQ);

  return (getResponse());
}

//Change volume to zero (off) to 31 (max)
void MP3TRIGGER::setVolume(byte volumeLevel)
{
  if (volumeLevel > 31)
    volumeLevel = 31;                           //Error check
  sendCommand(COMMAND_SET_VOLUME, volumeLevel); //Change volume
}

//Get current volume level: 0 (off) to 31 (max)
uint8_t MP3TRIGGER::getVolume()
{
  sendCommand(COMMAND_GET_VOLUME);

  return (getResponse());
}

//Get the current status of the Qwiic MP3
//0=OK, 1=Fail, 2=No such file, 5=SD Error
byte MP3TRIGGER::getStatus()
{
  return (getResponse());
}

//Checks to see if MP3 player has a valid SD card
boolean MP3TRIGGER::hasCard()
{
  sendCommand(COMMAND_GET_CARD_STATUS);

  delay(50); //Give the QMP3 time to get the status byte from MP3 IC before we ask for it

  return (getResponse());
}

//Get the 8 characters of the song currently playing
//Returns a character array of the song currently playing, terminated with \0
char *MP3TRIGGER::getSongName()
{
  static char thisSongName[9]; //Max is 12345678\0
  sendCommand(COMMAND_GET_SONG_NAME);

  delay(50); //QMP3 can take ~22 to 35ms to get song name.

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)8); //Song names are max 8 chars

  uint8_t spot = 0;
  while (_i2cPort->available())
  {
    uint8_t incoming = _i2cPort->read();
    if (spot < 8)
    {
      thisSongName[spot++] = incoming;
    }
  }
  thisSongName[spot] = '\0'; //Terminate string

  return (thisSongName);
}

//Get the number of songs on the SD card (in root and subfolders)
//Limited to 255
byte MP3TRIGGER::getSongCount()
{
  sendCommand(COMMAND_GET_SONG_COUNT); //Get current song count

  return (getResponse());
}

//getFirmwareVersion() returns the firmware version as a float.
float MP3TRIGGER::getVersion()
{
  sendCommand(COMMAND_GET_VERSION);

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)2); //2 bytes for Version

  if (_i2cPort->available() == 0)
    return 0;
  float versionNumber = _i2cPort->read();
  versionNumber += (float)_i2cPort->read() / 10.0;

  return (versionNumber);
}

//Send command to Qwiic MP3 with options
boolean MP3TRIGGER::sendCommand(byte command, byte option)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(command);
  _i2cPort->write(option);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

//Send just a command to Qwiic MP3
boolean MP3TRIGGER::sendCommand(byte command)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(command);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

//Ask for a byte from Qwiic MP3
//The response depends on what the last command was
//It is often the system status but can be song count or volume level
byte MP3TRIGGER::getResponse()
{
  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)1);

  if (_i2cPort->available())
    return (_i2cPort->read());

  return (0); //Error
}

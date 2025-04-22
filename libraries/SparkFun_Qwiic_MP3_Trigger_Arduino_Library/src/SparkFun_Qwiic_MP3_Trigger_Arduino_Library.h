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

#ifndef _SPARKFUN_QWIIC_MP3_TRIGGER_ARDUINO_LIBRARY_H
#define _SPARKFUN_QWIIC_MP3_TRIGGER_ARDUINO_LIBRARY_H
#include "Arduino.h"
#include "Wire.h"

#define QWIIC_MP3_TRIGGER_ADDR 0x37 //7-bit unshifted default I2C Address

//These are the commands we can send
#define COMMAND_STOP 0x00
#define COMMAND_PLAY_TRACK 0x01		 //Play a given track number like on a CD: regardless of file names plays 2nd file in dir.
#define COMMAND_PLAY_FILENUMBER 0x02 //Play a file # from the root directory: 3 will play F003xxx.mp3
#define COMMAND_PAUSE 0x03			 //Will pause if playing, or starting playing if paused
#define COMMAND_PLAY_NEXT 0x04
#define COMMAND_PLAY_PREVIOUS 0x05
#define COMMAND_SET_EQ 0x06
#define COMMAND_SET_VOLUME 0x07
#define COMMAND_GET_SONG_COUNT 0x08 //Note: This causes song to stop playing
#define COMMAND_GET_SONG_NAME 0x09	//Fill global array with 8 characters of the song name
#define COMMAND_GET_PLAY_STATUS 0x0A
#define COMMAND_GET_CARD_STATUS 0x0B
#define COMMAND_GET_VERSION 0x0C
#define COMMAND_CLEAR_INTERRUPTS 0x0D
#define COMMAND_GET_VOLUME 0x0E
#define COMMAND_GET_EQ 0x0F
#define COMMAND_GET_ID 0x10
#define COMMAND_SET_ADDRESS 0xC7

class MP3TRIGGER
{
public:
	MP3TRIGGER();

	boolean begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = QWIIC_MP3_TRIGGER_ADDR);
	boolean isConnected(); //Returns true if device's ID is what it should be
	uint8_t getID();	   //Queries device for its ID

	boolean isPlaying();			  //Returns true if song is playing
	void setAddress(byte newAddress); //Change the I2C address of this address to newAddress
	void clearInterrupts();			  //Clear any interrupts. Currently, the only interrupt is 'track is finished playing'.

	void playTrack(byte trackNumber); //Plays a given track number. Think of this like a CD. playTrack(4) will play whatever is in the 4th position on SD.
	void playFile(byte fileNumber);	  //Plays a file that has been named specifically. For example: passing in 6 will play F006xxx.mp3
	void playNext();				  //Play the next track. The user sets the file order. This plays the next one.
	void playPrevious();			  //Play the previous track. The user sets the file order. This plays the previous one.

	void pause(); //Pause a currently playing song, or begin playing if current track is paused
	void stop();  //Stop playing the current track

	void setEQ(byte eqType);		  //Change the equalizer to one of 6 types
	uint8_t getEQ();				  //Get current EQ level (0=normal, pop, rock, jazz, classic, 5=bass)
	void setVolume(byte volumeLevel); //Change volume to zero (off) to 31 (max)
	uint8_t getVolume();			  //Get current volume level

	byte getStatus();	 //Get the current status. 0=OK, 1=Fail, 2=No such file, 5=SD Error
	boolean hasCard();	 //Returns true if MP3 player has a valid SD card
	char *getSongName(); //Returns a character array of the song currently playing, terminated with \0
	byte getSongCount(); //Get the number of songs on the SD card (in root and subfolders). Limited to 255

	float getVersion(); //getFirmwareVersion() returns the firmware version as a float.

	boolean sendCommand(byte command, byte option); //Send command to Qwiic MP3 with options
	boolean sendCommand(byte command);				//Send just a command to Qwiic MP3
	byte getResponse();								//Ask for a byte from Qwiic MP3. The response depends on what the last command was. It is often the system status but can be song count or volume level.

private:
	TwoWire *_i2cPort;
	uint8_t _deviceAddress;
};

#endif
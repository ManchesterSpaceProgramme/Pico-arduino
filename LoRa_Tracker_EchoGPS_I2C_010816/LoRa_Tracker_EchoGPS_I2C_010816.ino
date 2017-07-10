#define programname "LoRa_Tracker_EchoGPS_I2C"
#define programversion "V1.2"
#define dateproduced "010816"
#define aurthorname "Stuart Robinson"


/*
**************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 01/08/2016

HTTP://WWW.LORATRACKER.UK

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

To do;


Changes:
010816 - Standard Tracker_Definitions file and correct typos


******************************************************************************************************
*/

/*
******************************************************************************************************
Hardware definitions for the various LoRaTracker PCBs.

Most of the LoRaTracker programs will run on any of the LoRaTracker boards, the only known limitation
is that any programs using softserial input (typically for a GPS) will not work when the RFM98PIHShield
board is plugged into ATMEGA2560 base. Be careful to ensure that all Pro Minis or Arduino bases are
3.3V versions.

PIHTracker2 - Tracker PCB, 50mm x 22mm with the date 02/16, for RFM98 only
PIHTracker3 - Tracker PCB, 50mm x 22mm with the date 04/16, for RFM98 or DRF1278F
RFM98PIHShield2 - Shiled PCB, 50mm x 50mm with date April 2016, for RFM98 only

*******************************************************************************************************
*/


/*
*********************************************************************************************************
The purpose of this program is to check that a I2C UBLOX GPS is working. Characters are read from the GPS
and sent to the Serial monitor at 115200 baud. 
An I2C address of 0x42 is assumed, the program is setup to send the commands for a UBLOX GPS that should
configure it out of GLONASS mode
**********************************************************************************************************
*/



#define PIHTracker3                          //select board type
//#define RFM98PIHShield
#include <Arduino.h>
#include <Wire.h>
#include "Tracker_Definitions.h"         
#include <avr/pgmspace.h>
#include <Flash.h>                           //for storing GPS config commands in Flash

const byte GPSI2CAddress = 0x42;

void loop()                                  
{
  byte i;

  Wire.begin();

  setup_GPS_Flash();

  Serial.println();
  Serial.println();

  Wire.beginTransmission(GPSI2CAddress);
  Wire.write(0xFF);

  while (1)
  {
    Wire.requestFrom(GPSI2CAddress, (uint8_t) 1);
    i = Wire.read();
    if (i != 0xFF)
    {
     Serial.write(i);
    }
  }


}

void led_FlashStart()
{
  //flash LED to show tracker is alive
  //lasts around 5 seconds to allow plenty of time for GPS to start.
  byte i;

  for (i = 0; i <= 24; i++)
  {
    digitalWrite(PLED1, HIGH);
    delay(100);
    digitalWrite(PLED1, LOW);
    delay(100);
  }
}


void setup_GPS_Flash()
{
  //Turn off GLONASS Mode
  byte i;
  byte j;
  int ptr;
  byte byteread;
  byte Messagesize;
  Serial.println(F("Start GPS Config"));
  
  FLASH_ARRAY(byte, GPSFlash,  0x15, 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, //clear current config
                                     0x00, 0x00, 0x01, 0x19, 0x98,
                               
                               0x14, 0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, //glonass
                                     0x01, 0x01, 0x8F, 0xB2,
                               
                               0x10, 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, //GPGLL off
                               
                               0x10, 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46, //GPGLL off
                               
                               0x10, 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31, //GPGLS off
                               
                               //0x10, 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38, //GPSGSV off
                               
                               0x2C, 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, //Flight mode on
                                     0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC,
                               
                               0x15, 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, //save current config
                                     0x00, 0x00, 0x01, 0x1B, 0xA9,
                               
                               0x00);  //messages end

  ptr = 0;

  do
  {
    Messagesize = GPSFlash[ptr];
    Serial.print(F("Message size "));
    Serial.println(Messagesize);
    Serial.print(F("Sending "));

    ptr++;

    Wire.beginTransmission(GPSI2CAddress);

    for (j = 1; j <= Messagesize; j++)
    {
      byteread = GPSFlash[ptr];
      Wire.write(byteread);
      Serial.print(byteread, HEX);
      Serial.print(F(" "));
      ptr++;
    }
    Serial.println();
    Serial.println();
    Messagesize = GPSFlash[ptr];
    Wire.endTransmission();
    delay(25);
  }
  while  (Messagesize != 0x00);                //0x00 message size terminates list

}

void setup()
{
  pinMode(GPSPOWER, OUTPUT);	               //setup pin for GPS Power Control
  digitalWrite(GPSPOWER, LOW);
  pinMode(PLED1, OUTPUT);
  
  Serial.begin(115200);                         //connect at 115200 so we can read the GPS fast enough and also spit it out
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.println();
  Serial.println();
   
  led_FlashStart();
}


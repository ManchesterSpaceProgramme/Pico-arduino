#define programname "LoRa_Tracker_EchoGPS_Serial"
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
The purpose of this program is to check that a Serial GPS is working. Characters are read from the GPS
and sent to the Serial monitor at 115200 baud. 
A 9600 baud GPS is assumed, and the program can be configured to send the commands for a UBLOX GPS that
should configure it out of GLONASS mode
******************************************************************************************************
*/


#define PIHTracker3                          //select board type here  
//#define RFM98PIHShield
#define UBLOX                                //comment in if using a UBLOX GPS

#include <Arduino.h>
#include "Tracker_Definitions.h"            //definitions for programs

#include <SoftwareSerial.h>
SoftwareSerial ss(GPSRX, GPSTX);            //Create the serial connection to the GPS device


void loop()                    
{
  while (ss.available() > 0)
  Serial.write(ss.read());
}


void sendConfig(unsigned char *Config, int Length)
{
  //sends config commands to GPS
  int i;
  for (i = 0; i < Length; i++)
  {
    ss.write(Config[i]);
  }
}

void setupGPMode()
{
  //Turn off GLONASS Mode
  unsigned char setGP[] = {0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x8F, 0xB2};
  sendConfig(setGP, sizeof(setGP));
  Serial.println(F("setupGPMode"));
}


void setupFlightMode()
{
  //Configuration GPS for flight mode
  unsigned char setFlight[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendConfig(setFlight, sizeof(setFlight));
  Serial.println(F("setupFlightMode"));
}


void setupCyclic()
{
  //Put GPS in cyclic, power save mode
  unsigned char setCyclic[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 }; // Setup for Power Save Mode (Default Cyclic 1s)
  sendConfig(setCyclic, sizeof(setCyclic));
  Serial.println(F("setupCyclic"));
}



void led_FlashStart()
{
  //flash LED to show tracker is alive
  //lasts around 3 seconds to allow plenty of time for GPS to start.
  byte i;

  for (i = 0; i <= 14; i++)
  {
    digitalWrite(PLED1, HIGH);
    delay(100);
    digitalWrite(PLED1, LOW);
    delay(100);
  }
}


void setup()
{
  pinMode(GPSPOWER, OUTPUT);	               //setup pin for GPS Power Control
  digitalWrite(GPSPOWER, LOW);
 
  Serial.begin(115200);                        //connect at 115200 so we can read the GPS fast enough and also spit it out
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));

  pinMode(PLED1, OUTPUT);
  led_FlashStart();
   
  ss.begin(GPSBaud);                           //Startup soft serial
    
  #ifdef UBLOX
  setupFlightMode();
  delay(500);
  //setupCyclic();
  //delay(500);
  setupGPMode();                               //on some UBLOX GPSs setting GPMode before Cyclic mode causes it to fail
  #endif                                       //back to GLONASS mode.
  
}



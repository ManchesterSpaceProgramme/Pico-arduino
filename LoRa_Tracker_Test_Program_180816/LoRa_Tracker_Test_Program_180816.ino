#define programname "LoRaTracker Test Program"
#define programversion "V1.2"
#define dateproduced "18/08/2016"
#define aurthorname "Stuart Robinson"

/*
**************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 18/08/2016

HTTP://WWW.LORATRACKER.UK

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
free from errors.

**************************************************************************************************
*/

/*
********************************************************************************************************************************
The test program has been written to check out that the hardware and connectors for the tracker boards have been assembled correctly
such that there are funtional, with no open or short circuits. 

Whilst some of the pins and connectors have multiple purposes, the I2C pins on the Pro Mini for instance (A4 and A5) its really
only necessary to check that the pins are functional for one purpose. So whilst A4 is in one mode of operation used to read a
RC Servo pulse input, if the connector works for with an I2C device, that pin has been proven to be functional. 
********************************************************************************************************************************
*/


#define PIHTracker3
//#define Relay                         //specify PCB type
//#define ATMEGA2560                             //commment in if using ATMEGA2560 
#define ATMEGA328                                //commment in if using ATMEGA328 


#define Frequency1 434.400                       //main frequency
#define const_TrTXFreqOffset 0.000               //tracker frequency offset

#define TXPower 10                               //transmit power in dBm, 10 = 10dBm = 10mW

//**************************************************************************
//These are the settings unique to each individual tracker board   
//**************************************************************************
const float kelvin_offset = 325;                 //if temperature reports high, increase this number
const float temp_conversion_slope = 1.0;         //defines the rate of change between low and high temperatures
const long  adc_constant = 1100000;              //if voltage reports high reduce this number 
//**************************************************************************

unsigned int RCPulseLen;
int SupplyVolts;
unsigned int wADC;                                //defined here rather than in internal_CPU so its globally available
float temp;          

#include <Arduino.h>
#include "Tracker_Definitions.h"
#include <SPI.h>
#include <NewTone.h>                               //Standard Arduino tone library does not work for AFSK RTTY on a Pro Mini
#include "LoRa.h"
#include "Internal_CPU.h"
#include <Wire.h>
#include "I2C_Scanner.h"
#include "AFSK_RTTY.h"



String TestString = "Hello World - www.LoRaTracker.uk";               //This string is sent as AFSK RTTY, 7 bit, 2 Stop bit, no parity, 300 baud.                 


void loop()
{
  byte i, j, len;
  Serial.println();
  Serial.println();
  Serial.println(F("LED Flash"));
  led_FlashStart();
  digitalWrite(GPSPOWER, LOW);                       //leave GPS on
  
  SupplyVolts = read_Voltage();
  Serial.print(F("Read VCC "));
  Serial.print(SupplyVolts);
  Serial.println(F("mV"));
 
  temp = read_TEMP();
  Serial.print(F("Processor Temperature "));
  Serial.print(temp, 1);
  Serial.println(F("C"));
  
  DisplaySupplyVolts();
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
   
  Serial.print(F("Checking LoRa Device"));

  if (lora_CheckDevice() == true)
  {
    Serial.println(F(" - Present"));
    lora_Setup();				     //Do the initial LoRa Setup
    lora_SetFreq(Frequency1, const_TrTXFreqOffset);
    Serial.print(F("Transmit FM Tone"));
    digitalWrite(PLED1, HIGH);
    lora_Tone(1000, 2500, TXPower);                  //Transmit an FM tone, 1000hz, 2500ms, 10dBm
    digitalWrite(PLED1, LOW);
    Serial.println(F(" - Done"));
    lora_Print();
    Serial.println();
  }
  else
  {
    Serial.println(F(" - LoRa Device Not Found"));
    lora_Print();
    Serial.println();
    error_LoRaDevice();
  }
 
  SPI.end();
 
  Serial.print(F("Send AFSK RTTY "));
  len = TestString.length();
  len--;
  start_AFSK_RTTY();
  SendAFSKRTTY(13);
  SendAFSKRTTY(13);
  SendAFSKRTTY(13);
    
  for (i = 0; i <= len; i++)
  {
    j = TestString.charAt(i);
    SendAFSKRTTY(j);
  }
  
  SendAFSKRTTY(13);
  SendAFSKRTTY(13);
  SendAFSKRTTY(13);
  
  digitalWrite(PLED1, LOW);
  Serial.println(); 
  noNewTone(Audio_Out);
  pinMode(lora_TonePin, INPUT_PULLUP);		         //set tone pin to input, avoid possible conflict with DIO2 as output

  Serial.println(F("Checking RC Pulse"));
  checkRCpulse();

  if (RCPulseLen == 0)
  {
  setup_I2CScan();
  run_I2CScan();
  }
  else
  {
  Serial.println(F("RC Pulse Detected - Skip I2C Scan"));
  }

  delay(1500);
}


void led_FlashStart()
{
  byte i;
  for (i = 0; i <= 4; i++)
  {
    digitalWrite(PLED1, HIGH);
    digitalWrite(PLED2, HIGH);
    digitalWrite(GPSPOWER, LOW);
    delay(250);
    digitalWrite(PLED1, LOW);
    digitalWrite(PLED2, LOW);
    digitalWrite(GPSPOWER, HIGH);
    delay(250);
  }
}


int GetSupplyVolts()
{
  //relies on 1V1 internal reference and 91K & 11K resistor divider
  //returns supply in mV @ 10mV per AD bit read
  int temp;
  byte i;
  SupplyVolts = 0;
  
  #ifdef ATMEGA328
  analogReference(INTERNAL);
  #endif
  
  #ifdef ATMEGA2560
  analogReference(INTERNAL1V1);
  #endif
  
  for (i = 0; i <= 9; i++)                      //sample AD 10 times
  {
    temp = analogRead(SupplyAD);
    SupplyVolts = SupplyVolts + temp;
  }
  SupplyVolts = (int) ((SupplyVolts / 10) * ADMultiplier);
  return SupplyVolts;
}


void DisplaySupplyVolts()
{
  Serial.print(F("AD Read Supply Volts "));
  Serial.print((int) GetSupplyVolts());
  Serial.println(F("mV"));
}


void lora_Print()
{
  //prints the contents of LoRa registers to serial monitor
  byte lora_LLoopv1;
  byte lora_LLoopv2;
  byte lora_LReg;
  byte lora_LRegData;
  Serial.print(F("Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();
  lora_LReg = 0;
  for (lora_LLoopv1 = 0; lora_LLoopv1 <= 7; lora_LLoopv1++)
  {
    Serial.print(F("0x"));
    Serial.print(lora_LLoopv1, HEX);	                           //print the register number
    Serial.print(F("0  "));
    for (lora_LLoopv2 = 0; lora_LLoopv2 <= 15; lora_LLoopv2++)
    {
      lora_LRegData = lora_Read(lora_LReg);
      if (lora_LRegData < 0x10) {
        Serial.print("0");
      }
      Serial.print(lora_LRegData, HEX);                 //print the register number
      Serial.print(F(" "));
      lora_LReg++;
    }
    Serial.println();
  }
}


void  error_LoRaDevice()
{
  int i;
  SPI.end();
  pinMode(13, OUTPUT);	                               //for Pro Mini PCB LED
  for (i = 0; i <= 49; i++)
  {
    digitalWrite(PLED1, HIGH);
    digitalWrite(PLED2, HIGH);
    delay(25);
    digitalWrite(PLED1, LOW);
    digitalWrite(PLED2, LOW);
    delay(25);
  }
}


void checkRCpulse()
{
  //reads RC pulse 
  pinMode(RCPulse, INPUT); 			        
  RCPulseLen = pulseIn(RCPulse, HIGH);
  Serial.print(F("Servo Pulse "));
  Serial.print(RCPulseLen);
  Serial.println(F("uS"));
}


void setup()
{
  pinMode(PLED1, OUTPUT); 			        //for PCB LED
  pinMode(PLED2, OUTPUT); 			        //for Pro Mini LED, Pin13
  pinMode(GPSPOWER, OUTPUT); 			        //for GPS Power control
  //digitalWrite(GPSPOWER, LOW);                          //GPS on 
  pinMode(lora_TonePin, INPUT_PULLUP);		        //ensure tone out pin is input
  pinMode(lora_PReset, OUTPUT);			        //LoRa Device reset line
  pinMode (lora_PNSS, OUTPUT);			        //LoRa Device select line
  digitalWrite(lora_PNSS, HIGH);
  digitalWrite(lora_PReset, HIGH);

  Serial.begin(38400);                                   //setup Serial console ouput
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.println();
}

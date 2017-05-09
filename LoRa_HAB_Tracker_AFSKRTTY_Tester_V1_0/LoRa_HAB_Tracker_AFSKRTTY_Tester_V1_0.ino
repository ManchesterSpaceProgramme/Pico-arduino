#define programname "LoRa_HAB_Tracker_AFSK_TEST"
#define programversion "V1.0"
#define dateproduced "030816"
#define aurthorname "Stuart Robinson"
#include <Arduino.h>

/*
**************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 03/08/2016

HTTP://WWW.LORATRACKER.UK

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

To do;


******************************************************************************************************
*/



//**********************************************
// Compiler Directives
//**********************************************

#define PIHTracker3
//#define RFM98PIHShield                             //Board receiver will run on       


//Program Variables

//File Includes
#include "Tracker_Definitions.h"
#include <NewTone.h>                                 //Standard Arduino tone library does not work for AFSK RTTY on a Pro Mini 
#include "AFSK_RTTY.h"


String TestString = "MISER1,1,00:00:00,51.1111,51.1111,10008.5,3999,-10";


void loop()
{
  byte i, j, len;
  char c;
  unsigned int CRC;
  
  len = TestString.length();
  len--;

  Serial.print(F("Send AFSK RTTY "));
  start_AFSK_RTTY();
  
  SendAFSKRTTY(13);
  SendAFSKRTTY(10);
  SendAFSKRTTY(36);
  SendAFSKRTTY(36);
  SendAFSKRTTY(36);
  SendAFSKRTTY(36);

  for (i = 0; i <= len; i++)
  {
    j = TestString.charAt(i);
    SendAFSKRTTY(j);
  }
  
  CRC = 0xffff;                                  //start value for CRC16

  for (i = 0; i <= len; i++)                     //element 4 is first character after $$$$ at start
  {
    CRC ^= (((unsigned int)TestString.charAt(i)) << 8);
    for (j = 0; j < 8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  }

  SendAFSKRTTY('*');
  
  c = Hex((CRC >> 12) & 15);                      //add the checksum bytes to the end
  SendAFSKRTTY(c);
  
  c = Hex((CRC >> 8) & 15);
  SendAFSKRTTY(c);
  
  c = Hex((CRC >> 4) & 15);
  SendAFSKRTTY(c);
  
  c = Hex(CRC & 15);
  SendAFSKRTTY(c);
    
  SendAFSKRTTY(13);
  SendAFSKRTTY(10);
  noNewTone(Audio_Out);
  pinMode(lora_TonePin, INPUT_PULLUP);		  //set tone pin to input, avoid possible conflict with DIO2 as output
  Serial.println();
  delay(2000);
}


char Hex(char lchar)
{
  //used in CRC calculation
  char Table[] = "0123456789ABCDEF";
  return Table[lchar];
}


void setup()
{
  int i;
  pinMode(PLED1, OUTPUT);		         //setup pin for PCB LED
  pinMode(lora_TonePin, INPUT_PULLUP);		 //set tone pin to input, avoid possible conflict with DIO2 as output
 
  digitalWrite(lora_PReset, LOW);	         //take reset line low
  delay(5);
  digitalWrite(lora_PReset, HIGH);	         //take it high
    
  Serial.begin(38400);                           //setup Serial console ouput
  pinMode(lora_PReset, OUTPUT);
  digitalWrite(lora_PReset, LOW);
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.println();

}



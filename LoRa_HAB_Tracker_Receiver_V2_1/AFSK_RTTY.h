//AFSK_RTTY.h
/*
Standard Routine 15/06/16
******************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 15/06/2016

HTTP://WWW.LORATRACKER.UK

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

******************************************************************************************************
*/

/*
******************************************************************************************************
Sends data as AFSK RTTY at 300 baud, 7 bit, 1 start bit, 2 stop bits and no parity. Tones are 634Hz
for a 0 bit and 1000hz for 1 bit.

Can be used for transmitting ASFK RTTY over the air or for a direct link to a PC sound card

AFSKrttybaud = 1580 for 300baud, 9761 for 50baud.
******************************************************************************************************
*/



const int AFSKrttybaud = 350;              //delay in uS x 2 for 1 bit 
const int afskleadin = 500;                 //number of ms for FSK constant lead in tone
const int tonehigh = 2000;
const int tonelow = 1634;

void start_AFSK_RTTY()
{
  byte i;
#ifdef RF_AFSK
  lora_DirectSetup();                                   //set for direct mode
  lora_Write(lora_RegFdevLsb, lora_Deviation);	        //We are generating FM tones so set the deviation, 5kHz
  lora_TXONDirect(2);
#endif
  tone(lora_TonePin, tonehigh);                              //lead in is high tone
  delay(afskleadin);
}


void SendAFSKRTTY(byte chartosend)
//send the byte in chartosend as FSK RTTY, assumes mark condition (idle) is already present
//Format is 7 bits, no parity and 2 stop bits
{
  byte numbits;
  byte test;
  Serial.write(chartosend);		         //send character to serial terminal for display
  digitalWrite(PLED1, LOW);
  tone(lora_TonePin, tonelow);
  delayMicroseconds(AFSKrttybaud);               //delay for 1 bit at baud rate,start bit
  delayMicroseconds(AFSKrttybaud);
  
  for (numbits = 1;  numbits <= 7; numbits++)	 //send 7 bits, LSB first
  {
    if ((chartosend & 0x01) != 0)
    {
      digitalWrite(PLED1, HIGH);
      tone(lora_TonePin, tonehigh);
    }
    else
    {
      digitalWrite(PLED1, LOW);
      tone(lora_TonePin, tonelow); 	                //send a 0 bit, 366hz shift, low
    }

    chartosend = (chartosend / 2);		//get the next bit
    delayMicroseconds(AFSKrttybaud);
    delayMicroseconds(AFSKrttybaud);
  }
  digitalWrite(PLED1, HIGH);			//start  mark condition
  tone(lora_TonePin, tonehigh);	                //send a 1 bit, 1342hz shift, high tone
  delayMicroseconds(AFSKrttybaud);              //leave time for the stop bit
  delayMicroseconds(AFSKrttybaud);              //and another stop bit
  delayMicroseconds(AFSKrttybaud);
  delayMicroseconds(AFSKrttybaud);
}



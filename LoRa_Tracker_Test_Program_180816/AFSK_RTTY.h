//AFSK_RTTY.h
/*
******************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 18/08/2016

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

AFSKrttybaud = 1580 for 300baud, 9761 for 50baud, 350 for 1200baud

You might find 1200baud reliable enough for your needs, if not 300baud seems to work very well

This program needs the NewTone library, the embedded Arduino Tone does not work very well at all
on some Arduino platforms, OK on a ATMEGA2560 though. You can find the Library here;

https://bitbucket.org/teckel12/arduino-new-tone/wiki/Home

******************************************************************************************************
*/



const int AFSKrttybaud = 1580;                      //delay in uS x 2 for 1 bit 
const int afskleadin = 1000;                        //number of ms for FSK constant lead in tone
const int tonehigh = 2000;
const int tonelow = 1625;

void start_AFSK_RTTY()
{
  byte i;

  NewTone(Audio_Out, tonehigh);                      //lead in is high tone
  delay(afskleadin);
}


void SendAFSKRTTY(byte chartosend)
//send the byte in chartosend as FSK RTTY, assumes mark condition (idle) is already present
//Format is 7 bits, no parity and 2 stop bits
{
  byte numbits;
  byte test;
  Serial.write(chartosend);		             //send character to serial terminal for display
  digitalWrite(PLED1, LOW);
  NewTone(Audio_Out, tonelow);
  delayMicroseconds(AFSKrttybaud);                   //delay for 1 bit at baud rate,start bit
  delayMicroseconds(AFSKrttybaud);                   //double delay used to allow for total dealys > 16384uS. 
  
  for (numbits = 1;  numbits <= 7; numbits++)	     //send 7 bits, LSB first
  {
    if ((chartosend & 0x01) != 0)
    {
      digitalWrite(PLED1, HIGH);
      NewTone(Audio_Out, tonehigh);
    }
    else
    {
      digitalWrite(PLED1, LOW);
      NewTone(Audio_Out, tonelow); 	             //send a 0 bit, 366hz shift, low
    }

    chartosend = (chartosend / 2);		     //get the next bit
    delayMicroseconds(AFSKrttybaud);
    delayMicroseconds(AFSKrttybaud);
  }
  digitalWrite(PLED1, HIGH);			     //start  mark condition
  NewTone(lora_TonePin, tonehigh);	             //send a 1 bit, 1342hz shift, high tone
  delayMicroseconds(AFSKrttybaud);                   //leave time for the stop bit
  delayMicroseconds(AFSKrttybaud);                   //and another stop bit
  delayMicroseconds(AFSKrttybaud);
  delayMicroseconds(AFSKrttybaud);

}



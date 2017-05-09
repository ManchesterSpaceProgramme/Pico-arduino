//FSK_RTTY.h
/*
**************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 02/06/2016

HTTP://WWW.LORATRACKER.UK

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

Sends data as USB FSK RTTY at 100 or 200 baud, 7 bit, 1 start bit, 2 stop bits and no parity. 
Frequency shift is 366Hz.

**************************************************************************************************
*/

void SendFSKRTTY(byte chartosend);
void start_FSK_RTTY();

void SendFSKRTTY(byte chartosend)
//send the byte in chartosend as FSK RTTY, assumes mark condition (idle) is already present
//format is 7 bits, no parity and 2 stop bits
{
  byte numbits;
  byte test;
  Serial.write(chartosend);		                  //send character to serial terminal for display
  digitalWrite(PLED1, LOW);
  lora_Write(lora_RegFrLsb, const_FSKrttynoshift);        //send a 0 bit, 366hz shift, low
  delayMicroseconds(ramv_FSKbaud);                        //delay for 1 bit at baud rate,start bit

  for (numbits = 1;  numbits <= 7; numbits++)	          //send 7 bits, LSB first
  {
    if ((chartosend & 0x01) != 0)
    {
      digitalWrite(PLED1, HIGH);
      lora_Write(lora_RegFrLsb, const_FSKrttyshift);      //send a 1 bit, 1342hz shift, high tone
    }
    else
    {
      digitalWrite(PLED1, LOW);
      lora_Write(lora_RegFrLsb, const_FSKrttynoshift);    //send a 0 bit, 366hz shift, low
    }

    chartosend = (chartosend/2);		          //get the next bit
    delayMicroseconds(ramv_FSKbaud);
  }
  digitalWrite(PLED1, HIGH);			          //start  mark condition
  lora_Write(lora_RegFrLsb, const_FSKrttyshift);	  //send a 1 bit, 1342hz shift, high tone
  delayMicroseconds(ramv_FSKbaud);                        //leave time for the stop bit
  delayMicroseconds(ramv_FSKbaud);                        //and another stop bit
}


void start_FSK_RTTY()
{
  byte i;
  lora_DirectSetup();                                     //set for direct mode
  lora_SetFreq(ramv_FlightFrequency,ramv_FreqOffset);
  lora_Write(lora_RegFdevLsb, 0x00); 			  //set deviation to 0
  lora_Write(lora_RegFrLsb, const_FSKrttyshift);	  //set to high frequency, 1342hz shift

  for (i = 0; i <= const_FSKpips; i++)                    //send FSK lead in pips
  {
    digitalWrite(PLED1, HIGH);
    lora_TXONDirect(10);
    delay(50);
    digitalWrite(PLED1, LOW);
    Serial.print(F("Pip "));
    lora_TXOFF();
    delay(950);
  }
  lora_TXONDirect(10);
  delay(const_FSKleadin);
  Serial.println();
  lora_Write(lora_RegPllHop, 0xAD); 			   //set fast hop modde, needed for fast changes of frequency 
  
}





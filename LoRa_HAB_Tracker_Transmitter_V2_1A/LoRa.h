//LoRa.h
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

******************************************************************************************************
*/

float ramv_FlightFrequency;             //Initial default for FlightFrequency
float ramv_CommandFrequency;            //Initial default for command and reporting frequency
float ramv_FreqOffset;                  //adjustment for frequency in Mhz, assumed at room temp
float ramv_BindFrequency;


byte  lora_RXStart;			//start of RX packet data in BUFF
byte  lora_RXEnd;			//end of RX packet data in BUFF
byte  lora_RXPacketType;		//type number of received packet
byte  lora_RXDestination;		//destination address of received packet
byte  lora_RXSource;			//source address of received packet
byte  lora_PacketRSSI;			//RSSI of received packet
byte  lora_PacketSNR;			//signal to noise ratio of received packet
byte  lora_RXPacketL;			//length of packet received, includes source, destination and packet type
byte  lora_Power;		        //power setting for mode

unsigned long lora_TXTime = 0;          //used to record TX on time
unsigned long lora_StartTXTime;         //used to record when TX starts

unsigned long lora_RXTime = 0;          //used to record RX on time
unsigned long lora_StartRXTime;         //used to record when RX starts

char  lora_BUFF[128];			//buffer where received packets are stored


/*
**************************************************************************************************
Library Functions
**************************************************************************************************
*/

void lora_ResetDev();
void lora_Write(byte lora_LReg, byte lora_LData);
byte lora_Read(byte lora_LReg);
void lora_SetFreq(float lora_LFreq, float lora_LOffset);
byte lora_CheckDevice();
void lora_Setup();
void lora_TXONDirect(byte lora_LTXPower);
void lora_TXOFF();
void lora_DirectSetup();
void lora_Tone(int lora_LFreq, int lora_LToneLen, int lora_LTXPower );
void lora_SetModem(byte lora_LBW, byte lora_LSF, byte lora_LCR, byte lora_LHDR, byte lora_LLDROPT);
void lora_PrintModem();
void init_LoRa();
void init_LoRaCommand();
void init_LoRaBind();
void init_LoRaflightTX();
void lora_BUFFPrint(byte lora_LPrint);
void lora_RXOFF();
void lora_RXPKTInfo();
void lora_ReadPacket();
void lora_RXtoReady();
byte lora_readRXready();
void lora_PrintLinkBudget();
void lora_TXONLoRa(byte lora_LTXPower);
void lora_Send(byte lora_LTXStart, byte lora_LTXEnd, char lora_LTXPacketType, char lora_LTXDestination, char lora_LTXSource, long lora_LTXTimeout, byte lora_LTXPower, byte StripAddress);
void lora_printfreq();


void lora_ResetDev()
{
  //resets the LoRa device
  digitalWrite(lora_PReset, LOW);	// take reset line low
  delay(5);
  digitalWrite(lora_PReset, HIGH);	// take it high
}


void lora_Write(byte lora_LReg, byte lora_LData)
{
  //write a byte to a LoRa register
  digitalWrite(lora_PNSS, LOW);		// set NSS low
  SPI.transfer(lora_LReg | 0x80);	// mask address for write
  SPI.transfer(lora_LData);		// write the byte
  digitalWrite(lora_PNSS, HIGH);	// set NSS high
}


byte lora_Read(byte lora_LReg)
{
  //read a byte from a LoRa register
  byte lora_LRegData;
  digitalWrite(lora_PNSS, LOW);		// set NSS low
  SPI.transfer(lora_LReg & 0x7F);	// mask address for read
  lora_LRegData = SPI.transfer(0);	// read the byte
  digitalWrite(lora_PNSS, HIGH);	// set NSS high
  return lora_LRegData;
}


void lora_SetFreq(float lora_LFreq, float lora_LOffset)
{
  //set the LoRa frequency
  lora_LFreq =  lora_LFreq + lora_LOffset;
  byte lora_LFMsb, lora_LFMid, lora_LFLsb;
  long lora_LLongFreq;
  lora_LLongFreq = ((lora_LFreq * 1000000) / 61.03515625);
  lora_LFMsb =  lora_LLongFreq >> 16;
  lora_LFMid = (lora_LLongFreq & 0x00FF00) >> 8;
  lora_LFLsb =  (lora_LLongFreq & 0x0000FF);
  lora_Write(lora_RegFrMsb, lora_LFMsb);
  lora_Write(lora_RegFrMid, lora_LFMid);
  lora_Write(lora_RegFrLsb, lora_LFLsb);
}


byte lora_CheckDevice()
{
  //check there is a device out there, program the 3 frequency setting registers and read back
  byte lora_Lvar1;
  lora_Write(lora_RegFrMid, 0xAA);
  lora_Lvar1 = lora_Read(lora_RegFrMid);	// Read RegFrMid
  if (lora_Lvar1 != 0xAA )
  {
  return false;
  }
  else
  {
  return true;
  }
}


void lora_Setup()
{
  //initialize LoRa device registers and check its responding
  lora_ResetDev();				// Clear all registers to default
  lora_Write(lora_RegOpMode, 0x08);		// RegOpMode, need to set to sleep mode before configure for LoRa mode
  lora_Write(lora_RegOcp, 0x2B);		// RegOcp
  lora_Write(lora_RegLna, 0x23);		// RegLna
  lora_Write(lora_RegSymbTimeoutLsb, 0xFF);	// RegSymbTimeoutLsb
  lora_Write(lora_RegPreambleLsb, 0x0C);	// RegPreambleLsb, default
  lora_Write(lora_RegFdevLsb, lora_Deviation);	// LSB of deviation, 5kHz
  if (!lora_CheckDevice())
  {
    Serial.print(F("LoRa Device Error"));
    Serial.println();
  }
}


void lora_TXONDirect(byte lora_LTXPower)
{
  //turns on transmitter,in direct mode for FSK and audio  power level is from 2 to 17
  lora_StartTXTime = millis();
  lora_Write(lora_RegPaConfig, (lora_LTXPower + 0xEE));
  lora_Write(lora_RegOpMode, 0x0B);		// TX on direct mode, low frequency mode
}


void lora_TXOFF()
{
  //turns off transmitter
  unsigned long i;
  lora_Write(lora_RegOpMode, 0x08);             // TX and RX to sleep, in direct mode
  i = lora_TXTime;
  lora_TXTime = i + (millis() - lora_StartTXTime);
}
 

void lora_DirectSetup()
{
  //setup LoRa device for direct modulation mode
  lora_Write(lora_RegOpMode, 0x08);
  lora_Write(lora_RegPacketConfig2, 0x00);	// set continuous mode
}


void lora_Tone(int lora_LFreq, int lora_LToneLen, int lora_LTXPower )
{
  //Transmit an FM tone
  lora_DirectSetup();
  lora_Write(lora_RegFdevLsb, lora_Deviation);	// We are generating a tone so set the deviation, 5kHz
  lora_TXONDirect(lora_LTXPower);		// TXON, power 10
  tone(lora_TonePin, lora_LFreq);
  delay(lora_LToneLen);
  noTone(lora_TonePin);
  lora_TXOFF();
}


void lora_SetModem(byte lora_LBW, byte lora_LSF, byte lora_LCR, byte lora_LHDR, byte lora_LLDROPT)
{
  //setup the LoRa modem parameters
  //Serial.print(F("BW-"));
  //Serial.print(lora_LBW);
  //Serial.print(F(",SF-"));
  //Serial.print(lora_LSF);
  //Serial.print(F(",CR-"));
  //Serial.print(lora_LCR);
  //Serial.print(F(",DOPT-"));
  //Serial.println(lora_LLDROPT);
  //Serial.println();
  lora_Write(lora_RegOpMode, 0x08);		 // RegOpMode, need to set to sleep mode before configure for LoRa mode
  lora_Write(lora_RegOpMode, 0x88);		 // goto LoRa mode
  lora_Write(lora_RegModemConfig1, (lora_LBW + lora_LCR + lora_LHDR));
  lora_Write(lora_RegModemConfig2, (lora_LSF * 16 + 7));
  lora_Write(lora_RegModemConfig3, lora_LLDROPT);
}


void lora_PrintModem()
{
  //Print the LoRa modem parameters
  Serial.print(F("lora_PrintModem() "));
  Serial.print(lora_Read(lora_RegModemConfig1));
  Serial.print(F(" "));
  Serial.print(lora_Read(lora_RegModemConfig2));
  Serial.print(F(" "));
  Serial.println(lora_Read(lora_RegModemConfig3));
}


void init_LoRa()
{
  lora_Setup();			                 //Do the initial LoRa Setup
}


void init_LoRaCommand()
{
  //Serial.println(F("Set Command Rate"));
  lora_SetFreq(ramv_CommandFrequency,ramv_FreqOffset);
  lora_SetModem(ramv_command_Bandwidth, ramv_command_SpreadFactor, ramv_command_CodeRate, lora_Explicit, ramv_command_RateOptimisation);	//Setup the LoRa modem parameters
  lora_Power = const_command_Power;
}


void init_LoRaBind()
{
  Serial.print(F("Set Bind Frequency "));
  Serial.print(const_BindFrequency, 3);
  Serial.println(F("Mhz"));
  lora_SetFreq(const_BindFrequency, 0);
  Serial.println(F("Set Bind Rate"));
  lora_SetModem(const_bind_Bandwidth, const_bind_SpreadFactor, const_bind_CodeRate, lora_Explicit, const_bind_RateOptimisation);	//Setup the LoRa modem parameters
  lora_Power = const_bind_Power;
}


void init_LoRaflightTX()
{
  //Serial.println(F("Set FlightTX Rate"));
  lora_SetFreq(ramv_FlightFrequency,ramv_FreqOffset);
  lora_SetModem(ramv_flightTX_Bandwidth, ramv_flightTX_SpreadFactor, ramv_flightTX_CodeRate, lora_Explicit, ramv_flightTX_RateOptimisation);	//Setup the LoRa modem parameters
  lora_Power = const_flightTX_Power;
}


/*
**************************************************************************************************
Library Functions RX
**************************************************************************************************
*/

void lora_BUFFPrint(byte lora_LPrint)
{
  byte lora_LLoop;
  byte lora_LData;

  Serial.write(lora_RXPacketType);
  Serial.write(lora_RXDestination);
  Serial.write(lora_RXSource);

  for (lora_LLoop = lora_RXStart; lora_LLoop <= lora_RXEnd; lora_LLoop++)
  {
    lora_LData = lora_BUFF[lora_LLoop];
    Serial.write(lora_LData);
  }
  Serial.print(F(","));
}


void lora_RXOFF()
{
  unsigned long i;
  lora_Write(lora_RegOpMode, 0x89);                         // TX and RX to sleep, in direct mode
  lora_RXTime = (millis() - lora_StartRXTime);
}


void lora_RXPKTInfo()
{
  //print the information for packet last received
  byte lora_Lvar1;
  char lora_LChar;
 
  if (lora_PacketSNR > 127)
  {
    lora_Lvar1 =  (255 - lora_PacketSNR) / 4;
    lora_LChar = '-';
  }
  else
  {
    lora_Lvar1 = lora_PacketSNR / 4;
    lora_LChar = '+';
  }

  Serial.print(lora_LChar);
  Serial.print(lora_Lvar1);
  Serial.print(F("dB"));

  lora_Lvar1 = 137 - lora_PacketRSSI;
  Serial.print(F(",-"));
  Serial.print(lora_Lvar1);
  Serial.print(F("dB"));
  Serial.print(F(","));
  Serial.println(lora_RXPacketL);
}


void lora_ReadPacket()
{
  byte lora_Lvar1;
  byte lora_LRegData;
  lora_RXPacketL = lora_Read(lora_RegRxNbBytes);
  lora_PacketRSSI = lora_Read(lora_RegPktRssiValue);
  lora_PacketSNR = lora_Read(lora_RegPktSnrValue);
  lora_Write(lora_RegFifoAddrPtr,0);  	                  // set RX FIFO ptr
    
  digitalWrite(lora_PNSS, LOW);		                  // start the burst read
  SPI.transfer(lora_RegFifo);			          // address for burst read
  
  lora_RXPacketType = SPI.transfer(0);
  lora_RXDestination = SPI.transfer(0);
  lora_RXSource = SPI.transfer(0);
  
  lora_RXStart = 0;
  lora_RXEnd = lora_RXPacketL - 4; 
 
  for (lora_Lvar1 = lora_RXStart; lora_Lvar1 <= lora_RXEnd; lora_Lvar1++)
  {
    lora_LRegData = SPI.transfer(0);
    lora_BUFF[lora_Lvar1] = lora_LRegData;
  }
  digitalWrite(lora_PNSS, HIGH);		           // finish, turn off LoRa device
}


void lora_RXONLoRa()
{
  //puts LoRa device into listen mode and receives packet exits with packet in array lora_BUFF
  byte lora_Lvar1, lora_LRegData, lora_LLoop;
  long lora_Lvar2;
  lora_RXPacketL = 0;
  lora_RXPacketType = 0;
  lora_RXDestination = 0;
  lora_RXSource = 0;
  lora_RXStart = 0;
  lora_RXEnd = 0;
  lora_Write(lora_RegOpMode, 0x09);
  lora_Write(lora_RegFifoRxBaseAddr, 0x00);
  lora_Write(lora_RegFifoAddrPtr, 0x00);
  lora_Write(lora_RegIrqFlagsMask, 0x9F);                  // only allow rxdone and crc error
  lora_Write(lora_RegIrqFlags, 0xFF);
  lora_Write(lora_RegOpMode, 0x8D);
  lora_StartRXTime = millis();
}


byte lora_readRXready()
{
  byte lora_LRegData;
  lora_LRegData = lora_Read(lora_RegIrqFlags);
  return lora_LRegData;
}


void lora_PrintLinkBudget()
{
  byte lora_Lvar1;
  char lora_LChar;
  if (lora_PacketSNR > 127)
  {
    lora_Lvar1 =  (255 - lora_PacketSNR) / 4;
    lora_LChar = '-';
  }
  else
  {
    lora_Lvar1 = lora_PacketSNR / 4;
    lora_LChar = '+';
  }

  Serial.print(lora_LChar);
  Serial.print(lora_Lvar1);
  Serial.print(F("dB"));

  lora_Lvar1 = 137 - lora_PacketRSSI;
  Serial.print(F(",-"));
  Serial.print(lora_Lvar1);
  Serial.print(F("dB"));
}



/*
**************************************************************************************************
Library Functions TX
**************************************************************************************************
*/

void lora_TXONLoRa(byte lora_LTXPower)
{
  //turns on LoRa transmitter, Sends packet, power level is from 2 to 17
  lora_StartTXTime = millis();
  lora_Write(lora_RegPaConfig, (lora_LTXPower + 0xEE));				// set TX power
  lora_Write(lora_RegOpMode, 0x8B);  						// TX on direct mode, low frequency mode
  //Serial.print(F(" ("));                                                      // Print TX power as a check
  //Serial.print(lora_LTXPower);
  //Serial.print(F(") "));
 }


void lora_TXONLoRa2(byte lora_LTXPower)
{
  //turns on LoRa transmitter, Sends packet, power level is from 2 to 17
  byte i;
  lora_StartTXTime = millis();
  lora_Write(lora_RegPaConfig, (lora_LTXPower + 0xEE));				// set TX power
  lora_Write(lora_RegOpMode, 0x8B);						// TX on direct mode, low frequency mode
}


void lora_Send(byte lora_LTXStart, byte lora_LTXEnd, char lora_LTXPacketType, char lora_LTXDestination, char lora_LTXSource, long lora_LTXTimeout, byte lora_LTXPower, byte StripAddress)
{
  int i;
  byte lora_LRegData;
  byte lora_LTXPacketL=0;
  lora_Write(lora_RegOpMode, 0x09);
  lora_Write(lora_RegIrqFlags, 0xFF);
  lora_Write(lora_RegIrqFlagsMask, 0xF7);
  lora_Write(lora_RegFifoTxBaseAddr, 0x00);
  lora_Write(lora_RegFifoAddrPtr, 0x00);  		                        // start burst read

  digitalWrite(lora_PNSS, LOW);					                // Set NSS low
  SPI.transfer(lora_WRegFifo);					                // address for burst write
  
  if (!StripAddress)
  {		
  SPI.transfer(lora_LTXPacketType);				                // Write the packet type
  SPI.transfer(lora_LTXDestination);			                        // Destination node
  SPI.transfer(lora_LTXSource);					                // Source node
  lora_LTXPacketL = 3;							        // We have added 3 header bytes
  }
    
  for (i = lora_LTXStart;  i <= lora_LTXEnd; i++)
  {
    lora_LTXPacketL++;

    if (lora_LTXPacketL > 253)					                // check for overlong packet here, wraps around from limit at 251 to 0
    {
      lora_LTXPacketL--;						        // remove increment to packet length
      break;
    }
    lora_LRegData = lora_BUFF[i];
    SPI.transfer(lora_LRegData);
  }

  digitalWrite(lora_PNSS, HIGH);					        // finish the burst write
  lora_Write(lora_RegPayloadLength, lora_LTXPacketL);
  lora_LTXTimeout = lora_LTXTimeout * 945;			                // convert seconds to mS, delay in TX done loop is 1ms
  
  lora_TXONLoRa(lora_LTXPower);

  do
  {
    delay(1);
    lora_LTXTimeout--;
    lora_LRegData = lora_Read(lora_RegIrqFlags);
  }
  while (lora_LTXTimeout > 0 && lora_LRegData == 0) ;		                // use a timeout counter, just in case the TX sent flag is missed

  lora_TXOFF();
}






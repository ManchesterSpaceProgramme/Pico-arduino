#define programname "LoRa_HAB_Tracker_Receiver"
#define programversion "V2.1"
#define dateproduced "250616"
#define aurthorname "Stuart Robinson"
#include <Arduino.h>

/*
**************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 25/06/2016

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
// Definitions specific to this receiver
//**********************************************

const unsigned char const_ThisNode = 'G';            //node number of this tracker receiver
const unsigned char const_ControlledNode = '1';      //node number of tracker node that will be controlled (applies to some commands)
const float const_RXFreqOffset = 0.0;                //tracker receiver frequency offset Mhz
const byte default_attempts = 5;                     //default number of times a command will attempt to be sent

//**********************************************
// Compiler Directives
//**********************************************

#define PIHTracker3                                //Board receiver will run on
//#define RFM98PIHShield                           //Board receiver will run on       
#define ClearEEPROM                                //zero the counts held in EEPROM
#define WriteEEPROM                                //Write initial values of changeable constants into EEPROM
#define ReceptionReport                            //print packet type, detination and source
#define SignalReport                               //As reception report plis SNR, RSSI, Length
//#define AFSKUpload                                 //set if we want to upload to FLDIGI


//Program Variables
char ramv_ThisNode;
char ramv_ControlledNode;

unsigned int ramv_TXnoGPSloops;                     //number of transmission loops for every GPS fix (unsigned int)
unsigned int ramv_numSleeps;                        //seconds for sleep at end of TX routine  (unsigned int)
unsigned int ramv_WaitGPSFixSeconds;                //in flight mode, time to wait for a fix  (unsigned int)

byte ramv_flightTX_Bandwidth;
byte ramv_flightTX_SpreadFactor;
byte ramv_flightTX_CodeRate;
byte ramv_flightTX_RateOptimisation;
byte ramv_flightTX_Power;

byte ramv_command_Bandwidth;
byte ramv_command_SpreadFactor;
byte ramv_command_CodeRate;
byte ramv_command_RateOptimisation;
byte ramv_command_Power;
byte ramv_default_config;                           //sets the config of whats transmitted etc

//File Includes
#include <SPI.h>
#include <EEPROM.h>
#include "Tracker_Definitions.h"
#include "LoRa_Definitions.h"
#include "Flight_Settings_1.h"
#include "LoRa2.h"									//dual buffer version of LoRa routines										
#include "AFSK_RTTY.h"


void loop()
{
  checkforpacket();                                //this is assumed to be listen in flight mode

  if (lora_RXPacketType == Test)
  {
    lora_RXBuffPrint(0);
  }

  if (Serial.available() > 0)
  {
    while (Serial.read() != -1);                   //clear serial input buffer
    domenu();
    go_FlightMode();
    lora_RXONLoRa();                               //ready for next and clear flags
  }
}


void go_CommandMode()
{
  Serial.print(F("Set Command Frequency "));
  Serial.print(ramv_CommandFrequency, 3);
  Serial.println(F("Mhz"));
  lora_SetFreq(ramv_CommandFrequency, ramv_FreqOffset);
  init_LoRaCommand();
}


void go_FlightMode()
{
  Serial.print(F("Set Flight Frequency "));
  Serial.print(ramv_FlightFrequency, 3);
  Serial.println(F("Mhz"));
  lora_SetFreq(ramv_FlightFrequency, ramv_FreqOffset);
  init_LoRaflightTX();
}


void go_BindMode()
{
  Serial.print(F("Set Bind Frequency "));
  Serial.print(const_BindFrequency, 3);
  Serial.println(F("Mhz"));
  lora_SetFreq(const_BindFrequency, 0);
  Serial.println(F("Set Bind Rate"));
  init_LoRaBind();
}


byte domenu()
{
  char keypress;
  float temp_FreqOffset;
  int i;

  go_CommandMode();

  Serial.println();
  Serial.println();
  Serial.println(F("Menu"));
  Serial.println(F("----"));
  Serial.println(F("0) Link Budget Request"));
  Serial.println(F("1) Enable FSK RTTY"));
  Serial.println(F("2) Disable FSK RTTY"));
  Serial.println(F("3) Enable Address Strip"));
  Serial.println(F("4) Disable Address Strip"));
  Serial.println(F("5) Enable GPS Power Off"));
  Serial.println(F("6) Disable GPS Power Off"));
  Serial.println(F("7) Add 1KHZ to Frequency Offset"));
  Serial.println(F("8) Subtract 1KHZ from Frequency Offset"));
  Serial.println(F("A) Packet Test (0)"));
  Serial.println(F("B) FSK RTTY Test (1)"));
  Serial.println(F("C) EEPROM Change"));
  Serial.println(F("D) Listen in Bind Mode"));
  Serial.println(F("E) Listen in Command Mode"));
  Serial.println(F("F) Enable Fence Check"));
  Serial.println(F("G) Disable Fence Check"));
  Serial.println(F("X) Reset Tracker"));
  Serial.print(F("> "));
  delay(1000);

  while (Serial.read() != -1);

  while (Serial.available() == 0)
  {
    //do nothing until a keypress
  }

  keypress = Serial.read();

  if (keypress == 27)
  {
    Serial.println(F("Going back to Flight Listen mode"));
    return 1;
  }


  Serial.println(keypress);
  Serial.println();

  if (keypress == '0')
  {

    lora_TXBUFF[0] = '0';
    Serial.println(F("Send Link Budget Request "));
    lora_QueuedSend(0, 0, LinkReport, Broadcast, ramv_ThisNode, 10, lora_Power, default_attempts);	         //send a Link Report Request
    delay(inter_Packet_delay);
  }

  if (keypress == '1')
  {
    Serial.println(F("Queue Enable FSKRTTY"));
    SendConfigCommand(Config0, ramv_ControlledNode, FSKRTTYEnable, '1');
  }

  if (keypress == '2')
  {
    Serial.println(F("Queue Disable FSKRTTY"));
    SendConfigCommand(Config0, ramv_ControlledNode, FSKRTTYEnable, '0');
  }

  if (keypress == '3')
  {
    Serial.println(F("Queue Enable Address Strip"));
    SendConfigCommand(Config0, ramv_ControlledNode, AddressStrip, '1');
    //Serial.print(F("87"));
  }

  if (keypress == '4')
  {
    Serial.println(F("Queue Disable Address Strip"));
    SendConfigCommand(Config0, ramv_ControlledNode, AddressStrip, '0');
    //Serial.print(F("88"));
  }

  if (keypress == '5')
  {
    //this is lat,long and alt at low data rate
    Serial.println(F("Queue Enable GPS Power Off"));
    SendConfigCommand(Config0, ramv_ControlledNode, GPSPowerSave, '1');
  }

  if (keypress == '6')
  {
    //this is lat,long and alt at low data rate
    Serial.println(F("Queue Disable GPS Power Off"));
    SendConfigCommand(Config0, ramv_ControlledNode, GPSPowerSave, '0');
  }

  if (keypress == '7')
  {

    Serial.println(F("Add 1KHZ to Tracker Frequency Offset"));
    EEPROM.get(addr_FreqOffset, temp_FreqOffset);      //get the current tracker offset from EEPROM
    temp_FreqOffset = temp_FreqOffset + 0.001;

load_key();
    lora_TXBUFF[4] = 4;                //Number of payload bytes to write to EEPROM
    lora_TXBUFF[5] = 0x14;             //low part of EEPROM address
    lora_TXBUFF[6] = 0x00;             //high part of EEPROM address

    writeFloat(7, temp_FreqOffset);    //put the bytes for float in buffer
    lora_TXBUFF[11] = 0x00;            //End of list, no more bytes to send

    i = lora_QueuedSend(0, 11, WritePacketEEPROM, ramv_ControlledNode, ramv_ThisNode, 10, lora_Power, default_attempts);
    delay(inter_Packet_delay);

    if (i == 1)
    {
      Serial.println(F("Offset update OK - Storing new value in Local EEPROM"));
      EEPROM.put(addr_FreqOffset, temp_FreqOffset);      //remote update OK, update local copy in EEPROM
    }
    else
    {
      Serial.println(F("Offset update failed"));
    }
  }

  if (keypress == '8')
  {

    Serial.println(F("Subtract 1KHZ from Tracker Frequency Offset"));
    EEPROM.get(addr_FreqOffset, temp_FreqOffset);      //get the current tracker offset from EEPROM
    temp_FreqOffset = temp_FreqOffset - 0.001;

 load_key();
    lora_TXBUFF[4] = 4;                //Number of payload bytes to write to EEPROM
    lora_TXBUFF[5] = 0x14;             //low part of EEPROM address
    lora_TXBUFF[6] = 0x00;             //high part of EEPROM address

    writeFloat(7, temp_FreqOffset);    //put the bytes for float in buffer
    lora_TXBUFF[11] = 0x00;            //End of list, no more bytes to send

    i = lora_QueuedSend(0, 11, WritePacketEEPROM, ramv_ControlledNode, ramv_ThisNode, 10, lora_Power, default_attempts);
    delay(inter_Packet_delay);

    if (i == 1)
    {
      Serial.println(F("Offset update OK - Storing new value in Local EEPROM"));
      EEPROM.put(addr_FreqOffset, temp_FreqOffset);      //remote update OK, update local copy in EEPROM
    }
    else
    {
      Serial.println(F("Offset update failed"));
    }
  }

  if ((keypress == 'A') || (keypress == 'a'))
  {
    Serial.println(F("Queue LoRa Packet Test Request"));
    sendTestRequest('0');
  }

  if ((keypress == 'B') || (keypress == 'b'))
  {
    Serial.println(F("Queue FSK RTTY Test Request"));
    sendTestRequest('1');
  }

  if ((keypress == 'C') || (keypress == 'c'))
  {
    sendEEPROMWrite();
  }

  if ((keypress == 'D') || (keypress == 'd'))
  {
    bind_Listen();
  }

  if ((keypress == 'E') || (keypress == 'e'))
  {
    command_Listen();
  }

  if ((keypress == 'F') || (keypress == 'f'))
  {
    Serial.println(F("Queue Enable Fence Check"));
    SendConfigCommand(Config0, ramv_ControlledNode, CheckFence, '1');
  }

 if ((keypress == 'G') || (keypress == 'g'))
  {
    Serial.println(F("Queue Disable Fence Check"));
    SendConfigCommand(Config0, ramv_ControlledNode, CheckFence, '0');
  }


  if (keypress == 'X')
  {
 load_key();
    Serial.println(F("Queue Reset Tracker Request"));
    lora_QueuedSend(0, 3, ResetTracker, ramv_ControlledNode, ramv_ThisNode, 10, lora_Power, default_attempts);	         //send a reset request

    delay(inter_Packet_delay);
  }

  while (Serial.read() != -1);

  Serial.flush();
  Serial.println(F("Flight Listen Mode"));
  return 1;
}

void load_key()
{
  lora_TXBUFF[0] = key0;       //key to protect change
  lora_TXBUFF[1] = key1;
  lora_TXBUFF[2] = key2;
  lora_TXBUFF[3] = key3;  
}

void writeByte(byte addr, byte x)
{
  lora_TXBUFF[addr] = lowByte(x);
  lora_TXEnd++;
}


void writeUint(byte addr, unsigned int x)
{
  lora_TXBUFF[addr] = lowByte(x);
  lora_TXBUFF[addr + 1] = highByte(x);
  lora_TXEnd++;
  lora_TXEnd++;
}


void writeFloat(byte addr, float x)
{
  int i;
  byte j;

  union
  {
    byte b[4];
    float f;
  } data;
  data.f = x;

  for (int i = 0; i < 4; i++)
  {
    j = data.b[i];
    Serial.print(j);
    Serial.print(" ");
    lora_TXBUFF[addr + i] = j;
  }
  Serial.println();
  lora_TXEnd = lora_TXEnd + 4;
}



void bind_Listen()
{
  while (Serial.read() != -1);			//empty serial buffer

  Serial.println(F("Bind Listen Mode"));
  go_BindMode();
  lora_RXONLoRa();

  do
  {
    checkforpacket();
  }
  while (Serial.available() == 0);

  Serial.println(F("Exit Bind Listen Mode"));
}


void command_Listen()
{
  while (Serial.read() != -1);			//empty serial buffer

  Serial.println(F("Command Listen Mode"));
  go_CommandMode();
  lora_RXONLoRa();

  do
  {
    checkforpacket();
    if (lora_RXPacketType == ClearToSend)
    {
      Serial.print(F("Config Byte "));
      Serial.println(lora_RXBUFF[0]);
    }
  }
  while (Serial.available() == 0);

  Serial.println(F("Exit Command Listen Mode"));
}


void sendTestRequest(char cmd)
{

  cmd = '0';
  Serial.print(F("Send Test Request "));
  Serial.write(cmd);
  Serial.println();

  lora_TXBUFF[0] = cmd;
  lora_QueuedSend(0, 0, Test, ramv_ControlledNode, ramv_ThisNode, 10, lora_Power, default_attempts);	         //send a Test Request
  delay(inter_Packet_delay);
}


void SendConfigCommand(char confignum, char configdestination, char bitnum, char bitval)
{
  byte i;
  init_LoRaCommand();
  lora_TXBUFF[0] = bitnum;          //set the bitnum
  lora_TXBUFF[1] = bitval;          //set the bitnum
  //lora_TXBuffPrint(0);

  i = lora_QueuedSend(0, 1, Config0, configdestination, ramv_ThisNode, 10, lora_Power, default_attempts);

  //Serial.print(F("##"));

  if (i == 1)
  {
    Serial.println(F("Config Sent OK"));
  }
  else
  {
    Serial.println(F("Config Send Failed"));
  }
  Serial.println();
  delay(inter_Packet_delay);
}


void sendEEPROMWrite()
{
  //init_LoRaCommand();
  lora_TXBUFF[0] = key0;       //key to protect change
  lora_TXBUFF[1] = key1;
  lora_TXBUFF[2] = key2;
  lora_TXBUFF[3] = key3;
  lora_TXBUFF[4] = 7;          //Number of payload bytes to send
  lora_TXBUFF[5] = 0x39;       //low part of EEPROM address
  lora_TXBUFF[6] = 0x00;       //high part of EEPROM address
  lora_TXBUFF[7] = 'M';        //byte to change
  lora_TXBUFF[8] = 'i';        //byte to change
  lora_TXBUFF[9] = 's';        //byte to change
  lora_TXBUFF[10] = 'e';        //byte to change
  lora_TXBUFF[11] = 'r';        //byte to change
  lora_TXBUFF[12] = 0;        //byte to change
  lora_TXBUFF[13] = 0;        //byte to change
  lora_QueuedSend(0, 13, WritePacketEEPROM, ramv_ControlledNode, ramv_ThisNode, 10, lora_Power, default_attempts);
  delay(inter_Packet_delay);
}


byte checkforpacket()
{
  byte lora_LRegData;
  byte lora_Ltemp;

  lora_Ltemp = lora_readRXready();

  if (lora_Ltemp == 64)
  {
    lora_ReadPacket();
    processPacket();
    lora_RXONLoRa();                    //ready for next and clear flags
    return 1;
  }

  if (lora_Ltemp == 96)
  {
#ifdef ReceptionReport
    Serial.println(F("CRC Error"));
#endif
    lora_RXONLoRa();                    //ready for next
  }
  return 0;
}


void processPacket()
{
  int i;
  byte pktend;


  if (lora_RXPacketType == LongPayload )
  {
    lora_RXBuffPrint(0);                 //print packet contents as ASCII
  }

  if (lora_RXPacketType == Test)
  {
    lora_RXBuffPrint(0);                 //print packet contents as ASCII
  }

  if (lora_RXPacketType == Bind )
  {
    lora_RXBuffPrint(2);                 //print packet contents as HEX
  }

  if (lora_RXPacketType == Wakeup)
  {
    Serial.write(7);                           //print a bell
    Serial.println();
  }

  if (lora_RXPacketType == NoFix)
  {
    Serial.print(F("No GPS Fix "));                           
    Serial.println(lora_RXBUFF[0]);
  }


  if (lora_RXPacketType == LinkReport)
  {
    Serial.print(F("Tracker Received packet at "));
    lora_PrintLinkBudget();
    Serial.println();
  }


#ifdef AFSKUpload
  if (lora_RXPacketType == LongPayload )
  {
    Serial.println();
    Serial.print(F("UplinkPayload,"));

    start_AFSK_RTTY();
    
    SendAFSKRTTY(36);
    SendAFSKRTTY(36);
    SendAFSKRTTY(36);
    SendAFSKRTTY(36);
    
    for (i = lora_RXStart; i <= lora_RXEnd; i++)
    {
      SendAFSKRTTY(lora_RXBUFF[i]);
    }
    SendAFSKRTTY(13);
    SendAFSKRTTY(10);
    noTone(lora_TonePin);
    init_LoRa();                                  //will also turn off LoRa TX
    go_FlightMode();
  }
#endif

  Serial.println();
}


void systemerror()
{
  while (1)
  {
    digitalWrite(PLED1, HIGH);
    delay(100);
    digitalWrite(PLED1, LOW);
    delay(100);
  }
}




void led_FlashStart()
{
  //flash LED to show tracker is alive
  byte i;

  for (i = 0; i <= 4; i++)
  {
    digitalWrite(PLED1, HIGH);
    delay(100);
    digitalWrite(PLED1, LOW);
    delay(100);
  }
}


void write_array_EEPROM(unsigned char *Config, int Length, int EEPROM_address)
{
  int i = 0;
  byte j;
  Serial.print(F("Saving Array "));

  while (i < Length)
  {
    j = Config[i++];
    EEPROM.update(EEPROM_address++, j);
    Serial.write(j);
  }
  Serial.println();
}


void read_array_EEPROM(unsigned char *Config, unsigned int Length, unsigned int EEPROM_address)
{
  int i = 0;
  byte j;
  Serial.print(F("Retrieving Array "));

  while (i < Length)
  {
    EEPROM.get(EEPROM_address++, j);
    Config[i++] = j;
    Serial.write(j);
  }
  Serial.println();
}


void write_constantsEEPROM()
{
  //these are the tracker transmitter settings, the receiver program needs to know most of them
  //they are also stored in EEPROM so that the receiver can record and keep track of changes to
  //it may make to the settings of the remote tracker
  EEPROM.put(addr_FlightFrequency, const_FlightFrequency);
  EEPROM.put(addr_FreqOffset, const_TrTXFreqOffset);                      //this is actually the frequency offset used by the transmitter
  EEPROM.put(addr_CommandFrequency, const_CommandFrequency);
  EEPROM.put(addr_TXnoGPSloops, const_TXnoGPSloops);
  EEPROM.put(addr_numSleeps, const_numSleeps);
  EEPROM.put(addr_WaitGPSFixSeconds, const_WaitGPSFixSeconds);
  EEPROM.put(addr_default_config, const_default_config);
  EEPROM.put(addr_ThisNode, const_ThisNode);
  EEPROM.put(addr_ControlledNode, const_ControlledNode);
  EEPROM.put(addr_flightTX_Bandwidth, const_flightTX_Bandwidth);
  EEPROM.put(addr_flightTX_SpreadFactor, const_flightTX_SpreadFactor);
  EEPROM.put(addr_flightTX_CodeRate, const_flightTX_CodeRate);
  EEPROM.put(addr_flightTX_RateOptimisation, const_flightTX_RateOptimisation);
  EEPROM.put(addr_command_Bandwidth, const_command_Bandwidth);
  EEPROM.put(addr_command_SpreadFactor, const_command_SpreadFactor);
  EEPROM.put(addr_command_CodeRate, const_command_CodeRate);
  EEPROM.put(addr_command_RateOptimisation, const_command_RateOptimisation);
  //write_array_EEPROM(Flight_ID, sizeof(Flight_ID), addr_FlightID);
}


void read_constantsEEPROM()
{
  //To ensure the program routines are as common as possible betweeen transmitter and receiver
  //this receiver program uses constants in RAM copied from EEPROM in the same way as the transmitter.
  //There are some exceptions, where the local programs need to use a setting unique to the particular
  //receiver.
  EEPROM.get(addr_FlightFrequency, ramv_FlightFrequency);
  ramv_FreqOffset = const_RXFreqOffset;                       //offset for local receiver is an exception, this needs to be calibrated ans set locally
  EEPROM.get(addr_CommandFrequency, ramv_CommandFrequency);
  EEPROM.get(addr_TXnoGPSloops, ramv_TXnoGPSloops);
  EEPROM.get(addr_numSleeps, ramv_numSleeps);
  EEPROM.get(addr_WaitGPSFixSeconds, ramv_WaitGPSFixSeconds);
  EEPROM.get(addr_default_config, ramv_default_config);
  EEPROM.get(addr_ThisNode, ramv_ThisNode);
  EEPROM.get(addr_ControlledNode, ramv_ControlledNode);
  EEPROM.get(addr_flightTX_Bandwidth, ramv_flightTX_Bandwidth);
  EEPROM.get(addr_flightTX_SpreadFactor, ramv_flightTX_SpreadFactor);
  EEPROM.get(addr_flightTX_CodeRate, ramv_flightTX_CodeRate);
  EEPROM.get(addr_flightTX_RateOptimisation, ramv_flightTX_RateOptimisation);
  EEPROM.get(addr_command_Bandwidth, ramv_command_Bandwidth);
  EEPROM.get(addr_command_SpreadFactor, ramv_command_SpreadFactor);
  EEPROM.get(addr_command_CodeRate, ramv_command_CodeRate);
  EEPROM.get(addr_command_RateOptimisation, ramv_command_RateOptimisation);
  //read_array_EEPROM(Flight_ID, sizeof(Flight_ID), addr_FlightID);
}


void  do_ClearEEPROM()
{
  unsigned long i = 0;
  EEPROM.put(addr_ResetCount, i);
  EEPROM.put(addr_SequenceNum, i);
  EEPROM.put(addr_mASecs, i);
}


void setup()
{
  int i;
  pinMode(PLED1, OUTPUT);		                 //setup pin for PCB LED
  led_FlashStart();
  Serial.begin(38400);                           //setup Serial console ouput

#ifdef ClearEEPROM
  Serial.println(F("Clearing EEPROM"));
  do_ClearEEPROM();
#endif

#ifdef WriteEEPROM
  Serial.println(F("Writing Constants to EEPROM"));
  write_constantsEEPROM();
#endif

  Serial.println(F("Retrieving Variables from EEPROM"));
  read_constantsEEPROM();


  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.print(F("Node "));
  Serial.print(ramv_ThisNode);
  Serial.println();
  Serial.println();

  pinMode(lora_PReset, OUTPUT);			// LoRa Device reset line
  digitalWrite(lora_PReset, LOW);		// Reset LoRa Device
  pinMode (lora_PNSS, OUTPUT);			// set the slaveSelectPin as an output:
  digitalWrite(lora_PNSS, HIGH);

  SPI.begin();					// initialize SPI:
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  if (lora_CheckDevice() == 1)
  {
    Serial.println("LoRa Device Error");
    systemerror();
  }
  init_LoRa();
  digitalWrite(PLED1, HIGH);                   //make sure LED is off
  lora_Tone(1000, 2500, 10);                   //Transmit an FM tone, 1000hz, 1000ms, 10dBm
  digitalWrite(PLED1, LOW);                   //make sure LED is off
  delay(1000);

  /*
  for (i = 600; i >= 0; i--)
  {
  Serial.print("Waiting ");
  Serial.print(i);
  Serial.println(" Seconds");
  delay(990);
  }
  */
  go_FlightMode();
  lora_RXONLoRa();
}



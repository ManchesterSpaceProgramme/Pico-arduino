#define programname "LoRa_HAB_Tracker_Transmitter"
#define programversion "V2.1A"
#define dateproduced "040816"
#define aurthorname "Stuart Robinson"

/*
**************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 24/06/2016

HTTP://WWW.LORATRACKER.UK

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
free from errors.

Location payload is constructed thus;

PayloadID,Sequence,Time,Lat,Lon,Alt,SupplyVolts,Temperature,Resets,Config0byte,RunmAhr,GPSFixTime,Checksum
    0       1       2    3   4   5      6            7        8        9         10       11         12


General program notes
---------------------

Most of the configurable parameters such as Frequency and LoRa settings are arranged so that they are stored in EEPROM.

At the time of first programming a compiler directive (#define WriteEEPROM) needs to be selected so that the default values
are initiall written to EEPROM. Once this initial write is done the #define WriteEEPROM directive needs to be disabled and
the board re-programmed. From that point on when the tracker starts the original default values are loaded from EEPROM into
ram variables used by the program.

This arrangement then allows for the tracker parameters to be changed over the air, either locally in a binding arrangement
or remotley from a groundstation. Obviously some care is needed when changing the frequencies. The bind frequncy and the LoRa
parameters used by bind cannot be changed.

This is a list of parameters that can be changed remotely.

Frequency of tracker payload transmissions
Frequncy offset adjustment (calibration)
Frequency of tracker commands and control
LoRa parameters of payload transmissions, bandwidth, spreading factor and coding rate
LoRa parameters of command and control transmissions, bandwidth, spreading factor and coding rate
Flight ID
Node number or transmitter
Node number of control receiver
Sequence number
Number of resets
Current mAseconds
Number of seconds for main sleep
Number of payload transmissions per GPS fix
Seconds to wait for a GPS fix on hot-start
Whether to strip addressing information from start of payload
Enable or disable FSK RTTY
Enable or disble fence check
Enable or disable GPS power save


**************************************************************************************************
*/

//***************************************
// Node Definitions
//***************************************

const unsigned char const_ThisNode = '1';       //node number of this tracker transmitter
const unsigned char const_ControlNode = 'G';    //node number of ground station control node

//***************************************
// Compiler Directives
//***************************************

#include <Arduino.h>
#define PIHTracker3                             //specify PCB type


#define ClearEEPROM                            //zero the counts held in EEPROM
#define WriteEEPROM                            //Write initial values of changeable constants into EEPROM
#define CalibrateTone                           //comment in to have a calibrate tone at startup
//#define I2CGPS
#define DebugNoGPS                            //test mode, does not read GPS
//#define EnableBind                            //at startup tracker will listen for a bind packet
//#define TestLocation                          //uses test location for GPS as defined in Flight_Settings         


char ramv_ThisNode;
char ramv_ControlNode;

unsigned int ramv_TXnoGPSloops;                 //number of transmission loops for every GPS fix update
unsigned int ramv_numSleeps;                    //seconds for sleep at end of TX routine
unsigned int ramv_WaitGPSFixSeconds;            //in flight mode, time to wait for a fix
unsigned int ramv_FSKbaud;

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
byte ramv_default_config;                       //sets the config of whats transmitted etc
float Tlon;

//*************************************************************************************************

#define Output_len_max 125             		//maximum length for built payload


#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <Flash.h>
#include <TinyGPS++.h>
TinyGPSPlus gps;                                //create the TinyGPS++ object
TinyGPSCustom GNGGAFIXQ(gps, "GNGGA", 5);
TinyGPSCustom SATSINVIEW(gps, "GPGSV", 3);

#include <LowPower.h>

#include "Tracker_Definitions.h"
#include "LoRa_Definitions.h"
#include "Flight_Settings_1.h"
#include "LoRa.h"
#include "FSK_RTTY.h"
#include "GPS.h"
#include "Internal_CPU.h"


//Global Program Variables
unsigned long UPTime = 0;
unsigned long UPStart = 0;
unsigned long ramv_mASecs = 0;                             //running total of mAseconds used
unsigned long SleepmAsecs = 0;
int internal_temperature = 0;



void loop()
{
  byte i;
  unsigned long int j;

#ifndef DebugNoGPS
  gps_WaitFix(ramv_WaitGPSFixSeconds, SwitchOn, LeaveOn);
#endif

#ifdef DebugNoGPS
  delay(1000);                                              //simulate time that processor reads GPS for more accurate temperature reading
#endif

  for (i = 1; i <= ramv_TXnoGPSloops; i++)
  {
    Serial.print(F("TX Loops "));
    Serial.println(i);
    Serial.println();
    incEEPROM(addr_SequenceNum);                            //increment sequence number
    sleep_secs(5);                                          //allow time to see GPS go off

    if (!readconfigByte(CheckFence))                        //need to check if we are within fence area
    {
      Serial.println(F("Fence Check Disabled"));
      DoStuff();
    }
    else
    {
      Serial.println(F("Perform Fence Check"));
      if (do_fenceCheck())
      {
        Serial.println(F("Within Fence Area"));
        DoStuff();
      }
      else
      {
        Serial.println(F("Outside Fence Area"));
        sleep_secs(1800);                              //goto sleep for 30 Minutes
        init_LoRa();
        sendCommand(ClearToSendCommand);
        Listen(5);
        sendCommand(ClearToSendCommand);
        Listen(5);
      }
    }
    start_GPSRead();
  }
}


byte do_fenceCheck()                                        //checks to see if GPS location is within an Western and Eastern limit
{
  //there has been a fix, so check fence limits
  Tlon = gps.location.lng();
  if ((Tlon > -32.0) && (Tlon < 45.0))                      //approximates to the limits for region 1 ISM band
  {
    return 1;
  }
  return 0;
}


void PrintPayload(byte lCount)
{
  byte i;
  for (i = 0; i <= lCount; i++)
  {
    Serial.write(lora_BUFF[i]);
  }
}


void DoStuff()
{
  byte i, Count, strip_value;
  float tempfloat;
  pulseWDI();
  init_LoRa();                                                //resets then sets up LoRa device, calls lora_setFrequency

  Count = buildLongPayload(lora_BUFF);

  if (readconfigByte(FSKRTTYEnable))
  {
    Serial.print(F("Baud Delay "));
    Serial.println(ramv_FSKbaud);
    start_FSK_RTTY();
    for (i = 0; i <= Count; i++)
    {
      SendFSKRTTY(lora_BUFF[i]);
    }
    SendFSKRTTY(13);                                          //finish RTTY with carriage return
    SendFSKRTTY(10);                                          //and line feed
    digitalWrite(PLED1, LOW);
    Serial.println();
    lora_TXOFF();                                             //to ensure TXTime updated correctly
  }

  sleep_secs(1);                                              //allows time if required to switch receiver antenna to LoRa receiver

  init_LoRaflightTX();
  PrintPayload(Count);

  strip_value = readconfigByte(AddressStrip);
  Serial.println("Lora start");
  lora_Send(0, Count, LongPayload, Broadcast, ramv_ThisNode, 10, lora_Power, strip_value);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  Serial.println();

  sleep_secs(1);                                               //allow time for receiver to be ready

  sendCommand(ClearToSendCommand);                             //indicate ready for command

  Listen(wait_command);                                       //wait for command packet

  tempfloat = ramv_FlightFrequency +  ramv_FreqOffset;
  Serial.print(F("Current Flight Frequency "));
  Serial.println(tempfloat, 3);

#ifdef EnableBind
  init_LoRaBind();
  delay(inter_Packet_delay);
  Serial.println(F("Waiting Remote Bind"));
  Serial.flush();
  sendCommand(ClearToSendCommand);
  Listen(2);
#endif

  update_mAUsed();
  print_Times();

  lora_TXTime = 0;
  lora_RXTime = 0;
  UPTime = 0;
  GPSFixTime = 0;
  digitalWrite(lora_PReset, LOW);	                        //take reset line low, makes sure LoRa device is off

  sleep_secs(ramv_numSleeps);

  UPStart = millis();                                           //set the start time for UPtime
  internal_temperature = read_TEMP();                           //read temp just after sleep, when CPU is closest to ambient
}


void update_mAUsed()
{
  add_mASecs(TXmA, lora_TXTime);                                //add TXTime current consumed current to total
  add_mASecs(RXmA, lora_RXTime);                                //add RXTime current consumed current to total

  if (readconfigByte(GPSPowerSave))
  {
    add_mASecs(GPSmA, GPSFixTime);                              //add GPS consumed current to total only if power save enabled
  }

  UPTime = (millis() - UPStart);
  add_mASecs(runmA, UPTime);                                    //add run current consumed current to total
  add_mASecs(SleepmA, SleepmAsecs);
  EEPROM.put(addr_mASecs, ramv_mASecs);                         //update stored value of mASecs so it survives reset
  SleepmAsecs = 0;
}


void add_mASecs(byte lmAamp, unsigned long lmSecs)
{
  //adds to the running total of mASecs, i.e 10mA for 2000mS = 20mAmS
  //for a long unsigned int max Count = 4294967296 or 4294967 mAMins or 71582 maHr
  unsigned long i, j;
  i = (lmSecs / 1000);                                          //convert the mS time into Seconds
  j = i * lmAamp;                                               //calculate the mASecs
  ramv_mASecs = ramv_mASecs + j;
  Serial.print(F("mASecs "));
  Serial.println(ramv_mASecs);
}


void print_Times()
{
  //print the times used to calculate mAhr used
  Serial.print(F("TXTime "));
  Serial.print(lora_TXTime);
  Serial.println(F("mS"));
  Serial.print(F("RXTime "));
  Serial.print(lora_RXTime);
  Serial.println(F("mS"));
  Serial.print(F("UPTime "));
  Serial.print((millis() - UPStart));
  Serial.println(F("mS"));
  Serial.print(F("mASecs "));
  Serial.println(ramv_mASecs);
  Serial.print(F("mAHour "));
  Serial.println((ramv_mASecs) / 3600);
}


int buildLongPayload(char *lora_BUFF)
{
  //build the long tracker payload
  unsigned int i, j, volts, resets, seq;
  unsigned int CRC;
  unsigned int runmAhr;
  int temp;
  byte Count;
  float floatGPSFixTime;
  char LatArray[10], LonArray[10], AltArray[10], FixArray[5];
  char node[2];

  EEPROM.get(addr_SequenceNum, seq);
  EEPROM.get(addr_ResetCount, resets);

  runmAhr = (ramv_mASecs / 3600);
  Serial.print(F("runmAhr "));
  Serial.println(runmAhr);

  volts = read_Voltage();
  floatGPSFixTime = ( (float) GPSFixTime / (float) 1000);

#ifndef TestLocation
  dtostrf(gps.location.lat(), 7, 5, LatArray);            //format is dtostrf(FLOAT,WIDTH,PRECISION,BUFFER);
  dtostrf(gps.location.lng(), 7, 5, LonArray);
  dtostrf(gps.altitude.meters(), 0, 0, AltArray);
  dtostrf(floatGPSFixTime, 0, 1, FixArray);
#endif

#ifdef TestLocation
  dtostrf(TestLatitude, 7, 5, LatArray);
  dtostrf(TestLongitude, 7, 5, LonArray);
  dtostrf(TestAltitude, 0, 0, AltArray);
  dtostrf(floatGPSFixTime, 0, 1, FixArray);
#endif

  memset(lora_BUFF, 0, sizeof(lora_BUFF));                //clear array

  node[0] = ramv_ThisNode;
  node[1] = 0;

  snprintf(lora_BUFF,
           Output_len_max,
           "$$$$%s%s,%d,%02d:%02d:%02d,%s,%s,%s,%d,%d,%d,%d,%d,%s",
           Flight_ID,
           node,
           seq,
           gps.time.hour(),
           gps.time.minute(),
           gps.time.second(),
           LatArray,
           LonArray,
           AltArray,
           volts,
           internal_temperature,
           resets,
           ramv_default_config,
           runmAhr,
           FixArray
          );

  Count = strlen(lora_BUFF);                      //how long is the array ?

  CRC = 0xffff;                                   //start value for CRC16

  for (i = 4; i < Count; i++)                     //element 4 is first character after $$$$ at start
  {
    CRC ^= (((unsigned int)lora_BUFF[i]) << 8);
    for (j = 0; j < 8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  }

  lora_BUFF[Count++] = '*';
  lora_BUFF[Count++] = Hex((CRC >> 12) & 15);      //add the checksum bytes to the end
  lora_BUFF[Count++] = Hex((CRC >> 8) & 15);
  lora_BUFF[Count++] = Hex((CRC >> 4) & 15);
  lora_BUFF[Count] = Hex(CRC & 15);
  return Count;
}


void Listen(unsigned int seconds)
{
  //listen (seconds) for an incoming packet using the current frequency and LoRa modem settings
  unsigned long tilltime;
  tilltime = (millis() + (seconds * 1000));
  Serial.print(F("Listen "));
  Serial.println(seconds);

  init_LoRaCommand();
  lora_RXONLoRa();

  while (millis() < tilltime)
  {
    checkForPacket();
  }
  lora_RXOFF();                                     //as we have finished listening
}


byte checkForPacket()
{
  //check LoRa device to see if a packet has arrived
  byte lora_LRegData;
  byte lora_Ltemp;

  lora_Ltemp = lora_readRXready();

  if (lora_Ltemp > 0)
  {
    if (lora_Ltemp == 64)
    {
      //packet has arrived
      lora_ReadPacket();
      Serial.print(F("RX "));
      Serial.write(lora_RXPacketType);
      Serial.println();
      processPacket();
      lora_RXONLoRa();                              //ready for next and clear flags
    }
    else
    {
      //packet arrived with error
      Serial.println(F("PktErr"));
      lora_RXOFF;
      lora_RXONLoRa();
    }

  }
}


void processPacket()
{
  //we have a packet so lets decide what to do with it
  byte i, j;

  Serial.print(F("Test Req "));
  Serial.print(lora_BUFF[0]);
  Serial.println();
  if (lora_RXPacketType == Test)
  {
    if (lora_BUFF[0] == '0')
    {
      Serial.println(F("Test Req 0"));
      delay(inter_Packet_delay);
      sendCommand(ACK);
      delay(inter_Packet_delay);
      sendTest();
    }

    if (lora_BUFF[0] == '1')
    {
      Serial.println(F("Test Req 1"));
      delay(inter_Packet_delay);
      sendCommand(ACK);
      delay(inter_Packet_delay);
      sendFSKRTTYTest();
    }

  }

  if (lora_RXPacketType == LinkReport)
  {
    Serial.println(F("Send Link Report"));
    delay(inter_Packet_delay);
    sendCommand(ACK);
    sleep_secs(1);
    sendLinkReport();
  }

  if (lora_RXPacketType == Config0)                 //is it a change config byte request ?
  {
    Serial.println(F("Program Configbyte"));

    i = ((lora_BUFF[0] - 48));
    j = ((lora_BUFF[1] - 48));
    programconfigByte(i, j);
    lora_BUFFPrint(0);                              //print packet contents as ASCII
    Serial.println();
    delay(inter_Packet_delay);
    sendCommand(ACK);                               //send the ack

    //its possible there has been a command to swap GPS out of power on\off mode
    //therefore the GPS needs to be woken up and setup for cyclic power save as the
    //software backup command will cancel power save

    gpsOn();
    set_powersave_cyclic();
  }

  if (lora_RXPacketType == ResetTracker)            //is it a reset ?
  {
    lora_BUFFPrint(0);                              //print packet contents as ASCII

    if ( (lora_RXDestination == ramv_ThisNode) && (lora_RXSource == ramv_ControlNode) && (lora_BUFF[0] == key0) && (lora_BUFF[1] == key1)  && (lora_BUFF[2] == key2)  && (lora_BUFF[3] == key3) )
    {
      Serial.println(F(" - Valid reset"));
      delay(inter_Packet_delay);
      sendCommand(ACK);
      Serial.flush();
      sleep_secs(2);
      softReset();
    }
    else
    {
      Serial.println(F(" - InValid reset"));
      delay(inter_Packet_delay);
      sendCommand(NACK);
    }
  }

  if (lora_RXPacketType == WritePacketEEPROM)
  {
    Serial.println(F("WritePacketEEPROM"));
    WritepacketEEPROM();
  }
}


void softReset()
{
  asm volatile ("  jmp 0");
}


void sendCommand(char cmd)
{
  Serial.write(cmd);
  init_LoRaCommand();
  lora_Send(0, 0, cmd, Broadcast, ramv_ThisNode, 10, lora_Power, 0);
  Serial.println();
}


void sendTest()
{
  byte power;
  for (power = 10; power >= 2; power--)
  {
    init_LoRaflightTX();
    lora_BUFF[0] = '0';
    lora_BUFF[1] = power + 48;

    if (power == 10)
    {
      lora_BUFF[0] = '1';
      lora_BUFF[1] = '0';
    }
    lora_Send(0, 1, Test, Broadcast, ramv_ThisNode, 10, power, 0);     //send the test packet
    sleep_secs(2);
  }
}


void sendFSKRTTYTest()
{
  byte power;
  Serial.print(F("FSKRTTYTest()"));

  start_FSK_RTTY();
  SendFSKRTTY(13);
  SendFSKRTTY(10);

  for (power = 10; power >= 2; power--)
  {
    lora_TXONDirect(power);
    delay(200);
    SendFSKRTTY(' ');


    if (power == 10)
    {
      SendFSKRTTY('1');
      SendFSKRTTY('0');
    }
    else
    {
      SendFSKRTTY('0');
      SendFSKRTTY(power + 48);
    }
    delay(200);
  }
  SendFSKRTTY(13);
  SendFSKRTTY(10);
  init_LoRa();
}


int buildLinkReport(char *lora_BUFF)
{
  byte Count;
  snprintf(lora_BUFF,
           Output_len_max,
           ",%d,%d",
           lora_PacketSNR,
           lora_PacketRSSI
          );
  Count = (strlen(lora_BUFF) - 1);                                                //how long is array ?
  return Count;
}


void sendLinkReport()
{
  byte Count;
  Count = buildLinkReport(lora_BUFF);
  PrintPayload(Count);
  init_LoRaflightTX();
  lora_Send(0, Count, LinkReport, Broadcast, ramv_ThisNode, 10, lora_Power, 0);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  Serial.println();
}


char Hex(char lchar)
{
  //used in CRC calculation
  char Table[] = "0123456789ABCDEF";
  return Table[lchar];
}


void sleep_secs(int LNumberSleeps)
{
  int i;

  Serial.print(F("zzz "));
  Serial.println(LNumberSleeps);

  Serial.flush();                                      //let print complete
  digitalWrite(lora_PNSS, HIGH);                       //ensure LoRa Device is off

  for (i = 1; i <= LNumberSleeps; i++)
  {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);    //sleep 1 second
    pulseWDI();
  }
  SleepmAsecs = SleepmAsecs + LNumberSleeps;
  Serial.print(F("SleepmAsecs "));
  Serial.println(SleepmAsecs);
}


void incEEPROM(int laddress)
{
  unsigned int i;
  EEPROM.get(laddress, i);
  i++;
  EEPROM.put(laddress, i);
}


void programconfigByte(byte bitnum, byte bitval)
{
  //program the config byte and save in EEPROM

  EEPROM.get(addr_default_config, ramv_default_config);

  if (bitval == 0)
  {
    bitClear(ramv_default_config, bitnum);
  }
  else
  {
    bitSet(ramv_default_config, bitnum);
  }

  EEPROM.put(addr_default_config, ramv_default_config);
  Serial.print(F("Set bit "));
  Serial.print(bitnum);
  Serial.print(F(" to "));
  Serial.println(bitval);
  Serial.println(ramv_default_config, BIN);
}


byte readconfigByte(byte bitnum)
{
  //read the config byte in EEPROM
  bitnum = bitnum - 48;
  return bitRead(ramv_default_config, bitnum);
}


byte setconfigByte(byte bitnum, byte bitval)
{
  //program the config byte
  bitnum = bitnum - 48;
  //Serial.print(F("Bitnum is "));
  //Serial.println(bitnum);

  if (bitval == 0)
  {
    bitClear(ramv_default_config, bitnum);
  }
  else
  {
    bitSet(ramv_default_config, bitnum);
  }
}


void pulseWDI()
{
  //if the watchdog is fitted its needs a regular pulse top prevent reset
  digitalWrite(WDI, LOW);
  delayMicroseconds(1);
  digitalWrite(WDI, HIGH);
}


byte gps_WaitFix(unsigned long lwait, byte StartState, byte LeaveState)
{
  //waits a specified number of seconds for a fix, returns true for good fix
  //StartState when set to 1 will turn GPS on at routine start
  //LeaveState when set to 0 will turn GPS off at routine end, when there is no fix

  unsigned long endwait, millistowait, currentmillis;

  if (StartState == 1)
  {
    gpsOn();
  }

  Serial.print(F("Wait GPS Fix "));
  Serial.println(lwait);

  currentmillis = millis();
  millistowait = lwait * 1000;
  endwait = currentmillis + millistowait;

  while (millis() < endwait)
  {
    getprocessGPSchar();

    if (gps.location.isUpdated() && gps.altitude.isUpdated())
    {
      Serial.println(F("GPS Fix !"));
      if (readconfigByte(GPSPowerSave))
      {
        gpsOff();
      }
      else
      {
        Serial.println(F("No Power Save"));
      }

      setconfigByte(GPSFix, 1);
      setconfigByte(GLONASSisoutput, 0);
      return 1;
    }
    pulseWDI();
  }

  //if here then there has been no fix

  Serial.print(F("No GPS Fix"));
  Serial.print(F(" - SATs in View "));
  Serial.println(SATSINVIEW.value());


  if (GNGGAFIXQ.age() < 2000)                     //check to see if GLONASS has gone active
  {
    Serial.print(F("GLONASS Active"));
    sendCommand(GLONASSDetected);
    setconfigByte(GLONASSisoutput, 1);
    setup_GPS_Flash();
    setup_PowerSave_Flash();                      //set power save mode
    sleep_secs(1);
    setup_Cyclic_Flash();
    sleep_secs(1);
  }

  if (LeaveState == 0)
  {
    gpsOff();
  }
  setconfigByte(GPSFix, 0);
  return 0;
}


void write_array_EEPROM(unsigned char *Config, int Length, int EEPROM_address)
{
  //saves the array at the address passed into EEPROM, used to save flight ID into EEPROM
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
  //reads from EEPROM into the the array at the address passed, used to recover flight ID
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
  //saves program constants into EEPROM
  EEPROM.put(addr_FlightFrequency, const_FlightFrequency);
  EEPROM.put(addr_FreqOffset, const_TrTXFreqOffset);
  EEPROM.put(addr_CommandFrequency, const_CommandFrequency);
  EEPROM.put(addr_TXnoGPSloops, const_TXnoGPSloops);
  EEPROM.put(addr_numSleeps, const_numSleeps);
  EEPROM.put(addr_WaitGPSFixSeconds, const_WaitGPSFixSeconds);
  EEPROM.put(addr_default_config, const_default_config);
  EEPROM.put(addr_ThisNode, const_ThisNode);
  EEPROM.put(addr_ControlNode, const_ControlNode);
  EEPROM.put(addr_flightTX_Bandwidth, const_flightTX_Bandwidth);
  EEPROM.put(addr_flightTX_SpreadFactor, const_flightTX_SpreadFactor);
  EEPROM.put(addr_flightTX_CodeRate, const_flightTX_CodeRate);
  EEPROM.put(addr_flightTX_RateOptimisation, const_flightTX_RateOptimisation);
  EEPROM.put(addr_command_Bandwidth, const_command_Bandwidth);
  EEPROM.put(addr_command_SpreadFactor, const_command_SpreadFactor);
  EEPROM.put(addr_command_CodeRate, const_command_CodeRate);
  EEPROM.put(addr_command_RateOptimisation, const_command_RateOptimisation);
  EEPROM.put(addr_FSKbaud, const_FSKbaud);
  write_array_EEPROM(Flight_ID, sizeof(Flight_ID), addr_FlightID);
}


void read_constantsEEPROM()
{
  //gets program constants from EEPROM into ram
  EEPROM.get(addr_FlightFrequency, ramv_FlightFrequency);
  EEPROM.get(addr_FreqOffset, ramv_FreqOffset);
  EEPROM.get(addr_CommandFrequency, ramv_CommandFrequency);
  EEPROM.get(addr_TXnoGPSloops, ramv_TXnoGPSloops);
  EEPROM.get(addr_numSleeps, ramv_numSleeps);
  EEPROM.get(addr_WaitGPSFixSeconds, ramv_WaitGPSFixSeconds);
  EEPROM.get(addr_default_config, ramv_default_config);
  EEPROM.get(addr_ThisNode, ramv_ThisNode);
  EEPROM.get(addr_ControlNode, ramv_ControlNode);
  EEPROM.get(addr_flightTX_Bandwidth, ramv_flightTX_Bandwidth);
  EEPROM.get(addr_flightTX_SpreadFactor, ramv_flightTX_SpreadFactor);
  EEPROM.get(addr_flightTX_CodeRate, ramv_flightTX_CodeRate);
  EEPROM.get(addr_flightTX_RateOptimisation, ramv_flightTX_RateOptimisation);
  EEPROM.get(addr_command_Bandwidth, ramv_command_Bandwidth);
  EEPROM.get(addr_command_SpreadFactor, ramv_command_SpreadFactor);
  EEPROM.get(addr_command_CodeRate, ramv_command_CodeRate);
  EEPROM.get(addr_command_RateOptimisation, ramv_command_RateOptimisation);
  EEPROM.get(addr_FSKbaud, ramv_FSKbaud);
  read_array_EEPROM(Flight_ID, sizeof(Flight_ID), addr_FlightID);
  ramv_command_Power = const_command_Power;
}


byte WritepacketEEPROM()
{
  //there is an incoming packet which is a request to write bytes to EEPROM.
  //the effect is to change stored program definitions and constants
  byte i, j, k = 0;
  byte ptr;
  byte low, high;
  unsigned int addr_EEPROM;
  float tempfloat;

  //packet format is key0, key1, key2, key3, number of bytes to write, address to write to, bytes to write
  //terminate list with 0 bytes to write.
  if ((lora_BUFF[0] != key0) || (lora_BUFF[1] != key1) || (lora_BUFF[2] != key2) || (lora_BUFF[3] != key3))
  {
    Serial.print(F("Key not Valid"));
    return 0;
  }

  ptr = 4;

  j = lora_BUFF[ptr++];
  low = lora_BUFF[ptr++];
  high = lora_BUFF[ptr++];
  addr_EEPROM = (high * 256) + low;

  for (i = 1; i <= j; i++)
  {
    EEPROM.update(addr_EEPROM, lora_BUFF[ptr]);
    addr_EEPROM++;
    ptr++;
  }
  Serial.println(F("Changes Written"));
  read_constantsEEPROM();
}


float readFloat(unsigned int addr)
{
  //read 4 bytes from buffer and return as float
  int i;
  byte j;

  union
  {
    byte b[4];
    float f;
  } readdata;

  for (int i = 0; i < 4; i++)
  {
    j = lora_BUFF[addr + i];
    readdata.b[i] = j;
  }
  lora_RXEnd = lora_RXEnd + 4;
  return readdata.f;
}

byte readByte(byte addr)
{
  //read 1 bytes from buffer and return as byte
  lora_RXEnd++;
  return lora_BUFF[addr];
}


unsigned int readUint(byte addr)
{
  //read 2 bytes from buffer and return as int
  byte llowbyte, lhighbyte;

  llowbyte = lora_BUFF[addr];
  lhighbyte = lora_BUFF[addr + 1];
  lora_RXEnd++;
  lora_RXEnd++;
  return (llowbyte + (lhighbyte * 256));
}


void  do_ClearEEPROM()
{
  unsigned long i = 0;
  EEPROM.put(addr_ResetCount, i);
  EEPROM.put(addr_SequenceNum, i);
  EEPROM.put(addr_mASecs, i);
}


void send_BindInfo()
{
  //send constants stored in EEPROM as bind packet
  unsigned int i;
  byte j, ptr = 0;
  for (i = addr_StartTXEEPROMData; i <= addr_EndTXEEPROMData; i++)
  {
    EEPROM.get(i, j);
    lora_BUFF[ptr++] = j;
  }
  lora_Send(0, ptr, Bind, Broadcast, ramv_ThisNode, 10, const_bind_Power, 0);
  Serial.println(F("Bind Information sent"));
}


void set_powersave_cyclic()
{
  //setup GPS
  setup_PowerSave_Flash();                      //set power save mode
  delay(1000);
  setup_Cyclic_Flash();
  delay(1000);
}


void setup()
{
  unsigned long int i = 0;
  char j;
  byte k;
  pinMode(PLED1, OUTPUT);			//for PCB LED
  pinMode(WDI, OUTPUT);			        //for Watchdog pulse input
  Serial.begin(38400);                          //Setup Serial console ouput
  pinMode(GPSPOWER, OUTPUT);		        //in case power switching components are fitted

  gpsOn();

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

  print_Voltage();
  print_TEMP();

  Serial.print(F("Resets "));
  EEPROM.get(addr_ResetCount, i);
  Serial.println(i);

  EEPROM.get(addr_mASecs, i);                   //get saved value of mASec so total survives resets
  Serial.print(F("Stored mASecs "));
  Serial.println(i);

  incEEPROM(addr_ResetCount);

  pinMode(lora_PReset, OUTPUT);			//LoRa device reset line
  pinMode (lora_PNSS, OUTPUT);			//set the slave select pin as an output:
  digitalWrite(lora_PNSS, HIGH);
  SPI.begin();					//initialize SPI
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  init_LoRa();

#ifdef CalibrateTone
  init_LoRaflightTX();                          //FM tone on flight frequency
  lora_Tone(1000, 3000, 10);                     //Transmit an FM tone, 1000hz, 3000ms, 10dBm
  sleep_secs(1);
#endif

  sendCommand(PowerUp);
  sleep_secs(1);

#ifdef EnableBind
  init_LoRaBind();
  send_BindInfo();
  delay(inter_Packet_delay);
  Serial.println(F("Waiting Bind"));
  Serial.flush();
  sendCommand(ClearToSendCommand);
  Listen(2);
#endif

  gpsSetup();                                  //GPS should have had plenty of time to initialise by now
  start_GPSRead();


#ifndef DebugNoGPS
  while (!gps_WaitFix(20, DontSwitch, LeaveOn)) //while there is no fix
  {
    init_LoRaflightTX();                       //FM tone on flight frequency
    digitalWrite(PLED1, HIGH);
    lora_Tone(500, 200, 10);                   //Transmit an FM tone, 500hz, 250ms, 2dBm
    digitalWrite(PLED1, LOW);
    k = (byte)(*(SATSINVIEW.value()));
    lora_BUFF[0] = k;
    sendCommand(NoFix);
  }
  add_mASecs(GPSmA, GPSFixTime);               //add GPS consumed current to total
#endif

  set_powersave_cyclic();

#ifdef DebugNoGPS
  Serial.println(F("Configured for No GPS"));
#endif
}





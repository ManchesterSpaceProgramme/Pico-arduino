/*HAB_Tracker_Settings.h
**************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 01/08/2016

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


#ifdef PIHTracker3
  #define lora_DIO0 2                   //pin connected to DIO0 on LoRa device
  #define Audio_Out 3                   //pin number for external tone generation
  #define TimePulse 3                   //For reading the GPS timepulse signal
  #define lora_TonePin 4                //pin number for radio tone generation, connects to LoRa device pin DIO2
  #define WDI 5                         //pin for using a watchdog device
  #define Switch1 6                     //pin for switch input
  #define GPSPOWER 7                    //pin controls power to GPS
  #define lora_PNSS 8	                //pin number where the NSS line for the LoRa device is connected
  #define lora_PReset 9	                //pin where LoRa device reset line is connected
  #define PLED1 10                      //pin number for LED on Tracker
  #define PLED2 13                      //pin number for standard LED on pin 13 of Pro Mini
  
  #define SupplyAD A0                   //pin for supply AD
  #define GPSBACKUP A1                  //use for GPS backup power
  #define CON2_1 A1                     //connected to CON2 pin 1 on external header
  #define GPSTX A2                      //pin number for TX output - RX into GPS
  #define GPSRX A3                      //pin number for RX input - TX from GPS
  #define RCPulse A4                    //pin used to read RC pulse
  #define DisplayTX A5                  //soft serial TX pin for Digole Serial LCD
  
  #define GPSBaud 9600                  //GPS baud rate
  #define ADMultiplier 10               //multiplier for supply volts calculation
#endif


#ifdef PIHTracker2
  #define lora_DIO0 2                   //pin connected to DIO0 on LoRa device
  #define Audio_Out 3                   //pin number for external tone generation
  #define lora_TonePin 4                //pin number for radio tone generation, connects to LoRa device pin DIO2
  #define WDI 5                         //pin for using a watchdog device
  #define Switch1 6                     //pin for switch input
  
  #define lora_PNSS 8	                //pin number where the NSS line for the LoRa device is connected
  #define lora_PReset 9	                //pin where LoRa device reset line is connected
  #define PLED1 10                      //pin number for LED on Tracker
  #define PLED2 13                      //pin number for standard LED on pin 13 of Pro Mini

  #define GPSPOWER A0                   //pin controls power to GPS
  #define CON2_1 A1                     //connected to CON2 pin 1 on external header
  #define GPSTX A2                      //pin number for TX output - RX into GPS
  #define GPSRX A3                      //pin number for RX input - TX from GPS
  #define RCPulse A4                    //pin used to read RC pulse
  #define DisplayTX A5                  //soft serial TX pin for Digole Serial LCD
  
  #define SupplyAD A7                   //pin for supply AD

  #define GPSBaud 9600                  //GPS baud rate
  #define ADMultiplier 10               //multiplier for supply volts calculation
#endif


#ifdef RFM98PIHShield
  #define Switch1 2                     //pin for switch
  
  #define RCPulse 4                     //External RC Pulse sense
  
  #define lora_TonePin 6                //pin number for tone generation, connects to LoRa device DIO2.
  #define Audio_Out 6                   //pin number for external tone generation
  #define FRAM_NSS 7
  #define Buzzer 7                      //Buzzer for receiver
  #define DisplayTX 8                   //Soft serial TX pin for Digole Serial LCD
  #define PLED1 9                       //pin number for LED on Tracker
  #define GPSPOWER 10                   //Controls power to GPS
  #define PLED2 13                      //pin number for LED fitted to most shield bases
    
  #define lora_PNSS A0	                //pin number where the NSS line for the LoRa device is connected.
  #define lora_PReset A1	        //pin where LoRa device reset line is connected
  #define GPSRX A2
  #define GPSTX A3
  #define SupplyAD A6                   //pin for supply AD
     
  #define GPSBaud 9600                  //GPS baud rate
  #define ADMultiplier 10               //multiplier for supply volts calculation
#endif



#define SwitchOn 1
#define DontSwitch 0
#define Leave0ff 0
#define LeaveOn 1
#define Strip 1
#define NoStrip 0

const char ACK = 'A';
const char LinkReport = 'B';
const char bLinkReport = 'b';           //binary style link report
const char ClearToSend = 'C';
const char ClearToSendCommand = 'c';
const char Error = 'E';                      
const char NoFix = 'F';
const char NoGPS = 'G';
const char GLONASSDetected = 'g';
const char LongPayload = '$';
const char Memory = 'M';
const char NACK = 'N';
const char NACKCommand = 'n';
const char PowerUp = 'P';            
const char Repeated = 'R';
const char ShortPayload = 'S';
const char Test = 'T';
const char Wakeup = 'W';
const char ResetTracker = 'X';
const char Config1 = 'Y';
const char Config0 = 'Z';
const char WritePacketEEPROM = '0';  //Write bytes to EEPROM 
const char Bind = '#';

const char Broadcast = '*';          //broadcast address
const char PacketStart = '$';        //command packets have a payload of at least one byte, so put this at start
const int inter_Packet_delay = 500; 

//default_config byte settings, defined as chars so they appear as ASCII in received packets
const char GPSFix = '0';           //flag bit number to indicate GPS fix
const char GLONASSisoutput = '1';    //flag bit number to indicate GLONASS found
const char FSKRTTYEnable = '2';      //bit num to set in config byte to enable FSK RTTY
const char CheckFence = '3';         //bit number to control whether fence is checked 0 = NoCheck
const char ShortPayloadEnable = '4'; //bit number to control short payload
const char RepeatEnable = '5';       //bit number to control short payload
const char AddressStrip = '6';       //bit number to control Address part of payload, 1 to strip, 0 to allow
const char GPSPowerSave = '7';       //bit when set enables GPS power save.

const byte wait_command = 2;         //base time in seconds to wait for command

const byte runmA = 4;                //used by HAB program  
const byte GPSmA = 27;               //used by HAB program, Ublox MAX8Q 
const byte RXmA = 11;                //used by HAB program
const byte TXmA = 40;                //used by HAB program, 10dBm  
const int GPSShutdownTimemS = 1900;  //Software backup mode takes around 1.9secs to power down
const byte SleepmA = 1;              //approx current in sleep, GPS consumes circa 500uA


//adressing information for variables stored in EEPROM 
const unsigned int addr_StartTXEEPROMData = 0x10;         //marks the start of the stored variables 
const unsigned int addr_FlightFrequency = 0x10;           //float 4bytes
const unsigned int addr_FreqOffset = 0x14;                //float 4bytes
const unsigned int addr_CommandFrequency = 0x18;          //float 4bytes
const unsigned int addr_ResetCount = 0x1C;                //unsigned long int 4bytes
const unsigned int addr_SequenceNum = 0x20;               //unsigned long int 4bytes
const unsigned int addr_mASecs = 0x24;                    //unsigned long int 4bytes
const unsigned int addr_TXnoGPSloops = 0x28;              //unsigned int 2 bytes
const unsigned int addr_numSleeps = 0x2A;                 //unsigned int 2 bytes
const unsigned int addr_WaitGPSFixSeconds = 0x2C;         //unsigned int 2 bytes
const unsigned int addr_default_config = 0x2E;            //byte 1byte
const unsigned int addr_TXNode = 0x2F;                    //char 1byte
const unsigned int addr_RXNode = 0x30;                    //char 1byte
const unsigned int addr_flightTX_Bandwidth = 0x31;        //byte 1byte
const unsigned int addr_flightTX_SpreadFactor = 0x32;     //byte 1byte
const unsigned int addr_flightTX_CodeRate = 0x33;         //byte 1byte
const unsigned int addr_flightTX_RateOptimisation = 0x34; //byte 1byte
const unsigned int addr_command_Bandwidth = 0x35;         //byte 1byte
const unsigned int addr_command_SpreadFactor = 0x36;      //byte 1byte
const unsigned int addr_command_CodeRate = 0x37;          //byte 1byte
const unsigned int addr_command_RateOptimisation = 0x38;  //byte 1byte
const unsigned int addr_FlightID = 0x39;                  //Character array 16 bytes max
const unsigned int addr_FSKbaud = 0x49;                   //int baud rate delay 
const unsigned int addr_EndTXEEPROMData = 0x4A;           //marks the end of the stored variables 



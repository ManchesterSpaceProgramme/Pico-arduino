//Flight_Settings_1.h
/*
******************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 04/08/2016

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
This Flight definition file contains the settings for a particular tracker board and flight. 

For convienence the different boards are tested and calibrated for the VCC and temperature read 
constants plus the frequency offset for the particular LoRa device. This settings file is then copied
across to the receive program software.

Note that most of the settings below are stored in and then read from EEPROM by the tracker software.
The effect of this is that a command packet to write values to EEPROM can be used to change the
default settings of the tracker remotely without the need to reprogram the tracker. 
******************************************************************************************************
*/

unsigned char Flight_ID[10] = "PicoChu";

//**************************************************************************
//These are the settings unique to each individual tracker transmitter board   
//**************************************************************************
const float const_TrTXFreqOffset = 0.0;                         //tracker transmitter frequency offset Mhz
const float kelvin_offset = 312;                                //if temperature reports high, increase this number
const float temp_conversion_slope = 1.0;                        //defines the rate of change between low and high temperatures
const long  adc_constant = 1109860;                             //if voltage reports high reduce this number 
//**************************************************************************


const float const_FlightFrequency = 434.350;                    //flight frequency
const float const_CommandFrequency = 434.550;                   //Command and reporting frequency
const float const_BindFrequency = 434.400;                      //bind frequency 


//*********************************************************************
//LoRa settings
//*********************************************************************

//FlightTX - 1562bps
const byte const_flightTX_Bandwidth = lora_BW62_5;
const byte const_flightTX_SpreadFactor = lora_SF8;
const byte const_flightTX_CodeRate = lora_CR4_5;
const byte const_flightTX_RateOptimisation = lora_LowDoptOFF;
const byte const_flightTX_Power = 10;

//Command - 488bps
const byte const_command_Bandwidth = lora_BW62_5;
const byte const_command_SpreadFactor = lora_SF10;
const byte const_command_CodeRate = lora_CR4_5;
const byte const_command_RateOptimisation = lora_LowDoptOFF;
const byte const_command_Power = 10;

//bind - 488bps
const byte const_bind_Bandwidth = lora_BW62_5;
const byte const_bind_SpreadFactor = lora_SF10;
const byte const_bind_CodeRate = lora_CR4_5;
const byte const_bind_RateOptimisation = lora_LowDoptOFF;
const byte const_bind_Power = 10;

const unsigned long const_ResetCount = 0;            //initial value of reset count
const unsigned long const_SequenceNum = 1;           //initial value of sequence number
const unsigned long const_mASecs = 0;
const unsigned int const_TXnoGPSloops = 1;
const unsigned int const_numSleeps = 10;             //gives approx 1 minute cycle 
const unsigned int const_WaitGPSFixSeconds = 30;
const unsigned int const_default_config = 196;

const unsigned int const_FSKbaud = 9900;            //delay for baud rate for FSK RTTY, 19930 for 50baud, 9900 for 100baud, 4930 for 200baud (100baud was 9800)  
const byte const_FSKrttyshift = 105; 		    //value to write to frequency offset register for RTTY mark shift
const byte const_FSKrttynoshift = 99;		    //value to write to frequency offset register for RTTY space shift
const byte const_FSKpips = 5;                       //number of FSK lead in pips
const int  const_FSKleadin = 500;                   //number of ms for FSK constant lead in tone

const char key0 = 'H';                              //goes out with EEPROM change packet as a protection key      
const char key1 = 'A';
const char key2 = 'B';
const char key3 = '1';

#define TestLatitude 53.5831                  //Cardiff castle keep, used for testing purposes
#define TestLongitude -2.2004
#define TestAltitude 48

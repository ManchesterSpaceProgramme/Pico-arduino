//LoRa_Definitions.h
/*
******************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 06/06/2016

HTTP://WWW.LORATRACKER.UK

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

******************************************************************************************************
*/



/*
**************************************************************************************************
LoRa Constant definitions
**************************************************************************************************
*/

//LoRa names for bandwidth settings
const byte lora_BW7_8 = 0;      //7.8khz
const byte lora_BW10_4 = 16;    //10.4khz
const byte lora_BW15_6 = 32;    //15.6khz
const byte lora_BW20_8 = 48;    //20.8khz
const byte lora_BW31_2 = 64;    //31.2khz
const byte lora_BW41_7 = 80;    //41.7khz
const byte lora_BW62_5 = 96;    //62.5khz
const byte lora_BW125 = 112;    //125khz
const byte lora_BW250 = 128;    //250khz
const byte lora_BW500 = 144;    //500khz

//Spreading Factors
const byte lora_SF6 = 6;
const byte lora_SF7 = 7;
const byte lora_SF8 = 8;
const byte lora_SF9 = 9;
const byte lora_SF10 = 10;
const byte lora_SF11 = 11;
const byte lora_SF12 = 12;

//LORA names for coding rate settings
const byte lora_CR4_5 = 2;	//4:5
const byte lora_CR4_6 = 4;	//4:6
const byte lora_CR4_7 = 6;	//4:7
const byte lora_CR4_8 = 8;	//4:8

//LORA Header Settings
const byte lora_Explicit    = 0;	//Use to set explicit header
const byte lora_Implicit    = 1;	//Use to set implicit header

//Misc definitions
const byte lora_Deviation = 0x52;
const byte lora_LowDoptON = 0x08;       //value to turn low data rate optimization on
const byte lora_LowDoptOFF = 0x00;      //value to turn low data rate optimization off
const byte lora_PrintASC = 0;           //value to cause buffer print to appear as ASCII
const byte lora_PrintNum = 1;           //value to cause buffer print to appear as decimal numbers
const byte lora_PrintHEX = 2;           //value to cause buffer print to appear as hexadecimal numbers

//SX1278 Register names
const byte lora_RegFifo = 0x00;
const byte lora_WRegFifo = 0x80;
const byte lora_RegOpMode = 0x01;
const byte lora_RegFdevLsb = 0x05;
const byte lora_RegFrMsb = 0x06;
const byte lora_RegFrMid = 0x07;
const byte lora_RegFrLsb = 0x08;
const byte lora_RegPaConfig = 0x09;
const byte lora_RegOcp = 0x0B;
const byte lora_RegLna = 0x0C;
const byte lora_RegFifoAddrPtr = 0x0D;
const byte lora_RegFifoTxBaseAddr = 0x0E;
const byte lora_RegFifoRxBaseAddr = 0x0F;
const byte lora_RegFifoRxCurrentAddr = 0x10;
const byte lora_RegIrqFlagsMask = 0x11;
const byte lora_RegIrqFlags = 0x12;
const byte lora_RegRxNbBytes = 0x13;
const byte lora_RegRxHeaderCntValueMsb = 0x14;
const byte lora_RegRxHeaderCntValueLsb = 0x15;
const byte lora_RegRxPacketCntValueMsb = 0x16;
const byte lora_RegRxPacketCntValueLsb = 0x17;
const byte lora_RegPktSnrValue = 0x19;
const byte lora_RegPktRssiValue = 0x1A;
const byte lora_RegRssiValue = 0x1B;
const byte lora_RegFsiMSB = 0x1D;
const byte lora_RegFsiLSB = 0x1E;
const byte lora_RegModemConfig1 = 0x1D;
const byte lora_RegModemConfig2 = 0x1E;
const byte lora_RegSymbTimeoutLsb = 0x1F;
const byte lora_RegPreambleLsb = 0x21;
const byte lora_RegPayloadLength = 0x22;
const byte lora_RegFifoRxByteAddr = 0x25;
const byte lora_RegModemConfig3 = 0x26;
const byte lora_RegPacketConfig2 = 0x31;
const byte lora_TXdefaultpower = 10;
const byte lora_RegPllHop = 0x44;

//End of standard definitions

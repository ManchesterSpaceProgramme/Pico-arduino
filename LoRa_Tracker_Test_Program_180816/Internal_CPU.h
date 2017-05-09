//Internal_CPU.h
/*
**************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 18/08/2016

HTTP://WWW.LORATRACKER.UK

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.
******************************************************************************************************
*/

/*
Routines for reading the supply voltage of the ATMEGA328 and the internal temperature
*/

long read_Voltage();
void print_Voltage();
float read_TEMP();
void print_TEMP();


long read_Voltage()
{
long result;                                           //use 1V1 ref 
ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
delay(10);                                             //wait for ref to be stable 
ADCSRA |= _BV(ADSC);  
while (bit_is_set(ADCSRA,ADSC));                       //wait for conversion
result = ADCL; 
result |= ADCH<<8;
result = adc_constant / result;                        //calculate VCC reading in mV 
return result;
}


void print_Voltage()
{
//for the internal VCC read
Serial.print(F("VCC "));  
Serial.print(read_Voltage());
Serial.println(F("mV"));
}


float read_TEMP()
{
  // Internal temperature uses the internal 1v1 reference.
  //unsigned int wADC;
  float temp;
  
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));              //set the internal reference and mux
  ADCSRA |= _BV(ADEN);                                        //enable the ADC

  delay(20);                                                  //wait for voltages to be stable.

  ADCSRA |= _BV(ADSC);                                        //Start ADC
  
  while (bit_is_set(ADCSRA,ADSC));                            //wait for conversion

  wADC = ADCW;                                                //read the result, low and high

  Serial.print(F("wADC "));
  Serial.println(wADC);
  
  temp = (wADC - kelvin_offset ) / (temp_conversion_slope);   //use the kelvin_offset to convert to centigrade, slope scales reading


 return (temp);                                               //return temp as float in C
}


void print_TEMP()
{
Serial.print(F("Temperature "));
Serial.print(read_TEMP(),1);
Serial.println(F("C"));
}

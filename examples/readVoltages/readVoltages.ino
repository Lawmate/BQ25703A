/**
  Example of reading out the ADC registers to ascertain the different system voltages,
  power and current flows. The device needs some registers to be written as their
  default value is returned to after each device reset. There is no flash storage
  on the device, so they have to be written on every start up.

  This library is written to handle writing to and reading from the registers.
  Read the datasheet to get a full understanding of the device registers and an
  example circuit. http://www.ti.com/lit/ds/symlink/bq25703a.pdf
**/

//Libraries to be included
#include "Arduino.h"
#include <Lorro_BQ25703A.h>

//Default address for device. Note, it is without read/write bit. When read with analyser,
//this will appear 1 bit shifted to the left
#define BQ25703ADevaddr    0xD6

//Initialise the device and library
Lorro_BQ25703A BQ25703A;
const byte Lorro_BQ25703A::BQ25703Aaddr = BQ25703ADevaddr;

//Instantiate with reference to global set
extern Lorro_BQ25703A::Regt BQ25703Areg;

uint32_t previousMillis;
uint16_t loopInterval = 1000;

void setup() {

  Serial.begin(115200);

  //Set the watchdog timer to not have a timeout
  BQ25703Areg.chargeOption0.set_WDTMR_ADJ( 0 );
  BQ25703A.writeRegEx( BQ25703Areg.chargeOption0 );
  delay( 15 );

  //Set the ADC on IBAT and PSYS to record values
  //When changing bitfield values, call the writeRegEx function
  //This is so you can change all the bits you want before sending out the byte.
  BQ25703Areg.chargeOption1.set_EN_IBAT( 1 );
  BQ25703Areg.chargeOption1.set_EN_PSYS( 1 );
  BQ25703A.writeRegEx( BQ25703Areg.chargeOption1 );
  delay( 15 );

  //Set ADC to make continuous readings. (uses more power)
  BQ25703Areg.aDCOption.set_ADC_CONV( 1 );
  //Set individual ADC registers to read. All have default off.
  BQ25703Areg.aDCOption.set_EN_ADC_VBUS( 1 );
  BQ25703Areg.aDCOption.set_EN_ADC_PSYS( 1 );
  BQ25703Areg.aDCOption.set_EN_ADC_IDCHG( 1 );
  BQ25703Areg.aDCOption.set_EN_ADC_ICHG( 1 );
  BQ25703Areg.aDCOption.set_EN_ADC_VSYS( 1 );
  BQ25703Areg.aDCOption.set_EN_ADC_VBAT( 1 );
  //Once bits have been twiddled, send bytes to device
  BQ25703A.writeRegEx( BQ25703Areg.aDCOption );
  delay( 15 );

}

void loop() {

  uint32_t currentMillis = millis();

  if( currentMillis - previousMillis > loopInterval ){
    previousMillis = currentMillis;

    Serial.print( "Voltage of VBUS: " );
    Serial.print( BQ25703Areg.aDCVBUSPSYS.get_VBUS() );
    Serial.println( "mV" );
    delay( 15 );

    Serial.print( "System power usage: " );
    Serial.print( BQ25703Areg.aDCVBUSPSYS.get_sysPower() );
    Serial.println( "W" );
    delay( 15 );

    Serial.print( "Voltage of VBAT: " );
    Serial.print( BQ25703Areg.aDCVSYSVBAT.get_VBAT() );
    Serial.println( "mV" );
    delay( 15 );

    Serial.print( "Voltage of VSYS: " );
    Serial.print( BQ25703Areg.aDCVSYSVBAT.get_VSYS() );
    Serial.println( "mV" );
    delay( 15 );

    Serial.print( "Charging current: " );
    Serial.print( BQ25703Areg.aDCIBAT.get_ICHG() );
    Serial.println( "mA" );
    delay( 15 );

    Serial.print( "Voltage of VSYS: " );
    Serial.print( BQ25703Areg.aDCIBAT.get_IDCHG() );
    Serial.println( "mA" );
    delay( 15 );

  }

}

/**
  Example of how to read battery fuel percentage, total voltage, cell voltages and current
  from TI BQ4050 fuel gauge chip.
  Fuel percentage reading will not be correct unless the battery has been calibrated for this chip
**/

//Libraries to be included
#include "Arduino.h"
#include <Lorro_BQ25703A.h>

//Default address for device. Note, it is without read/write bit. When read with analyser,
//this will appear 1 bit shifted to the left
#define BQ25703Aaddr     0x0B
//Initialise the device and library
Lorro_BQ25703A BQ25703A( BQ25703Aaddr );
//Instantiate the structs
extern Lorro_BQ25703A::Regt registers;

uint32_t previousMillis;
uint16_t loopInterval = 1000;

void setup() {

  Serial.begin(115200);

}

void loop() {

  uint32_t currentMillis = millis();

  if( currentMillis - previousMillis > loopInterval ){
    previousMillis = currentMillis;

    BQ25703A.readReg( registers.relativeStateOfCharge );
    Serial.print( "State of charge: " );
    Serial.print( registers.relativeStateOfCharge.val );
    Serial.println( "%" );
    delay( 15 );

    BQ25703A.readReg( registers.voltage );
    Serial.print( "Pack voltage: " );
    Serial.print( registers.voltage.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( registers.cellVoltage1 );
    Serial.print( "Cell voltage 1: " );
    Serial.print( registers.cellVoltage1.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( registers.cellVoltage2 );
    Serial.print( "Cell voltage 2: " );
    Serial.print( registers.cellVoltage2.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( registers.cellVoltage3 );
    Serial.print( "Cell voltage 3: " );
    Serial.print( registers.cellVoltage3.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( registers.cellVoltage4 );
    Serial.print( "Cell voltage 4: " );
    Serial.print( registers.cellVoltage4.val );
    Serial.println( "mV" );
    delay( 15 );

    BQ4050.readReg( registers.current );
    Serial.print( "Current: " );
    Serial.print( registers.current.val );
    Serial.println( "mA" );
    delay( 15 );

  }

}

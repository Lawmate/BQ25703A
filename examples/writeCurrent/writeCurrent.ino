/**
  Example of setting the charge current so that charging commences. It also includes
  setting the max charge voltage. This defaults as 16800mV but in reality will fluctuate
  up above this by enough to trip a battery protector is there is one further down
  the system. This is why it is set slightly lower.
  Once the current is set, the current going into the batteries and coming out of
  them is read every second.

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

void setup(){

  //Setting the max voltage that the charger will charge the batteries up to.
  //This value gets rounded to multiples of 16mV
  //When setting numerical registers, calling the set_ function
  //will also send the bytes to the device.
  BQ25703Areg.maxChargeVoltage.set_voltage( 16400 );
  delay( 15 );

  //Sets the charge current. This needs to be set before any charging of
  //the batteries starts, as it is defaulted to 0. Any value entered will
  //be rounded to multiples of 64mA.
  BQ25703Areg.chargeCurrent.set_current( 1500 );
  delay( 15 );

  //Set the watchdog timer to not have a timeout
  //When changing bitfield values, call the writeRegEx function
  //This is so you can change all the bits you want before sending out the byte.
  BQ25703Areg.chargeOption0.set_WDTMR_ADJ( 0 );
  BQ25703A.writeRegEx( BQ25703Areg.chargeOption0 );
  delay( 15 );

  //Set the ADC on IBAT to record values
  BQ25703Areg.chargeOption1.set_EN_IBAT( 1 );
  BQ25703A.writeRegEx( BQ25703Areg.chargeOption1 );
  delay( 15 );

  //Set ADC to make continuous readings. (uses more power)
  BQ25703Areg.aDCOption.set_ADC_CONV( 1 );
  //Set individual ADC registers to read. All have default off.
  BQ25703Areg.aDCOption.set_EN_ADC_IDCHG( 1 );
  BQ25703Areg.aDCOption.set_EN_ADC_ICHG( 1 );
  //Once bits have been twiddled, send bytes to device
  BQ25703A.writeRegEx( BQ25703Areg.aDCOption );
  delay( 15 );

}

void loop(){

    uint32_t currentMillis = millis();

    if( currentMillis - previousMillis > loopInterval ){
      previousMillis = currentMillis;

      Serial.print( "Charging current: " );
      //Calls the get_IBAT() function in the struct. Calling the function
      //will read the register from the device and return the value, converted
      //into real units
      Serial.print( BQ25703Areg.aDCIBAT.get_ICHG() );
      Serial.println( "mA" );
      delay( 15 );

      Serial.print( "Discharge current: " );
      Serial.print( BQ25703Areg.aDCIBAT.get_IDCHG() );
      Serial.println( "mA" );
      delay( 15 );

    }
}

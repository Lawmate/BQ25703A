/**************************************************************************/
/*!
@file     Arduino_BQ4050.h
@author   Lorro


A library for interfacing with TI BQ4050 battery fuel gauge chip
*/
/**************************************************************************/
#include "Arduino.h"

#include <Wire.h>

#include "Lorro_BQ25703A.h"

// Prints a binary number with leading zeros (Automatic Handling)
#define PRINTBIN(Num) for (uint32_t t = (1UL<< ((sizeof(Num)*8)-1)); t; t >>= 1) Serial.write(Num  & t ? '1' : '0');

Lorro_BQ25703A::BitMaskt bitMask;


Lorro_BQ25703A::Lorro_BQ25703A( char addr ){
// Arduino_BQ4050::Arduino_BQ4050(  ){
  BQ25703Aaddr = addr;
  Wire.begin();
  // getChargerOption0();
  setChargerOption0();
  setChargerOption1();
  setContADC();
  setADCEns();
  uint16_t maxVoltage = 16480;
  setMaxChargeVoltage( maxVoltage );
}

void Lorro_BQ25703A::getVBUS(){

  setContADC();
  setADCEns();
  byte VBUSbyte = readByteReg( BQ25703Aaddr, adcVBUS );
  Serial.print( "VBUS voltage reg: " );
  PRINTBIN(VBUSbyte);
  Serial.println();
  float VBus = ( VBUSbyte * 0.064 ) + 3.2;
  Serial.print("VBUS voltage: ");
  Serial.print(VBus);
  Serial.println("V");
}

void Lorro_BQ25703A::getVSYS(){

  setContADC();
  setADCEns();
  byte VSYSbyte = readByteReg( BQ25703Aaddr, adcVSYS );
  Serial.print( "System voltage reg: " );
  PRINTBIN(VSYSbyte);
  Serial.println();
  float VSYS = ( VSYSbyte * 0.064 ) + 2.88;
  Serial.print("VSYS voltage: ");
  Serial.print(VSYS);
  Serial.println("V");
}

float Lorro_BQ25703A::getVBAT(){

  byte VBATbyte = readByteReg( BQ25703Aaddr, adcVBAT );
  Serial.print( "Battery voltage reg: " );
  PRINTBIN(VBATbyte);
  Serial.println();
  float VBAT = ( VBATbyte * 0.064 ) + 2.88;
  Serial.print("VBAT voltage: ");
  Serial.print(VBAT);
  Serial.println("V");
  return VBAT;
}

float Lorro_BQ25703A::getICHG(){
  // setChargerOption0();
  // setChargerOption1();
  // setContADC();
  // setADCEns();
  byte ICHGbyte = readByteReg( BQ25703Aaddr, adcICHG );
  // Serial.print( "Charging current reg: " );
  // PRINTBIN(ICHGbyte);
  // Serial.println();
  // printByteVal( ICHGbyte );
  float ICHG = ( ICHGbyte * 0.064 );
  // Serial.print("ICHG current: ");
  // Serial.print(ICHG);
  // Serial.println("A");
  return ICHG;
}

float Lorro_BQ25703A::getIDCHG(){

  byte IDCHGbyte = readByteReg( BQ25703Aaddr, adcIDCHG );
  Serial.print( "Discharge current reg: " );
  PRINTBIN(IDCHGbyte);
  Serial.println();
  // printByteVal( IDCHGbyte );
  float IDCHG = ( IDCHGbyte * 0.256 );
  // Serial.print("IDCHG current: ");
  // Serial.print(IDCHG);
  // Serial.println("A");
  return IDCHG;
}

float Lorro_BQ25703A::getIIN(){

  byte IINbyte = readByteReg( BQ25703Aaddr, adcIIN );
  Serial.print( "Incoming current reg: " );
  PRINTBIN(IINbyte);
  Serial.println();
  float IIN = ( IINbyte * 0.050 );
  return IIN;

}

float Lorro_BQ25703A::getCMPIN(){

  byte CMPINbyte = readByteReg( BQ25703Aaddr, adcCMPIN );
  Serial.print( "CMPIN reg: " );
  PRINTBIN(CMPINbyte);
  Serial.println();
  float CMPIN = ( CMPINbyte * 0.012 );
  return CMPIN;

}

void Lorro_BQ25703A::getManuID(){
  // byte manID = readByteReg( BQ25703Aaddr, manufacturerID );
  // Serial.print("Manufacturer ID:\t");
  // Serial.println( manID );
  // printByteVal( manID );
}

void Lorro_BQ25703A::getDevID(){
  // byte devID = readByteReg( BQ25703Aaddr, deviceID );
  // Serial.print("Device ID:\t\t");
  // Serial.println( devID );
  // printByteVal( devID );
}

void Lorro_BQ25703A::getADCOptions(){
  byte ADCOpt = readByteReg( BQ25703Aaddr, ADCOptions );
  Serial.print("ADC options:\t\t");
  Serial.println( ADCOpt , HEX);
  // printByteVal( ADCOpt );
}

void Lorro_BQ25703A::getADCEns(){
  byte ADCEn = readByteReg( BQ25703Aaddr, ADCEns );
  Serial.print("ADC enabled:\t\t");
  Serial.println( ADCEn , HEX);
  // printByteVal( ADCEn );
}

void Lorro_BQ25703A::getSysVoltage(){
  byte sysVoltage = readByteReg( BQ25703Aaddr, SysVolt );
  Serial.print("System voltage:\t\t");
  // printByteVal( sysVoltage );
  float SysVoltSetting = ( sysVoltage * 0.256 );// + 2.88;
  Serial.print(SysVoltSetting);
  Serial.println("V");
}

void Lorro_BQ25703A::getChargerStatus(){
  byte chrgStatus1 = readByteReg( BQ25703Aaddr, chargerStatus1 );
  Serial.print("Charger status 1:\t");
  Serial.println( chrgStatus1 );
  // printByteVal( chrgStatus1 );
  byte chrgStatus2 = readByteReg( BQ25703Aaddr, chargerStatus2 );
  Serial.print("Charger status 2:\t");
  Serial.println( chrgStatus2 );
  // printByteVal( chrgStatus2 );
}

void Lorro_BQ25703A::getChargeCurrent(){
  byte chrgCurrent1 = readByteReg( BQ25703Aaddr, chargeCurrentreg1 );
//  Serial.print("Charger current 1:\t");
//  printByteVal( chrgCurrent1 );
  byte chrgCurrent2 = readByteReg( BQ25703Aaddr, chargeCurrentreg2 );
//  Serial.print("Charger current 2:\t");
//  printByteVal( chrgCurrent2 );
  uint16_t chargeCurr = chrgCurrent1 << 8 | chrgCurrent2;
//  PRINTBIN(chargeCurr)
//  Serial.println();
  chargeCurr = 64 * ( chargeCurr >> 6 );
//  PRINTBIN(chargeCurr)
//  Serial.println();
  float chargeCurrent = float( chargeCurr ) / 1000;
//  PRINTBIN(chargeVolt)
//  Serial.println();
  Serial.print( "Charge current: " );
  Serial.print( chargeCurrent, 3 );
  Serial.println( " A" );
}

void Lorro_BQ25703A::getMaxChargeVoltage(){
  byte chrgVoltage1 = readByteReg( BQ25703Aaddr, maxChargeVoltageReg2 );
  byte chrgVoltage2 = readByteReg( BQ25703Aaddr, maxChargeVoltageReg1 );
  uint16_t chargeVolt = chrgVoltage1 << 8 | chrgVoltage2;
  chargeVolt = 16 * ( chargeVolt >> 4 );
  float chargeVoltage = float( chargeVolt ) / 1000;
//  PRINTBIN(chargeVolt)
//  Serial.println();
  Serial.print( "Charge voltage: " );
  Serial.print( chargeVoltage, 3 );
  Serial.println( "V" );
}

void Lorro_BQ25703A::setMaxChargeVoltage( uint16_t maxVoltage ){

  maxVoltage /= 16; //to make sure value is in 16mV steps
  maxVoltage *= 16;

  byte chrgVoltage1 = maxVoltage >> 8; //shift 8 right to preserve MSB's
  byte chrgVoltage2 = maxVoltage; //no shifting here as converting from uint16_t to byte discards 8 MSB's

  write2ByteReg( BQ25703Aaddr, maxChargeVoltageReg1, chrgVoltage2, chrgVoltage1 );

}

void Lorro_BQ25703A::getChargerOption0(){
  byte chrgOption1 = readByteReg( BQ25703Aaddr, chargerOption0reg1 );
  PRINTBIN(chrgOption1);
  Serial.println();
  byte chrgOption2 = readByteReg( BQ25703Aaddr, chargerOption0reg2 );
  PRINTBIN(chrgOption2);
  Serial.println();
  // // byte chargerOption01 = 0x06;
  // byte chargerOption01 = 0xE2;
  // PRINTBIN(chargerOption01);
  // Serial.println();
  // // writeByteReg( BQ25703Aaddr, chargerOption0reg1, chargerOption01 );
  // byte chargerOption02 = 0x0E;
  // PRINTBIN(chargerOption02);
  // Serial.println();
  // // writeByteReg( BQ25703Aaddr, chargerOption0reg2, chargerOption02 );
  // write2ByteReg( BQ25703Aaddr, chargerOption0reg2, chargerOption01, chargerOption02 );
}

void Lorro_BQ25703A::setChargerOption0(){
  //don't turn watchdog timer on
  byte chargerOption01 = 0x82;
  // PRINTBIN(chargerOption01);
  // Serial.println();
  // writeByteReg( BQ25703Aaddr, chargerOption0reg1, chargerOption01 );
  byte chargerOption02 = 0x0E;
  // PRINTBIN(chargerOption02);
  // Serial.println();
  // writeByteReg( BQ25703Aaddr, chargerOption0reg2, chargerOption02 );
  write2ByteReg( BQ25703Aaddr, chargerOption0reg2, chargerOption01, chargerOption02 );
}

void Lorro_BQ25703A::setChargerOption1(){
  // byte chargerOption11 = 0x9C;
  byte chargerOption11 = 0x92;
  writeByteReg( BQ25703Aaddr, chargerOption1reg1, chargerOption11 );
}

void Lorro_BQ25703A::setChargerCurrent( uint16_t chargeCurrentVal ){
  // uint16_t chargeCurrentVal = 2060; //mA
  uint8_t valInmA = ( chargeCurrentVal / 64 );// / 10; // divide by 10 because sense resistors are out by 10
  byte chargeCurrentByte2 = valInmA >> 2;
  byte chargeCurrentByte1 = valInmA << 6;
  write2ByteReg( BQ25703Aaddr, chargeCurrentreg2, chargeCurrentByte1, chargeCurrentByte2 );
}

void Lorro_BQ25703A::setContADC(){
  byte contADC = 0xE0;
  writeByteReg( BQ25703Aaddr, ADCOptions, contADC );
}

void Lorro_BQ25703A::setADCEns(){
  byte EnAll = 0xFF;
  // byte contADC = 0xE0;
  writeByteReg( BQ25703Aaddr, ADCEns, EnAll );
  // write2ByteReg( BQ25703Aaddr, ADCEns, contADC, EnAll );
}

void Lorro_BQ25703A::setSysVoltage(){
  byte highVoltage = 0x3a;
  writeByteReg( BQ25703Aaddr, SysVolt, highVoltage );
}





//I2C function below here
//------------------------------------------------------------------------

uint16_t Lorro_BQ25703A::read2ByteReg(char devAddress, byte regAddress){

  byte dataByte[2] = {0};
  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  Wire.endTransmission();

  Wire.requestFrom( devAddress , 3);
  if( Wire.available() > 0 ){
    dataByte[0] = Wire.receive();
    dataByte[1] = Wire.receive();
  }
  uint16_t val = ( dataByte[1] << 8 ) + dataByte[0];
  return val;

}

byte Lorro_BQ25703A::readByteReg( char devAddress, byte regAddress ){
  byte dataByte = 0;

  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  Wire.endTransmission();

  Wire.requestFrom( devAddress , 1);
  if( Wire.available() > 0 ){
    dataByte = Wire.receive();
  }
  return dataByte;
}

byte Lorro_BQ25703A::readDFByteReg( char devAddress, byte regAddress1, byte regAddress2 ){

  byte dataByte = 0;
  byte sentData[3] = { regAddress2, regAddress1 };

  Wire.beginTransmission( devAddress );
  Wire.send( 0x00 );
  Wire.send( sentData, 2 );
  Serial.print("return status: ");
  Serial.println(Wire.endTransmission());
  delay(5);

  Wire.requestFrom( devAddress , 3);
  if( Wire.available() > 0 ){
    dataByte = Wire.receive();
  }

  return dataByte;
}

byte Lorro_BQ25703A::readDFByteReg2( char devAddress, byte regAddress1, byte regAddress2 ){

  byte dataByte = 0;
//   byte sentData[3] = { regAddress2, regAddress1 };
//
//   Wire2.i2c_start( B0010110 );
//   Wire2.i2c_write( 0x44 );
//   Wire2.i2c_write( sentData[0] );
//   Wire2.i2c_write( sentData[1] );
//   Serial.print("return status: ");
//   Wire2.i2c_stop();
// //  Serial.println(Wire.endTransmission());
//   delay(5);
//
//   Wire2.i2c_start( B0010111 );
// //  Wire2.i2c_write( 0x44 );
//   dataByte = Wire2.i2c_read(false);
//   Wire2.i2c_read(false);
//   Wire2.i2c_read(false);
//   Wire2.i2c_read(false);
//   Wire2.i2c_read(false);
//   Wire2.i2c_stop();
//  Wire.write( 0x00 );
//  if( Wire.available() > 0 ){
//    dataByte = Wire.receive();
//  }

  return dataByte;
}

uint16_t Lorro_BQ25703A::readBlockReg( char devAddress, byte regAddress, byte *block ){

  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  Wire.endTransmission();

  Wire.requestFrom( devAddress , 34);
  Wire.write( 0x20 ); //write the number of bytes in the block (32)
  if( Wire.available() > 0 ){
    for(int i = 0; i < 32; i++ ){
      block[i] = Wire.receive();
    }
  }
  return *block;
}

void Lorro_BQ25703A::writeByteReg( byte devAddress, byte regAddress, byte dataByte ){

  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  Wire.write( dataByte );
  Wire.endTransmission();

}

void Lorro_BQ25703A::write2ByteReg( byte devAddress, byte regAddress, byte dataByte1, byte dataByte2 ){

  Wire.beginTransmission( devAddress );
  Wire.write( regAddress );
  Wire.write( dataByte1 );
  Wire.write( dataByte2 );
  Wire.endTransmission();

}

boolean Lorro_BQ25703A::readDataReg( char devAddress, byte regAddress, byte *dataVal, uint8_t arrLen ){
  return true;
}

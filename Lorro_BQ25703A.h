/**************************************************************************/
/*!
    @file     Arduino25703A.h
    @author   Lorro


Library for basic interfacing with BQ25703A battery management IC from TI


*/
/**************************************************************************/

#include "Arduino.h"

#define adcVBUS                               0x27
#define adcPSYS                               0x26
#define adcVSYS                               0x2D
#define adcVBAT                               0x2C
#define adcICHG  													  	0x29
#define adcIDCHG  														0x28
#define adcIIN    														0x2B
#define adcCMPIN    													0x2A
#define chargerStatus1 												0x21
#define chargerStatus2                        0x20
// #define manufacturerID                        0x2E
// #define deviceID 														  0x2D
#define ADCOptions  													0x3B
#define ADCEns                                0x3A
#define SysVolt 														  0x0D
#define chargerOption0reg1  									0x01
#define chargerOption0reg2  									0x00
#define chargerOption1reg1   									0x31
#define chargerOption1reg2                    0x30
#define chargeCurrentreg1                     0x03
#define chargeCurrentreg2                     0x02
#define maxChargeVoltageReg1 									0x04
#define maxChargeVoltageReg2                  0x05


class Lorro_BQ25703A{
 public:
	Lorro_BQ25703A( char addr );
	void getVBUS();
 	void getVSYS();
	float getVBAT();
	float getICHG();
	float getIDCHG();
	float getIIN();
	float getCMPIN();
	void getManuID();
	void getDevID();
	void getADCOptions();
	void getADCEns();
	void getSysVoltage();
	void getChargerStatus();
	void getChargeCurrent();
	void getMaxChargeVoltage();
	void setMaxChargeVoltage( uint16_t );
  void getChargerOption0();
	void setChargerOption0();
	void setChargerOption1();
	void setChargerCurrent( uint16_t chargeCurrentVal );
	void setContADC();
	void setADCEns();
	void setSysVoltage();
  char BQ25703Aaddr;
  template<typename T>
  boolean readReg( T& dataParam ){

    //This is a function for reading data words.
    //The number of bytes that make up a word is either 1 or 2.

    //Measure the number of bytes to look for
    constexpr uint8_t byteLen = sizeof( dataParam.val );
    //Create an array to hold the returned data
    byte valBytes[ byteLen ];
    //Function to handle the I2C comms.
    if( readDataReg( BQ25703Aaddr, ( byte )dataParam.addr, valBytes, byteLen ) ){
      //Cycle through array of data
      for( int i = 0; i < byteLen; i++ ){
        //Shift each byte in to the right, in steps of 8 bits. The resulting data is type cast, by getting the type with decltype
        dataParam.val = ( decltype( dataParam.val ) ) ( dataParam.val | ( valBytes[ i ] << ( 8 * i ) ) );
      }
      return true;
    }else{
      return false;
    }

  }
  struct BitMaskt{
    struct ChargerOptByte0Hight{
      byte vals[12] = {  0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
      byte masks[12] = {  0x7FFF, //byte 1, bit 7
                          0x9FFF, //byte 1, bit 6 & 5
                          0xEFFF, //byte 1, bit 4
                          0xF7FF, //byte 1, bit 3
                          0xFAFF, //byte 1, bit 2
                          0xFDFF, //byte 1, bit 1
                          0xFFDF, //byte 1, bit 5
                          0xFFEF, //byte 1, bit 4
                          0xFFF7, //byte 1, bit 3
                          0xFFFB, //byte 1, bit 2
                          0xFFFD, //byte 1, bit 1
                          0xFFFE, //byte 1, bit 0
                      };
      byte shift[6] = { 0x07, 0x06, 0x04, 0x03, 0x02, 0x01 };
      void EN_LWPWR( byte thisVal ){ vals[ 0 ] = thisVal; } //enable low power mode. Default is enabled
      void WDTMR_ADJ( byte thisVal ){ vals[ 1 ] = thisVal; } //Watchdog timer. Default is 175sec between commands (0x03)
      void IDPM_AUTO_DISABLE( byte thisVal ){ vals[ 2 ] = thisVal; } //Disable IDPM. Default is low (IDPM enabled)
      void OTG_ON_CHRGOK( byte thisVal ){ vals[ 3 ] = thisVal; } //Turn Chrgok on if OTG is enabled. Default is low
      void EN_OOA( byte thisVal ){ vals[ 4 ] = thisVal; } //Out of audio switch frequency. Default is low(disabled)
      void PWM_FREQ( byte thisVal ){ vals[ 5 ] = thisVal; } //PWM switching frequency, 800kHz or 1.2MHz. Default is high (800kHz)
      // struct EN_LWPWRt{ //enable low power mode. Default is enabled
      //   byte val = 0x01;
      //   byte mask = 0x7F; //bit 7
      //   byte shift = 7;
      // } eN_LWPWR;
      // struct WDTMR_ADJt{ //Watchdog timer. Default is 175sec between commands (0x03)
      //   byte val = 0x00;
      //   byte mask = 0x9F; //bit 6 & 5
      //   byte shift = 6;
      // } wDTMR_ADJ;
      // struct IDPM_AUTO_DISABLEt{ //Disable IDPM. Default is low (IDPM enabled)
      //   byte val = 0x00;
      //   byte mask = 0xEF; //bit 4
      //   byte shift = 4;
      // } iDPM_AUTO_DISABLE;
      // struct OTG_ON_CHRGOKt{ //Turn Chrgok on if OTG is enabled. Default is low
      //   byte val = 0x00;
      //   byte mask = 0xF7; //bit 3
      //   byte shift = 3;
      // } oTG_ON_CHRGOK;
      // struct EN_OOAt{ //Out of audio switch frequency. Default is low(disabled)
      //   byte val = 0x00;
      //   byte mask = 0xFA; //bit 2
      //   byte shift = 2;
      // } eN_OOA;
      // struct PWM_FREQt{ //PWM switching frequency, 800kHz or 1.2MHz. Default is high (800kHz)
      //   byte val = 0x01;
      //   byte mask = 0xFD; //bit 1
      //   byte shift = 1;
      // } pWM_FREQ;
    } chargerOptByte0Hight;
    struct ChargerOptByte0Lowt{
      struct EN_LEARNt{ //Learn mode. Discharges with power connected. Default disabled
        byte val = 0x00;
        byte mask = 0xDF; //bit 5
      } eN_LEARN;
      struct IADPT_GAINt{ //Current shunt amplifier 20x or 40x. Default is 20x
        byte val = 0x00;
        byte mask = 0xEF; //bit 4
      } iADPT_GAIN;
      struct IBAT_GAINt{ //Bat current shunt amplifier 8x or 16x. Default is 16x
        byte val = 0x01;
        byte mask = 0xF7; //bit 3
      } iBAT_GAIN;
      struct EN_LDOt{ //LDO mode - use of pre charge. Default is precharge enabled
        byte val = 0x01;
        byte mask = 0xFA; //bit 2
      } eN_LDO;
      struct EN_IDPMt{ //Enable IDPM current control. Default is high(enabled)
        byte val = 0x00;
        byte mask = 0xFD; //bit 1
      } eN_IDPM;
      struct CHRG_INHIBITt{ //Inhibit charging. Default is low(enabled)
        byte val = 0x00;
        byte mask = 0xFE; //bit 0
      } cHRG_INHIBIT;
    } chargerOptByte0Lowt;
  } ;
  struct Regt{
      struct ChargeOption0t{
        byte val[ 2 ] = { 0x1A, 0x34 };
        uint8_t addr = 0x00;
      } chargeOption0;
      struct ChargeCurrentt{
        uint16_t val = 2048;
        uint8_t addr = 0x02;
      } chargeCurrent;
      struct MaxChargeVoltaget{
        uint16_t val = 16800;
        uint8_t addr = 0x04;
      } maxChargeVoltage;
      struct ChargeOption1t{
        int16_t val = 0;
        uint8_t addr = 0x30;
      } chargeOption1;
      struct ChargeOption2t{
        uint16_t val = 0;
        uint8_t addr = 0x32;
      } chargeOption2;
      struct ChargeOption3t{
        uint16_t val = 0;
        uint8_t addr = 0x34;
      } chargeOption3;
      struct ProchotOption0t{
        uint16_t val = 0;
        uint8_t addr = 0x36;
      } prochotOption0;
      struct ProchotOption1t{
        uint16_t val = 0;
        uint8_t addr = 0x38;
      } prochotOption1;
      struct ADCOptiont{
        uint16_t val = 0;
        uint8_t addr = 0x3A;
      } aDCOption;
      struct ChargerStatust{
        uint16_t val = 0;
        uint8_t addr = 0x20;
      } chargerStatus;
      struct ProchotStatust{
        uint16_t val = 0;
        uint8_t addr = 0x22;
      } prochotStatus;
      struct IIN_DPMt{
        uint16_t val = 0;
        uint8_t addr = 0x24;
      } iIN_DPM;
      struct ADCVBUSPSYSt{
        uint16_t val = 0;
        uint8_t addr = 0x26;
      } aDCVBUSPSYS;
      struct ADCIBATt{
        uint16_t val = 0;
        uint8_t addr = 0x28;
      } aDCIBAT;
      struct ADCIINCMPINt{
        uint16_t val = 0;
        uint8_t addr = 0x2A;
      } aDCIINCMPIN;
      struct ADCVSYSVBATt{
        uint16_t val = 0;
        uint8_t addr = 0x2C;
      } aDCVSYSVBAT;
      struct OTGVoltaget{
        uint16_t val = 0;
        uint8_t addr = 0x06;
      } oTGVoltage;
      struct OTGCurrentt{
        uint16_t val = 0;
        uint8_t addr = 0x08;
      } oTGCurrent;
      struct InputVoltaget{
        uint16_t val = 0;
        uint8_t addr = 0x0A;
      } inputVoltage;
      struct MinSystemVoltaget{
        uint16_t val = 0;
        uint8_t addr = 0x0C;
      } minSystemVoltage;
      struct IIN_HOSTt{
        uint16_t val = 0;
        uint8_t addr = 0x0E;
      } iIN_HOST;
      struct ManufacturerIDt{
        byte val = 0x40;
        uint8_t addr = 0x2E;
      } manufacturerID;
      struct DeviceID{
        byte val = 0x78;
        uint8_t addr = 0x2F;
      } deviceID;
    } ;

 private:
	byte readByteReg( char devAddress, byte regAddress );
	byte readDFByteReg( char devAddress, byte regAddress1, byte regAddress2 );
	byte readDFByteReg2( char devAddress, byte regAddress1, byte regAddress2 );
	uint16_t read2ByteReg( char devAddress, byte regAddress );
	uint16_t readBlockReg( char devAddress, byte regAddress, byte *block );
	void writeByteReg( byte devAddress, byte regAddress, byte dataByte );
	void write2ByteReg( byte devAddress, byte regAddress, byte dataByte1, byte dataByte2 );
  boolean readDataReg( char devAddress, byte regAddress, byte *dataVal, uint8_t arrLen );

};

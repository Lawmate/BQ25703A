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
      // byte bitVals[12] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01 };
      // uint16_t masks[12] = {  0xDFFF, //byte 2, bit 5
      //                     0xEFFF, //byte 2, bit 4
      //                     0xF7FF, //byte 2, bit 3
      //                     0xFBFF, //byte 2, bit 2
      //                     0xFDFF, //byte 2, bit 1
      //                     0xFEFF, //byte 2, bit 0
      //                     0xFF7F, //byte 1, bit 7
      //                     0xFF9F, //byte 1, bit 6 & 5
      //                     0xFFEF, //byte 1, bit 4
      //                     0xFFF7, //byte 1, bit 3
      //                     0xFFFA, //byte 1, bit 2
      //                     0xFFFD, //byte 1, bit 1
      //                 };
      // byte shift[6] = { 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x07, 0x06, 0x04, 0x03, 0x02, 0x01 };
      // //Learn mode. Discharges with power connected. Default disabled
      // void EN_LEARN( byte thisVal ){ bitVals[ 0 ] = thisVal; }
      // //Current shunt amplifier 20x or 40x. Default is 20x
      // void IADPT_GAIN( byte thisVal ){ bitVals[ 1 ] = thisVal; }
      // //Bat current shunt amplifier 8x or 16x. Default is 16x
      // void IBAT_GAIN( byte thisVal ){ bitVals[ 2 ] = thisVal; }
      // //LDO mode - use of pre charge. Default is precharge enabled
      // void EN_LDO( byte thisVal ){ bitVals[ 3 ] = thisVal; }
      // //Enable IDPM current control. Default is high(enabled)
      // void EN_IDPM( byte thisVal ){ bitVals[ 4 ] = thisVal; }
      // //Inhibit charging. Default is low(enabled)
      // void CHRG_INHIBIT( byte thisVal ){ bitVals[ 5 ] = thisVal; }
      // //enable low power mode. Default is enabled
      // void EN_LWPWR( byte thisVal ){ bitVals[ 6 ] = thisVal; }
      // //Watchdog timer. Default is 175sec between commands (0x03)
      // void WDTMR_ADJ( byte thisVal ){ bitVals[ 7 ] = thisVal; }
      // //Disable IDPM. Default is low (IDPM enabled)
      // void IDPM_AUTO_DISABLE( byte thisVal ){ bitVals[ 8 ] = thisVal; }
      // //Turn Chrgok on if OTG is enabled. Default is low
      // void OTG_ON_CHRGOK( byte thisVal ){ bitVals[ 9 ] = thisVal; }
      // //Out of audio switch frequency. Default is low(disabled)
      // void EN_OOA( byte thisVal ){ bitVals[ 10 ] = thisVal; }
      // //PWM switching frequency, 800kHz or 1.2MHz. Default is high (800kHz)
      // void PWM_FREQ( byte thisVal ){ bitVals[ 11 ] = thisVal; }
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
        byte bitVals[12] = { 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01 };
        uint16_t masks[12] = {  0xDFFF, //byte 2, bit 5
                                0xEFFF, //byte 2, bit 4
                                0xF7FF, //byte 2, bit 3
                                0xFBFF, //byte 2, bit 2
                                0xFDFF, //byte 2, bit 1
                                0xFEFF, //byte 2, bit 0
                                0xFF7F, //byte 1, bit 7
                                0xFF9F, //byte 1, bit 6 & 5
                                0xFFEF, //byte 1, bit 4
                                0xFFF7, //byte 1, bit 3
                                0xFFFA, //byte 1, bit 2
                                0xFFFD, //byte 1, bit 1
                            };
        byte shift[12] = { 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x04, 0x03, 0x02, 0x01 };
        //Learn mode. Discharges with power connected. Default disabled
        void EN_LEARN( byte thisVal ){ bitVals[ 0 ] = thisVal; }
        //Current shunt amplifier 20x or 40x. Default is 20x
        void IADPT_GAIN( byte thisVal ){ bitVals[ 1 ] = thisVal; }
        //Bat current shunt amplifier 8x or 16x. Default is 16x
        void IBAT_GAIN( byte thisVal ){ bitVals[ 2 ] = thisVal; }
        //LDO mode - use of pre charge. Default is precharge enabled
        void EN_LDO( byte thisVal ){ bitVals[ 3 ] = thisVal; }
        //Enable IDPM current control. Default is high(enabled)
        void EN_IDPM( byte thisVal ){ bitVals[ 4 ] = thisVal; }
        //Inhibit charging. Default is low(enabled)
        void CHRG_INHIBIT( byte thisVal ){ bitVals[ 5 ] = thisVal; }
        //enable low power mode. Default is enabled
        void EN_LWPWR( byte thisVal ){ bitVals[ 6 ] = thisVal; }
        //Watchdog timer. Default is 175sec between commands (0x03)
        void WDTMR_ADJ( byte thisVal ){ bitVals[ 7 ] = thisVal; }
        //Disable IDPM. Default is low (IDPM enabled)
        void IDPM_AUTO_DISABLE( byte thisVal ){ bitVals[ 8 ] = thisVal; }
        //Turn Chrgok on if OTG is enabled. Default is low
        void OTG_ON_CHRGOK( byte thisVal ){ bitVals[ 9 ] = thisVal; }
        //Out of audio switch frequency. Default is low(disabled)
        void EN_OOA( byte thisVal ){ bitVals[ 10 ] = thisVal; }
        //PWM switching frequency, 800kHz or 1.2MHz. Default is high (800kHz)
        void PWM_FREQ( byte thisVal ){ bitVals[ 11 ] = thisVal; }
        byte val[ 2 ] = { 0x1A, 0x34 };
        uint8_t addr = 0x00;
        void update(){
          uint16_t tempVar = 0;
          //cycle through arrays
          for( uint16_t i = 0; i < sizeof( shift ); i++ ){
            //mask relevant bit(s) in question to 0
            tempVar = ( tempVar & masks[i] );
            //OR in the value, shifted to the correct position
            tempVar = ( tempVar | ( uint16_t )( bitVals[ i ] << shift[ i ] ) );
          }
          val[ 0 ] = ( byte )( tempVar );
          val[ 1 ] = ( byte )( tempVar >> 8 );
        }
      } chargeOption0;
      struct ChargeCurrentt{
        uint16_t current = 2048;
        byte val[ 2 ] = { 0x00, 0x08 };
        uint8_t addr = 0x02;
        void update(){
          //catch out of bounds
          if( current < 64 ) current = 64;
          if( current > 8128 ) current = 8128;
          //catch out of resolution
          current = current / 64;
          current = current * 64;
          //extract bytes
          val[ 0 ] = ( byte )( current );
          val[ 1 ] = ( byte )( current >> 8 );
        }
      } chargeCurrent;
      struct MaxChargeVoltaget{
        uint16_t voltage = 16800;
        byte val[ 2 ] = { 0xA0, 0x41 };
        uint8_t addr = 0x04;
        void update(){
          //catch out of bounds
          if( voltage < 1024 ) voltage = 1024;
          if( voltage > 19200 ) voltage = 19200;
          //catch out of resolution
          voltage = voltage / 16;
          voltage = voltage * 16;
          //extract bytes
          val[ 0 ] = ( byte )( voltage );
          val[ 1 ] = ( byte )( voltage >> 8 );
        }
      } maxChargeVoltage;
      struct ChargeOption1t{
        byte bitVals[12] = { 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01 };
        uint16_t masks[12] = {  0x7FFF, //byte 2, bit 7
                                0xBFFF, //byte 2, bit 6
                                0xCFFF, //byte 2, bit 5 & 4
                                0xF7FF, //byte 2, bit 3
                                0xFDFF, //byte 2, bit 1
                                0xFEFF, //byte 2, bit 0
                                0xFF7F, //byte 1, bit 7
                                0xFF9F, //byte 1, bit 6 & 5
                                0xFFEF, //byte 1, bit 4
                                0xFFF7, //byte 1, bit 3
                                0xFFFB, //byte 1, bit 2
                                0xFFFD, //byte 1, bit 1
                            };
        byte shift[12] = { 0x0F, 0x0E, 0x0D, 0x0B, 0x09, 0x08, 0x07, 0x06, 0x04, 0x03, 0x02, 0x01 };
        //Internal comparator reference 2.3V or 1.2V. Default is 2.3V
        void CMP_REF( byte thisVal ){ bitVals[ 0 ] = thisVal; }
        //Internal comparator polarity
        void CMP_POL( byte thisVal ){ bitVals[ 1 ] = thisVal; }
        //Internal comparator deglitch time; off, 1us, 2ms, 5s. Default is 1us
        void CMP_DEG( byte thisVal ){ bitVals[ 2 ] = thisVal; }
        //Force power path to switch off. Default is disabled
        void FORCE_LATCHOFF( byte thisVal ){ bitVals[ 3 ] = thisVal; }
        //Discharge SRN ppin for shipping. Default is disabled
        void EN_SHIP_DCHG( byte thisVal ){ bitVals[ 4 ] = thisVal; }
        //Automatically charge for 30mins at 128mA if voltage is below min. Default enabled.
        void AUTO_WAKEUP_EN( byte thisVal ){ bitVals[ 5 ] = thisVal; }
        //Enable IBAT output buffer. Default is disabled
        void EN_IBAT( byte thisVal ){ bitVals[ 6 ] = thisVal; }
        //PROCHOT during battery only; disabled, IDCHG, VSYS. Default is disabled
        void EN_PROCHOT_LPWR( byte thisVal ){ bitVals[ 7 ] = thisVal; }
        //Disable IDPM. Default is low (IDPM enabled)
        void EN_PSYS( byte thisVal ){ bitVals[ 8 ] = thisVal; }
        //Charge sense resistor; 10mR or 20mR. Default is 10mR
        void RSNS_RAC( byte thisVal ){ bitVals[ 9 ] = thisVal; }
        //Input sense resistor; 10mR or 20mR. Default is 10mR
        void RSNS_RSR( byte thisVal ){ bitVals[ 10 ] = thisVal; }
        //PSYS gain; 0.25uA/W or 1uA/W. Default is 1uA/W
        void PSYS_RATIO( byte thisVal ){ bitVals[ 11 ] = thisVal; }
        byte val[ 2 ] = { 0x81, 0x11};
        uint8_t addr = 0x30;
        void update(){
          uint16_t tempVar = 0;
          //cycle through arrays
          for( uint16_t i = 0; i < sizeof( shift ); i++ ){
            //mask relevant bit(s) in question to 0
            tempVar = ( tempVar & masks[i] );
            //OR in the value, shifted to the correct position
            tempVar = ( tempVar | ( uint16_t )( bitVals[ i ] << shift[ i ] ) );
          }
          val[ 0 ] = ( byte )( tempVar );
          val[ 1 ] = ( byte )( tempVar >> 8 );
        }
      } chargeOption1;
      struct ChargeOption2t{
        byte bitVals[14] = { 0x01, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };
        uint16_t masks[14] = {  0xFF7F, //byte LSB, bit 7
                                0xFFBF, //byte LSB, bit 6
                                0xFFDF, //byte LSB, bit 5
                                0xFFEF, //byte LSB, bit 4
                                0xFFF7, //byte LSB, bit 3
                                0xFFFB, //byte LSB, bit 2
                                0xFFFD, //byte LSB, bit 1
                                0xFFFE, //byte LSB, bit 0
                                0x3FFF, //byte MSB, bit 7 & 6
                                0xDFFF, //byte MSB, bit 5
                                0xEFFF, //byte MSB, bit 4
                                0xF7FF, //byte MSB, bit 3
                                0xFBFF, //byte MSB, bit 2
                                0xFCFF, //byte MSB, bit 1 & 0
                            };
        byte shift[14] = { 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x0F, 0x0D, 0x0C, 0x0B, 0x0A, 0x09 };
        //Allow ILIM_HIZ pin to set current limit. Default is enabled
        void EN_EXTILIM( byte thisVal ){ bitVals[ 0 ] = thisVal; }
        //Function of IBAT pin; discharge or charge. Default is discharge
        void EN_ICHG_IDCHG( byte thisVal ){ bitVals[ 1 ] = thisVal; }
        //Over Current Protection for Q2 by sensing VDS; 210mV or 150mV. Default is 150mV
        void Q2_OCP( byte thisVal ){ bitVals[ 2 ] = thisVal; }
        //Over Current Protection for input between ACP and ACN; 210mV or 150mV. default is 150mV
        void ACX_OCP( byte thisVal ){ bitVals[ 3 ] = thisVal; }
        //Input OCP enable. Default is disabled
        void EN_ACOC( byte thisVal ){ bitVals[ 4 ] = thisVal; }
        //Input OCP disabled current limit; 125% of ICRIT or 210% of ICRIT. Default is 210%
        void ACOC_VTH( byte thisVal ){ bitVals[ 5 ] = thisVal; }
        //Bat OCP; disabled or related to PROCHOT IDCHG. Default is IDPM
        void EN_BATOC( byte thisVal ){ bitVals[ 6 ] = thisVal; }
        //OCP related to PROCHOT IDCHG; 125% or 200%. Default is 200%
        void BATOC_VTH( byte thisVal ){ bitVals[ 7 ] = thisVal; }
        //Input overload time; 1ms, 2mS, 10mS, 20mS. Default is 1mS
        void PKPWR_TOVLD_DEG( byte thisVal ){ bitVals[ 8 ] = thisVal; }
        //Enable peak power mode from over current. Default is disabled
        void EN_PKPWR_IDPM( byte thisVal ){ bitVals[ 9 ] = thisVal; }
        //Enable peak power mode from under voltage. Default is disabled
        void EN_PKPWR_VSYS( byte thisVal ){ bitVals[ 10 ] = thisVal; }
        //Indicator that device is in overloading cycle. Default disabled
        void PKPWR_OVLD_STAT( byte thisVal ){ bitVals[ 11 ] = thisVal; }
        //Indicator that device is in relaxation cycle. Default disabled
        void PKPWR_RELAX_STAT( byte thisVal ){ bitVals[ 12 ] = thisVal; }
        //Peak power mode overload and relax cycle times; 5mS, 10mS, 20mS, 40mS. Default is 20mS
        void PKPWR_TMAX( byte thisVal ){ bitVals[ 13 ] = thisVal; }
        byte val[ 2 ] = { 0xB7, 0x02};
        uint8_t addr = 0x32;
        void update(){
          uint16_t tempVar = 0;
          //cycle through arrays
          for( uint16_t i = 0; i < sizeof( shift ); i++ ){
            //mask relevant bit(s) in question to 0
            tempVar = ( tempVar & masks[i] );
            //OR in the value, shifted to the correct position
            tempVar = ( tempVar | ( uint16_t )( bitVals[ i ] << shift[ i ] ) );
          }
          val[ 0 ] = ( byte )( tempVar );
          val[ 1 ] = ( byte )( tempVar >> 8 );
        }
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

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
  //macro to generate bit mask to access bits
  #define GETMASK(index, size) (((1 << (size)) - 1) << (index))
  //macro to read bits from variable, using mask
  #define READFROM(data, index, size) (((data) & GETMASK((index), (size))) >> (index))
  //macro to write bits into variable, using mask
  #define WRITETO(data, index, size, value) ((data) = ((data) & (~GETMASK((index), (size)))) | ((value) << (index)))
  //macro to wrap functions for easy access
  //if name is called with empty brackets, read bits and return value
  //if name is prefixed with set_, write value in brackets into bits defined in FIELD
  #define FIELD(data, name, index, size) \
  inline decltype(data) name() { return READFROM(data, index, size); } \
  inline void set_##name(decltype(data) value) { WRITETO(data, index, size, value); }

  struct Regt{
      struct ChargeOption0t{
        uint8_t addr = 0x00;
        byte val0 = 0x1A;
        byte val1 = 0x34;
        //Learn mode. Discharges with power connected. Default disabled
        FIELD( val0, EN_LEARN, 0x05, 0x01 )
        //Current shunt amplifier 20x or 40x. Default is 20x
        FIELD( val0, IADPT_GAIN, 0x04, 0x01 )
        //Bat current shunt amplifier 8x or 16x. Default is 16x
        FIELD( val0, IBAT_GAIN, 0x03, 0x01 )
        //LDO mode - use of pre charge. Default is precharge enabled
        FIELD( val0, EN_LDO, 0x02, 0x01 )
        //Enable IDPM current control. Default is high(enabled)
        FIELD( val0, EN_IDPM, 0x01, 0x01 )
        //Inhibit charging. Default is low(enabled)
        FIELD( val0, CHRG_INHIBIT, 0x00, 0x01 )
        //Enable low power mode. Default is enabled
        FIELD( val1, EN_LWPWR, 0x07, 0x01 )
        //Watchdog timer. Default is 175sec between commands (0x03)
        FIELD( val1, WDTMR_ADJ, 0x06, 0x02 )
        //Disable IDPM. Default is low (IDPM enabled)
        FIELD( val1, IDPM_AUTO_DISABLE, 0x04, 0x01 )
        //Turn Chrgok on if OTG is enabled. Default is low
        FIELD( val1, OTG_ON_CHRGOK, 0x03, 0x01 )
        //Out of audio switch frequency. Default is low(disabled)
        FIELD( val1, EN_OOA, 0x02, 0x01 )
        //PWM switching frequency, 800kHz or 1.2MHz. Default is high (800kHz)
        FIELD( val1, PWM_FREQ, 0x01, 0x01 )
      } chargeOption0;
      struct ChargeCurrentt{
        uint16_t current = 2048;
        byte val0 = 0x00;
        byte val1 = 0x08;
        uint8_t addr = 0x02;
        void set_current( uint16_t set_cur ){
          //catch out of bounds
          if( set_cur < 64 ) set_cur = 64;
          if( set_cur > 8128 ) set_cur = 8128;
          //catch out of resolution
          set_cur = set_cur / 64;
          set_cur = set_cur * 64;
          //extract bytes
          val0 = ( byte )( set_cur );
          val1 = ( byte )( set_cur >> 8 );
          current = set_cur;
        }
      } chargeCurrent;
      struct MaxChargeVoltaget{
        uint16_t voltage = 16800;
        byte val0 = 0xA0;
        byte val1 = 0x41;
        uint8_t addr = 0x04;
        void set_voltage( uint16_t set_volt ){
          //catch out of bounds
          if( set_volt < 1024 ) set_volt = 1024;
          if( set_volt > 19200 ) set_volt = 19200;
          //catch out of resolution
          set_volt = set_volt / 16;
          set_volt = set_volt * 16;
          //extract bytes
          val0 = ( byte )( set_volt );
          val1 = ( byte )( set_volt >> 8 );
          voltage = set_volt;
        }
      } maxChargeVoltage;
      struct ChargeOption1t{
        uint8_t addr = 0x30;
        byte val0 = 0x81;
        byte val1 = 0x11;
        //Internal comparator reference 2.3V or 1.2V. Default is 2.3V
        FIELD( val0, CMP_REF, 0x07, 0x01 )
        //Internal comparator polarity
        FIELD( val0, CMP_POL, 0x06, 0x01 )
        //Internal comparator deglitch time; off, 1us, 2ms, 5s. Default is 1us
        FIELD( val0, CMP_DEG, 0x05, 0x02 )
        //Force power path to switch off. Default is disabled
        FIELD( val0, FORCE_LATCHOFF, 0x03, 0x01 )
        //Discharge SRN pin for shipping. Default is disabled
        FIELD( val0, EN_SHIP_DCHG, 0x01, 0x01 )
        //Automatically charge for 30mins at 128mA if voltage is below min. Default enabled.
        FIELD( val0, AUTO_WAKEUP_EN, 0x00, 0x01 )
        //Enable IBAT output buffer. Default is disabled
        FIELD( val1, EN_IBAT, 0x07, 0x01 )
        //PROCHOT during battery only; disabled, IDCHG, VSYS. Default is disabled
        FIELD( val1, EN_PROCHOT_LPWR, 0x06, 0x02 )
        //Enable PSYS buffer. Default is disabled
        FIELD( val1, EN_PSYS, 0x04, 0x01 )
        //Charge sense resistor; 10mR or 20mR. Default is 10mR
        FIELD( val1, RSNS_RAC, 0x03, 0x01 )
        //Input sense resistor; 10mR or 20mR. Default is 10mR
        FIELD( val1, RSNS_RSR, 0x02, 0x01 )
        //PSYS gain; 0.25uA/W or 1uA/W. Default is 1uA/W
        FIELD( val1, PSYS_RATIO, 0x01, 0x01 )
      } chargeOption1;
      struct ChargeOption2t{
        uint8_t addr = 0x32;
        byte val0 = 0xB7;
        byte val1 = 0x02;
        //Allow ILIM_HIZ pin to set current limit. Default is enabled
        FIELD( val0, EN_EXTILIM, 0x07, 0x01 )
        //Function of IBAT pin; discharge or charge. Default is discharge
        FIELD( val0, EN_ICHG_IDCHG, 0x06, 0x01 )
        //Over Current Protection for Q2 by sensing VDS; 210mV or 150mV. Default is 150mV
        FIELD( val0, Q2_OCP, 0x05, 0x01 )
        //Over Current Protection for input between ACP and ACN; 210mV or 150mV. default is 150mV
        FIELD( val0, ACX_OCP, 0x04, 0x01 )
        //Input adapter OCP enable. Default is disabled
        FIELD( val0, EN_ACOC, 0x03, 0x01 )
        //Input adapter OCP disabled current limit; 125% of ICRIT or 210% of ICRIT. Default is 210%
        FIELD( val0, ACOC_VTH, 0x02, 0x01 )
        //Bat OCP; disabled or related to PROCHOT IDCHG. Default is IDPM
        FIELD( val0, EN_BATOC, 0x01, 0x01 )
        //OCP related to PROCHOT IDCHG; 125% or 200%. Default is 200%
        FIELD( val0, BATOC_VTH, 0x00, 0x01 )
        //Input overload time; 1ms, 2mS, 10mS, 20mS. Default is 1mS
        FIELD( val1, PKPWR_TOVLD_DEG, 0x07, 0x02 )
        //Enable peak power mode from over current. Default is disabled
        FIELD( val1, EN_PKPWR_IDPM, 0x05, 0x01 )
        //Enable peak power mode from under voltage. Default is disabled
        FIELD( val1, EN_PKPWR_VSYS, 0x04, 0x01 )
        //Indicator that device is in overloading cycle. Default disabled
        FIELD( val1, PKPWR_OVLD_STAT, 0x03, 0x01 )
        //Indicator that device is in relaxation cycle. Default disabled
        FIELD( val1, PKPWR_RELAX_STAT, 0x02, 0x01 )
        //Peak power mode overload and relax cycle times; 5mS, 10mS, 20mS, 40mS. Default is 20mS
        FIELD( val1, PKPWR_TMAX, 0x01, 0x02 )
      } chargeOption2;
      struct ChargeOption3t{
        uint8_t addr = 0x34;
        byte val0 = 0x00;
        byte val1 = 0x00;
        //Control BAT FET during Hi-Z state. Default is disabled
        FIELD( val0, BATFETOFF_HIZ, 0x01, 0x01 )
        //PSYS function during OTG mode. PSYS = battery discharge - IOTG or PSYS = battery discharge. Default 0
        FIELD( val0, PSYS_OTG_IDCHG, 0x00, 0x01 )
        //Enable Hi-Z(low power) mode. Default is disabled
        FIELD( val1, EN_HIZ, 0x07, 0x01 )
        //Reset registers. Set this bit to 1 to reset all other registers
        FIELD( val1, RESET_REG, 0x06, 0x01 )
        //Reset VINDPM register. Default is idle (0)
        FIELD( val1, RESET_VINDPM, 0x05, 0x01 )
        //Enable OTG mode to output power to VBUS. EN_OTG pin needs to be high. Default is disabled.
        FIELD( val1, EN_OTG, 0x04, 0x01 )
        //Enable Input Current Optimiser. Default is disabled
        FIELD( val1, EN_ICO_MODE, 0x03, 0x01 )
      } chargeOption3;
      struct ProchotOption0t{
        uint8_t addr = 0x36;
        byte val0 = 0x50;
        byte val1 = 0x92;
        //VSYS threshold; 5.75V, 6V, 6.25V, 6.5V. Default is 6V
        FIELD( val0, VSYS_VTH, 0x07, 0x02 )
        //Enable PROCHOT voltage kept LOW until PROCHOT_CLEAR is written. Default is disabled
        FIELD( val0, EN_PROCHOT_EX, 0x05, 0x01 )
        //Minimum PROCHOT pulse length when EN_PROCHOT_EX is disabled; 100us, 1ms, 10ms, 5ms. Default is 1ms
        FIELD( val0, PROCHOT_WIDTH, 0x04, 0x02 )
        //Clears PROCHOT pulse when EN_PROCHOT_EX is enabled. Default is idle.
        FIELD( val0, PROCHOT_CLEAR, 0x02, 0x01 )
        //INOM deglitch time; 1ms or 50ms. Default is 1ms
        FIELD( val0, INOM_DEG, 0x01, 0x01 )
        //ILIM2 threshold as percentage of IDPM; 110%-230%(5% step), 250%-450%(50% step). Default is 150%
        FIELD( val1, ILIM2_VTH, 0x07, 0x05 )
        //ICRIT deglitch time. ICRIT is 110% of ILIM2; 15us, 100us, 400us, 800us. Default is 100us.
        FIELD( val1, ICRIT_DEG, 0x02, 0x02 )
      } prochotOption0;
      struct ProchotOption1t{
          uint8_t addr = 0x38;
          byte val0 = 0x20;
          byte val1 = 0x41;
          //PROCHOT profile comparator. Default is disabled.
          FIELD( val0, PROCHOT_PROFILE_COMP, 0x06, 0x01 )
          //Prochot is triggered if ICRIT threshold is reached. Default enabled.
          FIELD( val0, PROCHOT_PROFILE_ICRIT, 0x05, 0x01 )
          //Prochot is triggered if INOM threshold is reached. Default disabled.
          FIELD( val0, PROCHOT_PROFILE_INOM, 0x04, 0x01 )
          //Enable battery Current Discharge current reading. Default is disabled.
          FIELD( val0, PROCHOT_PROFILE_IDCHG, 0x03, 0x01 )
          //Prochot is triggered if VSYS threshold is reached. Default disabled.
          FIELD( val0, PROCHOT_PROFILE_VSYS, 0x02, 0x01 )
          //PROCHOT will be triggered if the battery is removed. Default is disabled.
          FIELD( val0, PROCHOT_PROFILE_BATPRES, 0x01, 0x01 )
          //PROCHOT will be triggered if the adapter is removed. Default is disabled.
          FIELD( val0, PROCHOT_PROFILE_ACOK, 0x00, 0x01 )
          //IDCHG threshold. PROCHOT is triggered when IDCHG is above; 0-32356mA in 512mA steps. Default is 16384mA
          FIELD( val1, IDCHG_VTH, 0x07, 0x06 )
          //IDCHG deglitch time; 1.6ms, 100us, 6ms, 12ms. Default is 100us.
          FIELD( val1, IDCHG_DEG, 0x01, 0x02 )
      } prochotOption1;
      struct ADCOptiont{
        uint8_t addr = 0x3A;
        byte val0, val1;
        //Enable comparator voltage reading. Default is disabled.
        FIELD( val0, EN_ADC_CMPIN, 0x07, 0x01 )
        //Enable VBUS voltage reading. Default is disabled.
        FIELD( val0, EN_ADC_VBUS, 0x06, 0x01 )
        //Enable PSYS voltage reading for calculating system power. Default is disabled.
        FIELD( val0, EN_ADC_PSYS, 0x05, 0x01 )
        //Enable Current In current reading. Default is disabled.
        FIELD( val0, EN_ADC_IIN, 0x04, 0x01 )
        //Enable battery Current Discharge current reading. Default is disabled.
        FIELD( val0, EN_ADC_IDCHG, 0x03, 0x01 )
        //Enable battery Current Charge current reading. Default is disabled.
        FIELD( val0, EN_ADC_ICHG, 0x02, 0x01 )
        //Enable Voltage of System voltage reading. Default is disabled.
        FIELD( val0, EN_ADC_VSYS, 0x01, 0x01 )
        //Enable Voltage of Battery voltage reading. Default is disabled.
        FIELD( val0, EN_ADC_VBAT, 0x00, 0x01 )
        //ADC mode; one shot reading or continuous. Default is one shot
        FIELD( val1, ADC_CONV, 0x07, 0x01 )
        //Start a one shot reading of the ADC. Resets to 0 after reading
        FIELD( val1, ADC_START, 0x06, 0x01 )
        //ADC scale; 2.04V or 3.06V. Default is 3.06V
        FIELD( val1, ADC_FULLSCALE, 0x05, 0x01 )
      } aDCOption;
      struct ChargerStatust{
        uint8_t addr = 0x20;
        byte val0, val1;
        //Latched fault flag of adapter over voltage. Default is no fault.
        FIELD( val0, Fault_ACOV, 0x07, 0x01 )
        //Latched fault flag of battery over current. Default is no fault.
        FIELD( val0, Fault_BATOC, 0x06, 0x01 )
        //Latched fault flag of adapter over current. Default is no fault.
        FIELD( val0, Fault_ACOC, 0x05, 0x01 )
        //Latched fault flag of system over voltage protection. Default is no fault.
        FIELD( val0, SYSOVP_STAT, 0x04, 0x01 )
        //Resets faults latch. Default is disabled
        FIELD( val0, Fault_Latchoff, 0x02, 0x01 )
        //Latched fault flag of OTG over voltage protection. Default is no fault.
        FIELD( val0, Fault_OTG_OVP, 0x01, 0x01 )
        //Latched fault flag of OTG over current protection. Default is no fault.
        FIELD( val0, Fault_OTG_UCP, 0x00, 0x01 )
        //Input source present. Default is not connected.
        FIELD( val1, AC_STAT, 0x07, 0x01 )
        //After ICO routine is done, bit goes to zero.
        FIELD( val1, ICO_DONE, 0x06, 0x01 )
        //Charger is in VINDPM or OTG mode. Default is not
        FIELD( val1, IN_VINDPM, 0x04, 0x01 )
        //Device is in current in DPM mode. Default is not
        FIELD( val1, IN_IINDPM, 0x03, 0x01 )
        //Device is in fast charge mode. Default is not
        FIELD( val1, IN_FCHRG, 0x02, 0x01 )
        //Device is in pre charge mode. Default is not
        FIELD( val1, IN_PCHRG, 0x01, 0x01 )
        //Device is in OTG mode. Default is not
        FIELD( val1, IN_OTG, 0x00, 0x01 )
      } chargerStatus;
      struct ProchotStatust{
        uint8_t addr = 0x22;
        byte val0, val1;
        //PROCHOT comparator trigger status. Default is not triggered.
        FIELD( val0, STAT_COMP, 0x06, 0x01 )
        //PROCHOT current critical trigger status. Default is not triggered.
        FIELD( val0, STAT_ICRIT, 0x05, 0x01 )
        //PROCHOT input current exceeds 110% threshold trigger. Default is not triggered.
        FIELD( val0, STAT_INOM, 0x04, 0x01 )
        //PROCHOT discharge current trigger status. Default is not triggered.
        FIELD( val0, STAT_IDCHG, 0x03, 0x01 )
        //PROCHOT system voltage trigger status. Default is not triggered.
        FIELD( val0, STAT_VSYS, 0x02, 0x01 )
        //PROCHOT battery removal trigger status. Default is not triggered.
        FIELD( val0, STAT_Battery_Removal, 0x01, 0x01 )
        //PROCHOT adapter removal trigger status. Default is not triggered.
        FIELD( val0, STAT_Adapter_Removal, 0x00, 0x01 )
      } prochotStatus;
      struct IIN_DPMt{ //read only
        uint16_t current = 0;
        byte val0 = 0x00; //not used
        byte val1 = 0x00;
        uint8_t addr = 0x24;
        //Incoming current threshold before device lowers charging current.
        //50mA to 6400 in 50mA steps, with 50mA offset.
        uint16_t get_current( ){
          //multiply up to mA value
          current = val1 * 50;
          //Add in offset
          current = current + 50;
          return current;
        }
      } iIN_DPM;
      struct ADCVBUSPSYSt{ //read only
        uint16_t sysPower = 0;
        uint16_t VBUS = 0;
        uint32_t Rsys = 30000; //Value of resistor on PSYS pin
        byte val0 = 0x00;
        byte val1 = 0x00;
        uint8_t addr = 0x26;
        //VBUS voltage and system power. VBUS is direct reading.
        //System power(W) is Vsys(mV)/Rsys(R) * 10^3
        uint16_t get_VBUS(){
          //multiply up to mV value
          VBUS = val1 * 64;
          //Add in offset
          VBUS = VBUS + 3200;
          return VBUS;
        }
        uint16_t get_sysPower(){
          //multiply by the 12mV resolution
          sysPower = val0 * 12;
          //create temp large var and multiply by 1000 (uA/W)
          uint32_t tempPower = sysPower * 1000;
          //Divide by the resistor value Rsys
          tempPower = tempPower / Rsys;
          //convert back to smaller variable
          sysPower = ( uint16_t )tempPower;
          return sysPower;
        }
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
        uint16_t voltage = 0;
        byte val0 = 0x00;
        byte val1 = 0x00;
        uint8_t addr = 0x06;
        //OTG output voltage.
        //4480mV to 20864mV in 64mA steps, with 4480mV offset.
        void set_voltage( uint16_t set_volt ){
          voltage = set_volt;
          //catch out of bounds
          if( set_volt < 4480 ) set_volt = 4480;
          if( set_volt > 20864 ) set_volt = 20864;
          //remove offset
          set_volt = set_volt - 4480;
          //catch out of resolution
          set_volt = set_volt / 64;
          set_volt = set_volt * 64;
          //extract byte
          val0 = ( byte )( set_volt );
          val1 = ( byte )( set_volt >> 8 );
        }
      } oTGVoltage;
      struct OTGCurrentt{
        uint16_t current = 0;
        byte val0 = 0x00; //not used
        byte val1 = 0x00;
        uint8_t addr = 0x08;
        //OTG output current limit.
        //0mA to 6400mA in 50mA steps.
        void set_current( uint16_t set_cur ){
          current = set_cur;
          //catch out of bounds
          if( set_cur > 6400 ) set_cur = 6400;
          //convert to steps value
          set_cur = set_cur / 50;
          //extract byte
          val1 = ( byte )( set_cur );
        }
      } oTGCurrent;
      struct InputVoltaget{
        uint16_t voltage = 18720;
        byte val0 = 0x20;
        byte val1 = 0x49;
        uint8_t addr = 0x0A;
        //Incoming voltage threshold before device lowers charging current.
        //3200mV to 19584mV in 64mA steps, with 3200mV offset.
        void set_voltage( uint16_t set_volt ){
          voltage = set_volt;
          //catch out of bounds
          if( set_volt < 3200 ) set_volt = 3200;
          if( set_volt > 19584 ) set_volt = 19584;
          //remove offset
          set_volt = set_volt - 3200;
          //catch out of resolution
          set_volt = set_volt / 64;
          set_volt = set_volt * 64;
          //extract byte
          val0 = ( byte )( set_volt );
          val1 = ( byte )( set_volt >> 8 );
        }
      } inputVoltage;
      struct MinSystemVoltaget{
        uint16_t val = 0;
        uint8_t addr = 0x0C;
      } minSystemVoltage;
      struct IIN_HOSTt{
        uint16_t current = 3250;
        byte val0 = 0x00; //not used
        byte val1 = 0x40;
        uint8_t addr = 0x0E;
        //Incoming current threshold before device lowers charging current.
        //50mA to 6400 in 50mA steps, with 50mA offset.
        void set_current( uint16_t set_cur ){
          current = set_cur;
          //catch out of bounds
          if( set_cur < 50 ) set_cur = 50;
          if( set_cur > 6400 ) set_cur = 6400;
          //remove offset
          set_cur = set_cur - 50;
          //convert to steps value
          set_cur = set_cur / 50;
          //extract byte
          val1 = ( byte )( set_cur );
        }
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

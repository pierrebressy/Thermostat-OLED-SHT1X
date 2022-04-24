/*
# -------------------------------------------------------------------
# Filename  : thermostat.h
# Created on: 18/06/2017 - 22:50:12  by Pierre BRESSY (HEIG-VD)
# Purpose   : 
# -------------------------------------------------------------------
*/

#ifndef _THERMOSTAT_H_
#define _THERMOSTAT_H_

#define SOFTWARE_VERSION "V1.0 20171201"

// INIT_EEPROM
// for the first configuration of the Arduino NANO:
// - set this symbol to 1
// - compile and program the µC board
// - run the software => tMin, tMax and tCsg will be correctly set in EEPROM
// - set this symbol to 0
// - compile and program the µC board
// - run the software => tMin, tMax and tCsg will be read from EEPROM

#define INIT_EEPROM 1

// I2C pinout for temperature sensor
#define WIRE_LIB_DATAPIN 4
#define WIRE_LIB_CLOCKPIN 5

#define ANSWER_MAX_SIZE 16
#define I2C_ADDRESS_BASE 8

#define DS18S20_PIN 2

// addresses for tMax, tMin and tCsg in EEPROM
#define EEPROM_PARAM_BASE_ADDRESS 0x00
#define EEPROM_OFFSET_ADDRESS_TMIN 0
#define EEPROM_OFFSET_ADDRESS_TMAX 1
#define EEPROM_OFFSET_ADDRESS_TTARGET 2


#define CELSIUS_TO_INT_SCALE (2.)
#define CELSIUS_TO_INT(x) ( (x)*CELSIUS_TO_INT_SCALE )
#define INT_TO_CELSIUS(x) ( (x)/CELSIUS_TO_INT_SCALE )

// default values for target temperature
#define DEFAULT_MIN_TEMPERATURE (CELSIUS_TO_INT(+63.5))
#define DEFAULT_MAX_TEMPERATURE (0.0)
#define DEFAULT_TARGET_TEMPERATURE (CELSIUS_TO_INT(23.0))

#define DEFAULT_ICE_CSG (CELSIUS_TO_INT(5.0))

// regulation parameters
#define HYSTERESIS (CELSIUS_TO_INT(0.5))
#define DEFAULT_CSG_STEP (CELSIUS_TO_INT(0.5))
#define DEFAULT_CSG_MAX (CELSIUS_TO_INT(30.0))




// ----------------------------------------------------------  enumerated types
typedef enum {
  ADDRESS_ZERO = 0,
  ADDRESS_DATA = 1,
  ADDRESS_TARGET_VALUE = 2,
  ADDRESS_SENSOR_VALUE = 3,
  ADDRESS_CMD_VALUE = 4,
  ADDRESS_HYST_VALUE = 5,
  ADDRESS_LED = 6
} eAddress;


// ----------------------------------------------------------  structured types
typedef struct {
  eAddress address;
  uint8_t data[ANSWER_MAX_SIZE];

} sI2CData, *pI2CData;

typedef struct {
  int16_t sensorValue;
  int8_t targetValue;
  int8_t cmd;
  int8_t hystValue;
} sRawData, *pRawData;


// -128 = -64.0°C
// +127 = +63.5°C

typedef struct {
  
  int8_t tMin;
  int8_t tMax;
  int8_t defaultTarget;
  
} sEepromParameters, *pEepromParameters;




// --------------------------------------------------------------------  unions

// ----------------------------------------------------------------  prototypes
// reply to python cmd : bus.read_i2c_block_data(8,0,16);
void I2CTxData(void);
// reply to python cmd : bus.write_i2c_block_data(8,address,data);
void I2CRxData(int byteCount);
void rxDataAnalysis(pI2CData pData);

void setup(void);
void loop(void);

void displaySplashScreenInitEeprom(void);

void writeTInEeprom(int8_t _t, int _address);
int8_t readTFromEeprom(int _address);

bool advancedRegulation(int8_t _target, int8_t _t, bool _cmd);

void displayTemp(float t, int x, int y);
void displayMinMaxTemp(int8_t tMin, int8_t tMax, int x, int y);
void displayCsg(int8_t _t, int _x, int _y);
void displayCmd(bool _cmd, int _x, int _y, bool _blinkMode);
void draw(int8_t _t, int8_t _tMin, int8_t _tMax, int8_t _csg, bool _cmd);




#endif


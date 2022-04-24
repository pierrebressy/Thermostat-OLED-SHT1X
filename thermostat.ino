// 23268 / 463
// 25856 / 126
#define WITH_SHT1X 0
#define WITH_DS18X20 1
#define WITH_HW_I2C 0
#define WITH_I2C 1

// --------------------------------------------------------------------------------
// include files

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#if WITH_SHT1X
#include <SHT1x.h>
#endif
#if WITH_DS18X20
#include <OneWire.h>
#include "thermostat.h"
#endif


// --------------------------------------------------------------------------------
// global variables

#if WITH_DS18X20
OneWire  ds(DS18S20_PIN);  // on pin 2 of arduino nano (a 4.7K resistor is necessary)
const byte addr[8] = {0x10, 0x9E, 0x4A, 0x42, 0x3, 0x8, 0x0, 0x78};
byte data[12];
#endif

// display object
#if WITH_HW_I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 16, 17 );
// SCL on pin 16 (A0)
// SDA on pin 17 (A1)
#endif

// sensor object
#if WITH_SHT1X
SHT1x sht1x(WIRE_LIB_DATAPIN, WIRE_LIB_CLOCKPIN);
#endif

static bool cmd = 0;
static int8_t temp_c = CELSIUS_TO_INT(19.0);
static int8_t tCsg = DEFAULT_ICE_CSG;
static int8_t hystValue = HYSTERESIS;
static float realTemp = 0.;

#if WITH_I2C
sI2CData I2CData;
sRawData rawData;
#endif

sEepromParameters params = {
  DEFAULT_MIN_TEMPERATURE,
  DEFAULT_MAX_TEMPERATURE,
  DEFAULT_TARGET_TEMPERATURE,
};

/**
   \fn void getRawDataFromReal(pRealData in, pRawData out)
   \brief transform real data into raw format
   \param[in] pRealData in : pointer on real data
   \param[out] pRawData out : pointer on raw data
   \return nothing
*/

// --------------------------------------------------------------------------------

void  writeParamsInEeprom(sEepromParameters _params, uint8_t _address) {

  writeTInEeprom(_params.tMin, _address + EEPROM_OFFSET_ADDRESS_TMIN);
  writeTInEeprom(_params.tMax, _address + EEPROM_OFFSET_ADDRESS_TMAX);
  writeTInEeprom(_params.defaultTarget, _address + EEPROM_OFFSET_ADDRESS_TTARGET);

}

void  readParamsFromEeprom(sEepromParameters *_params, uint8_t _address) {

  _params->tMin = readTFromEeprom(_address + EEPROM_OFFSET_ADDRESS_TMIN);
  _params->tMax = readTFromEeprom(_address + EEPROM_OFFSET_ADDRESS_TMAX);
  _params->defaultTarget = readTFromEeprom(_address + EEPROM_OFFSET_ADDRESS_TTARGET);

}

// --------------------------------------------------------------------------------

void setup(void) {

#if WITH_SERIAL
  Serial.begin(9600);
#endif


  Wire.begin(I2C_ADDRESS_BASE);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.onReceive(I2CRxData);
  Wire.onRequest(I2CTxData);

  u8g2.begin();

#if INIT_EEPROM
  writeParamsInEeprom(params, EEPROM_PARAM_BASE_ADDRESS);
#endif
  readParamsFromEeprom(&params, EEPROM_PARAM_BASE_ADDRESS);

#if INIT_EEPROM
  displaySplashScreenInitEeprom();
  delay(500);
#endif
#if WITH_SHT1X
  sht1x.readTemperatureC();
#endif

}


float readTemperature(void) {
#if WITH_SHT1X
  sht1x.readTemperatureC();
#elif WITH_DS18X20
  byte i;

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  delay(1000);     // maybe 750ms is enough, maybe not
  ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  raw = raw << 3; // 9 bit resolution default
  if (data[7] == 0x10) {
    // "count remain" gives full 12 bit resolution
    raw = (raw & 0xFFF0) + 12 - data[6];
  }
  return  (float)(raw / 16.0);
#else
  return (float)(19.0);
#endif

}


void displaySplashScreenInitEeprom(void) {

  u8g2.clearBuffer();
  u8g2.setFontMode(0); // mode white char, black background
  u8g2.setDrawColor(1);
  u8g2.setFontDirection(0);
  u8g2.setFont(u8g2_font_t0_13_tf);
  u8g2.setCursor(0, 13);
  u8g2.print("INIT. EEPROM");
  u8g2.sendBuffer();

  return;
}


// --------------------------------------------------------------------------------

void displayTemp(float _t, int _x, int _y) {

  u8g2.setFontDirection(0);
  u8g2.setFont(u8g2_font_logisoso28_tf);
  u8g2.setCursor(_x + 0, _y + 28);
  u8g2.print(_t, 1);
  u8g2.setFont(u8g2_font_u8glib_4_tf);
  u8g2.setCursor(_x + 65, _y + 5);
  u8g2.print("o");
  u8g2.setFont(u8g2_font_logisoso28_tf);
  u8g2.setCursor(_x + 70, _y + 28);
  u8g2.print("C");

}

// --------------------------------------------------------------------------------

void displayMinMaxTemp(int8_t _tMin, int8_t _tMax, int _x, int _y) {

  u8g2.setFontDirection(0);
  u8g2.setFont(u8g2_font_7x13_tr);
  u8g2.setCursor(_x, _y + 13);
  u8g2.print(INT_TO_CELSIUS(_tMax), 1);
  u8g2.setCursor(_x, _y + 28);
  u8g2.print(INT_TO_CELSIUS(_tMin), 1);
}

// --------------------------------------------------------------------------------

void displayCsg(int8_t _t, int _x, int _y) {

  u8g2.setFontDirection(0);
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.setCursor(_x + 0, _y + 16);
  u8g2.print(INT_TO_CELSIUS(_t), 1);
  u8g2.setFont(u8g2_font_u8glib_4_tf);
  u8g2.setCursor(_x + 40, _y + 5);
  u8g2.print("o");
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.setCursor(_x + 45, _y + 16);
  u8g2.print("C");
}

// --------------------------------------------------------------------------------

void displayCmd(bool _cmd, int _x, int _y, bool _blinkMode) {

  u8g2.setFontDirection(0);
  u8g2.setFont(u8g2_font_pressstart2p_8r);
  u8g2.setCursor(_x + 0, _y + 8);
  if (!_cmd)
    u8g2.print("OFF ");
  else {
    u8g2.setFontMode(0);
    u8g2.setDrawColor(_blinkMode ? 1 : 0);
    u8g2.print(" ON ");
  }
  u8g2.setDrawColor(1);
}

// --------------------------------------------------------------------------------

void draw(float _t, int8_t _tMin, int8_t _tMax, int8_t _csg, bool _cmd) {

  static bool blinkMode = false;

  blinkMode = blinkMode ? false : true;

  u8g2.clearBuffer();
  u8g2.setFontMode(0); // mode white char, black background
  u8g2.setDrawColor(1);
  displayTemp(_t, 0, 0);
#if 1
  displayMinMaxTemp(_tMin, _tMax, 98, 0);
  displayCsg(_csg, 64 + 3, 32 + 3);
  displayCmd(_cmd, 64 + 3, 55, blinkMode);

  u8g2.drawLine(0, 32, 127, 32); // horizontal separator
  u8g2.drawLine(94, 0, 94, 32); // upper vertical separator
  u8g2.drawLine(49, 32, 49, 63); // lower vertical separator
#endif
  u8g2.sendBuffer();
}

// --------------------------------------------------------------------------------

bool advancedRegulation(int8_t _target, int8_t _t, bool _cmd) {


  if (!(_cmd) && _t <= (_target - hystValue))
    return true;
  else if ((_cmd) && _t >= (_target + hystValue))
    return false;
  else
    return _cmd;
}

// --------------------------------------------------------------------------------

void writeTInEeprom(int8_t _t, int _address) {

  EEPROM.write(_address, (unsigned char)(_t));
  return;
}

int8_t readTFromEeprom(int _address) {

  return (int8_t)(EEPROM.read(_address) );
}

// --------------------------------------------------------------------------------

void loop(void) {

  static int8_t tMin = 0;
  static int8_t tMax = 0;

  static bool oldLeftButton = false;
  static bool oldRightButton = false;
  bool leftButton = analogRead(A1) > 500 ? true : false;
  bool rightButton = analogRead(A0) > 500 ? true : false;

  bool pushLeftButton = (leftButton != oldLeftButton && leftButton) ? true : false;
  bool pushRightButton = (rightButton != oldRightButton && rightButton) ? true : false;

  oldLeftButton = leftButton;
  oldRightButton = rightButton;


  // Read values from the sensor
#if WITH_SHT1X
  tempReadCounter = 120;
  temp_c = sht1x.readTemperatureRaw();
  temp_c = tempCompensation(temp_c);
  humidity = sht1x.readHumidity();
#else
  realTemp = readTemperature();
  temp_c = CELSIUS_TO_INT(realTemp);
#endif


  if (tCsg == DEFAULT_ICE_CSG) {
    tCsg = readTFromEeprom(EEPROM_PARAM_BASE_ADDRESS + EEPROM_OFFSET_ADDRESS_TTARGET);
  }

  if (tMin == 0 && tMax == 0) {
#if INIT_EEPROM
    tMin = tMax = temp_c;
    writeTInEeprom(tMin, EEPROM_PARAM_BASE_ADDRESS + EEPROM_OFFSET_ADDRESS_TMIN);
    writeTInEeprom(tMax, EEPROM_PARAM_BASE_ADDRESS + EEPROM_OFFSET_ADDRESS_TMAX);
#else
    tMin = readTFromEeprom(EEPROM_PARAM_BASE_ADDRESS + EEPROM_OFFSET_ADDRESS_TMIN);
    tMax = readTFromEeprom(EEPROM_PARAM_BASE_ADDRESS + EEPROM_OFFSET_ADDRESS_TMAX);
#endif
  }
  else {

    if (temp_c < tMin) {
      tMin = temp_c;
      writeTInEeprom(tMin, EEPROM_PARAM_BASE_ADDRESS + EEPROM_OFFSET_ADDRESS_TMIN);
    }
    if (temp_c > tMax) {
      tMax = temp_c;
      writeTInEeprom(tMax, EEPROM_PARAM_BASE_ADDRESS + EEPROM_OFFSET_ADDRESS_TMAX);
    }
  }

  if (pushLeftButton && tCsg >= DEFAULT_ICE_CSG + DEFAULT_CSG_STEP) {
    tCsg -= DEFAULT_CSG_STEP;
    writeTInEeprom(tCsg, EEPROM_PARAM_BASE_ADDRESS + EEPROM_OFFSET_ADDRESS_TTARGET);

  }
  if (pushRightButton && tCsg <= DEFAULT_CSG_MAX - DEFAULT_CSG_STEP) {
    tCsg += DEFAULT_CSG_STEP;
    writeTInEeprom(tCsg, EEPROM_PARAM_BASE_ADDRESS + EEPROM_OFFSET_ADDRESS_TTARGET);
  }


  cmd = advancedRegulation(tCsg, temp_c, cmd);
  digitalWrite(LED_BUILTIN, cmd ? HIGH : LOW);

  draw(realTemp, tMin, tMax, tCsg, cmd);


}


/**
   \fn void I2CTxData(void)
   \brief reply to python cmd : bus.read_i2c_block_data(I2C node, address, data size);
   \param[in] none
   \param[out] none
   \return nothing
*/
void I2CTxData(void) {

#if WITH_I2C
  unsigned char index = 0;
  Serial.print("------ sendData (address=");
  Serial.print(I2CData.address);
  Serial.println(")");

  rawData.sensorValue = realTemp*10;
  rawData.targetValue = tCsg;
  rawData.cmd = cmd;
  rawData.hystValue = hystValue;

  switch (I2CData.address) {
    case ADDRESS_DATA:
      Wire.write((char*)&rawData, sizeof(sRawData) ); // respond with message
      break;
    case ADDRESS_SENSOR_VALUE:
      I2CData.data[index++] = (rawData.sensorValue&0xFF00)>>8;
      I2CData.data[index++] = (rawData.sensorValue&0x00FF);
      Wire.write(I2CData.data, 2); // respond with message
      break;
    case ADDRESS_TARGET_VALUE:
      I2CData.data[index++] = rawData.targetValue;
      Wire.write(I2CData.data, 1); // respond with message
      break;
    case ADDRESS_CMD_VALUE:
      I2CData.data[index++] = rawData.cmd;
      Wire.write(I2CData.data, 1); // respond with message
      break;
    case ADDRESS_HYST_VALUE:
      I2CData.data[index++] = rawData.hystValue;
      Wire.write(I2CData.data, 1); // respond with message
      break;
  }
#endif
}

// reply to python cmd : bus.write_i2c_block_data(8,address,data);
/**
   \fn void I2CTxData(int byteCount)
   \brief reply to python cmd : bus.write_i2c_block_data(I2C node,address,data);
   \param[in] int byteCount : number of bytes to read
   \param[out] none
   \return nothing
*/
void I2CRxData(int byteCount) {
#if WITH_I2C

  unsigned char inputData = 0;
  unsigned char count = 0;

  Serial.print("------ receiveData (");
  Serial.print(byteCount);
  Serial.println(")");

  while (Wire.available()) {

    inputData = Wire.read();
    if (count == 0) {
      I2CData.address = inputData;
      Serial.print("address: ");
      Serial.println((int)I2CData.address);
    }
    else {
      I2CData.data[count - 1] = inputData;
      Serial.print("data   : ");
      Serial.println((unsigned int)(I2CData.data[count - 1]));
    }
    count++;
  }

  if (byteCount > 1) {
    rxDataAnalysis(&I2CData);
  }
#endif
  return;
}



/**
   \fn void rxDataAnalysis(pI2CData pData)
   \brief received data analysis ; update realdata
   \param[in] pI2CData pData : pointer on received raw data
   \param[out] realData (global variable)
   \return nothing
*/
void rxDataAnalysis(pI2CData pData) {
#if WITH_I2C

  switch (pData->address) {

    case ADDRESS_LED: // cmd to set/reset LED
      digitalWrite(13, pData->data[0] ? HIGH : LOW);
      break;

    case ADDRESS_DATA: // set all
      tCsg = (pData->data[0]);
      temp_c = (pData->data[1]);
      cmd = (pData->data[2]);
      hystValue = (pData->data[3]);
      break;

    case ADDRESS_TARGET_VALUE: // set targetValue
      Serial.print("set tCsg to ");
      tCsg = (pData->data[0]);
      Serial.println(tCsg);
      break;

    case ADDRESS_SENSOR_VALUE: // set sensorValue
      Serial.print("set temp_c to ");
      temp_c = pData->data[0];
      Serial.println(temp_c);
      break;

    case ADDRESS_HYST_VALUE: // set sensorValue
      Serial.print("set hystValue to ");
      hystValue = pData->data[0];
      Serial.println(hystValue);
      break;
  }
#endif

  return;
}



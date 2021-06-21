// **************************************************************************************************
// Bees Monitoring System - Node
// This file do following:
// - read temperature, pressure, humidity (and altitude) of one or two connected BME280 sensors
// - read weight of connected Hx711 ADC
// - transmit data via LoRaWan connectivity as Cayenne LPP formatted data
// - all periphery and MKR1300 board will periodically sleep and wake (to safe battery power)
//
// Thanks for libraries which were used in the project.
// This project is a part of my diploma thesis for technical computer sience at the university of
// applicance siences in Mittweida (Germany). The project is a show case for LoRaWan technology.
//
// Thanks for your interest.
// Ronald Creutz, 06.10.2019
// **************************************************************************************************

// **************************************************************************************************
//  insert needed libraries
// **************************************************************************************************
// ... powersafer for akr boards
#include "ArduinoLowPower.h" // note: the library RTCZero is also needed

// ... Hx711
#include "HX711.h"

// ... Bosch Bme280 sensor
#include <Wire.h>
#include <SparkFunBME280.h>

// ... LoRaWan
#include <MKRWAN.h>

// ... Cayenne LPP
#include <CayenneLPP.h>

// **************************************************************************************************
//  settings and datastructure
// **************************************************************************************************
// ... global stuff
const int SLEEP_TIME = 600000; // ms -> 10min.
const int MAX_TRY_AFTER_ERROR = 3;
const int WAITING_TIME_AFTER_ERROR = 5000; // ms
int _cntError = 0;

// ... for Battery Power measurement
const int BATTERY_PIN = A3;
float _batteryVoltage = 0.0;

// ... for HX711
const int LOADCELL_DOUT_PIN = A1;
const int LOADCELL_SCK_PIN = A2;
float _calibrationFactor = 0.0;
float _calibrationTemperature = 0.0;
HX711 _scale;
float _weight = 0.0;
float _weightRaw = 0.0;
float _deviationFactor = 0.0;
int _scaleOffset = 0;

// ... for BME280 sensor(s) - I2C connection
const int _indoorSensorAddress = 0x76;
const int _outdoorSensorAddress = 0x77;
BME280 _indoorSensor;
BME280 _outdoorSensor;
float _indoorTemperature = 0.0;
float _indoorPressure = 0.0;
float _indoorAltitude = 0.0;
float _indoorHumidity = 0.0;

float _outdoorTemperature = 0.0;
float _outdoorPressure = 0.0;
float _outdoorAltitude = 0.0;
float _outdoorHumidity = 0.0;

// ... for LoRaWan connectivity
const String APPEUI = "00000000000000000000000000000000"; //"0000000000000000"; // always this value
const String APPKEY = "e6a644d0e5be42037bd694eb10cc9ce7"; // use MSB and copy it of gateway website
LoRaModem _modem;

// ... for CayenneLPP
CayenneLPP _lpp(51);

// **************************************************************************************************
//  methods
// **************************************************************************************************
// The setup which executes when board starts after power on.
void setup() {
  Serial.begin(9600);
  // todo: only debugging issues!
  // while (!Serial);
  
  // config for battery voltage measurement
  pinMode(BATTERY_PIN, INPUT);

  // LoRaWan configuration
  Serial.println("Configure LoRaWan.");
  _cntError = 0;
  do {
    if (!_modem.begin(EU868)) {
      _cntError = _cntError + 1;
      Serial.println("Failed to start LoRa modem.");
      delay(WAITING_TIME_AFTER_ERROR);
    };
  } while ((_cntError != 0) & (_cntError < MAX_TRY_AFTER_ERROR));

  if (_cntError == MAX_TRY_AFTER_ERROR) {
    Serial.print("Error occured during LoRa modem configuration");
    return;
  }

  Serial.print("Your module version is: ");
  Serial.println(_modem.version()); // ARD-078 1.1.5
  Serial.print("Your device EUI is: ");
  Serial.println(_modem.deviceEUI()); // needed for gateway-device configuration
  
  // based on read deviceEUI -> configure HW
  if (_modem.deviceEUI() == "put your device eui here")
  { 
    configureHardwareForScale1();    
  }

  if (_modem.deviceEUI() == "put your device eui here")
  { 
    configureHardwareForScale2();    
  }

  ShowConfiguration();

  // Hx711 configuration
  Serial.println("Hx711 configuration.");
  _scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  _scale.set_offset(_scaleOffset);
  _scale.set_scale(_calibrationFactor);

  // indoor - bosch bme280 sensor configuration
  Wire.begin();

  Serial.println("Indoor bosch bme280 configuration.");
  _indoorSensor.setI2CAddress(_indoorSensorAddress);
  _cntError = 0;
  do {
    if (!_indoorSensor.beginI2C()) {
      _cntError = _cntError + 1;
      Serial.println("Could not find a valid indoor sensor, check wiring!");
      delay(WAITING_TIME_AFTER_ERROR);
    }
  } while ((_cntError != 0) & (_cntError < MAX_TRY_AFTER_ERROR));

  if (_cntError == MAX_TRY_AFTER_ERROR) {
    Serial.print("Error occured during indoor sensor configuration. Cancel process.");
    return;
  }
  
  Serial.println("Outdoor bosch bme280 configuration.");
  _outdoorSensor.setI2CAddress(_outdoorSensorAddress);
  _cntError = 0;
  do {
    if (!_outdoorSensor.beginI2C()) {
      _cntError = _cntError + 1;
      Serial.println("Could not find a valid outdoor sensor, check wiring!");
      delay(WAITING_TIME_AFTER_ERROR);
    }
  } while ((_cntError != 0) & (_cntError < MAX_TRY_AFTER_ERROR));
      
  if (_cntError == MAX_TRY_AFTER_ERROR) {
    Serial.print("Error occured during outdoor sensor configuration. Cancel process.");
    return;
  }
}

// Main method which runs in a loop.
void loop() {
  Serial.println("loop - entry");

  WakeUpPeriphery();

  GetBatteryVoltage();
  
  GetValuesBoschBmeSensors();

  GetWeight();

  SendLoraWanData();

  SleepPeriphery();

  Serial.println("Enter sleep mode for complete arduino board");
  LowPower.sleep(SLEEP_TIME);
  Serial.println("Arduino board waked up (time period).");

  Serial.println("loop - exit");
}

// Configure some values based on hardware environment.
void configureHardwareForScale1()
{
  Serial.println("configureHardwareForScale1 - entry");
  
  _calibrationFactor = 20110;
  _calibrationTemperature = 26;
  _deviationFactor = 0.144;
  _scaleOffset = 0;

  Serial.println("configureHardwareForScale1 - exit");
}

// Configure some values based on hardware environment.
void configureHardwareForScale2()
{
  Serial.println("configureHardwareForScale2 - entry");
  
  _calibrationFactor = 19800;
  _calibrationTemperature = 24;
  _deviationFactor = 0.1;
  _scaleOffset = 0;

  Serial.println("configureHardwareForScale2 - exit");
}

// Show configuration for debugging.
void ShowConfiguration()
{
  Serial.println("ShowConfiguration - entry");
  
  Serial.print("Configured calibration factor (HX711):");
  Serial.println(_calibrationFactor);
  Serial.print("Configured scale offset (HX711):");
  Serial.println(_scaleOffset);
  Serial.print("Configured deviation factor (weight):");
  Serial.println(_deviationFactor);
  Serial.print("Configured calibration temperature:");
  Serial.println(_calibrationTemperature);
  Serial.println("I2C address for indoor sensor:");
  Serial.println(_indoorSensorAddress);
  Serial.println("I2C address for outdoor sensor (if connected):");
  Serial.println(_outdoorSensorAddress);

  Serial.println("ShowConfiguration - exit");
}

// Wake up all connected periphery (exclusive arduino mkr board).
void WakeUpPeriphery()
{
  Serial.println("WakeUpPeriphery - entry");
  _indoorSensor.setMode(MODE_FORCED);

  Serial.println("WakeUp - indoor sensor.");
  while
  (
    //Wait for sensor to start measurment
    _indoorSensor.isMeasuring() == false
  ); 
    
  while
  (
    //Hang out while sensor completes the reading    
    _indoorSensor.isMeasuring() == true
  ); 

  Serial.println("WakeUp - outdoor sensor.");
  _outdoorSensor.setMode(MODE_FORCED);

  while
  (
    //Wait for sensor to start measurment
    _outdoorSensor.isMeasuring() == false
  ); 
    
  while
  (
    //Hang out while sensor completes the reading    
    _outdoorSensor.isMeasuring() == true
  );

  Serial.println("WakeUp - HX711 adc.");
  _scale.power_up();
  Serial.println("WakeUpPeriphery - exit");
}

// Send all connected periphery (exclusive arduino mkr board) in sleep mode.
void SleepPeriphery()
{
  Serial.println("SleepPeriphery - entry");
  
  Serial.println("Sleep - indoor sensor.");
  _indoorSensor.setMode(MODE_SLEEP);

  Serial.println("Sleep - outdoor sensor.");
  _outdoorSensor.setMode(MODE_SLEEP);
  
  Serial.println("Sleep - HX711 adc.");
  _scale.power_down();
  
  Serial.println("SleepPeriphery - exit");
}

// Get voltage of vcc (wired from vcc to input pin)
void GetBatteryVoltage()
{
  Serial.println("GetBatteryVoltage - entry");
  
  // ADC value = 1023 value with on 3.25V (VCC)
  // 3.15 full charged battery (measured with dmm)
  // 1023 / 3.25 = 314
  float adcValue = analogRead(BATTERY_PIN);
  _batteryVoltage = (adcValue / 314); 
  
  Serial.println("Battery adc value: ");
  Serial.println(adcValue);
  Serial.println("Battery voltage: ");
  Serial.println(_batteryVoltage);
  
  Serial.println("GetBatteryVoltage - exit");
}

// get weight of HX711
void GetWeight(){
  Serial.println("GetWeight - entry");
  
  if (_scale.wait_ready_retry(MAX_TRY_AFTER_ERROR, WAITING_TIME_AFTER_ERROR)) {
    _weightRaw = _scale.get_units(2); // includes offset, average of 10 values
    // linear compensation for temperature drift
    _weight = _weightRaw + ((_outdoorTemperature - _calibrationTemperature) * _deviationFactor * -1);

    Serial.println("raw weight:");
    Serial.println(_weightRaw);
    
    Serial.println("compensated weight:");
    Serial.println(_weight);

    Serial.println("scale factor");
    Serial.println(_scale.get_scale());
  }
  else {
    _weight = 0.0;
    Serial.println("Scale (HX711) not found. Cancel process.");
    return;
  }
  
  Serial.println("GetWeight - exit");
}

// Get all values of connected indoorsensor and outdoorsensor if it is connected.
void GetValuesBoschBmeSensors()
{
  Serial.println("GetValuesBoschBmeSensors - entry");
  // no error handling possible

  //Start with temperature, as that data is needed for accurate compensation.
  //Reading the temperature updates the compensators of the other functions.
  // https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/blob/master/examples/More_Advanced/I2C_and_SPI_Multisensor/I2C_and_SPI_Multisensor.ino
  _indoorTemperature = _indoorSensor.readTempC(); // °C
  _indoorPressure = _indoorSensor.readFloatPressure(); // return value in unit Pa -> mbar factor 100
  _indoorAltitude = _indoorSensor.readFloatAltitudeMeters(); // m
  _indoorHumidity = _indoorSensor.readFloatHumidity(); // %

  Serial.println("measured values of indoor sensor:");
  Serial.println(_indoorTemperature);
  Serial.println(_indoorPressure);
  Serial.println(_indoorAltitude);
  Serial.println(_indoorHumidity);
  
   _outdoorTemperature = _outdoorSensor.readTempC(); // °C
  _outdoorPressure = _outdoorSensor.readFloatPressure(); // // return value in unit Pa -> mbar factor 100
  _outdoorAltitude = _outdoorSensor.readFloatAltitudeMeters(); // m
  _outdoorHumidity = _outdoorSensor.readFloatHumidity(); // %

  Serial.println("measured values of outdoor sensor:");
  Serial.println(_outdoorTemperature);
  Serial.println(_outdoorPressure);
  Serial.println(_outdoorAltitude);
  Serial.println(_outdoorHumidity);
  
  Serial.println("GetValuesBoschBmeSensors - exit");
}

// Sends data formatted as CayenneLpp with LoRaWan modem.
void SendLoraWanData()
{
  Serial.println("SendLoraWanData - entry");
      
  Serial.println("Starting modem.");
  _cntError = 0;
  do {
    if (!_modem.begin(EU868)) {
      _cntError = _cntError + 1;
      Serial.println("Error occured during LoRa modem configuration (loop).");
      delay(WAITING_TIME_AFTER_ERROR);
    }
  } while ((_cntError != 0) & (_cntError < MAX_TRY_AFTER_ERROR));

  if (_cntError == MAX_TRY_AFTER_ERROR) {
    Serial.print("Cancel process.");
    return;
  }

  delay(500); // important to wait a while?!

  Serial.println("Connect to gateway (join request).");
  _cntError = 0;
  do {
    if (!_modem.joinOTAA(APPEUI, APPKEY)) {
      _cntError = _cntError + 1;
      Serial.println("Error occured during join to LoRaWan network.");
      delay(WAITING_TIME_AFTER_ERROR);
    }
  } while ((_cntError != 0) & (_cntError < MAX_TRY_AFTER_ERROR));

  if (_cntError == MAX_TRY_AFTER_ERROR) {
    Serial.print("Cancel process.");
    return;
  }
  
  delay(500); // important to wait a while?!

  _modem.setPort(3);
  _modem.beginPacket();

  Serial.println("Build cayenne lpp message.");
  BuildLppMessage();
  
  Serial.println("Send message.");
  _modem.write(_lpp.getBuffer(), _lpp.getSize());
  
  _cntError = 0;
  do {
    if (_modem.endPacket(true) < 0) {
        _cntError = _cntError + 1;
        Serial.println("Error occured during send LoRa message.");
        delay(WAITING_TIME_AFTER_ERROR);
    }
  } while ((_cntError != 0) & (_cntError < MAX_TRY_AFTER_ERROR));

  if (_cntError == MAX_TRY_AFTER_ERROR) {
    Serial.print("Cancel process.");
    return;
  }
    
  Serial.println("SendLoraWanData - exit");
}

// Build the message for cayenne lpp format.
// a workaround with one gps sensor is used to spend bytes.
// otherwise an overflow occured with two connected sensor.
void BuildLppMessage()
{
  Serial.println("BuildLppMessage - entry");
  
  // build message
  _lpp.reset();
    
  _lpp.addTemperature(1, _indoorTemperature);
  _lpp.addBarometricPressure(1, _indoorPressure / 100); // Pa -> hPa (expected unit!)
  _lpp.addRelativeHumidity(1, _indoorHumidity);
  // _lpp.addAltitude(1, _indoorAltitude); // not implemented on LoRaServer decoder
  // _lpp.addGPS(1, 0, 0, _indoorAltitude); // needs 9 bytes -> to big
    //_lpp.addGenericSensor(5, _weight); // not implemented on LoRaServer decoder
  _lpp.addAnalogInput(1, _weight);
  _lpp.addAnalogInput(2, _batteryVoltage);
  _lpp.addAnalogInput(3, _weightRaw);
  
  _lpp.addTemperature(2, _outdoorTemperature);
  _lpp.addBarometricPressure(2, _outdoorPressure / 100); // Pa -> hPa (expected unit)
  _lpp.addRelativeHumidity(2, _outdoorHumidity);
  // _lpp.addGPS(2, 0, 0, _outdoorAltitude); // needs 9 bytes -> to big 
  _lpp.addGPS(1, 0, _outdoorAltitude, _indoorAltitude);  // -> to spend some bytes -> altitude: sensor1 alt; longitude: sensor2 alt

  Serial.println("size of LPP message [Bytes]:");
  Serial.println(_lpp.getSize());

  Serial.println("cayenne LPP error:");
  Serial.println(_lpp.getError());

  Serial.println("BuildLppMessage - exit");
}

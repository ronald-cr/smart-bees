/*
  hardware: MKR 1300, Bosch BME280 connected over I2C
  download lib here: http://cactus.io/hookups/sensors/barometric/bme280/hookup-arduino-to-bme280-barometric-pressure-sensor
  - unzip to c:\user\documents\arduino\libraries
  - ... Sketch -> include library -> add zip library
  - compiled and uploaded with Arduino Studio
  - open tools -> serial monitor to see output

  - advantage: I2C address is configurable!
*/
// ... Bosch Bme280 sensor
#include <Wire.h>
#include <SparkFunBME280.h>

const int MAX_TRY_AFTER_ERROR = 3;
const int WAITING_TIME_AFTER_ERROR = 5000; // ms
const int MEASUREMENT_INTERVAL = 5000; // ms
int _cntError = 0;

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("setup - entry");
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

  Serial.println("I2C address for indoor sensor:");
  Serial.println(_indoorSensorAddress);
  Serial.println("I2C address for outdoor sensor (if connected):");
  Serial.println(_outdoorSensorAddress);

  Serial.println("setup - exit");
}

void loop() {
  GetValuesBoschBmeSensors();
  delay(MEASUREMENT_INTERVAL);
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

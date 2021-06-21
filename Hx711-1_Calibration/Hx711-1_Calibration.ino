                                                                              // original code taken from here: https://github.com/sparkfun/HX711-Load-Cell-Amplifier/blob/master/firmware/SparkFun_HX711_Calibration/SparkFun_HX711_Calibration.ino
#include "HX711.h"

// HX711 circuit wiring to arduino mkr 1300
const int LOADCELL_DOUT_PIN = A1;
const int LOADCELL_SCK_PIN = A2;

// calibration settings
float calibration_factor = 20110;

HX711 scale;

void setup() {
  Serial.begin(9600);
  Serial.println("********* Calibration program started *********");
  // set hardware pins
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  // initial set to 0 (without extra weight!)
  scale.set_scale();
  scale.tare();
  // get zero factor
  long zero_factor = scale.read_average();
  Serial.print("Zero factor:");
  Serial.println(zero_factor);
}

void loop() {
  if (scale.is_ready()) {  
    scale.set_scale(calibration_factor);
    Serial.print(scale.get_units(), 2);
    Serial.println("kg");
    // calibration with known weight
    Serial.print("Calibration factor:");
    Serial.println(calibration_factor);
    Serial.print("Offset (put 0 kg on the weight):");
    Serial.println(scale.get_offset());
    
    // with +/- an anjustment is possible
    if(Serial.available())
    {
      String temp = Serial.readString();
      if(temp.length() > 0)
      {
        calibration_factor = temp.toInt();
      }
      //if(temp == '+' || temp == 'a')
      //  calibration_factor += 10;
      // else if(temp == '-' || temp == 'z')
      //  calibration_factor -= 10;
    }
  } else {
    Serial.println("HX711 not found.");
  }

  delay(1000);
}

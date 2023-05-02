
#include "main.h"

void setup() {
  Serial.begin(115200);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  while (!Serial);

  // Initialize sensors, actuators and software components
  IMU_Init();
  Motor_Init();  
  TimedFunctionCaller_Init();
}

void loop() {
  // Timer based function caller to have precise sampling of data and control
  TimedFunctionCaller();
}

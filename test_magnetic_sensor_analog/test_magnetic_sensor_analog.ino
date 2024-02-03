/* test_magnetic_sensor_analog
 https://docs.simplefoc.com
 */

#include "src/SimpleFOC.h" // ver. 1.6.1

/**
 * Magnetic sensor reading analog voltage on pin A0 or A1.  This voltage is proportional to rotation position.
 * Tested on AS5600 magnetic sensor running in 'analog mode'.  Note AS5600 works better in 'i2C mode' (less noise) but only supports one sensor per i2c bus. 
 * 
 * MagneticSensorAnalog(uint8_t _pinAnalog, int _min, int _max)
 * - pinAnalog      - the pin that is reading the pwm from magnetic sensor
 * - min_raw_count  - the smallest expected reading.  Whilst you might expect it to be 0 it is often ~15.  Getting this wrong results in a small click once per revolution
 * - max_raw_count  - the largest value read.  whilst you might expect it to be 2^10 = 1023 it is often ~ 1020. Note ESP32 will be closer to 4096 with its 12bit ADC
 */

MagneticSensorAnalog sensorL = MagneticSensorAnalog(A1, 14, 1020); // for read the left sensor
MagneticSensorAnalog sensorR = MagneticSensorAnalog(A0, 14, 1020); // for read the right sensor

void setup() {
  // monitoring port
  Serial.begin(9600);

  // initialise magnetic sensor hardware
  sensorL.init();
  sensorR.init();
  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // display the angles to the terminal
  Serial.print("L: ");
  Serial.print(sensorL.getAngle());
  Serial.print(" ");
  Serial.print("R: ");
  Serial.println(sensorR.getAngle());
}

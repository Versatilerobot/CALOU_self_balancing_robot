/* find_sensor_offset_and_direction
 https://docs.simplefoc.com

 * Simple example intended to help users find the zero offset and natural direction of the sensor.  
 * These values can further be used to avoid motor and sensor alignment procedure. 
 * motor.initFOC(zero_offset, sensor_direction);
 * This will only work for abosolute value sensors - magnetic sensors. 
 * Bypassing the alignment procedure is not possible for the encoders and for the current implementation of the Hall sensors. 
 * 
 */
#include "src/SimpleFOC.h" // ver. 1.6.1

MagneticSensorAnalog sensorL = MagneticSensorAnalog(A1, 14, 1020);
MagneticSensorAnalog sensorR = MagneticSensorAnalog(A0, 14, 1020);

// BLDC motor instance
BLDCMotor motorL = BLDCMotor(9, 10, 11, 7); // pins + pole pairs
BLDCMotor motorR = BLDCMotor(3, 5, 6, 7); // pins + pole pairs

void setup() {

  // initialise magnetic sensor hardware
  sensorL.init();
  sensorR.init();
  // link the motor to the sensor
  motorL.linkSensor(&sensorL);
  motorR.linkSensor(&sensorR);
  // power supply voltage
  // default 12V
  motorL.voltage_power_supply = 12;
  motorR.voltage_power_supply = 12;
  // aligning voltage 
  motorL.voltage_sensor_align = 7;
  motorR.voltage_sensor_align = 7;
  // set motion control loop to be used
  motorL.controller = ControlType::voltage;
  motorR.controller = ControlType::voltage;
  // initialize motor
  motorL.init();
  motorR.init();  
  // align sensor and start FOC
  motorL.initFOC();
  motorR.initFOC();
  
  Serial.begin(9600);
  Serial.println("motorL");
  Serial.print("Offset: ");
  Serial.print(motorL.zero_electric_angle, 4);
  Serial.println(sensorL.natural_direction == 1 ? " Direction: CW" : " Direction: CCW"); 

  Serial.println("motorR");
  Serial.print("Offset: ");
  Serial.print(motorR.zero_electric_angle, 4);   
  Serial.println(sensorR.natural_direction == 1 ? " Direction: CW" : " Direction: CCW");  
  Serial.println("To use these values provide them to the robot sketch:");
  Serial.println("motorL.initFOC(offset, direction)");
  Serial.println("motorR.initFOC(offset, direction)");  
  _delay(1000);
  Serial.println("If motor are not moving the alignment procedure was not successfull!!");

}

void loop() {
    
  // main FOC algorithm function
  motorL.loopFOC();
  motorR.loopFOC();

  // motion control function
  motorL.move(2);
  motorR.move(2);
  
}


/**
 * 
 * Velocity motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor 
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from serial terminal
 * 
 * 
 * By using the serial terminal set the velocity value you want to motor to obtain
 *
 */
#include <SimpleFOC.h>

// magnetic sensor instance
MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);
// magnetic sensor instance
//MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0x0E, 4);
// magnetic sensor instance - analog output
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor instance
BLDCMotor motor = BLDCMotor(9, 5, 6, 11, 8);
// Stepper motor instance
//StepperMotor motor = StepperMotor(9, 5, 10, 6, 50, 8);

void setup() {
 
  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;

  // set motion control loop to be used
  motor.controller = ControlType::velocity;

  // contoller configuration 
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  
  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  Serial.println("Motor ready.");
  Serial.println("Set the target velocity using serial terminal:");
  _delay(1000);
}

// velocity set point variable
float target_velocity = 0;

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();
  
  // user communication
  serialReceiveUserCommand();
}

// utility function enabling serial communication with the user to set the target values
// this function can be implemented in serialEvent function as well
void serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {
      
      // change the motor target
      target_velocity = received_chars.toFloat();
      Serial.print("Target velocity: ");
      Serial.println(target_velocity);
      
      // reset the command buffer 
      received_chars = "";
    }
  }
}


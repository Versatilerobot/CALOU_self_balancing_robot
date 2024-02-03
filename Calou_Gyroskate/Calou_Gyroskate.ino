
/* Calou_Gyroskate
 * This is a sketch for a two wheel balancing robot done for a BGC 3.1 Brushless Gimbal Controller with gimbal BLDC motors and SimpleFOClibrary. 
 * It was derived from the application example done for a STM32 Nucleo-64 board you can find on the github "SimpleFOCproject": Arduino SimpleFOCBalancer
 https://docs.simplefoc.com
 https://github.com/simplefoc/Arduino-FOC-balancer
 */

#include "imu_helpers.h"
#include "src/SimpleFOC.h" // ver. 1.61

//// create motor and sensor instances
//BLDCMotor motorL = BLDCMotor(3, 5, 6, 7); // pins + pole pairs
//MagneticSensorAnalog sensorL = MagneticSensorAnalog(A0, 14, 1020);
//
//BLDCMotor motorR = BLDCMotor(9, 10, 11, 7); // pins + pole pairs
//MagneticSensorAnalog sensorR = MagneticSensorAnalog(A1, 14, 1020);

//magnetic sensor instance
MagneticSensorAnalog sensorL = MagneticSensorAnalog(A1, 14, 1020);
MagneticSensorAnalog sensorR = MagneticSensorAnalog(A0, 14, 1020);
// motor instance
BLDCMotor motorL = BLDCMotor(9, 10, 11, 7); // pins + pole pairs
BLDCMotor motorR = BLDCMotor(3, 5, 6, 7); // pins + pole pairs

// control algorithm parameters
// stabilisation pid
PIDController pid_stb{.P = 50, .I = 90, .D = 2, .ramp = 100000, .limit = 10};
// velocity pid
PIDController pid_vel{.P = 0.015, .I = 0.01, .D = 0, .ramp = 2000, .limit = _PI / 10};
// velocity control filtering
LowPassFilter lpf_pitch_cmd{.Tf = 0.35};
// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle{.Tf = 0.4};
LowPassFilter lpf_steering{.Tf = 0.01};

// Bluetooth variables
int state = 1;           // stabilization enabling
float steering = 0;
float throttle = 0;
float max_throttle = 20; // 20 rad/s
float max_steering = 1;  // 1 V
float MaxAngle = PI/8;   // dead angle activation limit 
char Incoming_value = 0; // Variable for storing serial value

// constants won't change. Used here to set a pin number :
const int ledPin =  A3;             // the pin of the LED antenna
int ledState = LOW;                 // ledState used to set the LED
unsigned long previousMillis = 0;   // will store last time LED was updated
long interval = 1000;               // interval at which to blink (milliseconds)

void setup() {
  // use monitoring with Serial for motor init
  // monitoring port
  Serial.begin(9600);
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);  
  delay(1000);
  // imu init and configure
  if ( !initIMU() ) {
    Serial.println(F("IMU connection problem... Disabling!"));
    return;
  }
  delay(1000);
  
  // initialise magnetic sensor hardware
  sensorL.init();
  sensorR.init();                                             
  // link the motor to the sensor
  motorL.linkSensor(&sensorL);
  motorR.linkSensor(&sensorR);
  // power supply voltage
  // default 12V
  motorL.voltage_power_supply = 11;
  motorR.voltage_power_supply = 11;
  // set motion control loop to be used  
  motorL.controller = ControlType::voltage;
  motorR.controller = ControlType::voltage;
  // choose FOC modulation (optional)  
  motorL.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorR.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // initialize motor 
  motorL.init();
  motorR.init();
  // align encoder and start FOC
  // before upload this sketch please look at the instructions for find sensor
  // offset and direction of the motors corresponding to your robot (sketch: find_sensor_offset_and_direction.ino)                                                 
  motorL.initFOC(); // must be updated with your values e.g: motorL.initFOC(6.0520,CCW);
  motorR.initFOC();  // must be updated with your values e.g: motorR.initFOC(0.1287,CW);                                   
  delay(2000);
  Serial.println(F("Balancing robot ready!"));
}

void loop() {
  
  // antenna blink update
  BlinkLed();
  
  // main FOC algorithm function
  motorL.loopFOC();
  motorR.loopFOC();

  // motion control function
  motorL.move();
  motorR.move();

  if (!state) { // if balancer disabled
    motorL.target = 0;
    motorR.target = 0;
  } else if ( hasDataIMU() ) { // when IMU has received the package
    // read pitch from the IMU
    float pitch = getPitchIMU();
    //Serial.println(pitch);
    if (pitch < MaxAngle && pitch > -MaxAngle){
    pitch = pitch - PI/180; //offset of the verticale position link to the sensor mounting (optional)
    // calculate the target angle for throttle control
    float target_pitch = lpf_pitch_cmd(pid_vel((motorL.shaft_velocity + motorR.shaft_velocity) / 2 - lpf_throttle(throttle)));  
    // calculate the target voltage
    float voltage_control = pid_stb(target_pitch - pitch);
    // filter steering
    float steering_adj = lpf_steering(steering);
    // set the target voltage value
    motorL.target = voltage_control + steering_adj;
    motorR.target = voltage_control - steering_adj;
  }
  else {
    throttle = 0;
    steering = 0;
  }
}

// read the user command from bluetooth
  Bluetooth();
}

void BlinkLed() {

  unsigned long currentMillis = millis()/32; // timer was changed by the SimpleFOC library

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}

void Bluetooth() {
  if (Serial.available()> 0){  
  Incoming_value = Serial.read();     
  if(Incoming_value != '\n')
  {
    if (Incoming_value == '1') {
      // stabilization enabled
      steering = 0;
      throttle = 0;
      state = 1;
    } else if (Incoming_value == '0') {
      // stabilization disabled
      steering = 0;
      throttle = 0;
      state = 0;
    } else if (Incoming_value == 'A') {
      if (throttle < max_throttle) throttle = throttle + 2; // move forward - send several time for accelerate
    } else if (Incoming_value == 'R') {
      if (throttle > - max_throttle) throttle = throttle - 2; // move backward - send several time for accelerate
    } else if (Incoming_value == 'G') {
      if (steering > - max_steering) steering = steering - 0.05; // turn left - send several time for shorting the radius   
    } else if (Incoming_value == 'D') {
      if (steering < max_steering) steering = steering + 0.05; // turn right - send several time for shorting the radius
    } else if (Incoming_value == 'L') {
      steering = 0; // stop steering
    } else if (Incoming_value == 'S'){
      // stop moving
      steering = 0;
      throttle = 0;
      }
    }
  }
}


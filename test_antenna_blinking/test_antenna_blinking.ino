/* Blink without Delay
 http://www.arduino.cc/en/Tutorial/BlinkWithoutDelay
 */

// constants won't change :
const int ledPin =  A3;      // the number of the LED pin on command board

// variables will change :
int ledState = LOW;             // ledState used to set the LED

// use "unsigned long" for variables that hold time :
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
const long interval = 1000;           // interval at which to blink (milliseconds)

void setup() {
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
}

void loop() {

  unsigned long currentMillis = millis();

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


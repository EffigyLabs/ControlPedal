    /*
    Tester for Emprenda / multiple OTA devices
     continually read sensor a0, a1, and a2 and output to the debug com port.
     used to tune and position the OTAs, and record centering and maxing parameters.
    Output compatible with both plottter and serial monitor
*/
#include <systemblock.h>
#include <pedal.h>
#include <ResponsiveAnalogRead.h> // input smoothing - Damien Clarke

// set sensdelaymode to 0 to wait 0 ms between samples
// set sensdelaymode to 1 to wait n ms between samples where n = value of the sensitivity knob.
// set to > 1 to wait a fixed amount between samples
int sensdelaymode = 0;
int doblem = 0; // blink emitters

long sample = 0;
//int modeSwitchPin = 6;      // mode switch button
int modeSwitchState = 0;
//int downSensorPin = A3;  // L OTA sensor input pin f7 for on breakout/leonardo
//int upSensorPin = A2;    // R OTA sensor input pin
//int pgmSensorPin = A1;   // Center OTA input pin
//int sensitivityPin = A0; // knob pin

//int sensorPowerPin = 1;   // power to voltage divider circuits for OTA sensors?
//int pedalLedPin = 2;      // ota emitters


// input smoothing
ResponsiveAnalogRead downSensorPin(pos1Pin, true,.1); // OTA1
ResponsiveAnalogRead upSensorPin(pos2Pin, true,.1); // OTA2
ResponsiveAnalogRead pgmSensorPin(pos3Pin, true,.1); // OTA3
ResponsiveAnalogRead sensitivityPin(knobPin, true, .1); // Knob doesn't need snap adjust


int pedalLedTimer = 1000;  // ms to wait
unsigned long pedalLedCounterTimer;
int commLedState; // 0 = off 1 = on
//int commLedPin = 3;       // comm led blue light
//int redLedPin = 9;       // comm led red light
int LEDtoggleState = 0;   // for toggling asynchronously
int commLEDtoggleState = 1;   // for toggling asynchronously
int redLEDtoggleState = 0;   // for toggling asynchronously

int downSensorValue = 0;  // variable to store the value coming from the sensor
int upSensorValue = 0;  // variable to store the value coming from the sensor
int pgmSensorValue = 0;  // variable to store the value coming from the sensor
int sensitivityKnobValue = 0;  // variable to store the value coming from the sensor
int last_downSensorValue = 0;  // variable to store the value coming from the sensor
int last_upSensorValue = 0;  // variable to store the value coming from the sensor
int last_pgmSensorValue = 0;  // variable to store the value coming from the sensor
int last_sensitivityKnobValue = 0;  // variable to store the value coming from the sensor

int starray[4][3];
int wlarr = 0;
int wlarr2 = 0;

void setup() {
  Serial.begin(9600); // comment this out and uncomment the midi rate when doing midi - use this for debugging //Serial.println()
  pinMode(pedalLedPin, OUTPUT);
  pinMode(commLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  //pinMode(sensorPowerPin, OUTPUT);
  //pinMode(modeSwitchPin, INPUT);
  //digitalWrite(sensorPowerPin, HIGH);  // turn on sensor power (not getting from general vcc circuit)
  digitalWrite(redLedPin, HIGH);  // turn on leds
  digitalWrite(pedalLedPin, HIGH);  // turn on leds
  commLedState = 1;
  //analogWrite(commLedPin, 500);  // turn on leds
  digitalWrite(redLedPin, HIGH);  // turn on leds
  //blinkLED(dimLedPin,333,5);
  blinkLED(commLedPin, 333, 3);
  blinkLED(redLedPin, 333, 3);
  LEDtoggleState = 1;
  //blinkLED(pedalLedPin,333,3);

  
    while (!Serial) {
    ; // wait
  }

  pedalLedCounterTimer = millis(); // initialize timer
}

void loop() {
  wlarr = millis();
  //modeSwitchState = digitalRead(modeSwitchPin);
  modeSwitchState = analogRead(modeSwitchPin);
  modeSwitchState = analogRead(modeSwitchPin);

  //int[4][2] starray;
  int i;
  for (i = 0; i < 4; i++) {
    starray[i][0] = millis();

    switch (i) {
      case 0:
        downSensorValue = analogRead(pos1Pin);
        downSensorPin.update();
        downSensorValue = downSensorPin.getValue();
//        Serial.print("down=");
//        Serial.println(downSensorValue);
//        downSensorValue = analogRead(downSensorPin);
        break;
      case 1:
        upSensorValue = analogRead(pos2Pin);
        //delay(1);
        upSensorPin.update();
        upSensorValue = upSensorPin.getValue();
//        upSensorValue = analogRead(upSensorPin);
        break;
      case 2:
        pgmSensorValue = analogRead(pos3Pin);
        pgmSensorPin.update();
        pgmSensorValue = pgmSensorPin.getValue();

//        pgmSensorValue = analogRead(pgmSensorPin);
        break;
      case 3:
        sensitivityKnobValue = analogRead(knobPin);
        sensitivityPin.update();
        sensitivityKnobValue = sensitivityPin.getValue();
//        Serial.print("knob=");
//        Serial.println(sensitivityKnobValue);

//        sensitivityKnobValue = analogRead(sensitivityPin);
        break;
      default:
        // statements
        break;
    }
  }
 
  wlarr2 = millis() - wlarr;
  if ((upSensorValue != last_upSensorValue) | (downSensorValue != last_downSensorValue) | (pgmSensorValue != last_pgmSensorValue)) {
    //Serial.println("here!");
    //      Serial.print("s: ");
    //      Serial.print(sample++);

    //      Serial.print(" (");
    //      Serial.print(wlarr2);
    //      Serial.print(")");
    //Serial.print("\t d: ");
    Serial.print("\t");
    Serial.print(downSensorValue);
    //      Serial.print(" (");
    //      Serial.print(starray[0][2]);
    //      Serial.print(")");
    Serial.print("\t");
//    Serial.print("\t u: ");
    Serial.print(upSensorValue);
    //    Serial.print(" (");
    //    Serial.print(starray[1][2]);
    //    Serial.print(")");
    Serial.print("\t");
//    Serial.print("\t c: ");
    Serial.print(pgmSensorValue);
    //    Serial.print(" (");
    //    Serial.print(starray[2][2]);
    //    Serial.print(")");
    Serial.print("\t");
//    Serial.print("\t k: ");
    Serial.print(sensitivityKnobValue);
    //    Serial.print(" (");
    //    Serial.print(starray[3][2]);
    //    Serial.print("));
    Serial.print("\t");
//    Serial.print("\tM: ");
    Serial.println(modeSwitchState);

    switch (sensdelaymode)  {
      case 0:
        if (doblem > 0) blem();
        break;
      case 1:
        delay(sensitivityKnobValue);
        break;
      default:
        delay(sensdelaymode);
        break;

    }


    //     Serial.print("A0=");
    //   Serial.println(A0);

    last_upSensorValue = upSensorValue;
    last_downSensorValue = downSensorValue;
    last_pgmSensorValue = pgmSensorValue;
    last_sensitivityKnobValue = sensitivityKnobValue;

    toggleCommLED();
    toggleRedLED();

  }
}


void blem() {
  // blink LED emitters asynchrounously

  if ( millis() - pedalLedCounterTimer > pedalLedTimer) {
    pedalLedCounterTimer = millis();
    switch (commLedState)  {
      case 0:  // is low, turn high
        digitalWrite(pedalLedPin, HIGH);  // turn on emitters
        commLedState = 1;
        break;
      case 1:  // is high, turn low
        digitalWrite(pedalLedPin, LOW);  // turn off emitters
        commLedState = 0;
        break;
      default:
        //delay(0);
        break;
    }
  }
}
// turn LEDs on and off
void blinkLED(int LED, int time, int repetitions) {
  for (int t = 1; t < repetitions; t ++) {
    digitalWrite(LED, LOW);  // turn off led
    delay(time);
    digitalWrite(LED, HIGH);  // turn on led
    delay(time);
  }
}
// toggle LED from whatever state it is to the other state
void toggleLED(int LED) {
  if (LEDtoggleState == 0) {
    digitalWrite(LED, HIGH);  // turn on led
    LEDtoggleState = 1;
  } else {
    digitalWrite(LED, LOW);  // turn off led
    LEDtoggleState = 0;
  }
}

void toggleRedLED() {
  if (redLEDtoggleState == 0) {
    digitalWrite(redLedPin, HIGH);  // turn on led
    redLEDtoggleState = 1;
  } else {
    digitalWrite(redLedPin, LOW);  // turn off led
    redLEDtoggleState = 0;
  }
}
void toggleCommLED() {
  if (commLEDtoggleState == 0) {
    digitalWrite(commLedPin, HIGH);  // turn on led
    commLEDtoggleState = 1;
  } else {
    digitalWrite(commLedPin, LOW);  // turn off led
    commLEDtoggleState = 0;
  }
}




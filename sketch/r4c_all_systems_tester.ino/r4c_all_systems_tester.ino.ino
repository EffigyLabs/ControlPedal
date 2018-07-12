/*
    Tester for Emprenda / multiple OTA devices
     continually read sensor a0, a1, and a2 and output to the debug com port.
     used to tune and position the OTAs, and record centering and maxing parameters.
 */
// these pins are as of Emprenda box #4 14 Oct 2012 J Roberts
// these pins are as of Emprenda #21 26 March 2014 J Roberts
// these pins are as of Emprenda #34 9 November 2016 J Roberts
// sensitivity knob max for #34 seems to be 502 - 11/10/16 - with Face playing in the background!


// set sensdelaymode to 0 to wait 0 ms between samples
// set sensdelaymode to 1 to wait n ms between samples where n = value of the sensitivity knob.
// set to > 1 to wait a fixed amount between samples  
int sensdelaymode = 0;
int doblem = 0;

long sample = 0;
//for teeonardu
//int downSensorPin = A3;    //OTA1
//int upSensorPin = A2;      //OTA2
//int pgmSensorPin = A1;     //OTA3
//int sensitivityPin = A0;   //Sensitivity knob
int modeSwitchPin = 6;      // mode switch button
int modeSwitchState = 0;
//for leonardo
int downSensorPin = A3;  // input pin f7 for on breakout/leonardo
int upSensorPin = A2;    // input pin for LDR1
int pgmSensorPin = A1;   // input pin for LDR
int sensitivityPin = A0; // sensitivity pin



int sensorPowerPin = 1;   // power to voltage divider circuits for OTA sensors?
int pedalLedPin = 2;      // ota emitters
int pedalLedPin2 = 2;      // ota emitters

int pedalLedTimer = 1000;  // ms to wait
unsigned long pedalLedCounterTimer;
int commLedState; // 0 = off 1 = on
int commLedPin = 3;       // comm led blue light
int fadeLedPin = 9;       // comm led red light
int LEDtoggleState = 0;   // for toggling asynchronously 


int downSensorValue = 0;  // variable to store the value coming from the sensor
int upSensorValue = 0;  // variable to store the value coming from the sensor
int pgmSensorValue = 0;  // variable to store the value coming from the sensor
int sensitivityKnobValue = 0;  // variable to store the value coming from the sensor
int last_downSensorValue = 0;  // variable to store the value coming from the sensor
int last_upSensorValue = 0;  // variable to store the value coming from the sensor
int last_pgmSensorValue = 0;  // variable to store the value coming from the sensor
int last_sensitivityKnobValue = 0;  // variable to store the value coming from the sensor
//int starray[][3] = { {0,0,0},{0,0,0},{0,0,0},{0,0,0} };
int starray[4][3];
int wlarr = 0;
int wlarr2 = 0;

void setup() {
  Serial.begin(9600); // comment this out and uncomment the midi rate when doing midi - use this for debugging //Serial.println()
  pinMode(pedalLedPin, OUTPUT);  
  pinMode(commLedPin, OUTPUT);  
  pinMode(fadeLedPin, OUTPUT);  
  //pinMode(pedalLedPin, OUTPUT);  
  //pinMode(sensorPowerPin, OUTPUT);  
  //pinMode(modeSwitchPin, INPUT);  
  //pinMode(dimLedPin, OUTPUT);  
  //digitalWrite(sensorPowerPin, HIGH);  // turn on sensor power (not getting from general vcc circuit)
  digitalWrite(fadeLedPin, HIGH);  // turn on leds
  digitalWrite(pedalLedPin, HIGH);  // turn on leds
  commLedState = 1;
  //analogWrite(commLedPin, 500);  // turn on leds
  digitalWrite(fadeLedPin, HIGH);  // turn on leds
  //blinkLED(dimLedPin,333,5); 
  blinkLED(commLedPin,333,3);
  blinkLED(fadeLedPin,333,3);
  LEDtoggleState = 1; 
  //blinkLED(pedalLedPin,333,3); 
  pedalLedCounterTimer = millis(); // initialize timer
}
    
void loop() {
    wlarr = millis();
    //modeSwitchState = digitalRead(modeSwitchPin);
    modeSwitchState = analogRead(modeSwitchPin);
    modeSwitchState = analogRead(modeSwitchPin);
  
   //int[4][2] starray;
   int i;   
   for(i = 0; i < 4; i++) {
    starray[i][0] = millis();   

    switch (i) {
  case 0:
    downSensorValue = analogRead(downSensorPin);
    downSensorValue = analogRead(downSensorPin);
    break;
  case 1:
    upSensorValue = analogRead(upSensorPin);
    upSensorValue = analogRead(upSensorPin);
    break;
  case 2:
    pgmSensorValue = analogRead(pgmSensorPin);
    pgmSensorValue = analogRead(pgmSensorPin);
    break;
  case 3:
    sensitivityKnobValue = analogRead(sensitivityPin);
    sensitivityKnobValue = analogRead(sensitivityPin);
    break;
  default: 
    // statements
    break;
    }
   }

    wlarr2 = millis() - wlarr;
    if((upSensorValue != last_upSensorValue) | (downSensorValue != last_downSensorValue) | (pgmSensorValue != last_pgmSensorValue)) {
      Serial.print("s: ");
      Serial.print(sample++);

      Serial.print(" (");
      Serial.print(wlarr2);
      Serial.print(")");
      Serial.print("\t d: ");
      Serial.print(downSensorValue);
      Serial.print(" (");
      Serial.print(starray[0][2]);
      Serial.print(")");
      Serial.print("\t u: ");
      Serial.print(upSensorValue);
      Serial.print(" (");
      Serial.print(starray[1][2]);
      Serial.print(")");
      Serial.print("\t c: ");
      Serial.print(pgmSensorValue);
      Serial.print(" (");
      Serial.print(starray[2][2]);
      Serial.print(")");
      Serial.print("\t k: ");
      Serial.print(sensitivityKnobValue);
      Serial.print(" (");
      Serial.print(starray[3][2]);
      Serial.print(")\tM: ");
      Serial.println(modeSwitchState);
      
     switch(sensdelaymode)  {
      case 0: 
          if(doblem > 0) blem();
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
      
      toggleLED(commLedPin); 
      toggleLED(fadeLedPin); 
      
    }
}


void blem() {
  // blink LED emitters asynchrounously

  if( millis() - pedalLedCounterTimer > pedalLedTimer) {
     pedalLedCounterTimer = millis();
     switch(commLedState)  {
      case 0:  // is low, turn high
          digitalWrite(pedalLedPin, HIGH);  // turn off emitters
          commLedState = 1;
          break;
      case 1:  // is high, turn low
          digitalWrite(pedalLedPin, LOW);  // turn on emitters
          commLedState = 0;
          break;
      default: 
          delay(0);
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
 if(LEDtoggleState == 0) {
  digitalWrite(LED, HIGH);  // turn on led
  LEDtoggleState = 1;
 } else {
  digitalWrite(LED, LOW);  // turn off led
  LEDtoggleState = 0;
  }
 }
 


// Effigy Control Pedal system block definition
// (C) 2018, Effigy Labs LLC

const static String ver = "R4C_15"; // Source of Record.  This is where we say the version for all. Maybe.


// R4C pin maps
const static int pos1Pin = A3;
const static int pos2Pin = A2;
const static int pos3Pin = A1;
const static int knobPin = A0;
const static int pedalLedPin = 2;     // emitters
const static int commLedPin = 3;      // comm led port pin?????
const static int redLedPin = 9;       // red color of coomm Led Pin
const static int BLUE = 1;            // blue led lead
const static int RED = 2;             // red led lead
const static int BOTH = 3;            // blue and red leads on at the same time

// mode switch
const static int modeSwitchPin = 6;   // A6  pulled up, on atmega32u4 R4a+ boards


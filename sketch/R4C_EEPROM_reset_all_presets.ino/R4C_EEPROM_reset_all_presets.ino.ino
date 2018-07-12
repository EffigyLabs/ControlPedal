short serialnum = 0x03; // set to burn
short ceilsz = 100;     // set to burn
short porchsz = 5;      // set to burn
 
/*
 * EEPROM system factory settings and factory Preset burner for Emprenda R4C pedals
 * 
 * This program is the same as the preset burner except empties ALL user presets.  A completely empty preset uses 36 bytes, all zero
 * 
 * 
 * High-level structure: 
 * 0-99 - system block
 * 100-279 - preset 0
 * 280-449 - preset 1
 * 460-639 - preset 2
 * 640-819 - preset 3
 * 820-999 - preset 4
 * 1000-1023 - reserved
 * 
 * bytes 5-70: factory preset - 524 bits, or 66 bytes 
 * presets  = 180 bytes fully loaded ** 5 = 900 bytes + 4 = 86 bytes remaining in eeprom inc. a full live preset.
 * This is also easier and more efficient to implement preset and everything else addressability: preset Addres = sizeof(systemBlock) + (preset# x 180)
 * the remaining eeprom space is used to store downloaded user-defindd presets
 * 
 * 
 * 
 * This is the reference implementation of the system block.  
 * This burns the eeproms on outgoing pedals. 
 * In the event of a conflict, this reference is the correct one.
 * Alter as required to make work.  Maybe make a copy first.
 * 
 */
#include <EEPROM.h>
#include <systemblock.h> // Effigy-specific configuration for the device
#include <pedal.h>
// slot column name mappings
const static int MSP_CMD_COL = 0;
const static int MSP_CHANNEL_COL = 1;
const static int MSP_SUBCMD_COL = 2;
const static int MSP_CURVETYPE_COL = 3;
const static int MSP_CURVEDIR_COL = 4;
const static int MSP_MIN_COL = 5;
const static int MSP_MAX_COL = 6;
const static int MSP_LATCHING = 7;

// midi command types - note on/off, etc. including cc and pitch
int static const MIDI_NOTEOFF = 0x08; // not implemented
int static const MIDI_NOTEON = 0x09; // not implemented
int static const MIDI_CHG = 0x0B;            // midi change command - no channel
int static const MIDI_PKP = 0x0C;            // not implemented
int static const MIDI_AFTERTOUCH = 0x0D;     // aftertouch
int static const MIDI_PITCHBEND = 0x0E;      // midi pitch bend
int static const MIDI_SYSEX = 0x0F;          // sysex maybe
int static const MIDI_SUSTAIN = 0x40;        // midi sustain channel 1, dec 64
int static const MIDI_MOD_CMD = 0x01;        // midi MOD wheel control - CC1
int static const MIDI_BREATH_CMD = 0x02;     // midi BREATH control - CC2
int static const MIDI_XPRS_CMD = 0x0B;       // midi EXPRESSION control - CC7
int static const MIDI_BRIGHTNESS_CMD = 0x4A;       // midi EXPRESSION control - CC7
int static const MIDI_SUSTAINON_CMD = 0x7F;  // midi value for sustain on (>64)
int static const MIDI_SUSTAINOFF_CMD = 0x00; // midi value for sustain off (<64)

// Position Array
const static int numberOfInputs = 4; // set the number of inputs on the device, Positions, knobs, and mode switches, etc.
/*

// Effigy Control Pedal system block definition
// (C) 2018, Effigy Labs LLC
typedef struct systemblock { // currently 100 bytes
  int serialNumber: 16; // 2 bytes dddd of R4Cddddd little-endian style, i.e. serial number 9 would be 09 00
  int knobMaxValue: 16 ; // max knob value calibrated at factory - 0-1024 for a 1M potentiometer e.g. B1M
  // and 0~500 for a 100K e.g. B100K pots.  any linear pot works in the R4Cs.
  int knobCurrentValue: 16 ; // saved knob value as saved by last flip to save sensitivity
  int bootmode: 2; // 0=mode select, 1-3 = autoselect mode 1-3
  int presetToLoad: 3; // 0=factory preset, 1-5 = selects user-defined preset
  boolean knobControl: 1;  // toggle between knob as sensitivity control and as a position
  byte fademax: 8; // led fade max (brightness) (%)
  byte porchsize: 8; // stored/altered porch size
  byte ceilsize: 8; // stored/altered porch size
  // adding new attributes should be bitsize-specified
  // and remove the field size from the pool of remaining reserved bits
  int byteborder: 2; // makes as of r4c_15 10 bytes, including the 2 pad bits here
  byte factoryreset: 288; // 36 bytes for the factory preset 0
  byte reserved: 432; // reserved for factory reset, and also a cool version of the A4 note

  // possibly a calibration array of the original factory caliibration at nominal tension strap - to compare against over time and, maybe, compensate??
};


*/
// 1st four bytes of eeprom are reserved for system settings, leaving 1020 bytes for presets.

struct SysExSlotAddress {
  int preset:3;
  int mode:2;
  int pos:2;
  int slot:2;
};
//SysExSlotAddress slotaddr;

struct Slot {
  unsigned int MidiCmd : 4;
  unsigned int MidiChannel : 4;
  unsigned int MidiSubCommand: 7;
  unsigned int curvetype: 2;
  unsigned int curvedir: 1;
  unsigned int minrange: 7;
  unsigned int maxrange: 7;
  unsigned int latching: 1;
};
//Slot slotdata;

struct Position {
  Slot slot[3];
};

struct Mode {
  Position pos[4];
};
struct Preset {
  Mode mode[3];
};

    Preset factoryPreset;
    Preset emptyPreset;

 
void setup() {
    Serial.begin(9600);
    pinMode(commLedPin,OUTPUT);
    pinMode(redLedPin,OUTPUT);
    digitalWrite(redLedPin,HIGH);
    digitalWrite(commLedPin,LOW);
    delay(2000);
    Serial.println("Effigy Labs R4C EEPROM set factory settings and reset all uesr presets");    
    Serial.println("");
    Serial.println("this will reset preset 0 to the factory default, and all user-defined presets will be deleted.");
    Serial.println("Continue only with this knowledge.  Otherwise, pull the plug now!");
    Serial.println("");
    delay(1000);    
    Serial.println("calibrating knob...");
    Serial.println("Ensure pedal is idling.");
    Serial.println("Ensure knob is all the way to the left.");
    for(int timer1=5;timer1>0;timer1--) {
      Serial.print(timer1);
      Serial.print("...");
      for (int second=0;second<1000;second++) {
        delay(1);
      }
    }
    delay(2000);
    Serial.println("writing system block...");
    systemblock SystemBlock;
    SystemBlock.bootmode = 0; // do select mode
    Serial.print("bootmode = ");
    Serial.println(SystemBlock.bootmode);
    
    SystemBlock.presetToLoad = 0; // live preset
    Serial.print("presetToLoad = ");
    Serial.println(SystemBlock.presetToLoad);

    SystemBlock.knobControl = 0; // knob is sensitivity control mode, 1=controller mode
    Serial.print("knobControl = ");
    Serial.println(SystemBlock.knobControl);
    SystemBlock.knobMaxValue = calibrateKnob(1000);
    SystemBlock.knobCurrentValue = SystemBlock.knobMaxValue;//left position
    Serial.print("knobMaxValue = ");
    Serial.println(SystemBlock.knobMaxValue);
    Serial.print("knobCurrentValue = ");
    Serial.println(SystemBlock.knobCurrentValue);
    SystemBlock.porchsize = porchsz; // 0 is very jittery and sloppy on open and full gates.  too high and you have a bigger and bigger gap of motion before activation of minimum gate. Factory setting of 5 should take care of all hardware jitter.
    Serial.print("porchsize = ");
    Serial.println(SystemBlock.porchsize);
    SystemBlock.ceilsize = ceilsz; // 0 is very jittery and sloppy on open and full gates.  too high and you have a bigger and bigger gap of motion before activation of minimum gate. Factory setting of 100 should take care of all hardware jitter (was 5 but that caused "stiffness" on the top, hard to go the last bit to full gate, especially when operating two positions simultaneously.
    Serial.print("ceilsize = ");
    Serial.println(SystemBlock.ceilsize);
    SystemBlock.fademax = (byte) 16; // min 10 max 255
    Serial.print("fademax = ");
    Serial.println(SystemBlock.fademax);
    
    
      SystemBlock.serialNumber = serialnum; 
    Serial.print("serialNumber = ");
    Serial.println(SystemBlock.serialNumber);
    
    
    
    int SystemBlockAddr = 0;
    EEPROM.put(SystemBlockAddr,SystemBlock);
    Serial.println("wrote System Block.");
    Serial.print("system block size=");
    Serial.println(sizeof(SystemBlock));
    delay(500);
    Serial.print("initializing factory preset at addr ");
  
    int presetBlockAddr = sizeof(systemblock); // preset block starts at byte 4
  // mode 1, position 1, slot 1
    Serial.println(presetBlockAddr);    

    // mode 1
    Serial.print("mode 1");
    factoryPreset.mode[0].pos[0].slot[0].MidiCmd = MIDI_PITCHBEND;
    factoryPreset.mode[0].pos[0].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[0].pos[0].slot[0].MidiSubCommand = 0; // 0 = pitch down
    factoryPreset.mode[0].pos[0].slot[0].curvetype = 0; // 0 = linear
    factoryPreset.mode[0].pos[0].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[0].pos[0].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[0].pos[0].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[0].pos[0].slot[0].latching = 0; // value latching behaviour, 1 = latched
    Serial.print(".");
    factoryPreset.mode[0].pos[0].slot[1].MidiCmd = 0x0; // empty
    Serial.print(".");
    factoryPreset.mode[0].pos[0].slot[2].MidiCmd = 0x0; // empty
    Serial.print("pos1.");

    factoryPreset.mode[0].pos[1].slot[0].MidiCmd = MIDI_PITCHBEND;
    factoryPreset.mode[0].pos[1].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[0].pos[1].slot[0].MidiSubCommand = 0x7F; // > 0 = pitch up
    factoryPreset.mode[0].pos[1].slot[0].curvetype = 0; // 0 = linear
    factoryPreset.mode[0].pos[1].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[0].pos[1].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[0].pos[1].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[0].pos[1].slot[0].latching = 0; // value latching behaviour, 1 = latched
    Serial.print(".");

    factoryPreset.mode[0].pos[1].slot[1].MidiCmd = 0x0; // empty
    Serial.print(".");
    factoryPreset.mode[0].pos[1].slot[2].MidiCmd = 0x0; // empty
    Serial.print("pos2.");

    factoryPreset.mode[0].pos[2].slot[0].MidiCmd = MIDI_CHG;
    factoryPreset.mode[0].pos[2].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[0].pos[2].slot[0].MidiSubCommand = MIDI_SUSTAIN; // > 0 = pitch up
    factoryPreset.mode[0].pos[2].slot[0].curvetype = 0x3; // 0 = linear, 1=audio, 2=log, 3=latched
    factoryPreset.mode[0].pos[2].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[0].pos[2].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[0].pos[2].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[0].pos[2].slot[0].latching = 1; // value latching behaviour, 1 = latched
    Serial.print(".");

    factoryPreset.mode[0].pos[2].slot[1].MidiCmd = 0; // empty
    Serial.print(".");
    factoryPreset.mode[0].pos[2].slot[2].MidiCmd = 0; // empty
    Serial.print("pos3.");

    factoryPreset.mode[0].pos[3].slot[0].MidiCmd = MIDI_CHG;
    factoryPreset.mode[0].pos[3].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[0].pos[3].slot[0].MidiSubCommand = 0x0B; // post exprs ctl e.g. vol, kinda
    factoryPreset.mode[0].pos[3].slot[0].curvetype = 0x0; // 0 = linear, 1=audio, 2=log, 3=latched
    factoryPreset.mode[0].pos[3].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[0].pos[3].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[0].pos[3].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[0].pos[3].slot[0].latching = 0; // value latching behaviour, 1 = latched

    Serial.print(".");
    factoryPreset.mode[0].pos[3].slot[1].MidiCmd = 0; // empty
    Serial.print(".");
    factoryPreset.mode[0].pos[3].slot[2].MidiCmd = 0; // empty
    Serial.println("pos4.");

    // mode 2
    Serial.print("mode 2");
    factoryPreset.mode[1].pos[0].slot[0].MidiCmd = MIDI_PITCHBEND;
    factoryPreset.mode[1].pos[0].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[1].pos[0].slot[0].MidiSubCommand = 0; // 0 = pitch down
    factoryPreset.mode[1].pos[0].slot[0].curvetype = 0; // 0 = linear
    factoryPreset.mode[1].pos[0].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[1].pos[0].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[1].pos[0].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[1].pos[0].slot[0].latching = 0; // value latching behaviour, 1 = latched
    Serial.print(".");

    factoryPreset.mode[1].pos[0].slot[1].MidiCmd = 0x0; // empty
    Serial.print(".");
    factoryPreset.mode[1].pos[0].slot[2].MidiCmd = 0x0; // empty
    Serial.print("pos1.");

    factoryPreset.mode[1].pos[1].slot[0].MidiCmd = MIDI_PITCHBEND;
    factoryPreset.mode[1].pos[1].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[1].pos[1].slot[0].MidiSubCommand = 0x7F; // > 0 = pitch up
    factoryPreset.mode[1].pos[1].slot[0].curvetype = 0; // 0 = linear
    factoryPreset.mode[1].pos[1].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[1].pos[1].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[1].pos[1].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[1].pos[1].slot[0].latching = 0; // value latching behaviour, 1 = latched
    Serial.print(".");

    factoryPreset.mode[1].pos[1].slot[1].MidiCmd = 0x0; // empty
    Serial.print(".");
    factoryPreset.mode[1].pos[1].slot[2].MidiCmd = 0x0; // empty
    Serial.print("pos2.");

    factoryPreset.mode[1].pos[2].slot[0].MidiCmd = MIDI_CHG;
    factoryPreset.mode[1].pos[2].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[1].pos[2].slot[0].MidiSubCommand = MIDI_MOD_CMD; //
    factoryPreset.mode[1].pos[2].slot[0].curvetype = 0; // 0 = linear, 1=audio, 2=log, 3=latched
    factoryPreset.mode[1].pos[2].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[1].pos[2].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[1].pos[2].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[1].pos[2].slot[0].latching = 0; // value latching behaviour, 1 = latched
    Serial.print(".");

    factoryPreset.mode[1].pos[2].slot[1].MidiCmd = 0x0; // empty
    Serial.print(".");
    factoryPreset.mode[1].pos[2].slot[2].MidiCmd = 0x0; // empty
    Serial.print("pos3.");

    factoryPreset.mode[1].pos[3].slot[0].MidiCmd = MIDI_CHG;
    factoryPreset.mode[1].pos[3].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[1].pos[3].slot[0].MidiSubCommand = 0x0B; // post exprs ctl e.g. vol, kinda
    factoryPreset.mode[1].pos[3].slot[0].curvetype = 0x0; // 0 = linear, 1=audio, 2=log, 3=latched
    factoryPreset.mode[1].pos[3].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[1].pos[3].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[1].pos[3].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[1].pos[3].slot[0].latching = 0; // value latching behaviour, 1 = latched
    Serial.print(".");
    factoryPreset.mode[1].pos[3].slot[1].MidiCmd = 0x0; // empty
    Serial.print(".");
    factoryPreset.mode[1].pos[3].slot[2].MidiCmd = 0x0; // empty
    Serial.println("pos4.");


    // mode 3
    Serial.print("mode 3");
    factoryPreset.mode[2].pos[0].slot[0].MidiCmd = MIDI_PITCHBEND;
    factoryPreset.mode[2].pos[0].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[2].pos[0].slot[0].MidiSubCommand = 0;  // pitch down
    factoryPreset.mode[2].pos[0].slot[0].curvetype = 0; // 0 = linear
    factoryPreset.mode[2].pos[0].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[2].pos[0].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[2].pos[0].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[2].pos[0].slot[0].latching = 0; // value latching behaviour, 1 = latched

    Serial.print(".");
    factoryPreset.mode[2].pos[0].slot[1].MidiCmd = 0x0; // empty
    Serial.print(".");
    factoryPreset.mode[2].pos[0].slot[2].MidiCmd = 0x0; // empty
    Serial.print("pos1.");

    factoryPreset.mode[2].pos[1].slot[0].MidiCmd = MIDI_PITCHBEND;
    factoryPreset.mode[2].pos[1].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[2].pos[1].slot[0].MidiSubCommand = 0x7F; // > 0 = pitch up
    factoryPreset.mode[2].pos[1].slot[0].curvetype = 0; // 0 = linear
    factoryPreset.mode[2].pos[1].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[2].pos[1].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[2].pos[1].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[2].pos[1].slot[0].latching = 0; // value latching behaviour, 1 = latched
    Serial.print(".");

    factoryPreset.mode[2].pos[1].slot[1].MidiCmd = 0; // empty
    Serial.print(".");
    factoryPreset.mode[2].pos[1].slot[2].MidiCmd = 0; // empty
    Serial.print("pos2.");

    factoryPreset.mode[2].pos[2].slot[0].MidiCmd = MIDI_CHG;
    factoryPreset.mode[2].pos[2].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[2].pos[2].slot[0].MidiSubCommand = 0x4A; // CC74
    factoryPreset.mode[2].pos[2].slot[0].curvetype = 0; // 0 = linear, 1=audio, 2=log, 3=latched
    factoryPreset.mode[2].pos[2].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[2].pos[2].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[2].pos[2].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[2].pos[2].slot[0].latching = 0; // value latching behaviour, 1 = latched
    Serial.print(".");

    factoryPreset.mode[2].pos[2].slot[1].MidiCmd = 0x0; // empty
    Serial.print(".");
    factoryPreset.mode[2].pos[2].slot[2].MidiCmd = 0x0; // empty
    Serial.print("pos3.");
 
    factoryPreset.mode[2].pos[3].slot[0].MidiCmd = MIDI_CHG;
    factoryPreset.mode[2].pos[3].slot[0].MidiChannel = 0; // channel 1
    factoryPreset.mode[2].pos[3].slot[0].MidiSubCommand = 0x0B; // post exprs ctl e.g. vol, kinda
    factoryPreset.mode[2].pos[3].slot[0].curvetype = 0x0; // 0 = linear, 1=audio, 2=log, 3=latched
    factoryPreset.mode[2].pos[3].slot[0].curvedir = 0; // 0 = neg, 1 = pos, when curve type > 0 (nonlinear)
    factoryPreset.mode[2].pos[3].slot[0].minrange = 0; // input lower range
    factoryPreset.mode[2].pos[3].slot[0].maxrange = 100; // input upper range
    factoryPreset.mode[2].pos[3].slot[0].latching = 0; // value latching behaviour, 1 = latched
    Serial.print(".");
    factoryPreset.mode[2].pos[3].slot[1].MidiCmd = 0x0; // empty
    Serial.print(".");
    factoryPreset.mode[2].pos[3].slot[2].MidiCmd = 0x0; // empty
    Serial.println("pos4.");

    EEPROM.put(presetBlockAddr,factoryPreset);

    // make an empty preset
    for(int moder=0;moder<3;moder++) {
      for(int poser=0;poser<4;poser++) {
        for(int slotter=0;slotter<3;slotter++) {
            emptyPreset.mode[moder].pos[poser].slot[slotter].MidiCmd = 0x0;          
        }
      }
    }

     // write the empty preset to the user-defined slots (so there is a slot there in case)
     int pn = 0; 
     for(int preaddr=sizeof(systemblock)+sizeof(factoryPreset);preaddr<1000;preaddr+=sizeof(factoryPreset)) {
      Serial.print("initializing preset at addr ");
      Serial.println(preaddr);
      EEPROM.put(preaddr,factoryPreset); // write the factory preset to this preset#
      //EEPROM.put(preaddr,emptyPreset);
      Serial.print("wrote factory preset ");
      Serial.println(pn);
      pn++;
     }


    Serial.print("preset size=");
    Serial.println(sizeof(factoryPreset));

    Serial.println("EEPROM factory settings and all presets successfully burned.");
    Serial.println("done.");
    digitalWrite(commLedPin,HIGH);
    digitalWrite(redLedPin,LOW);
}

void loop() {
  // don't do anything here
}


int calibrateKnob(int numberOfSamples) {
  int returnval = 930; // best guess just in case;
  int knobPin = A0;
  // these pins are as of Emprenda #3x 17 Nov 2017 J Roberts

// mode switch button
const static int modeSwitchPin = 6;   // A6  pulled up, atmega32u4 R4a+ boards

  // calibration of knob is turning all the way to the left and taking some samples.  Blink when ready and indicate when done, for usefulness.
  // read the inputs for numberOfSamples times, with the emitters first on, then off
  //   determine the lower limit and the noise level
  //   max and min value, average value - for now
  //   max value + portch size becomes the lower limit.

  long inputAvg[3]; // 0 = counter, then avg, 1 = max, 2 = min
  
  // initialize accumulator array
    inputAvg[0] = 0; // initialize counter
    inputAvg[1] = 0; // max
    inputAvg[2] = 1024;  // min
    analogRead(knobPin);  // flushing read
    analogRead(knobPin);  // good read
    //modeSwitchPin = 6;   // A6  pulled up, atmega32u4 R4a+ boards
 
  long tempVal = 0;
  //timecounter = millis(); // initialize timer for blink
  Serial.println("calculating knobMaxValue, sampling knob...");
  // turn the red led on 
  pinMode(commLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  //pinMode(dimLedPin, OUTPUT);
//  digitalWrite(pedalLedPin, HIGH);  // turn on emitters - give time to 'warm up'?
  digitalWrite(redLedPin, HIGH);  // turn on red led lead
  //digitalWrite(commLedPin, HIGH);  // turn on red led lead
  
  // read sensors and update counters and bounds
  for (int q = 0; q < numberOfSamples; q++) {
        digitalWrite(redLedPin, LOW);  // turn off red led lead
        analogRead(knobPin);  // flushing read
        tempVal = analogRead(knobPin);  // good read
        inputAvg[1] = max(tempVal, inputAvg[1]); // check max
        inputAvg[2] = min(tempVal, inputAvg[2]); // check min
        inputAvg[0] += tempVal; // accumulate average read
        digitalWrite(redLedPin, HIGH);  // turn on red led lead
  }
  
  Serial.println("calculating knobMaxValue, analyzing...");
  // set sensitivity and lower limits
    // calculate average positions (idle value)
      inputAvg[0] = (long) (inputAvg[0] / numberOfSamples);
      //Serial.print("knob max value = ");
      //Serial.println(inputAvg[1]);
       digitalWrite(commLedPin, HIGH);  // turn on blue led lead
       digitalWrite(redLedPin, LOW);  // turn on red led lead

      returnval = inputAvg[1];
      return returnval;      
}


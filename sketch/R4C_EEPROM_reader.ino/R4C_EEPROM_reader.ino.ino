/*
 * verify EEPROM system factory settings and factory Preset burn for Emprenda R4C pedals
 */
#include <EEPROM.h>
#include <systemblock.h> // Effigy Specific configuration
#include <pedal.h>  // R4C pin maps and stuff

// midi command types - note on/off, etc. including cc and pitch
short static const MIDI_NOTEOFF = 0x08; // not implemented
short static const MIDI_NOTEON = 0x09; // not implemented
short static const MIDI_CHG = 0x0B;            // midi change command - no channel
short static const MIDI_PKP = 0x0C;            // not implemented
short static const MIDI_AFTERTOUCH = 0x0D;     // aftertouch
short static const MIDI_PITCHBEND = 0x0E;      // midi pitch bend
short static const MIDI_SYSEX = 0x0F;          // sysex maybe
short static const MIDI_SUSTAIN = 0x40;        // midi sustain channel 1, dec 64
short static const MIDI_MOD_CMD = 0x01;        // midi MOD wheel control - CC1
short static const MIDI_BREATH_CMD = 0x02;     // midi BREATH control - CC2
short static const MIDI_XPRS_CMD = 0x0B;       // midi EXPRESSION control - CC7
short static const MIDI_BRIGHTNESS_CMD = 0x4A;       // midi EXPRESSION control - CC7
short static const MIDI_SUSTAINON_CMD = 0x7F;  // midi value for sustain on (>64)
short static const MIDI_SUSTAINOFF_CMD = 0x00; // midi value for sustain off (<64)

// Position Array
const static short numberOfInputs = 4; // set the number of inputs on the device, Positions, knobs, and mode switches, etc.

short systemBlockAddr = 0;
// 1st four bytes of eeprom are reserved for system settings, leaving 1020 bytes for presets.

struct SysExSlotAddress {
  short preset:3;
  short mode:2;
  short pos:2;
  short slot:2;
};
//SysExSlotAddress slotaddr;

struct Slot {
  unsigned short MidiCmd : 4;
  unsigned short MidiChannel : 4;
  unsigned short MidiSubCommand: 7;
  unsigned short curvetype: 2;
  unsigned short curvedir: 1;
  unsigned short minrange: 7;
  unsigned short maxrange: 7;
  unsigned short latching: 1;
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

    Preset bufferPreset;
    Preset emptyPreset;
 
void setup() {
    Serial.begin(9600);
    while(!Serial) {
      ; // wait for serial port to initialize
    }
    Serial.println("Effigy Labs R4C EEPROM reader");    

    pinMode(commLedPin, OUTPUT);
    pinMode(redLedPin, OUTPUT);

    digitalWrite(redLedPin, HIGH);   // turn the LED on (HIGH is the voltage level)

    Serial.println("reading system block...");
    systemblock systemBlock;
    systemBlockAddr = 0; // preset block starts at byte 4
    EEPROM.get(systemBlockAddr,systemBlock);

    Serial.print("serialNumber="); 
    Serial.println(systemBlock.serialNumber); 

    Serial.print("bootmode=");
    Serial.println(systemBlock.bootmode);
    Serial.print("presetToLoad="); 
    Serial.println(systemBlock.presetToLoad);
    Serial.print("knobControl="); 
    Serial.println(systemBlock.knobControl); 
    Serial.print("knobMaxValue=");
    Serial.println(systemBlock.knobMaxValue); 
    Serial.print("knobCurrentValue=");
    Serial.println(systemBlock.knobCurrentValue); 
    Serial.print("porchsize="); 
    Serial.println(systemBlock.porchsize); 
    Serial.print("ceilsize="); 
    Serial.println(systemBlock.ceilsize); 
    Serial.print("fademax="); 
    Serial.println(systemBlock.fademax); 
    Serial.print("system block size=");
    Serial.println(sizeof(systemBlock));
    Serial.println();

    Serial.println("Presets");

 for(short preset=0;preset<5;preset++) { 
    Serial.print("Preset ");
    Serial.println(preset);

    // load preset into bufferPreset
    EEPROM.get(sizeof(systemBlock)+(preset * sizeof(bufferPreset)),bufferPreset);

    
    for(short moder=0;moder<3;moder++) {
      Serial.print("\tMode  ");
      Serial.println(moder);
      for(short poser=0;poser<numberOfInputs;poser++) {
        Serial.print("\t\tPos  ");
        Serial.println(poser);
        for(short slotter=0;slotter<3;slotter++) {
          Serial.print("\t\t\tSlot  ");
          Serial.print(slotter);
            short MidiCmd = bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd;
            if(MidiCmd == 0) {
              Serial.println("\tempty");                      
            }
            else {
              Serial.print("\tMidiCmd=");
              Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd);
              Serial.print("\t\t\t\tMidiChannel=");
              Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].MidiChannel);
              Serial.print("\t\t\t\tMidiSubCommand=");
              Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].MidiSubCommand);
              Serial.print("\t\t\t\tcurvetype=");
              Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].curvetype);
              Serial.print("\t\t\t\tcurvedir=");
              Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].curvedir);
              Serial.print("\t\t\t\tmin=");
              Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].minrange);
              Serial.print("\t\t\t\tmax=");
              Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].maxrange);
              Serial.print("\t\t\t\tlatching=");
              Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].latching);
            }
        } // slot
      } // position 
    } // mode
 } // preset

    Serial.println("done.");
    digitalWrite(commLedPin, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(redLedPin, HIGH);   // turn the LED on (HIGH is the voltage level)

}

void loop() {
  // don't do anything here
}



/* Control Program for the Effigy Labs Control Pedal
    vR4C.15
    9 July 2018
    author: Jody Roberts
    All rights reserved, Effigy Labs LLC
    
    This program is licensed under the FOSS (Free Open Source Software) license (https://opensource.org/faq#commercial)
    For more information, visit Effigy's web site at https://www.EffigyLabs.com
    Please feel free to report bugs and suggestions to Effigy Labs.

   Program can be written as if the hardware were an Arduino Leonardo:
    ATMEGA32U4 MCU
    28672 bytes of program storage space
    2560 bytes of dynamic memory
    1K EEPROM,
    16Mhz external clock
   Effigy-mapped hardware pins to sensors, emitters, and output in effigy.h
   eeprom map in pedal.h
   
   Program high-level structure:
 	 
 	 global variables
   
   setup()
 		initialize
 		calibrate
 		load preset
 		select mode
 	loop()
   
    read inputs
    generate outputs
    updates/actions
    
   subroutines:
     midi output
     LED handling,
     timer control,
     calibration,
     api implementation,
     input handling,
     dejitter,
     mode switch depression detection and timing

  change history, possible bugs, caveats:

     not well-tested:
        off-latching for slots.  So the value must be zero (or "open gate") prior to "latching" onto the effect.
     So when you select with the knob a knob function, the position of the selection function does not immediately
     become the selection of the effect, so you don't get a big blaring volume if your volume is using knobb position 6.
     You will zero it out, once the knob sees zero, it will then respond with the effect output.
    not well-tested: min-max limits
    not well-tested: curve type and direction.  Only curve type 4 "off-on" is used for sustain for now.  
    low pri bug - fadeonsensitivity seems to leave out at a high blink rate and not return
    
    constrain midval fed to fadeonposition - so the fade doesn't go above full or lower than open gate - fixed? - masked by the Warble Bug (opp OTA HC dep > porch sz)
    
    R4C_15 changes:
     - implement api code 17 to send system block
     - implement api code 18 to send bank (sends all 5 presets)
     - support auto-sync attribute, ensure timing and performance of consecutive commands are handled gracefully
     - allow midi handling during pickPosition aka mode select, so if pedal goes into mode select on boot, it will still respond to the API
     - send software version indicator bytes (00 0F)
     - use shorts instead of ints to save space
     - implement min-max on mapped value calculations-
*/
// uncomment to turn on console output
//#define DEBUG

#include <EEPROM.h>       // eeprom
#include "MIDIUSB.h"      // midi
#include <systemblock.h>  // Effigy Pedal-specific persistent configuration - includes map struct of system block
#include <pedal.h>        // Effigy hardware pin mappings and R4c-specific constants
//#include <JeeLib.h>	  // smoothing methods TBIL

// mode switch button timers and counters
unsigned long pedalLedCounterTimer; // Hz timer, currently turned off
boolean msdep = false;  // Mode Switch Depressed.  This is turned on as long as the mode switch is depressed.  Counters and timers key off of this condition.
unsigned long mstimer = 0;
unsigned long secondcounter = 0;
int ticker = 0;
unsigned long tickertimer = 0;
unsigned long loopctr = 0;

// LED blink parameters
long pickPositionblinkinterval = 100;     // indicator of ota picker cycle
short pickPositionconfirmblinkspeed = 500;  // 500 = slow time = 1/2 second, fast is is very fast like 40 (1/40)

// default calibration sample count, global for api to ask for recalibration with a given # of samples
int numberOfSamples = 1000;

// LED phasing and dimming
short hbval = 1;
int8_t hbdelta = 8;
short fadeSpeedMax = 8;
short fadeSpeedMin = 1;
short fadespeed = 1;            // vary from 10 to 100 - when centered, cycle according to sensitivity.  when pedal is operating, vary according to OTA position.
unsigned long cyclestart;     // led fader timer
short delayint = 60;
short absolutefademax = 32;     // fademax can't rise above this
short fademax = 32;             // maximum brightness to output default - overridden by system block
short fademin = 1;              // minimum brightness to output

short preset_number = 0;        // live mode, if this is 0 this is what the pedal plays from.  will be loaded from eeprom or sent from USB.
short preset_sens_knob_value = 0;
short preset_master_MIDI_channel = 0;
const static short deviceKnobMin = 0;
short maxLowerLimit = 0;
short minLowerLimit = 0;
short minUpperLimit = 0;
short dwr = 0; // dynamic max working range for all otas, based on lowest upper and largest lower limits for all otas.

float pickPosition_select_threshold = .5;  // single press on/off threshold.  this is the percent of the initial sampled value above, not the percent of working range depressed.  maybe that should change...

boolean setOnce = false; // for condition of setting sensitivity from eeprom if knob controller enabled (otherwise sensitivity is obtained from knob)

// porch and ceiling, configurable size buffers on the ends of the working range
// bigger value to porchSize sets the size of the smallest difference between the average (idle) of an OTA and it's lower limit trigger.
// min size of non-activating motion added to max idle measurement
short porchsize = 5; // for now, here, but eeprom overrides
// ceiling = amount of buffer to give to the upper maximum value of operation.  the minimum max sensor value is the upper limit use to set dynamic working range (dwr).  This number is subtracted from the max.  Use zero for nothing, and more to increase the ceiling buffer size.
short ceiling = 100; // default, this number is to avoid too much stiffness at the top

// Position Array and hardware constants
const static short numberOfInputs = 4; // set the number of position inputs on the device, OTA and knobs, but not mode switches.
const static short numberOfCols = 6;   // remember to cardinally count zero
short Inputs[numberOfInputs + 1][numberOfCols + 1]; // second number is the number of columns or global variables we have at this point in time
const static short numberOfSlots = 3; // R4C design is for three  slots per position.  The UI and other things are hard coded around this.  DO NOT CHANGE THIS.
const static short numberOfModes = 3; // Factory hard coded DO NOT CHANGE
const static short knobInput = 3;

// consts for the columns in the inputs table
const static short PIN_COL = 0;         // Pin mapping to atmega328 analog pins
const static short TYPE_COL = 1;        // 0 = Position, 1 = knob, 2 = momentary sw
const static short VALUE_COL = 2;       // current value measured
const static short LASTVAL_COL = 3;     // last value measured
const static short LL_COL = 4;          // lower limit input value
const static short UL_COL = 5;          // upper limit
const static short PORCH_COL = 6;       // porch - added to ll to prevent warble and other feel considerations
const static short CEIL_COL = 7;        // ceil - added to up to remove stiffness (excessive pressure to full gate)
short lastVals[numberOfInputs + 1][numberOfSlots]; // array to remember last values to avoid duplicates per slot

int last_master_val_sent = 65535; // set a guaranteed unique value for the first value sent from any ota, then remember to filter duplicate values by any ota

// output slot is identified by its cardinality in the array?
unsigned long timecounter = 0;    // comm LED Phaser

// presets, modes, spots, and slots
short preset = 0; // preset 0 = live mode
short mode = 0; // mode 1=1, 2=2, 3=3 do not count mode 0 as a mode, 0=mode unset

// MIDI section, codes, defaults, command constants, etc.
//int CURRENT_MIDI_CHANNEL = 1;
midiEventPacket_t rx; //  for midiusb library, defines a packet to receive

// midi command types - note on/off, etc. including cc and pitch
//int static const MIDI_NOTEOFF = 0x08; // not implemented
//int static const MIDI_NOTEON = 0x09; // not implemented
int static const MIDI_CHG = 0x0B;            // midi change command - no channel
//int static const MIDI_PKP = 0x0C;            // not implemented
int static const MIDI_AFTERTOUCH = 0x0D;     // aftertouch
int static const MIDI_PITCHBEND = 0x0E;      // midi pitch bend
int static const MIDI_SYSEX = 0x0F;          // sysex maybe

// system configuration
systemblock systemBlock;  // 1st 100 bytes of eeprom are reserved for system settings, leaving 924 bytes for presets.

// Effigy API Slot Addressability in a SysEx format - Effigy Vendor ID is 00h 02h 21h
short bufAddr = 0;
short blockCount = 0;
bool ignore = false;
byte sysexBuffer[186]; // incoming buffer to receive a preset or smaller items.

struct slotAddress {
  short preset: 3;
  short mode: 2;
  short pos: 2;
  short slot: 2;
};
//slotAddress slotaddr;

// slot is bitmapped.
// bit fields are truncated and will return incorrect values unless the maximum value is known and respected.
// For example, if curvetype is 2 bits and responds to bit positions 0 and 1.
//     If bit position 2 (4) is set, thinking 4 is greater than 11b and will round up with a limit, this is incorrect.
//     only bits 1 and will be used and stored in the value so caller/handler is responsible.
struct Slot {
  unsigned short MidiCmd : 4;
  unsigned short MidiChannel : 4;
  unsigned short MidiSubCommand: 7;
  unsigned short curvetype: 2;
  unsigned short curvedir: 1;
  unsigned short minrange: 7;
  unsigned short maxrange: 7;
  unsigned short latching: 1;
  unsigned short latched: 1;
};

Slot slotdata;

struct Position {
  Slot slot[numberOfSlots];
};

struct Mode {
  Position pos[numberOfInputs];
};

struct Preset {
  Mode mode[numberOfModes];
};

// define the live preset containing the current operating configuration.
Preset livePreset;
short presetSize;
bool full_message_received = false;
short apimidicmd; // the API call number sent in the SysEx message representing which API function is being called.

void setup() {
  Serial.begin(9600); // do not alter this even though your console might say 115200 bps
  Serial1.begin(31250);  // MIDI DIN-5 port
  pinMode(pedalLedPin, OUTPUT);
  pinMode(commLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  //pinMode(dimLedPin, OUTPUT);
  digitalWrite(pedalLedPin, HIGH);  // turn on emitters - give time to 'warm up'?
  analogWrite(redLedPin, HIGH);  // turn on red led lead
    
  delay(2000); // time for emitters and sensors to level

  // Effigy Control Pedal R4C input device array
  // input pins - only the Positions should be type 0 as they are looked at together to determine consistent working range.  knob or mode switch should not be used to calibrate Position working ranges.
  Inputs[0][PIN_COL] = pos1Pin; // left Position
  Inputs[0][TYPE_COL] = 0; // analog input

  Inputs[1][PIN_COL] = pos2Pin; // right Position
  Inputs[1][TYPE_COL] = 0; // analog input

  Inputs[2][PIN_COL] = pos3Pin; // center Position
  Inputs[2][TYPE_COL] = 0; // analog input

  Inputs[knobInput][PIN_COL] = knobPin; // knob
  Inputs[knobInput][TYPE_COL] = 1; // analog input

  Inputs[4][PIN_COL] = modeSwitchPin; // mode switch
  Inputs[4][TYPE_COL] = 2; // digital momentary switch
  // pinMode(modeswitch, INPUT);

  for (short i = 0; i < numberOfInputs; i++) {
    Inputs[i][LL_COL] = 0;
    Inputs[i][UL_COL] = 0;
    Inputs[i][VALUE_COL] = 0;
    Inputs[i][LASTVAL_COL] = 0;
    // initialize last value/dupe avoidance columns
    for (short s = 0; s < 3; s++) {
      lastVals[i][s] = 0;
    }
  }

  // load system settings from EEPROM
  EEPROM.get(0, systemBlock);
#ifdef DEBUG
  Serial.print("Effigy Labs Control Pedal ");
  Serial.print("v");
  Serial.println(ver);
#endif

  fademax = systemBlock.fademax;
#ifdef DEBUG
  //Serial.print("fMax=");
  //Serial.println(fademax);
#endif

#ifdef DEBUG
  Serial.print("kC=");
  Serial.println(systemBlock.knobControl);
#endif
  presetSize = sizeof(livePreset);

  // retrieve kob max (factory calibrated)
  Inputs[knobInput][LL_COL] = 0;
  Inputs[knobInput][UL_COL] = systemBlock.knobMaxValue;
  //Serial.print("k ll=");
  //Serial.print(Inputs[knobInput][LL_COL]);
#ifdef DEBUG
  Serial.print(" ul=");
  Serial.println(Inputs[knobInput][UL_COL]);
#endif
  // load the live preset.
  // presets are stored in EEPROM starting at byte 100 and are 180 bytes long.  The preset number x the size, starting at byte 100, gives the address of that preset's data
  EEPROM.get(sizeof(systemBlock) + (sizeof(livePreset) * (systemBlock.presetToLoad)), livePreset);

  porchsize = systemBlock.porchsize; // set saved porch size
  ceiling = systemBlock.ceilsize; // set saved ceiling size

  calibrateOTAs();  // set porch size and lower limits

  if (systemBlock.knobControl == 1) { // if knob is controller, populate sensitivity values from saved values

    for ( short k = 0; k < numberOfInputs; k++) { // hard coding just OTA sensitivity controlling now - knob doesn't control it's own sensitivity
      // adjust sensitivity only for OTAs, not the knob
      if (Inputs[k][TYPE_COL] == 0) { // it's an OTA so change the upper limit, also skip self as knob
        short newUL = Inputs[k][LL_COL] + constrain(map(systemBlock.knobCurrentValue, deviceKnobMin, systemBlock.knobMaxValue, 0, dwr), 1, dwr);
        if (!(Inputs[k][UL_COL] == newUL)) {
          Inputs[k][UL_COL] = newUL; // adjust upper limit for each ota to be the same minimum range but starting from each ota's individual lower limit.
        }
      }
    } // set new uls
    fadeOnSensitivity(systemBlock.knobCurrentValue); // so what if we do this three times, we only do it when a value has changed.
    setOnce = true;
  }

  pedalLedCounterTimer = millis(); // initialize timer
  secondcounter = millis();

  // mode select goes last
  if (systemBlock.bootmode == 0) {
    selectMode();
  } else {
    mode = systemBlock.bootmode; // nothing else except the mode is needed to address the correct mode in the preset.  No data or processing need occur.
#ifdef DEBUG
    Serial.print("btmd:");
    Serial.println(mode);
#endif
}

  // initialize the rest of the timers here also
}

// main loop repeats forever or until reset or power off
void loop() {
  //countloop(); // determine sample rate
  readInputs();  // update all the current and last values
  generateOutputs(); // output signals to the ports
  updateAll(); // timers, interrupts, api work, etc.
}

void readInputs() {
  for (short i = 0; i < numberOfInputs; i++) {
    Inputs[i][LASTVAL_COL] = Inputs[i][VALUE_COL];
    analogRead(Inputs[i][PIN_COL]); // flushing read
    if (Inputs[i][TYPE_COL] == 1) delay(1); // delay for knob to cooldown internal ADC, otherwise the value is coupled to OTA3
    Inputs[i][VALUE_COL] = analogRead(Inputs[i][PIN_COL]); // good read }
  }
}

void generateOutputs() {
  /*
      the idea is that an idling value is constrained down to a zero open gate value, and is sent
      then the last-value duplicate just never sends anotherr open gate-constrained value.
      so there should be no extra action required to go to both full and open gate naturally
      using a properly constraining map function.  The highest full gate value should always be sent when going to full gate.
  */
  //Serial.println("generateOutputs()");
  short modeIndex = mode - 1; // mode 1 is indexed by array index [0] and so on...

  for (short pos = 0; pos < numberOfInputs; pos++) {
    /*
       Process all input positions.
       Knob handling:  systemBlock.knobControl determines whether knob is handled as sensitivity control or as position 4.
    */
    if ((Inputs[pos][TYPE_COL] == 1) && (systemBlock.knobControl == 0)) {// knob role is as a sensitivity control
      // the knob value changes the working range of the other inputs (of type 0) dynamically:
      boolean willChgSens = false;
      for ( short k = 0; k < numberOfInputs; k++) { // hard coding just OTA sensitivity controlling now - knob doesn't control it's own sensitivity
        if (Inputs[k][TYPE_COL] == 0) { // it's an OTA so change the upper limit, also skip self as knob
          short newUL = Inputs[k][LL_COL] + constrain(map(Inputs[pos][VALUE_COL], deviceKnobMin, systemBlock.knobMaxValue, 0, dwr), 1, dwr);
          if ( abs(Inputs[k][UL_COL] - newUL) > 1  ) {
            /*
              Serial.print("old=");
              Serial.print(Inputs[k][UL_COL]);
              Serial.print(",new=");
              Serial.println(newUL);
            */
            Inputs[k][UL_COL] = newUL; // adjust upper limit for each ota to be the same minimum range but starting from each ota's individual lower limit.
            willChgSens = true;
          }
        }
      } // set new uls
      if (willChgSens) {
        fadeOnSensitivity(Inputs[pos][VALUE_COL]); // so what if we do this three times, we only do it when a value has changed.
        //Serial.print("XX");
      }
    }

    // if the position is an ota, and knob control is 1, then set the sensitivity from the systemblock
    //  done in calibration but also when knob is reflipped
    if (!setOnce) {
      if ((Inputs[pos][TYPE_COL] == 0) && (systemBlock.knobControl == 1)) {// knob role is as a controller so handle sensitivity for the other OTAs differently
        for ( short k = 0; k < numberOfInputs; k++) { // hard coding just OTA sensitivity controlling now - knob doesn't control it's own sensitivity
          short newUL = Inputs[k][LL_COL] + constrain(map(systemBlock.knobCurrentValue, deviceKnobMin, systemBlock.knobMaxValue, 0, dwr), 1, dwr);
          if ((!(Inputs[k][UL_COL] == newUL)) && (Inputs[k][TYPE_COL] == 0)  ) { // still only vary sensitivity for OTAs, not the knob itself ever
            Inputs[k][UL_COL] = newUL; // adjust upper limit for each ota to be the same minimum range but starting from each ota's individual lower limit.
            fadeOnSensitivity(Inputs[pos][VALUE_COL]); // so what if we do this three times, we only do it when a value has changed.
            setOnce = true;
          }
        } // set new uls
      }
    }

    // if this is an ota or if it's the knob as an ota, then Set the sensitivity for it if necessary
    if (
      (Inputs[pos][TYPE_COL] == 0) ||
      (
        (Inputs[pos][TYPE_COL] == 1) && (!(systemBlock.knobControl == 0))
      ) // if knob and knob is in controller mode
    ) {

      //  flip the value if our controller is the knob
      if (pos == 3) {
        //   Serial.print("knob orig val=");
        //   Serial.println(Inputs[pos][VALUE_COL]);
        Inputs[pos][VALUE_COL] = constrain(systemBlock.knobMaxValue - Inputs[pos][VALUE_COL], 0, systemBlock.knobMaxValue);
      }
      // }

      // go through each slot and generate possible output
      for (short slot = 0; slot < 3; slot++) {
        //  if(pos == 3) Serial.println("processing knob slots!");

        // handle off latching
        short latchlock = 0;
        if (livePreset.mode[modeIndex].pos[pos].slot[slot].latching == 1) {
          if (livePreset.mode[modeIndex].pos[pos].slot[slot].latched == 0) {
            // see if we can latch, if we are at zero
            if (Inputs[pos][VALUE_COL] <= Inputs[pos][LL_COL]) {
              livePreset.mode[modeIndex].pos[pos].slot[slot].latched = 1; // if a preset is loaded again it should zero-latch again so don'tt preserve this in the eeprom
            } else { // we are not latched and will not latch now
              latchlock = 1;
            }
          }
          else {
            // allow operation to continue normally
          }
        }

        // handle min and max, these are expressed as a percentage of the full input range.  this may change if we want a 'whole' or 'smooth' operation just in the otuptu range.
        // however, the latter may just create more space on the porch and not feel good.  we'll see how this goes.....02/20/2018 - jr ******
        // - may need to move this past midval generation to ensure open/full gate always sent
        short sval = Inputs[pos][VALUE_COL];

        short minpct = constrain(livePreset.mode[modeIndex].pos[pos].slot[slot].minrange, 0, 100);
        short maxpct = constrain(livePreset.mode[modeIndex].pos[pos].slot[slot].maxrange, 0, 100);

        short wr = Inputs [pos][UL_COL] - Inputs[pos][LL_COL]; // calculate working range
        short tval = ((sval - Inputs[pos][LL_COL]) / wr) * 100; // calculate % value of intput range

        // all the reasons not to continue processing go here, else the command is allowed to process and output
        if ( (livePreset.mode[modeIndex].pos[pos].slot[slot].MidiCmd == 0) || // slot is empty
             (tval < minpct ) ||  // min lock
             (tval > maxpct ) ||  // max lock
             (latchlock == 1)     // latch lock
           ) {
          // do nothing rather than process the value
        } else {
          // process and output the value

          short midval = 0; // main var holding output value.  this is the value that will be sent to the midi outputter.

          // calculate the output value.  not the midi word, but the raw logical value to be sent via midi.
          switch (livePreset.mode[modeIndex].pos[pos].slot[slot].MidiCmd) {
            case 0: {
                // provide for doing nothing if slot is empty
              }
              break;

            case  MIDI_PITCHBEND: {
                if (livePreset.mode[modeIndex].pos[pos].slot[slot].MidiSubCommand == 0) {  // 0 = pitch down
                  // apply min max pct in place of the defaults ***** tag **** min = 2000 max = 0, so the first number would be minpct * (min - max), and 2nd# would be maxpct * (max - min)) 
                  midval = constrain( map(Inputs[pos][VALUE_COL],
                                          Inputs[pos][LL_COL], Inputs[pos][UL_COL],
                                          0x2000, 0x0000), // use min and max values here and the inversion problem gets solved ************** tag 
                                      0x0000, 0x2000);
                    
//                    // prototypical implementation of min/max that would support inversion
//                  midval = constrain( map(Inputs[pos][VALUE_COL],
//                                          Inputs[pos][LL_COL], Inputs[pos][UL_COL],
//                                          0x2000 * minpct, 0x0000 * maxpct), // use min and max values here and the inversion problem gets solved ************** tag 
//                                      0x0000, 0x2000);
                  break;
                } // pitch down
                else {
                  if (livePreset.mode[modeIndex].pos[pos].slot[slot].MidiSubCommand > 0) { // > 0 = pitch up
                  // opposite of "apply min max pct in place of the defaults ***** tag **** min = 2000 max = 0, so the first number would be minpct * (min - max), and 2nd# would be maxpct * (max - min)) "
                    midval = constrain(map(Inputs[pos][VALUE_COL],
                                           Inputs[pos][LL_COL], Inputs[pos][UL_COL],
                                           0x2000, 0x3FFF),
                                       0x2000, 0x3FFF);
//                    midval = constrain(map(Inputs[pos][VALUE_COL],
//                                           Inputs[pos][LL_COL], Inputs[pos][UL_COL],
//                                           0x2000, 0x3FFF),
//                                       0x2000 * minpct, 0x3FFF * maxpct);
                    break;
                  }
                }
                break; // if the break is here, then the output starts oscillating!!!!
              }

            default: { // pretty much everything besides pitch bend is a 0-127 type output.  I said pretty much in case something else comes along...
                midval = constrain(map(Inputs[pos][VALUE_COL], Inputs[pos][LL_COL], Inputs[pos][UL_COL], 0, 127), 0, 127);
//                midval = constrain(map(Inputs[pos][VALUE_COL], Inputs[pos][LL_COL], Inputs[pos][UL_COL], 0 * minpct, 127 * maxpct), 0, 127);
              }
          } // switch

          // apply curve type and curve dir - only latching curve is implemented, need to implement curve functions as of R4c.15
          switch (livePreset.mode[modeIndex].pos[pos].slot[slot].curvetype) {
            case 0: { // linear
              // default
              }
              break;
            case 1: { // logarithmic
#ifdef DEBUG
Serial.println("log crv not yet implemented");
#endif
              // not yet supported
              // curved value = linear value (curve function)
              // curve function = linear value which is the distance along the range
              // positive adds more and more to the midpoint, and then adds less and less
              // negative curve subtracts more until the center then subtracts less
              // the curve type determines the amount added or subtracted from the ends to the midpoint
              
              }
              break;
            case 2: { // audio ?  exponential ?
              // not yet supported 
#ifdef DEBUG
Serial.println("log crv not yet implemented");
#endif
              }
              break;
            case 3: { // on-off latching, anything past open gate is full gate
                if (!(livePreset.mode[modeIndex].pos[pos].slot[slot].MidiCmd == MIDI_PITCHBEND)) { // for 0-127 type commands only, further filter later for sysex etc. ***********
                  if (midval > 0) { // let it go back to zero unimpeded
                    midval = 127; // such as sustain
                    //Serial.print("latching value to 127");
                  }
 
                }
              }
              break;
            default: {
              }
          }

          //////////////////////////////////////////////////////////
          // possibly do whatever other processing/hook/exit here  //
          //////////////////////////////////////////////////////////


          /////////////////////////////////////////////////////////////////////////////////////////////////////
          // Doing nothing past this point without an understanding of what is happening is the wise choice. //
          /////////////////////////////////////////////////////////////////////////////////////////////////////

          // generate an output value from the input value
          if ( !(midval == lastVals[pos][slot]) ) { // filter duplicate output values at the slot level
            lastVals[pos][slot] = midval;
            if (!(midval == last_master_val_sent)) { // filter at the master level so no matter who sent the value it doesn't get sent again
              last_master_val_sent = midval;

              // indicate to debug port.  This output will work on either the Serial monitor or plotter in the arduino IDE
#ifdef DEBUG
              Serial.print(pos);
              Serial.print("/");
              Serial.print(slot);
              Serial.print("\t");
              Serial.print(Inputs[pos][VALUE_COL]);
              Serial.print("\t");
              Serial.println(midval);
#endif

              lastVals[pos][slot] = midval;

              sendMidiOut(
                livePreset.mode[modeIndex].pos[pos].slot[slot].MidiCmd, // midi cmd byte, B=CC, E=pitch, D=aftertouch, etc.
                livePreset.mode[modeIndex].pos[pos].slot[slot].MidiChannel, // midi channel, 0 = channel 1
                livePreset.mode[modeIndex].pos[pos].slot[slot].MidiSubCommand, // pitch direction, 00=down, FF = up, or, CC command subtype, e.g. MOD, sustain, expression, etc.
                midval); // output value

              fadeOnPosition(Inputs[pos][VALUE_COL], Inputs[pos][LL_COL], Inputs[pos][UL_COL]);
            }
          }
        }
        // channel, command, curve type, curve direction, min, max
      } // slot
    }
  }
  // generate the right output, midi and other, and set the comm LED states.
}

void report() {
#ifdef DEBUG
  Serial.println("I\tTP\tV\tLV\tLL\tUL");

  for (short i = 0; i < numberOfInputs; i++) {
    for (short c = 0; c < numberOfCols; c++) {

      Serial.print(Inputs[i][c]);
      Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println("v:\tsn:\tpSz:\tcSz:\tdwr:\tcal:\tdKm:");
  Serial.print(ver);
  Serial.print("\t");
  Serial.print(systemBlock.serialNumber);
  Serial.print("\t");
  Serial.print(porchsize);
  Serial.print("\t");
  Serial.print(ceiling);
  Serial.print("\t");
  Serial.print(dwr);
  Serial.print("\t");
  Serial.print(numberOfSamples);
  Serial.print("\t");
  Serial.println(systemBlock.knobMaxValue);
#endif
}

// handle all timers, API calls, etc.
void updateAll() {

  // listen to mode switch, handle all timers triggered by mode switch there
  handleModeSwitch();

  // comm led timer
  heartbeat();

  // handle receiving midi e.g. api commands and maybe other stuff
  handleMidiTraffic();
}

void rpt(short blockCount, short bufAddr) {
#ifdef DEBUG
  Serial.print(blockCount);
  Serial.print(" blks,sz=");
  Serial.println(bufAddr);
#endif
}


void handleMidiTraffic() {
  // header codes from usb.org:
  /* 0 - function codes (reserved) */
  /* 1 - cable events (reserved) */
  /* 2 - two-byte system common message */
  /* 3 - three-byte system common message */
  /* 4 - sysex starts or continues */
  /* 5 - sysex ends with one byte, or single-byte system common message */
  /* 6 - sysex ends with two bytes */
  /* 7 - sysex ends with three bytes */
  /* 8 - note-off */
  /* 9 - note-on */
  /* A - poly-keypress */
  /* B - control change */
  /* C - program change */
  /* D - channel pressure */
  /* E - pitch bend change */
  /* F - single byte */
  //

  midiEventPacket_t recpac = MidiUSB.read();
  //  if ((recpac.header >3) && (recpac.header < 8)) {  // we've noticed that a midi sysex message is in progress
  //if (recpac.header > 0) { // we've noticed that a midi sysex message is in progress
  if(recpac.header == 0) return;

    Serial.print("M:");
    Serial.print(recpac.header, HEX);
    Serial.print(":");
    Serial.print(recpac.byte1, HEX);
    Serial.print(":");
    Serial.print(recpac.byte2, HEX);
    Serial.print(":");
    Serial.println(recpac.byte3, HEX);


    switch (recpac.header) {

      case 4: { // start or middle of sysex message
          // we are either starting or continuing a sysex message
          switch (blockCount) {
            case 0:
              // process block 1
              // begin receiving, confirm sysex and get first 2 bytes of address
#ifdef DEBUG
              Serial.println("msg rcv");
#endif
              bufAddr = 0;
              ignore = false;
              full_message_received = false;
              addToSysexBuffer(recpac);

              break;

            case 1:

              // process block 2
              addToSysexBuffer(recpac);
              // must check for Effigy's MMMA vendor ID prior to receiving the entire message
              // because if it is not addressed to us, the message might be larger than our buffer
              // and we could overrun the buffer before we see a delimiter.

              // turn ignore on if this isn't a sysex message addressed to us
              if ( (!(sysexBuffer[0] == 0xF0)) ||
                   (!(sysexBuffer[1] == 0x00)) ||
                   (!(sysexBuffer[2] == 0x02)) ||
                   (!(sysexBuffer[3] == 0x21))
                 ) {
                ignore = true;
#ifdef DEBUG
                Serial.print("rcv err");
#endif
              }

              break;

            default:
              // process block 2-n
              if (!(ignore == true)) {
                addToSysexBuffer(recpac);
              }
          }
        } // sysex start or continue message
        break;


      case 5: {
          if (recpac.byte1 == 0xF7) {

            blockCount++;

            full_message_received = true;
            rpt(blockCount, bufAddr);
          }
        }
        break;

      case 6: {
          sysexBuffer[bufAddr++] = recpac.byte1;

          blockCount++;
          full_message_received = true;
          rpt(blockCount, bufAddr);
        }
        break;

      case 7: {
          sysexBuffer[bufAddr++] = recpac.byte1;
          sysexBuffer[bufAddr++] = recpac.byte2;
          blockCount++;
          full_message_received = true;
          rpt(blockCount, bufAddr);
        }
        break;

      default: {
          //Serial.print("*");
          // non-sysex unimplemented
        }
    } // switch header
  

  // gatekeepers to avoid putting everything else in yet another nested block
  if (!(full_message_received == true) ) return; // only go past here if the message is complete and addressed to us
    
  apimidicmd = sysexBuffer[4]; // fifth byte is the api cmd.  the first four bytes are the sysex header and our address

#ifdef DEBUG
  Serial.print("cmd=");
  Serial.println(apimidicmd, HEX);
#endif
  if (!(ignore == true)) apiDispatcher(apimidicmd);
  blockCount = 0;
  ignore = false;
  full_message_received = false;
  // process all api midi commands either directly or via subroutines.

} // end of handleMidiCmd


void apiDispatcher(short apicmd) {

  /*
     Effigy Labs control pedal API Cmd List

     0 - Select Pedal Preset
     1 - Set single value in a preset
     2 - Send preset to pedal
     3 - Get preset from pedal
     4 - Get slot from pedal
     5 - Send slot to pedal
     6 - Set mode in current preset
     7 - User Mode Switch
     8 - Set sensitivity % (knob)
     9 - Recalibrate sensors
     A - Recalibrate w/#samples
     B - Reboot immediately
     C - Change all live channels
     D - set Porch size
     E - set Boot mode
     F - set Ceiling size
     10 - set Fade max (led brightness max)

     host-side automation:
     11 - send Bank to pedal (send Preset x 5)

  */
  switch (apicmd) {

    case 0:  { // load from eeprom and switch to preset
#ifdef DEBUG
        Serial.print("ld pre:");
        Serial.println(sysexBuffer[5]);
#endif
        loadPreset(sysexBuffer[5]);
        full_message_received = true;
      }
      break;

    case 1:  { // receive single attribute value
#ifdef DEBUG
        Serial.print("set attr sa:");
        Serial.print(sysexBuffer[5]); // preset
        Serial.print(sysexBuffer[6]); // mode
        Serial.print(sysexBuffer[7]); // position
        Serial.print(sysexBuffer[8]); // slot
        Serial.print(":"); 
        Serial.print(sysexBuffer[9]); // attribute#
        Serial.print(":"); 
        Serial.println(sysexBuffer[10]); // attribute val
#endif
        Preset bufferPreset;
        EEPROM.get(sizeof(systemBlock) + (sizeof(livePreset) * sysexBuffer[5]), bufferPreset);
        switch (sysexBuffer[9]) {
          case 0:
              bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].MidiCmd = sysexBuffer[10];
            break;
          case 1:
            bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].MidiChannel = sysexBuffer[10];
            break;
          case 2:
            bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].MidiSubCommand = sysexBuffer[10];
            break;
          case 3:
            bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].curvetype = sysexBuffer[10];
            break;
          case 4:
            bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].curvedir = sysexBuffer[10];
            break;
          case 5:
            bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].minrange = sysexBuffer[10];
            break;
          case 6:
            bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].maxrange = sysexBuffer[10];
            break;
          case 7:
            bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].latching = sysexBuffer[10];
            break;
          default:
          ;
#ifdef DEBUG
            Serial.print("attr err:");
            Serial.println(sysexBuffer[9]);
#endif
        }
        // save preset back
        EEPROM.put(sizeof(systemBlock) + (sizeof(livePreset) * sysexBuffer[5]), bufferPreset);
        if (sysexBuffer[5] == 0) loadPreset(0); // load live preset if the change was to preset 0
        full_message_received = true;
      } // cmd=1
      break;

    case 2:  { // receive and save preset
#ifdef DEBUG
        Serial.print("rcv pre:");
#endif
        savePreset(sysexBuffer[5], sysexBuffer, 6); // offset = 4 points to the data and skips the final sysex header:  last byte of the mma vendor id, + the api cmd, + the preset # arg of the cmd
        if (sysexBuffer[5] == 0) loadPreset(0); // load live preset if changes were made
        full_message_received = true;
      }
      break;

    case 3: { // send requested preset

#ifdef DEBUG
        Serial.print("snd pre:"); // preset to send
        Serial.println(sysexBuffer[5]); // preset to send
#endif

        short prestart = sysexBuffer[5]; // which preset to send
        Preset bufferPreset; // for incoming or outgoing presets vi api
        EEPROM.get(sizeof(systemBlock) + (sizeof(livePreset)*prestart), bufferPreset);
        byte sendBuffer[sizeof(bufferPreset) + 2]; // put the api bytes in front of the preset bytes
        short butPtr = 0; // space for sysex header
        byte apiBytes[2] = {0x03, sysexBuffer[5]};

        // serialize the preset
        for (short moder = 0; moder < 3; moder++) {
          //Serial.print("m ");
          //Serial.println(moder);
          for (short poser = 0; poser < numberOfInputs; poser++) {
            //Serial.print("\tp ");
            //Serial.println(poser);
            for (short slotter = 0; slotter < 3; slotter++) {
              //Serial.print("\t\ts ");
              //Serial.print(slotter);
              sendBuffer[butPtr] = bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd;
              butPtr++;
              if (bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd > 0) {
                //Serial.print("\tc=");
                //Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd);
                sendBuffer[butPtr] = bufferPreset.mode[moder].pos[poser].slot[slotter].MidiChannel;
                sendBuffer[butPtr + 1] = bufferPreset.mode[moder].pos[poser].slot[slotter].MidiSubCommand;
                sendBuffer[butPtr + 2] = bufferPreset.mode[moder].pos[poser].slot[slotter].curvetype;
                sendBuffer[butPtr + 3] = bufferPreset.mode[moder].pos[poser].slot[slotter].curvedir;
                sendBuffer[butPtr + 4] = bufferPreset.mode[moder].pos[poser].slot[slotter].minrange;
                sendBuffer[butPtr + 5] = bufferPreset.mode[moder].pos[poser].slot[slotter].maxrange;
                sendBuffer[butPtr + 6] = bufferPreset.mode[moder].pos[poser].slot[slotter].latching;
                butPtr += 7;

              } // slot contents
              else {
                //Serial.println("\t-");
              }
            } // slot
          } // position
        } // mode
        // append sysex F7 to end of buffer here
        //delay(60);
        MidiUSB_sendSysex(sendBuffer, butPtr, apiBytes, sizeof(apiBytes));
        //MidiUSB.flush();
#ifdef DEBUG
        Serial.print("snt ");
        Serial.println(butPtr);
#endif
      }
      full_message_received = true;
      break;

    case 4:  { // send slot data
#ifdef DEBUG
        Serial.print("snd sl:");
#endif
        short preaddr = sizeof(systemBlock); //start of preset memory - where the first byte starts
        short prestart = sysexBuffer[5]; // which preset to send
        preaddr += (prestart * presetSize); // preset address
        Preset bufferPreset; // for incoming or outgoing presets vi api
        EEPROM.get(preaddr, bufferPreset);
#ifdef DEBUG
        Serial.print(sysexBuffer[5]); // preset of slot to send
        Serial.print(sysexBuffer[6]); // mode of slot to send
        Serial.print(sysexBuffer[7]); // position of slot to send
        Serial.print(sysexBuffer[8]); // slot number of slot to send
        Serial.print(":"); 
#endif
        byte apiBytes[5];
        apiBytes[0] = 0x04;
        apiBytes[1] = sysexBuffer[5]; // preset
        apiBytes[2] = sysexBuffer[6]; // mode
        apiBytes[3] = sysexBuffer[7]; // position
        apiBytes[4] = sysexBuffer[8];// slot
        // implement slot addressability
        short bufLen = 0;
        byte sendBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0} ; // sysex header + systex footer + slot address + slot data
        sendBuffer[bufLen] = bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].MidiCmd;
        bufLen++;
        if (bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].MidiCmd > 0) {
#ifdef DEBUG
          Serial.print(bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].MidiCmd);
          Serial.print(" ");
#endif
          sendBuffer[bufLen] = bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].MidiChannel;
#ifdef DEBUG
          Serial.print(bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].MidiChannel);
          Serial.print(" ");
#endif
          sendBuffer[bufLen + 1] = bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].MidiSubCommand;
#ifdef DEBUG
          Serial.print(bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].MidiSubCommand);
          Serial.print(" ");
#endif
          sendBuffer[bufLen + 2] = bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].curvetype;
#ifdef DEBUG
          Serial.print(bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].curvetype);
          Serial.print(" ");
#endif
          sendBuffer[bufLen + 3] = bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].curvedir;
#ifdef DEBUG
          Serial.print(bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].curvedir);
          Serial.print(" ");
#endif
          sendBuffer[bufLen + 4] = bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].minrange;
#ifdef DEBUG
          Serial.print(bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].minrange);
          Serial.print(" ");
#endif
          sendBuffer[bufLen + 5] = bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].maxrange;
#ifdef DEBUG
          Serial.print(bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].maxrange);
          Serial.print(" ");
#endif
          sendBuffer[bufLen + 6] = bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].latching;
#ifdef DEBUG
          Serial.println(bufferPreset.mode[sysexBuffer[6]].pos[sysexBuffer[7]].slot[sysexBuffer[8]].latching);
          Serial.print(" ");
#endif
          bufLen += 7;
        } else {
#ifdef DEBUG
          Serial.println("-");
#endif
        }
        MidiUSB_sendSysex(sendBuffer, bufLen, apiBytes, sizeof(apiBytes) );
#ifdef DEBUG
        Serial.print("snt ");
        Serial.println(bufLen);
#endif
      }
      full_message_received = true;
      break;

    case 5: { // receive and set slot data
        Preset bufferPreset; // for incoming or outgoing presets vi api
#ifdef DEBUG
        Serial.print("set sl,addr:");
#endif
        short c = 5;
        short presx = sysexBuffer[c]; // preset of slot
        short modex = sysexBuffer[c + 1]; // mode of slot
        short posex = sysexBuffer[c + 2]; // position of slot
        short slotx = sysexBuffer[c + 3]; // slot number
        c += 3;

        //          Serial.print("buf[5]=");
#ifdef DEBUG
        Serial.print(sysexBuffer[5]);
        Serial.print(sysexBuffer[6]);
        Serial.print(sysexBuffer[7]);
        Serial.print(sysexBuffer[8]);
        Serial.print(":");
        Serial.print(sysexBuffer[9]);
        Serial.print(":");
        Serial.println(sysexBuffer[10]);
        //         Serial.print(" buf[11]=");
        //    Serial.println(sysexBuffer[11]);
        //
#endif

        EEPROM.get(sizeof(systemBlock) + (presetSize * presx), bufferPreset); // load the preset
        c++;
        bufferPreset.mode[modex].pos[posex].slot[slotx].MidiCmd = sysexBuffer[c];
        if (bufferPreset.mode[modex].pos[posex].slot[slotx].MidiCmd > 0) {
#ifdef DEBUG
          Serial.print(bufferPreset.mode[modex].pos[posex].slot[slotx].MidiCmd);
          Serial.print(" ");
#endif
          c++;
          bufferPreset.mode[modex].pos[posex].slot[slotx].MidiChannel = sysexBuffer[c];
#ifdef DEBUG
          Serial.print(bufferPreset.mode[modex].pos[posex].slot[slotx].MidiChannel);
          Serial.print(" ");
#endif
          c++;
          bufferPreset.mode[modex].pos[posex].slot[slotx].MidiSubCommand = sysexBuffer[c];
#ifdef DEBUG
          Serial.print(bufferPreset.mode[modex].pos[posex].slot[slotx].MidiSubCommand);
          Serial.print(" ");
#endif
          c++;
          bufferPreset.mode[modex].pos[posex].slot[slotx].curvetype = sysexBuffer[c];
#ifdef DEBUG
          Serial.print(bufferPreset.mode[modex].pos[posex].slot[slotx].curvetype);
          Serial.print(" ");
#endif
          c++;
          bufferPreset.mode[modex].pos[posex].slot[slotx].curvedir = sysexBuffer[c];
#ifdef DEBUG
          Serial.print(bufferPreset.mode[modex].pos[posex].slot[slotx].curvedir);
          Serial.print(" ");
#endif
          c++;
          bufferPreset.mode[modex].pos[posex].slot[slotx].minrange = sysexBuffer[c];
          Serial.print(" ");
#ifdef DEBUG
          Serial.print(bufferPreset.mode[modex].pos[posex].slot[slotx].minrange);
          Serial.print(" ");
#endif
          c++;
          bufferPreset.mode[modex].pos[posex].slot[slotx].maxrange = sysexBuffer[c];
#ifdef DEBUG
          Serial.print(bufferPreset.mode[modex].pos[posex].slot[slotx].maxrange);
          Serial.print(" ");
#endif
          c++;
          bufferPreset.mode[modex].pos[posex].slot[slotx].latching = sysexBuffer[c];
#ifdef DEBUG
          Serial.println(bufferPreset.mode[modex].pos[posex].slot[slotx].latching);
          Serial.print(" ");
#endif
        } //else {
        //Serial.println("empty");
        //}

        EEPROM.put(sizeof(systemBlock) + (presetSize * presx), bufferPreset); // save the preset
        if (presx == 0) {
#ifdef DEBUG
          Serial.println("ld pre:0");
#endif
          loadPreset(0); // refresh live preset if preset 0
        }
#ifdef DEBUG
        //Serial.println("ok");
#endif
      }
      full_message_received = true;
      break;

    case 6: { // switch to mode - right now
        mode = sysexBuffer[5]; // for now
#ifdef DEBUG
        Serial.print("set md:");
        Serial.println(mode);
#endif
        full_message_received = true;
      }
      break;

    case 7: { // enter mode select - no parameters
#ifdef DEBUG
        Serial.println("sel md");
#endif
        mode = 99;
        selectMode();
        full_message_received = true;
      }
      break;

    case 8: { // set sensitivity
#ifdef DEBUG
        Serial.println("set sns%:");
        Serial.println(sysexBuffer[5]);
#endif
        for ( short k = 0; k < numberOfInputs; k++) { // hard coding just OTA sensitivity controlling now - knob doesn't control it's own sensitivity
          if (Inputs[k][TYPE_COL] == 0) { // it's an OTA so change the upper limit, also skip self as knob
            short newUL = Inputs[k][LL_COL] + constrain(map(sysexBuffer[5], 0, 100, 0, dwr), 1, dwr);
            if (!(Inputs[k][UL_COL] == newUL)) {
              Inputs[k][UL_COL] = newUL; // adjust upper limit for each ota to be the same minimum range but starting from each ota's individual lower limit.
            }
          }
        } // set new uls
        // set in eeprom?  **** TAG
        fadeOnSensitivity(sysexBuffer[5]); // so what if we do this three times, we only do it when a value has changed.
        full_message_received = true;
      }
      break;

    case 9: { // recalibrate
#ifdef DEBUG
        Serial.println("recal");
#endif
        calibrateOTAs(); // leave numberOfSamples the same as last time, restart the program to set to the default value
        full_message_received = true;
      }
      break;

    case 10: { // recalibrate with a certain # of samples
        numberOfSamples = sysexBuffer[5] * 100;
#ifdef DEBUG
        Serial.print("recal:");
        Serial.println(numberOfSamples);
#endif
        calibrateOTAs();
        full_message_received = true;
      }
      break;

    case 11: { //restart the program (but do not reboot the hardware chip itself)
#ifdef DEBUG
        Serial.println("rst");
        Serial.end(); // try to gracefully shut down 
#endif
        Serial1.end(); // try to gracefully shut down
        void (*softReset) (void) = 0; //declare reset function @ address 0
        softReset();
      }
      break; // pro forma break

    case 12: { // change all channels - this may be wrong *********
        short newchannel = sysexBuffer[5];
#ifdef DEBUG
        Serial.print("all chn:");
        Serial.println(newchannel);
#endif
        chgAllChannels(newchannel); // change all the channels short he live preset, do not write to eeprom.  can be used as an 'emergency' switch to put everything on a single channel if something is, say, corrupted at/during/right before a gig.
        full_message_received = true;
      }
      break;

    case 13: { // change porch size
        porchsize = sysexBuffer[5];
        systemBlock.porchsize = porchsize;
        EEPROM.put(0, systemBlock);
#ifdef DEBUG
        Serial.print("porch:");
        Serial.println(porchsize);
#endif
      }
      full_message_received = true;
      break;

    case 14: { // change boot mode
        short bm = sysexBuffer[5];
#ifdef DEBUG
        Serial.println("boot md:");
        Serial.println(bm);
#endif
        if (bm != systemBlock.bootmode) {
          systemBlock.bootmode = bm;
          EEPROM.put(0, systemBlock); // must test this **********
        }
      }
      full_message_received = true;
      break;

    case 15: { // change ceiling size
        ceiling = sysexBuffer[5];
        systemBlock.ceilsize = ceiling;
        EEPROM.put(0, systemBlock);
#ifdef DEBUG
        Serial.print("ceil:");
        Serial.println(ceiling);
#endif
        //calibrateOTAs(); // must recalibrate when changing?
      }
      full_message_received = true;
      break;

    case 16: { // change fademax - max brightness of LED

        fademax = map(sysexBuffer[5], 0, 100, fademin, absolutefademax); // fix this - unless fade min and absolute fade max are also percents, this doesn't work ***** tag
        systemBlock.fademax = constrain(fademax, fademin + 1, absolutefademax); // 
        fadeOnSensitivity(fademax); // so what if we do this three times, we only do it when a value has changed.
        //hbval = fademax; // so the oscillator won't lock up
        EEPROM.put(0, systemBlock);
#ifdef DEBUG
        Serial.print("brt%:");
        Serial.println(fademax);
#endif
      }
      full_message_received = true;
      break;

    case 17: { // send system block
        byte apiByte[1] = { 0x11 };
#ifdef DEBUG
        Serial.println("sndcfg");
#endif
        //byte sbb[sizeof(systemBlock)]; // put the api bytes in front of the preset bytes
//        Serial.print("sysblksz=");
//        Serial.println(sizeof(systemBlock));
        //byte sendBuffer[100]; // put the api bytes in front of the preset bytes

        // here, we should pack the data into twice as many words, each with a nibble of the data stream, and unpack it on the other end
        union bssb{
         systemblock sysblk;
         byte serz[sizeof(systemblock)];
        };        
        union bssb BS;
        BS.sysblk = systemBlock;
        short actualsysblocksize = 12; // used section of system block
        short xmitarray = actualsysblocksize * 2;
        byte xmtarr[xmitarray];
        for(short ir=0;ir<actualsysblocksize;ir++) {
          xmtarr[ir*2] = BS.serz[ir]  >> 4; // shift high nibble, hopefully the fill bits are zeroes I guess **** tag
          xmtarr[ir*2+1] = BS.serz[ir] & 0x0F; // low nibble
          /*
          Serial.print("ir=");
          Serial.print(ir);
          Serial.print(", sysblkbyte=");
          Serial.print(BS.serz[ir]);
          Serial.print(", highnib=");
          Serial.print(xmtarr[ir*2]);
          Serial.print(", lownib=");
          Serial.println(xmtarr[(ir*2)+1]);
        */
        }
        
        //loop and print out here *******************************************

/*
#ifdef DEBUG
        Serial.print("blksys sz=");
        Serial.print(sizeof(BS));
        Serial.print(", data=");
        Serial.print("serz sz=");
        Serial.print(sizeof(BS.serz));
        Serial.print(", ");
        for(int xt=0;xt<sizeof(BS);xt++) {
            Serial.print("BS[");
            Serial.print(xt);
            Serial.print("]='");
            Serial.print(BS.serz[xt]);
            Serial.println("'");
        }
#endif
        */

        MidiUSB_sendSysex(xmtarr, sizeof(xmtarr), apiByte, sizeof(apiByte) );

#ifdef DEBUG
    
        //Serial.println("snt " + String(sizeof(xmtarr)));
#endif
        full_message_received = true;
        break;
      }
   
    case 18: { // send bank (all 5 presets)

#ifdef DEBUG
        Serial.print("sndbnk");
#endif

        //Serial.println(sysexBuffer[5]); // preset to send

        byte apiBytes[1] = {0x12}; // api command to send the pedal bank, all 5 presets
        //int prestart = sysexBuffer[5]; // which preset to send
        Preset bufferPreset; // for incoming or outgoing presets vi api
        byte sendBuffer[sizeof(bufferPreset) + 1]; // put the api bytes in front of the preset bytes
        for (short bpr = 0; bpr < 5; bpr++) { // iterate through all presets
          EEPROM.get(sizeof(systemBlock) + (sizeof(livePreset)*bpr), bufferPreset); // retrieve preset from EEPROM

          short butPtr = 0; // space for sysex header

          // serialize the preset
          for (short moder = 0; moder < 3; moder++) {
            //Serial.print("m ");
            //Serial.println(moder);
            for (short poser = 0; poser < numberOfInputs; poser++) {
              //Serial.print("\tp ");
              //Serial.println(poser);
              for (short slotter = 0; slotter < 3; slotter++) {
                //Serial.print("\t\ts ");
                //Serial.print(slotter);
                sendBuffer[butPtr] = bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd;
                butPtr++;
                if (bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd > 0) {
                  //Serial.print("\tc=");
                  //Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd);
                  sendBuffer[butPtr] = bufferPreset.mode[moder].pos[poser].slot[slotter].MidiChannel;
                  sendBuffer[butPtr + 1] = bufferPreset.mode[moder].pos[poser].slot[slotter].MidiSubCommand;
                  sendBuffer[butPtr + 2] = bufferPreset.mode[moder].pos[poser].slot[slotter].curvetype;
                  sendBuffer[butPtr + 3] = bufferPreset.mode[moder].pos[poser].slot[slotter].curvedir;
                  sendBuffer[butPtr + 4] = bufferPreset.mode[moder].pos[poser].slot[slotter].minrange;
                  sendBuffer[butPtr + 5] = bufferPreset.mode[moder].pos[poser].slot[slotter].maxrange;
                  sendBuffer[butPtr + 6] = bufferPreset.mode[moder].pos[poser].slot[slotter].latching;
                  butPtr += 7;

                } // slot contents
                else {
                  //Serial.println("\t-");
                }
              } // slot
            } // position
          } // mode
          // append sysex F7 to end of buffer here
          //delay(60);
          MidiUSB_sendSysex(sendBuffer, butPtr, apiBytes, sizeof(apiBytes));
          //MidiUSB.flush();
#ifdef DEBUG
          Serial.print("snt ");
          Serial.println(butPtr);
#endif
        }
      }

      full_message_received = true;
      break;

    case 19: { // set preset to load
      //short ptl = sysexBuffer[5]; // location of arg 1
      systemBlock.presetToLoad = sysexBuffer[5];
      //hbval = fademax; // so the oscillator won't lock up
      EEPROM.put(0,systemBlock);
#ifdef DEBUG
      Serial.print("set PTL:");
      Serial.println(systemBlock.presetToLoad);
#endif
    }
      full_message_received = true;
    break;

    case 20: { // report SW version
#ifdef DEBUG
      Serial.println("R4C_15");
#endif
      byte swver[] = { 0x00, 0x0F }; // these shall be the bytes of the version, this version shall have these bytes' values. - Jody 061918
      byte apiBytes[1] = {0x14}; // 
      MidiUSB_sendSysex(swver, sizeof(swver), apiBytes, sizeof(apiBytes)); // the *2 is duplicatory of the process in prep(), sorry, saving space...
    }
      full_message_received = true;
    break;
    
    default: {
#ifdef DEBUG
        Serial.print("undef cmd ");
        Serial.println(sysexBuffer[5]);
#endif
        full_message_received = true;
      }

  }  // end of switch apimidicmd

} // apiDispatch


void savePreset(short presetToSave, byte presetData[], short offset) {
  //use offset to point to the preset data, past any sysex header, api data or arguments - caller will know
#ifdef DEBUG
  Serial.print("sv pre ");
  Serial.print(presetToSave);
  Serial.print(",o:");
  Serial.println(offset);
#endif
  Preset bufferPreset; // for incoming or outgoing presets vi api
  short preaddr = offset;
  for (short moder = 0; moder < 3; moder++) {
    for (short poser = 0; poser < numberOfInputs; poser++) {
      for (short slotter = 0; slotter < 3; slotter++) {
        bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd = presetData[preaddr];
        preaddr++;
        //Serial.print("preaddr=");
        //Serial.println(preaddr);
        if (bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd > 0) {
          //Serial.print("midicmd=");
          //Serial.println(bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd);
          bufferPreset.mode[moder].pos[poser].slot[slotter].MidiChannel = presetData[preaddr];
          bufferPreset.mode[moder].pos[poser].slot[slotter].MidiSubCommand = presetData[preaddr + 1];
          bufferPreset.mode[moder].pos[poser].slot[slotter].curvetype = presetData[preaddr + 2];
          bufferPreset.mode[moder].pos[poser].slot[slotter].curvedir = presetData[preaddr + 3];
          bufferPreset.mode[moder].pos[poser].slot[slotter].minrange = presetData[preaddr + 4];
          bufferPreset.mode[moder].pos[poser].slot[slotter].maxrange = presetData[preaddr + 5];
          bufferPreset.mode[moder].pos[poser].slot[slotter].latching = presetData[preaddr + 6];
          preaddr += 7;
        } // slot contents
      } // slot
    } // position
  } // mode
  // any sysex footer is naturally ignored
  EEPROM.put(sizeof(systemBlock) + (presetToSave * presetSize), bufferPreset);

  if (presetToSave == 0) {
    // copy this preset to the livePreset preset
    loadPreset(0);
  }
}


// copy one of the presets from eeprom to the livePreset
void loadPreset(short presetToLoad) {
#ifdef DEBUG
  Serial.print("ld pre:");
  Serial.println(presetToLoad);
#endif
  Preset bufferPreset; // for incoming or outgoing presets vi api

  EEPROM.get(sizeof(systemBlock) + (presetToLoad * presetSize), bufferPreset);
  for (short moder = 0; moder < numberOfModes; moder++) {
    for (short poser = 0; poser < numberOfInputs; poser++) {
      for (short slotter = 0; slotter < numberOfSlots; slotter++) {
        livePreset.mode[moder].pos[poser].slot[slotter].MidiCmd = bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd;
        if (bufferPreset.mode[moder].pos[poser].slot[slotter].MidiCmd > 0) {
          livePreset.mode[moder].pos[poser].slot[slotter].MidiChannel = bufferPreset.mode[moder].pos[poser].slot[slotter].MidiChannel;
          livePreset.mode[moder].pos[poser].slot[slotter].MidiSubCommand = bufferPreset.mode[moder].pos[poser].slot[slotter].MidiSubCommand;
          livePreset.mode[moder].pos[poser].slot[slotter].curvetype = bufferPreset.mode[moder].pos[poser].slot[slotter].curvetype;
          livePreset.mode[moder].pos[poser].slot[slotter].curvedir = bufferPreset.mode[moder].pos[poser].slot[slotter].curvedir;
          livePreset.mode[moder].pos[poser].slot[slotter].minrange = bufferPreset.mode[moder].pos[poser].slot[slotter].minrange;
          livePreset.mode[moder].pos[poser].slot[slotter].maxrange = bufferPreset.mode[moder].pos[poser].slot[slotter].maxrange;
          livePreset.mode[moder].pos[poser].slot[slotter].latching = bufferPreset.mode[moder].pos[poser].slot[slotter].latching;
        } // slot contents
        //Serial.println();
      } // slot
    } // position
  } // mode
}

/*
byte*  prepData(byte data[]) {
  //short orgsz = sizeof(data);
  byte result[sizeof(data) * 2];
  for(short ir=0;ir<sizeof(data);ir++) {
    result[ir*2] = data[ir]  >> 4; // shift high nibble, hopefully the fill bits are zeroes I guess **** tag
    result[ir*2+1] = data[ir] & 0x0F; // low nibble
  }
  return *result;
  //return result;
}
*/

void MidiUSB_sendSysex(const uint8_t *data, size_t size, byte apiBytes[], size_t apiBytesSize) {
  //Serial.print("MidiUSB_sendsysex");
  if (data == NULL || size == 0) return;
  
  unsigned short bc = 0;
  size_t totsize = size + apiBytesSize;
  byte buff[totsize + 1]; // account for delimiter (F7)

  // inject the api bytes - echo the command back to the response, usually
  for (bc = 0; bc < apiBytesSize; bc++) {
    buff[bc] = apiBytes[bc];
  }

  for (bc = 0; bc < size; bc++) {
    buff[apiBytesSize + bc] = data[bc];
  }
  const byte SYSEX_DELIMITER = 0xF7;
  const byte HXZ = 0x00;
  const uint8_t *d = buff;
  //uint8_t *p = midiData;
  size_t bufCtr = totsize + 1;

  // send the F0 sysex header packet,
  midiEventPacket_t packet;
  packet.header = 0x0F;
  packet.byte1 = 0xF0;
  packet.byte2 = HXZ; // is not included in the data!
  packet.byte3 = HXZ; // is not included in the data!
  MidiUSB.sendMIDI(packet);
  MidiUSB.flush();

  // Effigy MMA vendor ID
  packet.header = 0x04;
  packet.byte1 = HXZ; // MMA Vendor ID byte 1
  packet.byte2 = 0x02; // MMA Vendor ID byte 2
  packet.byte3 = 0x21; // MMA Vendor ID byte 3
  MidiUSB.sendMIDI(packet);
  MidiUSB.flush();

  while (bufCtr > 0) {
    delay(1);
    switch (bufCtr) {
      case 1:
        packet.header = 5;
        packet.byte1 = SYSEX_DELIMITER;
        packet.byte2 = HXZ;
        packet.byte3 = HXZ;
        bufCtr = 0;
        break;
      case 2:
        packet.header = 6;
        packet.byte1 = *d++;
        packet.byte2 = SYSEX_DELIMITER;
        packet.byte3 = HXZ;
        bufCtr = 0;
        break;
      case 3:
        packet.header = 7;
        packet.byte1 = *d++;
        packet.byte2 = *d++;
        packet.byte3 = SYSEX_DELIMITER;
        bufCtr = 0;
        break;
      default:
        packet.header = 4;
        packet.byte1 = *d++;
        packet.byte2 = *d++;
        packet.byte3 = *d++;
        bufCtr -= 3;
        break;
    }
    
            // uncomment out for debugging
#ifdef DEBUG  
            Serial.print("h:");
            Serial.print(packet.header,HEX);
            Serial.print("\t");
            Serial.print(packet.byte1,HEX);
            Serial.print("\t");
            Serial.print(packet.byte2,HEX);
            Serial.print("\t");
            Serial.println(packet.byte3,HEX);
#endif
    
    MidiUSB.sendMIDI(packet);
    MidiUSB.flush();
  }
}

// called by code that processes incoming sysex messages
void addToSysexBuffer(midiEventPacket_t block) {
  sysexBuffer[bufAddr] = block.byte1;
  sysexBuffer[bufAddr + 1] = block.byte2;
  sysexBuffer[bufAddr + 2] = block.byte3;
  bufAddr += 3;
  blockCount++;
#ifdef DEBUG
  Serial.print("+");
#endif

  
      Serial.print("blk ");
      Serial.print(blockCount);
      Serial.print("\t");
      Serial.print(block.byte1,HEX);
      Serial.print("\t");
      Serial.print(block.byte2,HEX);
      Serial.print("\t");
      Serial.println(block.byte3,HEX);
  
}

//}
//#endif

// calibrateOTAs establishes the upper and lower operating parameters of the OTA inputs.  the knob and mode switch are not calibrated here.
/*
  void calibrateKnob() {
  Inputs[knobInput][LL_COL] = 0;
  Inputs[knobInput][UL_COL] = systemBlock.knobMaxValue;
  //if(systemBlock.knobControl == 1) {
  //}
  Serial.print("k ll=");
  Serial.print(Inputs[knobInput][LL_COL]);
  Serial.print(" ul=");
  Serial.println(Inputs[knobInput][UL_COL]);
  }
*/
void calibrateOTAs() {

  // read the inputs for numberOfSamples times, with the emitters first on, then off
  //   determine the lower limit and the noise level
  //   max and min value, average value - for now
  //   max value + portch size becomes the lower limit.

  short r = 0;  // looper var
  long inputAvg[3][3]; // 0 = counter, then avg, 1 = max, 2 = min

  // initialize accumulator array
  for (r = 0; r < 3; r++) {  // there are only 3 inputs, the OTAs, of interest, the numberOfInputs variable doesn't work here so it's hard-coded for now
    inputAvg[r][0] = 0; // initialize counter
    inputAvg[r][1] = 0; // max
    inputAvg[r][2] = 1024;  // min
  }

  short tempVal = 0;
  //timecounter = millis(); // initialize timer for blink
  //Serial.println("averaging...");
  // read sensors and update counters and bounds
  
  for (short q = 0; q < numberOfSamples; q++) {
    for (r = 0; r < 3; r++) {
      if (Inputs[r][TYPE_COL] < 1) { // ensure only OTA inputs, not knob or momentary switches - *** chg 01/30/2018 iso full gate bug
        tempVal = analogRead(Inputs[r][PIN_COL]);  // good read
        analogRead(Inputs[r][PIN_COL]);  // flushing read
  /*      Serial.print("q=");
        Serial.print(q);
        Serial.print(",r=");
        Serial.print(r);
        Serial.print(", input=");
        Serial.println(tempVal);
 */
        inputAvg[r][1] = max(tempVal, inputAvg[r][1]); // check max
        inputAvg[r][2] = min(tempVal, inputAvg[r][2]); // check min
        inputAvg[r][0] += tempVal; // accumulate average read
      }
    }
  }

  //Serial.println("analyzing...");
  // set sensitivity and lower limits
  for (short r = 0; r < 3; r++) {
    // calculate average positions (idle value)
    if (Inputs[r][TYPE_COL] < 1) { // ensure only ota inputs, not knob or momentary switch *** chg 01/30/2018 iso full gate bug
      inputAvg[r][0] = (long) (inputAvg[r][0] / numberOfSamples);
      //Inputs[r][SENSITIVITY_COL] = (inputAvg[r][1] - inputAvg[r][2]) / 2 + porchsize; // sensitivity = (max - min)/2 + porchsize
/*
      Serial.print("r=");
      Serial.println(r);
      Serial.print("porchsize=");
      Serial.println(porchsize);
      Serial.print("inputAvg[r][1]=");
      Serial.println(inputAvg[r][1]);
*/
      Inputs[r][LL_COL] = inputAvg[r][1] + porchsize;
    }
  }

  // find the smallest and largest LL of all inputs
  //   Serial.println("finding highest lower limit...");
  // if the smallest ll is huge, then we have overtensioning or emitter failure
  maxLowerLimit = 0;
  minLowerLimit = 1024;
  for (r = 0; r < 3; r++) {
    if (Inputs[r][TYPE_COL] == 0) { // we only want Positions here, not knob or switch
      maxLowerLimit = max(Inputs[r][LL_COL], maxLowerLimit);
      minLowerLimit = min(Inputs[r][LL_COL], minLowerLimit);
    }
  }
  if (minLowerLimit > 500) {
    if (minLowerLimit > 1000) {
      // emitter failure
      while(true) {
#ifdef DEBUG
        Serial.println("emitter fail");
#endif
        blinkCommLED(250, 12, RED, true);
        delay(3000);
      }
    }
    else {
      // overtensioning warning
     while(true) {
#ifdef DEBUG
      Serial.println("tensioning err");
#endif
      //blinkLed(BOTH, 250, 6);
      blinkCommLED(250, 6, RED, true);
      delay(3000);
     }
    }
  }

  // calibrate with OTAs off to set the upper limits
  // we just sampled with emitters on so no need to take samples again with emitters on

  digitalWrite(pedalLedPin, LOW);  // turn off emitters
  delay(10); // wait for fall time

  // initialize accumulator array
  for (r = 0; r < 3; r++) {
    inputAvg[r][0] = 0;     // counter
    inputAvg[r][1] = 0;     // max
    inputAvg[r][2] = 1024;  // min

    analogRead(Inputs[r][PIN_COL]);  // flushing read
    analogRead(Inputs[r][PIN_COL]);  // good read
  }

  // collect samples
  for (r = 0; r < numberOfSamples; r++) {
    for (short c = 0; c < 3; c++) {
      if (Inputs[c][TYPE_COL] < 1) { // only OTA inputs, not momentary switches
        analogRead(Inputs[c][PIN_COL]);  // flushing read
        tempVal = analogRead(Inputs[c][PIN_COL]);  // good read
        inputAvg[c][1] = max(tempVal, inputAvg[c][1]); // check max
        inputAvg[c][2] = min(tempVal, inputAvg[c][2]); // check min
        inputAvg[c][0] += tempVal; // accumulate average read
      }
    }
  }

  //    Serial.println("analyzing...");
  // set sensitivity and lower limits
  minUpperLimit = 9999;
  for (short r = 0; r < 3; r++) {
    // calculate average positions (idle value)
    if (Inputs[r][TYPE_COL] == 0) { // we only want Positions here, not knob or switch
      inputAvg[r][0] = (long) (inputAvg[r][0] / numberOfSamples);
      minUpperLimit = min(inputAvg[r][2], minUpperLimit);
      //Serial.print("minUpperLimit=");
      //Serial.println(minUpperLimit);
    }
  }

  // find the lowest of all the upper limits of all inputs
  for (r = 0; r < numberOfInputs; r++) {
    if (Inputs[r][TYPE_COL] == 0) { // we only want Positions here, not knob or switch
      Inputs[r][UL_COL] =  Inputs[r][LL_COL] + (minUpperLimit - maxLowerLimit);
    }
  }

  dwr = (minUpperLimit - maxLowerLimit) - ceiling;
  digitalWrite(pedalLedPin, HIGH);  // turn on emitter leds

  // get knob calibration from eeprom
  for (r = 0; r < numberOfInputs; r++) {
    if (Inputs[r][TYPE_COL] == 1) { // find the knob
      Inputs[r][LL_COL] = deviceKnobMin; // just in case it changes from zero in the future
      Inputs[r][UL_COL] = systemBlock.knobMaxValue; // gotten from system block from eeprom
    }
  }
  report();
} // calibrateOTAs

void selectMode() {
#ifdef DEBUG
  Serial.print("md sel");
#endif
  if (mode == 99) {
    pickPositionconfirmblinkspeed = 50;
    mode = 0;
  }
  //short blinkstate = LOW;
  mode = pickPosition();
#ifdef DEBUG
  Serial.print("md:");
  Serial.println(mode);
  report();
#endif
  blinkCommLED(pickPositionconfirmblinkspeed, mode, BLUE, false);
}

short pickPosition() {
  short blinkstate = LOW;
  short P = 0;
  short s = 0;

  // get initial values
  short InitialValue[3];
  for (s = 0; s < 3; s++) { // only OTAs
    analogRead(Inputs[s][PIN_COL]); // flushing read
    InitialValue[s] = analogRead(Inputs[s][PIN_COL]); // good read
  }

  timecounter = millis(); // initialize timer for blink

  // wait for a position to be depressed
  do { // while P == 0
    for (s = 0; s < numberOfInputs; s++) { //loop through pedal inputs
      if (Inputs[s][TYPE_COL] == 0) { // only OTAs
        analogRead(Inputs[s][PIN_COL]); // flushing read
        short ov = analogRead(Inputs[s][PIN_COL]); // good read
        if ((ov > (InitialValue[s] + (InitialValue[s] * pickPosition_select_threshold)))) {  // check if a position has been depressed
          P = s + 1;  // a mode has been selected, loop counter + 1
        } // if a position has been pressed
      } // for OTAs only

    } // iterate through inputs

    // handle blinking
    if ((short) (millis() - timecounter) > pickPositionblinkinterval) {
#ifdef DEBUG
      Serial.print(".");
#endif
      //Serial.println(char(8));
      blinkstate = !blinkstate;
      analogWrite(redLedPin, blinkstate);   // blink
      analogWrite(commLedPin, blinkstate);   // which should be LOW at all times

      timecounter = millis(); // reset

      // handleMidiTraffic(); // don't completely do an updateAll() just be able to respond to midi commands if necessary while waiting.  be advised a recursive condition is possible to while handling this, enter into the api cmd of mode switch, which will then come back to mode switch but with another mode switch lower in the caller stack.  this may be bad, for example, creating an out of memory condition, or creating uncollectible garbage, or corruption.  caution is indicated.

    } // timer loop

      // like and updateAll but do not blink the LED (it's already blinking the way we want) and do not check the mode switch (we are already checking it)
      // this amounts to really just allowing MIDI commands to go to the pedal when it is in mode select, for example, when the unit is turned on.
      // unless we check here, the pedal will not respond to midi messages.  WRITE INTERRUPTS. *****
      handleMidiTraffic(); 
      

  } while (P == 0);
  analogWrite(redLedPin, LOW);
  return P;
} // pickPosition


// output to both the din-5 port and the USB
// caller is responsible for not sending duplicate values and bounding values, this function only sends what is given
void sendMidiOut(short cmd, short channel, short cmdSubType, short value) {
  midiEventPacket_t event;
  byte byte1 = (cmd << 4) | channel;

  switch (cmd) {
    case MIDI_CHG: {
        event = {(uint8_t)cmd, byte1, (uint8_t) cmdSubType, (uint8_t) value};  // subtype is pitch dir Or CC
        Serial1.write(byte1); // midi command type - MIDI_CHG, or something else
        Serial1.write(cmdSubType); // midi command: MOD wheel, breath, sustain, expression
        Serial1.write(constrain(value, 0, 127));
        break;
      }

    case MIDI_PITCHBEND: {
        uint8_t byte3 = value << 1;
        byte3 = byte3 >> 1;
        uint8_t byte2 = value >> 7;
        event = {(uint8_t) cmd, byte1, byte3, byte2};
        // write MIDI out the DIN-5 port
        Serial1.write(byte1);
        Serial1.write(byte3);
        Serial1.write(byte2);
        break;
      }
    case MIDI_AFTERTOUCH: {
        event = {(uint8_t) cmd, byte1, (uint8_t) value, 0x00};  // subtype is pitch dir Or CC
        Serial1.write(byte1); // midi command type - MIDI_CHG, or something else
        Serial1.write(constrain(value, 0, 127));
        break;
      }

    // for safety:  if it's not one of these, default to a 0-127 output.  all the others, program change, noteOn/Off etc. generally use 0-127
    default: {
        event = {(uint8_t) cmd, byte1, (uint8_t) value, 0x00};  // subtype is pitch dir Or CC
        Serial1.write(byte1); // midi command type - MIDI_CHG, or something else
        Serial1.write(constrain(value, 0, 127));

        break;
      }

  } // end of switch cmd

  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
} // sendmidiout


void handleModeSwitch() {
  analogRead(modeSwitchPin); // flushing read
  if (analogRead(modeSwitchPin) < 500) { // low state is something like 5 or 10 so for safety we say 500
    if (! msdep) { // switch is newly depressed
      mstimer = millis(); // start the timer
      msdep = true;  // indicate the switch is down
#ifdef DEBUG
      Serial.print("ms dn 0.");
#endif
      ticker = 0;
    }
    else {
      // switch was already depressed
      //msdep = true;
      if (millis() - tickertimer > 1000) {
        tickertimer = millis();
        ticker++; // second stopwatch
#ifdef DEBUG
        Serial.print(ticker);
        Serial.print(".");
#endif
        // blink led
        blinkCommLED(50, 1, RED, true);
      }
      // we're still counting, so do nothing for now
    }
  } else { // switch was but now is not depressed
    if (msdep == true) { // but switch was depressed before so now we have an event
      //Serial.print("switch released after ");
      //Serial.print(ticker);
#ifdef DEBUG
      Serial.println("s");
#endif

      // do stuff based on which timer was tripped here
      msdep = false;
      switch (ticker) {

        // zero to 1 second is mode switch
        case 0:
        case 1: {
            mode = 99;
            selectMode();
          }
          break;

        // 2-4 seconds
        case 2:
        case 3:
        case 4:  {
            // flip knob between sensitivity control and position 4
            if ( systemBlock.knobControl == 0) {
#ifdef DEBUG
              Serial.println("k->p4");
#endif
              systemBlock.knobControl = 1; // 1 = controller mode - this makes a -1 in the 1 bit field, so test always for either zero or not zero, do not test for 1
              systemBlock.knobCurrentValue = Inputs[3][VALUE_COL];
              setOnce = false; // tell sensitivity setter to use eeprom value while knob is also a controller
              // flip from knob to controller
              blinkCommLED(10, 1, RED, true);
            } else {
              // flip from controller to knob
#ifdef DEBUG
              Serial.println("p4->k");
#endif
              systemBlock.knobControl = 0; // 0=sensitivity control
              //systemBlock.knobbCurrentValue =
              blinkCommLED(10, 1, RED, true);
            }
            EEPROM.put(0, systemBlock);
          }
          break;

        //5-7 seconds
        case 5:
        case 6:
        case 7:  {
#ifdef DEBUG
            Serial.println("sel pr:");
#endif
            // switch preset
            loadPreset(pickPreset());
          }
          break;

        // 8-10 seconds
        case 8:
        case 9:
        case 10: {
#ifdef DEBUG
            Serial.println("sel ch:");
#endif
            // switch channel
            chgAllChannels(pickChannel());  // make this change all in the live preset
          }
          break;

        //case 12: case 13: case 14: case 15:  {
        //Serial.println("factory reset??");
        // reload factory preset to live preset??
        //}
        //break;

        default: {
            //Serial.print("other time triggered.");
          }
      }
    }
  }
}

short pickPreset() {
  // use pickPosition() in combination with the knob reading (regardless of what mode the knob is in)
  short pickedOTA = pickPosition();
  short numPickPositions = 2; // zone 1 = 123, zone 2 = 45 (5)
  short pickedPreset = constrain(getComplexValue(pickedOTA, numPickPositions, 5) - 1, 0, 4);

#ifdef DEBUG
  Serial.print("pr ");
  Serial.println(pickedPreset);
#endif
  return pickedPreset;
}


short getComplexValue(short pickedOTA, short numPickPositions, short limit) {

  short tmp = analogRead(knobPin);
  tmp = constrain((systemBlock.knobMaxValue) - analogRead(knobPin), 0, systemBlock.knobMaxValue); // flip on the goood read
  float rawzone = (float) tmp / systemBlock.knobMaxValue;
  //Serial.print("rawzoneb4=");
  //Serial.println(rawzone);
  rawzone = (float) rawzone * numPickPositions;
  short cleanzone = (int) rawzone;
  short rtnval = (int) constrain((cleanzone * 3) + pickedOTA, 0, limit);
  //Serial.print("rawzone=");
  //Serial.println(rawzone);
  //Serial.print(" rtnval=");
  //Serial.println(rtnval);
  return rtnval;
}

short pickChannel() {
  // use pickPosition() in combination with the knob reading (regardless of what mode the knob is in)

  // do like a mode select except just get what they hit - same code so reuse for mode select
  // rename mode select to like pickOTA() and return which OTA they picked.  leave the setup code in ModeSelect and we can now use pickOTA ourselves...
  short pickedOTA = pickPosition();
  short numPickPositions = 6; // number of zones to define on the knob
  short channel = getComplexValue(pickedOTA, numPickPositions, 16) - 1; // zn 1 = 123, 2=456,3=789,4=10 11 12, 5 = 13 14 15, 6 = 16 (16) (16)
#ifdef DEBUG
  Serial.print("ch ");
  Serial.println(channel);
#endif
  return channel;
}

void heartbeat() {
  if ((int)(millis() - cyclestart) >= delayint) {
    cyclestart = millis();
    if ((hbval >= fademax) || (hbval <= fademin)) {
      fadespeed = -fadespeed; // flip increment counter - possible bugs here b/c the come back into the range may not be how we went out
    }
    hbval += fadespeed;
    hbval = constrain(hbval, fademin, fademax);
    //hbval += fadespeed;
    analogWrite(commLedPin, hbval);

    /*
      Serial.print("fadespeed=");
      Serial.print(fadespeed);
      Serial.print(",raw hbval=");
      Serial.print(hbval);
      Serial.print(",hbval=");
      Serial.println(constrain(hbval, fademin, fademax));
    */

  }
}

/*
  set fade cycle speed to reflect the sensitivity knob's value - if the pedal is operating and fade cycle is ota-based,
  the fade cycle speed will reflect the sensitivity knob's position rather than the OTA position until the OTA position changes again
  which should not be a problem unless the OTA is, say, locked on with the sentivivity knob all the way up, or, being played by a robot,
  who happens to care about the exact fade speed.  either way it's good enough for now.  -JR
*/
void fadeOnSensitivity(short val) {

  fadespeed = map(val, systemBlock.knobMaxValue, deviceKnobMin, fadeSpeedMin, fadeSpeedMax);
  /*
    Serial.print("fos val=");
    Serial.println(val);
    Serial.print(", fadespeed=");
    Serial.println(fadespeed);
  */
}

void fadeOnPosition(short value, short min, short max) {
  short chk = map(value, min, max, fadeSpeedMin, fadeSpeedMax);

  if (!(fadespeed == chk)) {
    fadespeed = map(value, min, max, fadeSpeedMin, fadeSpeedMax);
  }

  /*
    Serial.print("fop fadespeed=");
    Serial.print(fadespeed);
    Serial.print(", min=");
    Serial.print(min);
    Serial.print(", max=");
    Serial.print(max);
    Serial.print(", fadeSpeedMin=");
    Serial.print(fadeSpeedMin);
    Serial.print(", fadeSpeedMax=");
    Serial.println(fadeSpeedMax);
  */
}

// since the new struct cannot address attributes iteratively, use a case stmt instead
// set all attributes of all slots
void setSlotAttribute(short attr, short value) { // e.g. setSlotAttribute(MSP_CHANNEL_COL, 3) = change all channels to 3 (4)
  for (short m = 0; m < numberOfModes; m++) { // mode looper
    for (short sp = 0; sp < numberOfInputs; sp++) { // position looper
      for (short sl = 0; sl < numberOfSlots; sl++) { // slot looper
        switch (attr) {
          case 0: // MidiCmd
            livePreset.mode[m].pos[sp].slot[sl].MidiCmd = value;
            break;
          case 1: // MidiChannel
            livePreset.mode[m].pos[sp].slot[sl].MidiChannel = value;
            break;
          case 2: // Midi Sub command
            livePreset.mode[m].pos[sp].slot[sl].MidiSubCommand = value;
            break;
          case 3: // Curve type
            livePreset.mode[m].pos[sp].slot[sl].curvetype = value;
            break;
          case 4: // Curve dir
            livePreset.mode[m].pos[sp].slot[sl].curvedir = value;
            break;
          case 5: // min range
            livePreset.mode[m].pos[sp].slot[sl].minrange = value;
            break;
          case 6: // max range
            livePreset.mode[m].pos[sp].slot[sl].maxrange = value;
            break;
          case 7: // latching
            livePreset.mode[m].pos[sp].slot[sl].latching = value;
            break;
        }
      }
    }
  }
}

void chgAllChannels(short channel) {
  for (short m = 0; m < numberOfModes; m++) { // mode looper
    for (short sp = 0; sp < numberOfInputs; sp++) { // position looper
      for (short sl = 0; sl < numberOfSlots; sl++) { // slot looper
        livePreset.mode[m].pos[sp].slot[sl].MidiChannel = channel;
      }
    }
  }
}

void chgChannel(short mode, short spot, short slot, short channel) {
  livePreset.mode[mode].pos[spot].slot[slot].MidiChannel = channel;
}

// change the channel of any slot of any preset
void chgChannel(slotAddress sa, short chn) {
  if (sa.preset == 0) { // the live preset
    livePreset.mode[sa.mode].pos[sa.pos].slot[sa.slot].MidiChannel = chn;
  } else {
    // not the live preset
    Preset bufferPreset;
    EEPROM.get(sizeof(systemBlock) + (sizeof(bufferPreset)*sa.preset), bufferPreset);
    bufferPreset.mode[sa.mode].pos[sa.pos].slot[sa.slot].MidiChannel = chn;
    EEPROM.put(sizeof(systemBlock) + (sizeof(bufferPreset)*sa.preset), bufferPreset);
  }
}

// redo this to fadeblink the LEDs according to the global fademax ******** TDL

void blinkCommLED(short time, short repetitions, short color, boolean bright) { // color 1 = blue, color 2 = red, color 3 = both
  // turn off fading for hard blinking

  // regBlinkFadeMax ******** tag
  //
  //short blinkstate = LOW;
  for (short t = 0; t < repetitions; t ++) {
    if (color == BLUE || color == BOTH ) {
      if (bright) {
        digitalWrite(commLedPin, HIGH);  // turn on led
      } else {
        analogWrite(commLedPin, HIGH);  // turn oon led
      }
    }
    if (color == RED || color == BOTH ) {
      if (bright) {
        digitalWrite(redLedPin, HIGH);  // turn on led
      } else {
        analogWrite(redLedPin, HIGH);  // turn on led
      }
    }
    //analogWrite(commLedPin, LOW & 1 & color);  // turn off led
    //analogWrite(redLedPin, LOW & 2 & color);  // turn off led
    delay(time);
    //#ifdef DEBUG(".");
    //#endif
    if (color == BLUE || color == BOTH ) {
      analogWrite(commLedPin, LOW);  // turn oon led
    }
    if (color == RED || color == BOTH ) {
      analogWrite(redLedPin, LOW);  // turn on led
    }
    //    analogWrite(commLedPin, HIGH & 1 & color);  // turn on led
    //    analogWrite(redLedPin, HIGH & 2 & color);  // turn on led
    delay(time);
  }
} //blinkCommLED


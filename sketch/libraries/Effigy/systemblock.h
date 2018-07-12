// Effigy Control Pedal system block definition
// (C) 2018, Effigy Labs LLC
struct systemblock { 	// currently 100 bytes
  short serialNumber: 16; 		//2 bytes dddd of R4Cddddd little-endian style, i.e. serial number 9 would be 09 00
  short knobMaxValue: 16 ; 		// max knob value calibrated at factory - 0-1024 for a 1M potentiometer e.g. B1M
  // and 0~500 for a 100K e.g. B100K pots.  any linear pot works in the R4Cs.
  short knobCurrentValue: 16 ; 	// saved knob value as saved by last flip to save sensitivity
  byte bootmode: 8; 			// 0=mode select, 1-3 = autoselect mode 1-3
  byte presetToLoad: 8; 		// 0=factory preset, 1-5 = selects user-defined preset
  byte knobControl: 8;  		// toggle between knob as sensitivity control and as a position
  byte fademax: 8; 				// led fade max (brightness) (%)
  byte porchsize: 8; 			// stored/altered porch size
  byte ceilsize: 8; 			// stored/altered porch size
  byte reserved[88]; 			// reserved for factory reset, and also a cool version of the A4 note
  // to extend the system block:
  // 1. allocate the size here
  // 2  add the code in the pedal to use it
  // 3  add the code in the app to allow usage via config dialog - this is about 4 places to list in the UI fields, the tool tip etc.
  // 4  implement in the main app to use or display

  // possibly a calibration array of the original factory caliibration at nominal tension strap - to compare against over time and, maybe, compensate??
};

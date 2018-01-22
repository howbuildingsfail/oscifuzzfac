
/*********************************************************************************/
/* CHANGE LOG        *************************************************************/
/*********************************************************************************/
/* 21/01/18
 *  
 *  Preparations for first live gig, using a guitar and touchosc to control:
 *  
 * - order of wave selection switced to sin-tri-saw-squ
 * - TODO - change oscil mag range from +-128 to +-64
 * - TODO - change range of oscil rates - currently top end is too fast, bottom 
 *          too slow
 * - only listening on MIDI channel 13 now         
 * 
 * - for the gig, we'll set STAB and GATE manually while we get used to the device
 * - (also need to tidy up the circuit so we get a cleaner sound with STAB & GATE
 */


//define this if you want received messages to be re-sent (for e.g. touchosc)
#define MIDI_ECHO


/*********************************************************************************/
/*  SPI              *************************************************************/
/*********************************************************************************/

#include "SPI.h"

//We are going to use the dedicated SPI stuff in the arduino - not reinvent the wheel.
// http://tronixstuff.com/2011/05/13/tutorial-arduino-and-the-spi-bus/

//We'll need two SS pins - one for each chip
//Using pins 7 & 8 for now as mozzi doesn't use them
#define SPI_SS_A 7
#define SPI_SS_B 8

/*********************************************************************************/
/*  PIN DEFINITIONS  *************************************************************/
/*********************************************************************************/

/* On Arduino we can usually use analogwrite on pins 3, 5, 6, 9, 10, and 11. 

  However, mozzi manipulates the timers, which means this isn't the case:
  instead, there's a bitbanging trick which works - see updateLED function

  We've put the SPI connections on the analog side, and the LED on the digital side
  
 */

//*Mozzi disables analogWrite() on pins 5 and 6 (Timer 0), 9 and 10 (Timer 1) in STANDARD_PLUS mode. 
//In HIFI mode, pins 3 and 11 (Timer 2) are also out. Pin numbers vary between boards.
//PIN 9 DOESN'T SEEM TO WOIK!work
//PIN 5 and 6 don't seem to 
#define LED_COMP_PIN 4      //output flashing LED in time with the LFO rate for COMP pot
#define LED_DIST_PIN 3      //output flashing LED in time with the LFO rate for DIST pot
#define LED_GATE_PIN 6      //output flashing LED in time with the LFO rate for GATE pot
#define LED_STAB_PIN 2      //output flashing LED in time with the LFO rate for STAB pot


/*********************************************************************************/
/*  GLOBAL VARIABLES  ************************************************************/
/*********************************************************************************/

//digipot values
byte compVal,distVal,gateVal,stabVal;

//'Central' values for oscillators: 
byte compCnt,distCnt,gateCnt,stabCnt;

//Magnitude of oscillation: 
byte compMag, distMag, gateMag, stabMag;

//Phase of oscillation:
byte compPha, distPha, gatePha, stabPha;


//TODO: we could #define these!
byte cmd_byte1    = B00010010 ;            // Command byte
byte cmd_byte0    = B00010001 ;            // Command byte
byte cmd_byteboth = B00010011 ;            // Command byte

/*********************************************************************************/
/*  http://sensorium.github.io/Mozzi/learn/a-simple-sketch/  *********************/
/*********************************************************************************/

//NB: `#define AUDIO_MODE STANDARD_PLUS` in mozzi config

#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>
#include <tables/triangle2048_int8.h>

#define CONTROL_RATE (128)

//LFO oscillators: 
Oscil <2048, CONTROL_RATE> compWav(SIN2048_DATA);
Oscil <2048, CONTROL_RATE> gateWav(SIN2048_DATA);
Oscil <2048, CONTROL_RATE> distWav(SIN2048_DATA);
Oscil <2048, CONTROL_RATE> stabWav(SIN2048_DATA);

/*********************************************************************************/
/* LED brightness - using bitbanging *********************************************/
/*********************************************************************************/

/* https://github.com/sensorium/Mozzi/blob/master/examples/11.Communication/Sinewave_PWM_leds_HIFI/Sinewave_PWM_leds_HIFI.ino
 * 
void updateRGB(byte r, byte g, byte b){
  // stagger pwm counter starts to try to reduce combined flicker
  static byte red_count=0;
  static byte green_count=85;
  static byte blue_count=170;
// PORTD maps to Arduino digital pins 0 to 7
// http://playground.arduino.cc/Learning/PortManipulation
  (red_count++ >= r) ? PORTD &= ~(1 << RED_PIN) : PORTD |= (1 << RED_PIN);
  (green_count++ >= g) ? PORTD &= ~(1 << GREEN_PIN) : PORTD |= (1 << GREEN_PIN);
  (blue_count++ >= b) ? PORTD &= ~(1 << BLUE_PIN) : PORTD |= (1 << BLUE_PIN);
}
*/


void updateLED(byte PIN, byte r){
  // stagger pwm counter starts to try to reduce combined flicker
  static byte red_count=0;
  
  //static byte green_count=85;
  //static byte blue_count=170;
  
  // PORTD maps to Arduino digital pins 0 to 7
  // http://playground.arduino.cc/Learning/PortManipulation

  (red_count++ >= r) ? PORTD &= ~(1 << PIN) : PORTD |= (1 << PIN);
}


/*********************************************************************************/
/* Generic setting functions  ****************************************************/
/*********************************************************************************/

/*Set pots to a fixed value*/
void setPots(byte comp, byte dist, byte gate, byte stab){

  compCnt = comp;
  distCnt = dist;
  gateCnt = gate;
  stabCnt = stab;
}



/*********************************************************************************/
/* MIDI - from jamjar drumkit ****************************************************/
/*********************************************************************************/

#include <MIDI.h>

MIDI_CREATE_DEFAULT_INSTANCE();

#include <mozzi_midi.h>


void HandleNoteOn(byte channel, byte note, byte velocity) {
  
  switch (note) {
    //case 24:
    //  break;
    default:
      //This will revert to the 'velcro fuzz' setting - meaning any keyboard press can immediately fix any madness
      setPots(120,10,100,100);
      //Turn of all oscillations
      compMag = distMag = gateMag = stabMag = 0;
      break;
  }
}



void HandleNoteOff(byte channel, byte note, byte velocity) {
  
  switch (note) {
    //case 24:
    //  break;
    //case 60:
    //  break;
    default:
      break;
  }
}

//REMEMBER! you'll need to set the magnitude as well!
//TODO: Find a good frequency sweep - might need a log or a pow or something
//might be quicker to split the rate - have a steeper gradient above 64
//and a slower gradient below - that way we'd cover a wider range..
void setLFOfreq(Oscil <2048, CONTROL_RATE> * osc, float multiplier, byte value){

      osc->setFreq(multiplier * value);
    
}


#define LFO_M (0.001165)
#define LFO_C (0.02)
//REMEMBER! you'll need to set the magnitude as well!
//TODO: Find a good frequency sweep - might need a log or a pow or something
//might be quicker to split the rate - have a steeper gradient above 64
//and a slower gradient below - that way we'd cover a wider range..
void setLFOfreq_2(Oscil <2048, CONTROL_RATE> * osc,  byte value){

      const byte vml = value-1;


      osc->setFreq((float) ((LFO_M * vml *vml) + LFO_C));
    
}





#define BIG_FREQ_STEP (0.4)
#define SML_FREQ_STEP (0.005)
void HandleControlChange (byte channel, byte number, byte value){


#ifdef MIDI_ECHO

  MIDI.sendControlChange(channel,number,value);

#endif


  
  switch (number){
    /***************************************************************/
    /* TODO: RESET CONTROLS - put params back to sensible defaults */
    case 10: //Reset Everything
    case 11: //Reset comp



    
    /***************************************************************/
    case 16:
      //setLFOfreq(&compWav, BIG_FREQ_STEP, value);
      setLFOfreq_2(&compWav, value);
      break;

    case 17:
      setLFOfreq(&compWav, 0.005, value);    
      break;

    case 18:
      if      (value < 32)  compWav.setTable(SIN2048_DATA);
      else  if(value < 64)  compWav.setTable(TRIANGLE2048_DATA);  
      else  if(value < 96)  compWav.setTable(SAW2048_DATA);  
      else                  compWav.setTable(SQUARE_NO_ALIAS_2048_DATA);  
      break;

    case 19:
      compMag = value;  
      break;

    case 20:
      compCnt = value << 1;
      break;

    case 21: //PHASE - UNTESTED!
      /* 2048 samples - so each midi value can be an increment of 16. Default value is therefore zero. 
       *  
       */
      compPha = value;
      compWav.setPhase(compPha << 4);
      break;

      
    /***************************************************************/
    case 32:
      //setLFOfreq(&distWav, BIG_FREQ_STEP, value);
      setLFOfreq_2(&distWav, value);  
      break;
      
    case 33:
      setLFOfreq(&distWav, 0.005, value);  
      break;

    case 34:
      if      (value < 32)  distWav.setTable(SIN2048_DATA);
      else  if(value < 64)  distWav.setTable(TRIANGLE2048_DATA);  
      else  if(value < 96)  distWav.setTable(SAW2048_DATA); 
      else                  distWav.setTable(SQUARE_NO_ALIAS_2048_DATA); 
      break;

    case 35:
      distMag = value;  
      break;

    case 36:
      distCnt = value << 1;
      break;

    case 37: //PHASE - UNTESTED!
      /* 2048 samples - so each midi value can be an increment of 16. Default value is therefore zero. 
       *  
       */
      distPha = value;
      distWav.setPhase(distPha << 4);
      break;


      
    /***************************************************************/
    case 48:
      //setLFOfreq(&gateWav, BIG_FREQ_STEP, value);  
      setLFOfreq_2(&gateWav, value);
      break;
      
    case 49:
      setLFOfreq(&gateWav, 0.005, value);  
      break;

    case 50:
      if      (value < 32)  gateWav.setTable(SIN2048_DATA);
      else  if(value < 64)  gateWav.setTable(TRIANGLE2048_DATA);  
      else  if(value < 96)  gateWav.setTable(SAW2048_DATA);  
      else                  gateWav.setTable(SQUARE_NO_ALIAS_2048_DATA); 
      break;

    case 51:
      gateMag = value;  
      break;

    case 52:
      gateCnt = value << 1;
      break;

    case 53: //PHASE - UNTESTED!
      /* 2048 samples - so each midi value can be an increment of 16. Default value is therefore zero. 
       *  
       */
      gatePha = value;
      gateWav.setPhase(gatePha << 4);
      break;




      
    /***************************************************************/
    case 64:
      //setLFOfreq(&stabWav, BIG_FREQ_STEP, value);  
      setLFOfreq_2(&stabWav, value);
      break;
      
    case 65:
      setLFOfreq(&stabWav, 0.005, value);  
      break;

    case 66:
      if      (value < 32)  stabWav.setTable(SIN2048_DATA);
      else  if(value < 64)  stabWav.setTable(TRIANGLE2048_DATA);  
      else  if(value < 96)  stabWav.setTable(SAW2048_DATA);  
      else                  stabWav.setTable(SQUARE_NO_ALIAS_2048_DATA);  
      break;

    case 67:
      stabMag = value;  
      break;

    case 68:
      stabCnt = value << 1;
      break;

    case 69: //PHASE - UNTESTED!
      /* 2048 samples - so each midi value can be an increment of 16. Default value is therefore zero. 
       *  
       */
      stabPha = value;
      stabWav.setPhase(stabPha << 4);
      break;

  }  
}

/*
//spi method was derived from this, but it doesn't use spi library!!
//https://github.com/rickit69/techrm/blob/master/test_potenziometro_digitale_1/test_potenziometro_digitale_1.ino
*/


//TODO: This is the new version of the send command using SPI.h
void spi_out(int CS, byte cmd_byte, byte data_byte){                        // we need this function to send command byte and data byte to the chip
    
    digitalWrite (CS, LOW);                                                 // to start the transmission, the chip select must be low
    SPI.transfer(cmd_byte); // invio il COMMAND BYTE
    //delay(2);
    SPI.transfer(data_byte); // invio il DATA BYTE
    //delay(2);
    digitalWrite(CS, HIGH);                                                 // to stop the transmission, the chip select must be high
}




void setup() {

  pinMode (LED_COMP_PIN, OUTPUT);
  pinMode (LED_DIST_PIN, OUTPUT);
  pinMode (LED_GATE_PIN, OUTPUT);
  pinMode (LED_STAB_PIN, OUTPUT);

  //SPI.h
  pinMode (SPI_SS_A,  OUTPUT);
  pinMode (SPI_SS_B,  OUTPUT);
  SPI.begin();
  //Most SPI chips are MSBfirst: https://www.arduino.cc/en/Reference/SPI
  SPI.setBitOrder(MSBFIRST);

  //TODO: Find good initial settings! - OR better still, send the midi control message for the default setting
  //NB: This action is voided on the first call to updateControl...
  //initialize_spi(aMOSI_signal,aCLK_signal);


  // Initiate MIDI communications, listen to all channels
  MIDI.begin(13);
  Serial.begin(38400);/* This baud rate is recommended for ALSA: http://alsa.opensrc.org/Serial  */

  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  MIDI.setHandleControlChange(HandleControlChange);


  //Centre of oscillation
  compCnt = distCnt = gateCnt = stabCnt = 128;

  //Magnitude of oscillation 
  compMag = distMag = gateMag = stabMag = 127;
  


  /* Freq: 6.5 is twice the speed of 3.25 */
  compWav.setFreq(2.0f);  //5.25f);//1.0f/2.0f);//(0.42f);   
  distWav.setFreq(1.0f);  //(0.0125f);//0.25f);//1.0f);//1.0f/3.0f);//(0.42f); 
  gateWav.setFreq(8.0f);  //6.75f);//5.0f);//(0.42f);               
  stabWav.setFreq(4.0f);  //1.0f);///7.0f);//(0.42f);             
  
  startMozzi(CONTROL_RATE);
}

void updateControl(){

  //Process any incoming MIDI: 
  MIDI.read();

  //NB These override the midi! but are useful for setting up / fixing in a hurry...
  //stabMag = 120;
  //stabCnt = 64;
  //gateMag = 64;
  //gateCnt = 48;
  //compMag = 32;
  //compMag = 96;


  //TODO: Rather than clumsily checking if we are oscillating or not, simply set xxxxMag to zero - processing time is slightly higher, but *guaranteed*, and code is easier to maintain - saves a variable too. 
  
  
  //Iterate the oscillators: 
  // compWav.next() returns a signed byte between -128 to 127 from the wave table
  compVal = compCnt + ((compMag * compWav.next())>>7);
  distVal = distCnt + ((distMag * distWav.next())>>7);
  gateVal = gateCnt + ((gateMag * gateWav.next())>>7);
  stabVal = stabCnt + ((stabMag * stabWav.next())>>7);


  //spi_out(CS_signal, cmd_byteboth, compVal); 
  //FURTHEST from Arduino
  //With Arduino to the LEFT, byte0 is on the bottom, byte1 is on the top (CHECK THIS)
  //spi_out(aCS_signal, cmd_byte0, stabVal,  aMOSI_signal, aCLK_signal); 
  //spi_out(aCS_signal, cmd_byte1, compVal,  aMOSI_signal, aCLK_signal); 
  spi_out(SPI_SS_A,  cmd_byte0, stabVal);
  spi_out(SPI_SS_A,  cmd_byte1, compVal);


  //NEAREST to Arduino
  //spi_out(bCS_signal, cmd_byte0, gateVal,  bMOSI_signal, bCLK_signal); 
  //spi_out(bCS_signal, cmd_byte1, distVal,  bMOSI_signal, bCLK_signal); 
  spi_out(SPI_SS_B,  cmd_byte0, gateVal);
  spi_out(SPI_SS_B,  cmd_byte1, distVal);

  //TODO: periodically send MIDI state (for touchosc)  
}


//TODO - we shouldn't have to update LEDs this often - figure out how to do this...
int updateAudio(){

  updateLED(LED_COMP_PIN, compVal);
  updateLED(LED_DIST_PIN, distVal);
  updateLED(LED_GATE_PIN, gateVal);
  updateLED(LED_STAB_PIN, stabVal);
  
  return 1;
}



void loop() {
  audioHook();
  
}

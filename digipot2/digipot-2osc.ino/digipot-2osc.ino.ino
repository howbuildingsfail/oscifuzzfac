
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
//#define MIDI_ECHO


/*********************************************************************************/
/*  SPI              *************************************************************/
/*********************************************************************************/

#include "SPI.h"

//We are going to use the dedicated SPI stuff in the arduino - not reinvent the wheel.
// http://tronixstuff.com/2011/05/13/tutorial-arduino-and-the-spi-bus/

//We'll need an SS pin
//Using pin 7 as mozzi doesn't use them
#define SPI_SS_A 7

#define POT_PIN_1 A1
#define POT_PIN_2 A2
#define POT_PIN_3 A3


//#define SPI_SS_B 8

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


#define MAG_CNT_PROP  0.015625  // = 1/64  - used to calculate max oscillation given a centre


/*********************************************************************************/
/*  GLOBAL VARIABLES  ************************************************************/
/*********************************************************************************/

//digipot values
byte compVal,gateVal;

//'Central' values for oscillators: 
byte compCnt,gateCnt;
float compCMprop, gateCMprop;

//Magnitude of oscillation: 
byte compMag, gateMag;

//Phase of oscillation:
byte compPha, gatePha;


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
  gateCnt = gate;
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
      compMag = gateMag = 0;
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

  if(value)
      osc->setFreq(multiplier * value);
  else
      osc->setFreq(0);
    
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

  //NB Gate and Stab not working proply at the mo-- check circuit...
  pinMode (LED_GATE_PIN, OUTPUT);
  pinMode (LED_STAB_PIN, OUTPUT);

  //SPI.h
  pinMode (SPI_SS_A,  OUTPUT);

  //Pot pins
  pinMode(POT_PIN_1, INPUT); 
  pinMode(POT_PIN_2, INPUT); 
  pinMode(POT_PIN_3, INPUT); 

  SPI.begin();
  //Most SPI chips are MSBfirst: https://www.arduino.cc/en/Reference/SPI
  SPI.setBitOrder(MSBFIRST);

  // Initiate MIDI communications, listen to all channels
  MIDI.begin(13);

#ifdef MIDI_ECHO
  Serial.begin(115200);/* This baud rate is recommended for MIDI: https://github.com/FortySevenEffects/arduino_midi_library/issues/35  
                          (seems to work even if ttymidi's baud rate is still 38400....*/
#else
  Serial.begin(38400);/* This baud rate is recommended for ALSA: http://alsa.opensrc.org/Serial  */
#endif  

  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  MIDI.setHandleControlChange(HandleControlChange);


  //Centre of oscillation and m
  compCnt = gateCnt = 64;

  compCMprop = gateCMprop = 1;
  
  //Magnitude of oscillation 
  compMag = gateMag = 127;
  
  /* Freq: 6.5 is twice the speed of 3.25 */
  compWav.setFreq(2.0f);  //5.25f);//1.0f/2.0f);//(0.42f);   
  gateWav.setFreq(8.0f);  //6.75f);//5.0f);//(0.42f);    
  
  startMozzi(CONTROL_RATE);
}


int potval1,potval2,potval3;
int lastpotval1=127,lastpotval2=127, lastpotval3=127;


void updateControl(){

  //Process any incoming MIDI: 
  MIDI.read();


  //Process pot1 - LFO Centre
  potval1 = analogRead(POT_PIN_1);          //Read and save analog value from potentiometer
  potval1 = map(potval1, 0, 1023, 0, 127);

  if(potval1 != lastpotval1){
    
      compCnt = potval1;
      gateCnt = potval1;

      lastpotval1 = potval1;

      //Update the maximum proportion that lag can be: 
      compCMprop =  gateCMprop = MAG_CNT_PROP * min(compCnt,(127-compCnt));
      
  }


  //Process pot2 - LFO Magnitude
  potval2 = analogRead(POT_PIN_2);          //Read and save analog value from potentiometer
  potval2 = map(potval2, 0, 1023, 0, 127);

  if(potval2 != lastpotval2){
    
      compMag = potval2;
      gateMag = potval2;

      lastpotval2 = potval2;      
  }


  //Process pot3 - LFO Frequency
  potval3 = analogRead(POT_PIN_3);          //Read and save analog value from potentiometer
  potval3 = map(potval3, 0, 1023, 0, 127);

  if(potval3 != lastpotval3){
    
      setLFOfreq_2(&compWav, potval3);  
      setLFOfreq_2(&gateWav, potval3+10);  

      lastpotval3 = potval3;
  }
  
  

  
  /*Iterate the oscillators: 
    - compWav.next() returns a signed byte between -128 to 127 from the wave table
    - compMag is the magnitude - from 0 to 127
    - the >>7 operation puts this in the range -64 to 64
    - compCMprop scales the mag to fit within the range 0 to 127 - the wave will be v. small at extreme values
  */  
  
  
  compVal = compCnt + (compCMprop *(( compMag * compWav.next())>>7) );
  gateVal = gateCnt + (gateCMprop *(( gateMag * gateWav.next())>>7) );


  //spi_out(CS_signal, cmd_byteboth, compVal);  
  spi_out(SPI_SS_A,  cmd_byte0, gateVal);
  spi_out(SPI_SS_A,  cmd_byte1, compVal); 

  //pot pin


  
}


//TODO - we shouldn't have to update LEDs this often - figure out how to do this...
int updateAudio(){

  updateLED(LED_DIST_PIN, (compVal+gateVal)>>2);
  updateLED(LED_COMP_PIN, lastpotval1);
  updateLED(LED_STAB_PIN, lastpotval2);
  updateLED(LED_GATE_PIN, lastpotval3);
  
  return 1;
}



void loop() {
  audioHook();
  
}

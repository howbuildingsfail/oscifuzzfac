

/*********************************************************************************/
/*  PIN DEFINITIONS  *************************************************************/
/*********************************************************************************/


/* On Arduino we can usually use analogwrite on pins 3, 5, 6, 9, 10, and 11. 

  However, mozzi manipulates the timers, which means this isn't the case:
  instead, there's a bitbanging trick which works - see updateLED function

  We've put the SPI connections on the analog side, and the LED on the digital side
  
 */




//TODO: we could #define these!
//This is the chip NEAREST TO the Arduino
//NB We were having trouble with using the analog pins - not sure why
byte aCS_signal   = 8;//A4;//5;//2;                      // Chip Select signal on pin 5 of Arduino
byte aCLK_signal  = 12;//A5;//6;//4;                     // Clock signal on pin 6 of Arduino
byte aMOSI_signal = 13;//A6;//7;//5;                    // MOSI signal on pin 7 of Arduino

//This is the chip FURTHEST FROM the Arduino
byte bCS_signal   = A0;//2;                      // Chip Select signal on pin 5 of Arduino
byte bCLK_signal  = A1;//3;                     // Clock signal on pin 6 of Arduino
byte bMOSI_signal = A2;//4;                    // MOSI signal on pin 7 of Arduino


//*Mozzi disables analogWrite() on pins 5 and 6 (Timer 0), 9 and 10 (Timer 1) in STANDARD mode. In HIFI mode, pins 3 and 11 (Timer 2) are also out. Pin numbers vary between boards.
//PIN 9 DOESN'T SEEM TO WOIK!work
//PIN 5 and 6 don't seem to 
#define LED_COMP_PIN 4     //output flashing LED in time with the LFO rate for GATE pot
#define LED_DIST_PIN 3      //output flashing LED in time with the LFO rate for GATE pot
#define LED_GATE_PIN 2//10     //output flashing LED in time with the LFO rate for GATE pot
#define LED_STAB_PIN 6    // trying to bitbang this ..


/*********************************************************************************/
/*  GLOBAL VARIABLES  ************************************************************/
/*********************************************************************************/

//LFO values
byte compVal,distVal,gateVal,stabVal;

//Boolean to say whether we are oscillating or not
bool  OSCcomp = true,
      OSCdist = true,
      OSCgate = true,
      OSCstab = true;


//TODO: we could #define these!
byte cmd_byte1    = B00010010 ;            // Command byte
byte cmd_byte0    = B00010001 ;            // Command byte
byte cmd_byteboth = B00010011 ;            // Command byte


//TODO: set initial value better - perhaps use the default...? 
int initial_value = 100;                // Setting up the initial value

/*********************************************************************************/
/*  http://sensorium.github.io/Mozzi/learn/a-simple-sketch/  *********************/
/*********************************************************************************/

#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <tables/saw2048_int8.h>

#define CONTROL_RATE (128)

//LFO oscillators: 
Oscil <2048, CONTROL_RATE> compSin(SIN2048_DATA);
Oscil <2048, CONTROL_RATE> gateSin(SIN2048_DATA);
Oscil <2048, CONTROL_RATE> distSin(SIN2048_DATA);
Oscil <2048, CONTROL_RATE> stabSin(SIN2048_DATA);

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


void setOsc(bool comp, bool dist, bool gate, bool stab){

  OSCcomp = comp;
  OSCdist = dist;
  OSCgate = gate;
  OSCstab = stab;
  
}

void setPot(byte comp, byte dist, byte gate, byte stab){

  compVal = comp;
  distVal = dist;
  gateVal = gate;
  stabVal = stab;

  setOsc(false,false,false,false);
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
      setPot(120,10,100,100);
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


void setLFOfreq(Oscil <2048, CONTROL_RATE> * osc, bool * onflag, float multiplier, byte value){
      //compSin.setFreq((float) 0.5 * value); 
      osc->setFreq(multiplier * value);
      *onflag = true;
  
}


void HandleControlChange (byte channel, byte number, byte value){
  switch (number){
    /***************************************************************/
    case 16:
      setLFOfreq(&compSin, &OSCcomp, 0.5, value);
      break;

    case 17:
      setLFOfreq(&compSin, &OSCcomp, 0.05, value);    
      break;

    case 18:
      if(value < 33) compSin.setTable(SIN2048_DATA);
      else if(value < 65) compSin.setTable(SAW2048_DATA);

      
    /***************************************************************/
    case 32:
      setLFOfreq(&distSin, &OSCdist, 0.5, value);  
      break;
      
    case 33:
      setLFOfreq(&distSin, &OSCdist, 0.05, value);  
      break;

      
    /***************************************************************/
    case 48:
      setLFOfreq(&gateSin, &OSCgate, 0.5, value);  
      break;
      
    case 49:
      setLFOfreq(&gateSin, &OSCgate, 0.05, value);  
      break;

      
    /***************************************************************/
    case 64:
      setLFOfreq(&stabSin, &OSCstab, 0.5, value);  
      break;
      
    case 65:
      setLFOfreq(&stabSin, &OSCstab, 0.05, value);  
      break;

    
  }
  
}



void initialize_spi(byte mosi, byte clk) {                     // send the command byte of value 100 (initial value)
    //TODO: need to initialise all four pots - 
    spi_out(aCS_signal, cmd_byteboth, initial_value, mosi, clk);
}

void spi_out(int CS, byte cmd_byte, byte data_byte, byte mosi, byte clk){                        // we need this function to send command byte and data byte to the chip
    
    digitalWrite (CS, LOW);                                                 // to start the transmission, the chip select must be low
    spi_transfer(cmd_byte,mosi,clk); // invio il COMMAND BYTE
    //delay(2);
    spi_transfer(data_byte,mosi,clk); // invio il DATA BYTE
    //delay(2);
    digitalWrite(CS, HIGH);                                                 // to stop the transmission, the chip select must be high
}

void spi_transfer(byte working, byte mosi, byte clk) {
  for(int i = 1; i <= 8; i++) {                                           // Set up a loop of 8 iterations (8 bits in a byte)
    if (working > 127) { 
      digitalWrite (mosi,HIGH) ;                                    // If the MSB is a 1 then set MOSI high
    } 
    else { 
      digitalWrite (mosi, LOW) ;                                     // If the MSB is a 0 then set MOSI low     
    }                                                                         
    digitalWrite (clk,HIGH) ;                                        // Pulse the CLK_signal high
    working = working << 1 ;                                                // Bit-shift the working byte
    digitalWrite(clk,LOW) ;                                          // Pulse the CLK_signal low
  }
}





void setup() {
  pinMode (aCS_signal, OUTPUT);
  pinMode (aCLK_signal, OUTPUT);
  pinMode (aMOSI_signal, OUTPUT);
  
  pinMode (bCS_signal, OUTPUT);
  pinMode (bCLK_signal, OUTPUT);
  pinMode (bMOSI_signal, OUTPUT);

  pinMode (LED_COMP_PIN, OUTPUT);
  pinMode (LED_DIST_PIN, OUTPUT);
  pinMode (LED_GATE_PIN, OUTPUT);
  pinMode (LED_STAB_PIN, OUTPUT);

  //TODO: Find good initial settings! - OR better still, send the midi control message for the default setting
  //NB: This action is voided on the first call to updateControl...
  initialize_spi(aMOSI_signal,aCLK_signal);


  // Initiate MIDI communications, listen to all channels
  MIDI.begin(MIDI_CHANNEL_OMNI);
  Serial.begin(38400);/* This baud rate is recommended for ALSA: http://alsa.opensrc.org/Serial  */

  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  MIDI.setHandleControlChange(HandleControlChange);
  


  /* Freq: 6.5 is twice the speed of 3.5 */
  //use primes to init - get the range!
  compSin.setFreq(0.25f);//1.0f/2.0f);//(0.42f);         This is STAB!
  distSin.setFreq(0.25f);//1.0f);//1.0f/3.0f);//(0.42f); 
  gateSin.setFreq(14.25f);//5.0f);//(0.42f);               
  stabSin.setFreq(0.25f);///7.0f);//(0.42f);             
  
  startMozzi(CONTROL_RATE);
}

void updateControl(){


  MIDI.read();

  //TODO: The following should be in the MIDI control commands
  // compSin.next() returns a signed byte between -128 to 127 from the wave table
  if(OSCcomp) compVal = 128 + (0.99 * compSin.next());
  if(OSCdist) distVal = 128 + (0.99 * distSin.next());
  if(OSCgate) gateVal = 128 + (0.99 * gateSin.next());
  if(OSCstab) stabVal = 128 + (0.99 * stabSin.next());


  //spi_out(CS_signal, cmd_byteboth, compVal); 
  //FURTHEST from Arduino
  //With Arduino to the LEFT, byte0 is on the bottom, byte1 is on the top (CHECK THIS)
  spi_out(aCS_signal, cmd_byte0, stabVal,  aMOSI_signal, aCLK_signal); 
  spi_out(aCS_signal, cmd_byte1, compVal,  aMOSI_signal, aCLK_signal); 

  //NEAREST to Arduino
  spi_out(bCS_signal, cmd_byte0, gateVal,  bMOSI_signal, bCLK_signal); 
  spi_out(bCS_signal, cmd_byte1, distVal,  bMOSI_signal, bCLK_signal); 

  //analogWrite(LED_COMP_PIN, compVal);
  //analogWrite(LED_DIST_PIN, distVal);
  //analogWrite(LED_GATE_PIN, gateVal);
  //analogWrite(LED_STAB_PIN, stabVal);
  //void updateLED(byte PIN, byte r)
}


int updateAudio(){

  updateLED(LED_COMP_PIN, compVal);
  updateLED(LED_DIST_PIN, distVal);
  updateLED(LED_GATE_PIN, gateVal);
  updateLED(LED_STAB_PIN, stabVal);
  //updateLED(LED_STAB_PIN,stabVal);
  
  return 1;
}



void loop() {
  audioHook();
  
}

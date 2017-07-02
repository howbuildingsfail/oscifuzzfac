

/*********************************************************************************/
/*  PIN DEFINITIONS  *************************************************************/
/*********************************************************************************/


/* On Arduino we can use analogwrite on pins 3, 5, 6, 9, 10, and 11. 
 *  /
 *

 Suggest we avoid pins that Mozzi uses where possible for spi - then test the LED pins one at a time
 
 
 
 */




//TODO: we could #define these!
byte aCS_signal   = A4;//5;//2;                      // Chip Select signal on pin 5 of Arduino
byte aCLK_signal  = A5;//6;//4;                     // Clock signal on pin 6 of Arduino
byte aMOSI_signal = A6;//7;//5;                    // MOSI signal on pin 7 of Arduino


byte bCS_signal   = A0;//2;                      // Chip Select signal on pin 5 of Arduino
byte bCLK_signal  = A1;//3;                     // Clock signal on pin 6 of Arduino
byte bMOSI_signal = A2;//4;                    // MOSI signal on pin 7 of Arduino


//*Mozzi disables analogWrite() on pins 5 and 6 (Timer 0), 9 and 10 (Timer 1) in STANDARD mode. In HIFI mode, pins 3 and 11 (Timer 2) are also out. Pin numbers vary between boards.
//PIN 9 DOESN'T SEEM TO WOIK!work
//PIN 5 and 6 don't seem to 
#define LED_comp 11     //output flashing LED in time with the LFO rate for GATE pot
#define LED_dist 3      //output flashing LED in time with the LFO rate for GATE pot
#define LED_gate 10     //output flashing LED in time with the LFO rate for GATE pot
#define LED_stab 6    



/*********************************************************************************/
/*  http://sensorium.github.io/Mozzi/learn/a-simple-sketch/  *********************/
/*********************************************************************************/

#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <tables/saw2048_int8.h>

#define CONTROL_RATE (128)

Oscil <2048, CONTROL_RATE> compSin(SIN2048_DATA);
//Sketch uses 10,100 bytes (32%) of program storage space. Maximum is 30,720 bytes.
//Global variables use 1,009 bytes (49%) of dynamic memory, leaving 1,039 bytes for local variables. Maximum is 2,048 bytes.

Oscil <2048, CONTROL_RATE> gateSin(SIN2048_DATA);
Oscil <2048, CONTROL_RATE> distSin(SIN2048_DATA);
Oscil <2048, CONTROL_RATE> stabSin(SIN2048_DATA);

float depth=0.25;

/*********************************************************************************/
/* MIDI - from jamjar drumkit ****************************************************/
/*********************************************************************************/

#include <MIDI.h>

MIDI_CREATE_DEFAULT_INSTANCE();

#include <mozzi_midi.h>


void HandleNoteOn(byte channel, byte note, byte velocity) {
  
  switch (note) {
    case 24:
      break;
  }
}



void HandleNoteOff(byte channel, byte note, byte velocity) {
  
  switch (note) {
    case 24:
      break;
  }
}


void HandleControlChange (byte channel, byte number, byte value){
  switch (number){
    /***************************************************************/
    case 16:
      compSin.setFreq((float) 0.5 * value);    
      break;

    case 17:
      compSin.setFreq((float) 0.05 * value);    
      break;

      
    /***************************************************************/
    case 32:
      gateSin.setFreq((float) 0.5 * value);    
      break;
      
    case 33:
      gateSin.setFreq((float) 0.05 * value);    
      break;

      
    /***************************************************************/
    case 48:
      distSin.setFreq((float) 0.5 * value);    
      break;
      
    case 49:
      distSin.setFreq((float) 0.05 * value);    
      break;

    
  }
  
}





//TODO: we could #define these!
byte cmd_byte1    = B00010010 ;            // Command byte
byte cmd_byte0    = B00010001 ;            // Command byte
byte cmd_byteboth = B00010011 ;            // Command byte

int initial_value = 100;                // Setting up the initial value


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

  pinMode (LED_comp, OUTPUT);
  pinMode (LED_dist, OUTPUT);
  pinMode (LED_gate, OUTPUT);
  pinMode (LED_stab, OUTPUT);

  //TODO: Find good initial settings! - OR better still, send the midi control message for the default setting
  initialize_spi(aMOSI_signal,aCLK_signal);


  /* Freq: 6.5 is twice the speed of 3.5 */
  //use primes to init - get the range!
  compSin.setFreq(0.001f);//1.0f/2.0f);//(0.42f);
  distSin.setFreq(0.001f);//1.0f);//1.0f/3.0f);//(0.42f); //Doesn't seem to do anything!
  gateSin.setFreq(0.001f);//5.0f);//(0.42f);                This is STAB!! led 3 (l-r)
  stabSin.setFreq(4.001f);///7.0f);//(0.42f);             //This is comp


  // Initiate MIDI communications, listen to all channels
  MIDI.begin(MIDI_CHANNEL_OMNI);
  Serial.begin(38400);/* This baud rate is recommended for ALSA: http://alsa.opensrc.org/Serial  */

  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  MIDI.setHandleControlChange(HandleControlChange);
  
  startMozzi(CONTROL_RATE);
}


int compLFO,distLFO,gateLFO,stabLFO;

void updateControl(){


  MIDI.read();

  //TODO: The following should be in the MIDI control commands
  // compSin.next() returns a signed byte between -128 to 127 from the wave table
  compLFO = 128 + (0.99 * compSin.next());
  distLFO = 128 + (0.99 * distSin.next());
  gateLFO = 128 + (0.99 * gateSin.next());
  stabLFO = 128 + (0.99 * stabSin.next());


  //spi_out(CS_signal, cmd_byteboth, compLFO); 
  spi_out(aCS_signal, cmd_byte0, compLFO,  aMOSI_signal, aCLK_signal); 
  spi_out(aCS_signal, cmd_byte1, distLFO,  aMOSI_signal, aCLK_signal); 
  spi_out(bCS_signal, cmd_byte0, gateLFO,  bMOSI_signal, bCLK_signal); 
  spi_out(bCS_signal, cmd_byte1, stabLFO,  bMOSI_signal, bCLK_signal); 

  analogWrite(LED_comp, compLFO);
  analogWrite(LED_dist, distLFO);
  analogWrite(LED_gate, gateLFO);
  analogWrite(LED_stab, stabLFO);
  
}


int updateAudio(){
 
  
  return 1;
}



void loop() {
  audioHook();
  
}

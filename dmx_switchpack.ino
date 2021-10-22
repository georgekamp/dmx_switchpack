#include <EEPROM.h>
#include <Arduino.h>
#include <TM1637Display.h>
#include <Conceptinetics.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x3F, 20, 4);


// Rotary encoder declarations
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile int encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile int oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

// Button reading, including debounce without delay function declarations
const byte buttonPin = 4; // this is the Arduino pin we are connecting the push button to
byte oldButtonState = HIGH;  // assume switch open because of pull-up resistor
const unsigned long debounceTime = 10;  // milliseconds
unsigned long buttonPressTime;  // when the switch last changed state
boolean buttonPressed = 0; // a flag variable




//set the number of DMX channels to be controlled
#define DMX_SLAVE_CHANNELS  8

DMX_Slave dmx_slave ( DMX_SLAVE_CHANNELS ); // Configure a DMX slave controller

//define pins to control relays
const int CH1 = 8;
const int CH2 = 9;
const int CH3 = 10;
const int CH4 = 5;
const int CH5 = 6;
const int CH6 = 7;
const int CH7 = 11;
const int CH8 = 12;

//set up EEPROM
int address  = 1;  //DMX starting address
int addr = 0;   //EEPROM address
int a = 0;  //holds value for EEPROM data division
int b = 0;  //holds value for EEPROM data division
int val;  //stores value of dmx
int mapVal; //stores mapped value of dmx

void setup() {

  // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
 // lcd.backlight();
 lcd.setCursor(5,0);
 lcd.print("Booting");
 delay(500);
 lcd.setCursor(12,0);
 lcd.print(".");
 delay(500);
 lcd.setCursor(13,0);
 lcd.print(".");
 delay(500);
 lcd.setCursor(14,0);
 lcd.print(".");
 delay(500);
 
  //Rotary encoder section of setup
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

  // button section of setup
  pinMode (buttonPin, INPUT_PULLUP); // setup the button pin
  // Button reading with non-delay() debounce - thank you Nick Gammon!
  byte buttonState = digitalRead (buttonPin);
  if (buttonState != oldButtonState) {
    if (millis () - buttonPressTime >= debounceTime) { // debounce
      buttonPressTime = millis ();  // when we closed the switch
      oldButtonState =  buttonState;  // remember for next time
      if (buttonState == LOW) {
        //this means the button has been pressed
        buttonPressed = 1;
      }
      else {
        //this means the button is not currently pressed
        buttonPressed = 0;
      }
    }  // end if debounce time up
  } // end of state change


  // Enable DMX slave interface and start recording
  // DMX data
  dmx_slave.enable ();

  // Set relay pins as outputs
  pinMode (CH1, OUTPUT );
  pinMode (CH2, OUTPUT );
  pinMode (CH3, OUTPUT );
  pinMode (CH4, OUTPUT );
  pinMode (CH5, OUTPUT );
  pinMode (CH6, OUTPUT );
  pinMode (CH7, OUTPUT );
  pinMode (CH8, OUTPUT );

  // Load saved DMX address
  int a = EEPROM.read(0);
  int b = EEPROM.read(1);
  address = a * 256 + b;
  encoderPos = (address); //set encoder value to start at saved address
  dmx_slave.setStartAddress (encoderPos);
delay(1000);

lcd.clear();
lcd.setCursor(3,0);
lcd.print("DMX Switchpack");
}



///////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  rotaryMenu();

  int k;


//  // Show decimal numbers with/without leading zeros
//  bool lz = false;
//  for (uint8_t z = 0; z < 2; z++) {
//    for (k = 0; k < 10000; k += k * 4 + 7) {


lcd.setCursor(0,1);
lcd.print("ADR:"); 

if (encoderPos < 1) encoderPos = 1; //if the encoder is spun backwards quickly, set it to 1. Don't allow negative numbers

else{                                             //eliminate stray numbers when going down                                      
if (encoderPos < 10) lcd.print("   ");                  
else if (encoderPos < 100) lcd.print("  ");
else if (encoderPos < 1000) lcd.print(' ');
lcd.print(encoderPos, DEC);
}

//    }
 //   lz = true;
 // }
  //adjust DMX start address with encoder change
  dmx_slave.setStartAddress (encoderPos);

  //save DMX address to EEPROM and display "done" when set
  if (digitalRead (buttonPin) == LOW) {
    a = encoderPos / 256; //split DMX address into two parts so that it can fit in the EEPROM
    b = encoderPos % 256;
    EEPROM.write(0, a);     //store dmx address in EEPROM
    EEPROM.write(1, b);
    // Done!
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("Done");
    delay (1000);

  }
  {
    // If the channel comes above 10% the relay will switch on
    // and below 10% the relay will be turned off
    //REMEMBER: Relays are on when "LOW" and off when "HIGH"

    // NOTE:
    // getChannelValue is relative to the configured start address
    if ( dmx_slave.getChannelValue (1) > 25 ){
      lcd.setCursor(0,2);
      lcd.print("1:1");
      digitalWrite ( CH1, LOW );
    }
    else{
      digitalWrite ( CH1, HIGH );
      lcd.setCursor(0,2);
      lcd.print("1:0");
    }
    
    if ( dmx_slave.getChannelValue (2) > 25 ) {
      lcd.setCursor(5,2);
      lcd.print("2:1");
      digitalWrite ( CH2, LOW );
    }
    else{
      digitalWrite ( CH2, HIGH );
      lcd.setCursor(5,2);
      lcd.print("2:0");
    }
    
    if ( dmx_slave.getChannelValue (3) > 25 ) {
      digitalWrite ( CH3, LOW );
      lcd.setCursor(10,2);
      lcd.print("3:1");
    }
    else{
      digitalWrite ( CH3, HIGH );
      lcd.setCursor(10,2);
      lcd.print("3:0");
    }
    
    if ( dmx_slave.getChannelValue (4) > 25 ){
      digitalWrite ( CH4, LOW );
      lcd.setCursor(15,2);
      lcd.print("4:1");
    }
    else{
      digitalWrite ( CH4, HIGH );
      lcd.setCursor(15,2);
      lcd.print("4:0");
    }

    if ( dmx_slave.getChannelValue (5) > 25 ){
      digitalWrite ( CH5, LOW );
      lcd.setCursor(0,3);
      lcd.print("5:1");
    }
    else{
      digitalWrite ( CH5, HIGH );
      lcd.setCursor(0,3);
      lcd.print("5:0");
    }

        if ( dmx_slave.getChannelValue (6) > 25 ){
      digitalWrite ( CH6, LOW );
      lcd.setCursor(5,3);
      lcd.print("6:1");
    }
    else{
      digitalWrite ( CH6, HIGH );
      lcd.setCursor(5,3);
      lcd.print("6:0");
    }
        if ( dmx_slave.getChannelValue (7) > 25 ){
      digitalWrite ( CH7, LOW );
      lcd.setCursor(10,3);
      lcd.print("7:1");
    }
    else{
      digitalWrite ( CH7, HIGH );
      lcd.setCursor(10,3);
      lcd.print("7:0");
    }
        if ( dmx_slave.getChannelValue (8) > 25 ){
      digitalWrite ( CH8, LOW );
      lcd.setCursor(15,3);
      lcd.print("8:1");
    }
    else{
      digitalWrite ( CH8, HIGH );
      lcd.setCursor(15,3);
      lcd.print("8:0");
    }


    
  }

}

void rotaryMenu() { //This handles the bulk of the menu functions without needing to install/include/compile a menu library
  //Rotary encoder update display if turned
  if (oldEncPos != encoderPos) { // DEBUGGING
    oldEncPos = encoderPos;// DEBUGGING

    delay(10); //how quick to update the adr after pot change
  }
  //Do not allow encoder to read below 1 or above 508
 // if (encoderPos < 1) {
 //   encoderPos = 1;
 // }
  if (encoderPos > 508) {
    encoderPos = 508;
  }

}

//Rotary encoder interrupt service routine for one encoder pin
void PinA() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

//Rotary encoder interrupt service routine for the other encoder pin
void PinB() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}
// end of sketch!

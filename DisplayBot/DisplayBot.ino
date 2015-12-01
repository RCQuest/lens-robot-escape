/*
 * DisplayBot - displays facial expressions
 */

// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.  Pololu3pi.h includes all of the
// other Orangutan Arduino libraries that can be used to control the
// on-board hardware such as LCD, buzzer, and motor drivers.
#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>

// This include file allows data to be stored in program space.  The
// ATmega168 has 16k of program space compared to 1k of RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>

Pololu3pi robot;

const char WELCOME_TUNE[] PROGMEM = ">g32>>c32";


//Data to display custom emojis.
const char normalEye[] PROGMEM = {
  0b10101,
  0b10101,
  0b00100,
  0b01010,
  0b10101,
  0b10101,
  0b01010,
  0b00100
};

const char eyeLeft[] PROGMEM = {
  0b10101,
  0b10101,
  0b00100,
  0b01010,
  0b11001,
  0b11001,
  0b01010,
  0b00100
};

const char eyeRight[] PROGMEM = {
  0b10101,
  0b10101,
  0b00100,
  0b01010,
  0b10011,
  0b10011,
  0b01010,
  0b00100
};

const char nose[] PROGMEM = {
  0b01010,
  0b01010,
  0b00000,
  0b00000,
  0b00000,
  0b00100,
  0b01010,
  0b00100
};

const char smileLeftSide[] PROGMEM = {
  0b00000,
  0b00000,
  0b10000,
  0b10000,
  0b01000,
  0b00111,
  0b00000,
  0b00000
};

const char smileMiddle[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b00000,
  0b00000
};

const char smileRightSide[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00001,
  0b00001,
  0b00010,
  0b11100,
  0b00000,
  0b00000
};

const char tongueOut[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b01010,
  0b00111
};

const char NORMAL_EYE = 0;
const char SMILE_LEFT = 1;
const char SMILE_MIDDLE = 2;
const char SMILE_RIGHT = 3;
const char NOSE = 4;
const char TONGUE = 5;
const char EYE_LEFT = 6;
const char EYE_RIGHT = 7;

void loadFaceParts()
{
  OrangutanLCD::loadCustomCharacter(normalEye, NORMAL_EYE);
  OrangutanLCD::loadCustomCharacter(eyeLeft, EYE_LEFT);
  OrangutanLCD::loadCustomCharacter(eyeRight, EYE_RIGHT);
  OrangutanLCD::loadCustomCharacter(nose, NOSE);
  OrangutanLCD::loadCustomCharacter(smileLeftSide, SMILE_LEFT);
  OrangutanLCD::loadCustomCharacter(smileMiddle, SMILE_MIDDLE);
  OrangutanLCD::loadCustomCharacter(smileRightSide, SMILE_RIGHT);
  OrangutanLCD::loadCustomCharacter(tongueOut, TONGUE);
  OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect
}

void displayFace(char mood)
{
  OrangutanLCD::clear();
  OrangutanLCD::gotoXY(2, 0);
  OrangutanLCD::print(NORMAL_EYE);
  OrangutanLCD::print(NOSE);
  OrangutanLCD::print(NORMAL_EYE);
  OrangutanLCD::gotoXY(2, 1);
  OrangutanLCD::print(SMILE_LEFT);
//  OrangutanLCD::print(SMILE_MIDDLE);
  OrangutanLCD::print(TONGUE);
  OrangutanLCD::print(SMILE_RIGHT);
}

//Function used to quickly test functionality
void test()
{
  loadFaceParts();
  
  displayFace((char)0);
  
  while(true)
  {
    delay (10000);
  }
}

void setup()
{
    OrangutanBuzzer::playFromProgramSpace(WELCOME_TUNE);

    test();
}

void loop() {
}

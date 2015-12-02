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

const char sadEye[] PROGMEM = {
  0b10101,
  0b10101,
  0b00000,
  0b01110,
  0b10001,
  0b10101,
  0b01110,
  0b00100
};

const char tiredEye[] PROGMEM = {
  0b10101,
  0b10101,
  0b00000,
  0b00000,
  0b11111,
  0b10101,
  0b01010,
  0b01110
};

const char queezyEye[] PROGMEM = {
  0b10101,
  0b10101,
  0b00000,
  0b10001,
  0b01110,
  0b01010,
  0b01110,
  0b10001
};

const char boredEye[] PROGMEM = {
  0b10101,
  0b10101,
  0b00100,
  0b01110,
  0b10111,
  0b10001,
  0b01010,
  0b00100
};

const char concernedEyeLeft[] PROGMEM = {
  0b10101,
  0b10101,
  0b00100,
  0b01010,
  0b10001,
  0b11101,
  0b01110,
  0b00100
};

const char concernedEyeRight[] PROGMEM = {
  0b10101,
  0b10101,
  0b00100,
  0b01010,
  0b10001,
  0b10111,
  0b01110,
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

const char confusedEyeLeft[] PROGMEM = {
  0b10101,
  0b10101,
  0b00100,
  0b01010,
  0b10111,
  0b10111,
  0b01010,
  0b00100
};

const char confusedEyeRight[] PROGMEM = {
  0b10101,
  0b10101,
  0b00100,
  0b01010,
  0b11101,
  0b11101,
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

const char straightMouth[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b00000,
  0b00000
};

const char bigSmileLeftSide[] PROGMEM = {
  0b00000,
  0b00000,
  0b10000,
  0b10000,
  0b01000,
  0b00111,
  0b00000,
  0b00000
};

const char rightFurtiveSmileRightSide[] PROGMEM = {
  0b00001,  // the 5 bits that make up the top row of the 5x8 character
  0b00001,
  0b00010,
  0b00100,
  0b11000,
  0b00000,
  0b00000,
  0b00000
};

const char rightFurtiveSmileMiddle[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00011,
  0b01100,
  0b00000,
  0b00000
};

const char leftFurtiveSmileLeftSide[] PROGMEM = {
  0b10000,  // the 5 bits that make up the top row of the 5x8 character
  0b10000,
  0b01000,
  0b00100,
  0b00011,
  0b00000,
  0b00000,
  0b00000
};

const char leftFurtiveSmileMiddle[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b11000,
  0b01100,
  0b00000,
  0b00000
};

const char queezyMouthRightSide[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00001,
  0b00110,
  0b11000,
  0b00000,
  0b00000,
  0b00000
};

const char queezyMouthMiddle[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00011,
  0b11100,
  0b00000,
  0b00000
};

const char queezyMouthLeftSide[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00011,
  0b01100,
  0b10000
};

const char confusedMouthRight[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11100,
  0b00011,
  0b00000
};

const char confusedMouthMiddle[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b10011,
  0b01100
};

const char confusedMouthLeft[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b01100,
  0b10011,
  0b00000
};

const char bigSmileRightSide[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00001,
  0b00001,
  0b00010,
  0b11100,
  0b00000,
  0b00000
};

const char smallSmileLeftSide[] PROGMEM = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b10000,
  0b01111,
  0b00000,
  0b00000
};

const char smallSmileRightSide[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00001,
  0b11110,
  0b00000,
  0b00000
};

const char frownLeftSide[] PROGMEM = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00111,
  0b01000,
  0b10000
};

const char frownRightSide[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11100,
  0b00010,
  0b00001
};

const char tongueOut[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b01010,
  0b01110
};

const char wideOpenMouthMiddle[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b11111,
  0b00000,
  0b00000,
  0b00000,
  0b11111
};

const char wideOpenMouthLeft[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00001,
  0b00110,
  0b01000,
  0b00110,
  0b00001
};

const char wideOpenMouthRight[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b10000,
  0b01100,
  0b00010,
  0b01100,
  0b10000
};

const char massiveSmileLeft[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b01100,
  0b10011,
  0b10000,
  0b01000,
  0b00110,
  0b00001
};

const char massiveSmileRight[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00110,
  0b11001,
  0b00001,
  0b00010,
  0b01100,
  0b10000
};

const char blank[] PROGMEM = {
  0b00000,  // the 5 bits that make up the top row of the 5x8 character
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};


const char* cheeky[2][3] = {{normalEye, nose, normalEye}, {bigSmileLeftSide, tongueOut, bigSmileRightSide}};
const char* smiley[2][3] = {{normalEye, nose, normalEye}, {bigSmileLeftSide, straightMouth, bigSmileRightSide}};
const char* sad[2][3] = {{sadEye, nose, sadEye}, {frownLeftSide, straightMouth, frownRightSide}};
const char* furtiveLeft[2][3] = {{eyeLeft, nose, eyeLeft}, {leftFurtiveSmileLeftSide, leftFurtiveSmileMiddle, blank}};
const char* furtiveRight[2][3] = {{eyeRight, nose, eyeRight}, {blank, rightFurtiveSmileMiddle, rightFurtiveSmileRightSide}};
const char* bored[2][3] = {{boredEye, nose, boredEye}, {frownLeftSide, straightMouth, frownRightSide}};
const char* queezy[2][3] = {{queezyEye, nose, queezyEye}, {queezyMouthLeftSide, queezyMouthMiddle, queezyMouthRightSide}};
const char* surprised[2][3] = {{normalEye, nose, normalEye}, {wideOpenMouthLeft, wideOpenMouthMiddle, wideOpenMouthRight}};
const char* excited[2][3] = {{normalEye, nose, normalEye}, {massiveSmileLeft, wideOpenMouthMiddle, massiveSmileRight}};
const char* concerned[2][3] = {{concernedEyeLeft, nose, concernedEyeLeft}, {queezyMouthLeftSide, queezyMouthMiddle, queezyMouthRightSide}};
const char* confused[2][3] = {{confusedEyeLeft, nose, confusedEyeRight}, {confusedMouthLeft, confusedMouthMiddle, confusedMouthRight}};
const char* concentratingLeft[2][3] = {{concernedEyeLeft, nose, concernedEyeLeft}, {smallSmileLeftSide, straightMouth, smallSmileRightSide}};
const char* concentratingRight[2][3] = {{concernedEyeRight, nose, concernedEyeRight}, {smallSmileLeftSide, straightMouth, smallSmileRightSide}};

const char NORMAL_EYE = 0;
const char SMILE_LEFT = 1;
const char SMILE_MIDDLE = 2;
const char SMILE_RIGHT = 3;
const char NOSE = 4;
const char TONGUE = 5;
const char EYE_LEFT = 6;
const char EYE_RIGHT = 7;

void loadFaceParts(const char* face[2][3])
{
  for (unsigned int row = 0; row < 2; row++)
  {
    for (unsigned column = 0; column < 3; column++)
    {
      unsigned int pos = (row * 3) + column;
      OrangutanLCD::loadCustomCharacter(face[row][column], pos);
    }
  }

  OrangutanLCD::clear();
}

void displayFace(const char* face[2][3])
{
  loadFaceParts(face);
  
  for (unsigned int row = 0; row < 2; row++)
  {
    OrangutanLCD::gotoXY(2, row);
    
    for (unsigned column = 0; column < 3; column++)
    {
      unsigned int pos = (row * 3) + column;
      OrangutanLCD::print((char)pos);
    }
  }
}

//Function used to quickly test functionality
void test()
{
//  displayFace(sad);
//  delay(3000);

//  displayFace(cheeky);
//  delay(3000);
//  
//  displayFace(smiley);
//  delay(3000);
//
//  displayFace(furtiveLeft);
//  delay(2000);
//  displayFace(furtiveRight);
//  delay(2000);
//
//  displayFace(bored);
//  delay(3000);
//
//  displayFace(queezy);
//  delay(3000);
//
//  displayFace(surprised);
//  delay(3000);
//
//  displayFace(excited);
//  delay(3000);
//
//  displayFace(concerned);
//  delay(3000);
//
//  displayFace(confused);
//  delay(3000);
//  displayFace(concentratingLeft);
//  delay(1000);
//  displayFace(concentratingRight);
//  delay(1000);
//  displayFace(concentratingLeft);
//  delay(1000);
//  displayFace(concentratingRight);
//  delay(1000);

  displayFace(smiley);
  delay(3000);

  scrollText("Hey there!", "          I'm Chip Magnet");
  delay (1000);
  
  displayFace(smiley);
  delay(3000);

  scrollText("Hey there!", "          I'm The Line King");
  delay (1000);

  displayFace(smiley);
  delay(3000);

  scrollText("Hey there!", "          I'm Random Access");
  delay (1000);

  displayFace(smiley);
  delay(3000);

  scrollText("Let's have some   ", "                  FUN!");
  delay (1000);

  displayFace(cheeky);
  delay(3000);

  scrollText("Work is sooo boring", "");
  delay (1000);

  displayFace(bored);
  delay(3000);

  scrollText("", "Hooray! Lunchtime!");
  delay (1000);

  displayFace(smiley);
  delay(3000);

  scrollText("I wonder if", "         the boss is looking...");
  delay (1000);

  displayFace(furtiveLeft);
  delay(2000);
  displayFace(furtiveRight);
  delay(2000);

  scrollText("I'm outta here!", "");
  delay (1000);

  displayFace(cheeky);
  delay(3000);
  
  scrollText("Woo-hoo!", "");
  delay (1000);

  displayFace(excited);
  delay(3000);

  scrollText("Yee-haa!", "");
  delay (1000);

  displayFace(excited);
  delay(3000);

  scrollText("Euurrggghhh", "");
  scrollText("", "I'm feeling kinda queezy");
  delay (1000);

  displayFace(queezy);
  delay(3000);

  scrollText("", "...Steady...");
  delay (1000);

  displayFace(concentratingLeft);
  delay(1000);
  displayFace(concentratingRight);
  delay(1000);
  displayFace(concentratingLeft);
  delay(1000);
  displayFace(concentratingRight);
  delay(1000);

  scrollText("Hmm...", "      roadblock");
  delay (1000);

  displayFace(concerned);
  delay(3000);

  scrollText("Forget this...", "              shortcut!");
  delay (1000);
  
  displayFace(cheeky);
  delay(3000);
  
  scrollText("What goes up...", "");
  delay (1000);

  displayFace(smiley);
  delay(3000);

  scrollText("Hey! Who turned out the lights?!", "");
  delay (1000);

  displayFace(concerned);
  delay(3000);

  scrollText("", "I wonder where this goes...");
  delay (1000);

  displayFace(confused);
  delay(3000);

  scrollText("Whheeeeeeeeee!!!!!", "");
  delay (1000);

  displayFace(excited);
  delay(3000);

  OrangutanLCD::clear();
  OrangutanLCD::gotoXY(0, 0);
  OrangutanLCD::print("  That");
  delay(1000);

  OrangutanLCD::clear();
  OrangutanLCD::print("   was");
  delay(1000);

  OrangutanLCD::clear();
  OrangutanLCD::print("  EPIC!");
  delay(2000);

  displayFace(excited);
  delay(3000);

  scrollText("EPIC!", "");
  delay (1000);

  displayFace(excited);
  delay(3000);

  scrollText("", "Rock'n'roll, baby!");
  delay (1000);

  displayFace(smiley);
  delay(3000);

  scrollText("Dammit!", "       Lunch break over");
  delay (1000);

  displayFace(sad);
  delay(3000);
  
  scrollText("Oh well", "       back to work, I guess");
  delay (1000);

  displayFace(bored);
  delay(3000);
}

//Scrolls two lines of text
void scrollText(char *line1, char *line2)
{
  unsigned int textLength = max(strlen(line1), strlen(line2));
  
  OrangutanLCD::clear();
  OrangutanLCD::gotoXY(8, 0);
  OrangutanLCD::print(line1);
  OrangutanLCD::gotoXY(8, 1);
  OrangutanLCD::print(line2); 
  delay(200);

  for (unsigned int i = 0; i < textLength + 8; i++)
  {
    OrangutanLCD::scroll(LCD_LEFT, 1, 200);
  }
}

void setup()
{
    OrangutanBuzzer::playFromProgramSpace(WELCOME_TUNE);

    test();
}

void loop() {
}

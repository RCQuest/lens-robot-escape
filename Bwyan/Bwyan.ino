/*
 * Bwyan - E2 Robot Challenge Contestant
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

//When in Debug mode, the buzzer and LCD are used to help verify correct operation
//When in Fast Start mode, welcome message and calibration steps are skipped (useful for quick testing)
boolean inDebugMode = true;
boolean inFastStartMode = false;

Pololu3pi robot;

const unsigned int NUM_SENSORS = 5;
unsigned int sensors[NUM_SENSORS];

unsigned int last_proportional = 0;
long integral = 0;

const unsigned int MAX_SPEED = 100;

//To enable accurate verification of whether the sensors are over white or black,
//we check several sensor readings over a short period.
const unsigned int READING_HISTORY_LENGTH = 5;
const unsigned int READING_HISTORY_MAX = READING_HISTORY_LENGTH - 1;
unsigned int readingHistory[READING_HISTORY_LENGTH] = { 0, 0, 0, 0, 0 };

//If the total of all sensor readings in the recent history is less than this amount,
//we must be over a white section of line (aka. a "signal")
const unsigned int WHITE_THRESHOLD = READING_HISTORY_LENGTH * 300;
boolean onWhite = false;

//Maximum time between two "signals" that will count them as part of the same "message"
const unsigned int MESSAGE_WINDOW_TIMEOUT = 500; //milliseconds
unsigned int messageWindowExpiryTime;
boolean messageIsBeingReceived = false;
unsigned int message = 0; //the message currently being received
unsigned int lastMessage = 3; //the last complete message received

// Introductory messages.  The "PROGMEM" identifier causes the data to
// go into program space.
const char welcome_line1[] PROGMEM = "  I'm";
const char welcome_line2[] PROGMEM = " Bwyan";
const char demo_name_line1[] PROGMEM = "I follow";
const char demo_name_line2[] PROGMEM = " lines";

// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";
const char signalTune[] PROGMEM = "L16 c";
const char messageTune[] PROGMEM = "L16 ad";

// Data for generating the characters used in load_custom_characters
// and display_readings.  By reading levels[] starting at various
// offsets, we can generate all of the 7 extra characters needed for a
// bargraph.  This is also stored in program space.
const char levels[] PROGMEM = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

enum State  { INIT, START_WORK, FOLLOW_LINE, GET_BORED, CHECK_FOR_THE_BOSS, GO_OFF_ROAD, ENTER_CABLE_CAR, BALANCE_ON_BEAM, LOOP_THE_LOOP, BARREL_ROLL, RETURN_TO_WORK };
State state = INIT;
State previousState = INIT;
State nextState;

const unsigned int NEVER = 0;
const unsigned int MORNING_DURATION = 6000;
const unsigned int BORED_DURATION = 3000;
const unsigned int CHECK_FOR_BOSS_PAUSE = 1000;
unsigned int nextTransitionTime = NEVER;

// This function loads custom characters into the LCD.  Up to 8
// characters can be loaded; we use them for 7 levels of a bar graph.
void load_custom_characters()
{
  OrangutanLCD::loadCustomCharacter(levels + 0, 0); // no offset, e.g. one bar
  OrangutanLCD::loadCustomCharacter(levels + 1, 1); // two bars
  OrangutanLCD::loadCustomCharacter(levels + 2, 2); // etc...
  OrangutanLCD::loadCustomCharacter(levels + 3, 3);
  OrangutanLCD::loadCustomCharacter(levels + 4, 4);
  OrangutanLCD::loadCustomCharacter(levels + 5, 5);
  OrangutanLCD::loadCustomCharacter(levels + 6, 6);
  OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect
}

// This function displays the sensor readings using a bar graph.
void display_readings(const unsigned int *calibrated_values)
{
  unsigned char i;

  for (i=0;i<5;i++) {
    // Initialize the array of characters that we will use for the
    // graph.  Using the space, an extra copy of the one-bar
    // character, and character 255 (a full black box), we get 10
    // characters in the array.
    const char display_characters[10] = { ' ', 0, 0, 1, 2, 3, 4, 5, 6, 255 };

    // The variable c will have values from 0 to 9, since
    // calibrated values are in the range of 0 to 1000, and
    // 1000/101 is 9 with integer math.
    char c = display_characters[calibrated_values[i] / 101];

    // Display the bar graph character.
    OrangutanLCD::print(c);
  }
}

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.  This function is automatically called
// by the Arduino framework at the start of program execution.
void setup()
{
  unsigned int counter; // used as a simple timer

  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  robot.init(2000);

  load_custom_characters(); // load the custom characters

  OrangutanBuzzer::playFromProgramSpace(welcome);

  if (!inFastStartMode)
  {
    // Play welcome music and display a message
    OrangutanLCD::printFromProgramSpace(welcome_line1);
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::printFromProgramSpace(welcome_line2);
    delay(1000);

    OrangutanLCD::clear();
    OrangutanLCD::printFromProgramSpace(demo_name_line1);
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::printFromProgramSpace(demo_name_line2);
    delay(1000);
  }

  // Display battery voltage and wait for button press
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    int bat = OrangutanAnalog::readBatteryMillivolts();

    OrangutanLCD::clear();
    OrangutanLCD::print(bat);
    OrangutanLCD::print("mV");
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print("Press B");

    delay(100);
  }

  // Always wait for the button to be released so that 3pi doesn't
  // start moving until your hand is away from it.
  OrangutanPushbuttons::waitForRelease(BUTTON_B);

  if (!inFastStartMode)
  {
    delay(1000);
    
    // Auto-calibration: turn right and left while calibrating the
    // sensors.
    for (counter=0; counter<80; counter++)
    {
      if (counter < 20 || counter >= 60)
        OrangutanMotors::setSpeeds(40, -40);
      else
        OrangutanMotors::setSpeeds(-40, 40);
  
      // This function records a set of sensor readings and keeps
      // track of the minimum and maximum values encountered.  The
      // IR_EMITTERS_ON argument means that the IR LEDs will be
      // turned on during the reading, which is usually what you
      // want.
      robot.calibrateLineSensors(IR_EMITTERS_ON);
  
      // Since our counter runs to 80, the total delay will be
      // 80*20 = 1600 ms.
      delay(20);
    }
    OrangutanMotors::setSpeeds(0, 0);
  
    // Display calibrated values as a bar graph.
    while (!OrangutanPushbuttons::isPressed(BUTTON_B))
    {
      // Read the sensor values and get the position measurement.
      unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
  
      // Display the position measurement, which will go from 0
      // (when the leftmost sensor is over the line) to 4000 (when
      // the rightmost sensor is over the line) on the 3pi, along
      // with a bar graph of the sensor readings.  This allows you
      // to make sure the robot is ready to go.
      OrangutanLCD::clear();
      OrangutanLCD::print(position);
      OrangutanLCD::gotoXY(0, 1);
      display_readings(sensors);
  
      delay(100);
    }
    OrangutanPushbuttons::waitForRelease(BUTTON_B);
  }
  
  OrangutanLCD::clear();
  OrangutanLCD::print("Go!");    

  // Play music and wait for it to finish before we start driving.
  OrangutanBuzzer::playFromProgramSpace(go);
  while(OrangutanBuzzer::isPlaying());

  state = START_WORK;
}

// The main function.  This function is repeatedly called by
// the Arduino framework.
void loop()
{
  if (nextTransitionTime != NEVER && millis() > nextTransitionTime)
  {
    state = nextState;
    nextTransitionTime = NEVER;
  }

  if (messageReceived())
  {
    if (inDebugMode)
    {
      messageBeep();
      displayLastMessage();
    }
  }

  switch(state)
  {
    case START_WORK: startWork(); break;
    case FOLLOW_LINE: followLine(); break;
    case GET_BORED: getBored(); break;
    case CHECK_FOR_THE_BOSS: checkForTheBoss(); break;
    case GO_OFF_ROAD: goOffRoad(); break;
    case ENTER_CABLE_CAR: break;
    case BALANCE_ON_BEAM: break;
    case LOOP_THE_LOOP: break;
    case BARREL_ROLL: break;
    case RETURN_TO_WORK: returnToWork(); break;
    default: break;
  }
}


///////////////////////STATES////////////////////////

//Until lunchtime, we will follow the line.
void startWork()
{
  displayState("StartWrk");
  
  state = FOLLOW_LINE;
//  nextState = GET_BORED;
//  transitionAfter(MORNING_DURATION);
}

//Tracks a black line on a light background and monitors for
//signals (which trigger state changes)
void followLine() {
  //TODO: Implement PID control (if Andrew Mason didn't already do this)
  displayState("FollowLn");
  
  // Get the position of the line.  Note that we *must* provide
  // the "sensors" argument to read_line() here, even though we
  // are not interested in the individual sensor readings.
  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

  unsigned int sensorSum = sum(sensors, NUM_SENSORS);
  addToSensorReadingHistory(sensorSum);
  checkForSignal();

//  OrangutanLCD::clear();
//  OrangutanLCD::print("SensorSum");
//  OrangutanLCD::gotoXY(0, 1);
//  OrangutanLCD::print(sensorSum);

  // PID line follower
  // The "proportional" term should be 0 when we are on the line.
  int proportional = (int)position - 2000;

  // Compute the derivative (change) and integral (sum) of the
  // position.
  int derivative = proportional - last_proportional;
  integral += proportional;

  // Remember the last position.
  last_proportional = proportional;

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.  You can adjust the constants by which
  // the proportional, integral, and derivative terms are multiplied to
  // improve performance.
  int power_difference = proportional/20 + integral/10000 + derivative*3/2;

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  const int maximum = MAX_SPEED;
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;

  if (position == 0 || position == 4000)
  {
    //If we see no line at all, just go straight
    OrangutanMotors::setSpeeds(maximum, maximum);
  }
  else
  {
    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);
  }
}

void getBored()
{
  displayState("GetBored");
  state = FOLLOW_LINE;
  nextState = CHECK_FOR_THE_BOSS;
  transitionAfter(BORED_DURATION);
}

//'Looks' left and right
void checkForTheBoss() {
  displayState("ChkBoss");

  unsigned int counter;
  
  for (counter=0; counter<80; counter++) {
    if (counter < 20 || counter >= 60) {
      OrangutanMotors::setSpeeds(-40, 40);
    }
    else {
      OrangutanMotors::setSpeeds(40, -40);
    }
  } 
  
  OrangutanMotors::setSpeeds(0, 0);

  delay (CHECK_FOR_BOSS_PAUSE);  //dramatic pause

  state = GO_OFF_ROAD;
}

//Turns then drives at high speed in a straight line (unmarked) until it finds a new line
void goOffRoad()
{
  displayState("GoOffRd");

  OrangutanMotors::setSpeeds(40, -40);

  //TODO: implement high-speed escape, and everything thereafter

  delay(1000);

  state = RETURN_TO_WORK;
}

void returnToWork()
{
    displayState("Rtn2Wrk");

    OrangutanMotors::setSpeeds(0, 0);
}


/////////////////UTILITIES/////////////////////

//Set time at which next transition will happen
void transitionAfter(unsigned int duration)
{
  nextTransitionTime = millis() + duration;
}

//Add an array of integers together
unsigned int sum(unsigned int values[], int numOfValues)
{
  unsigned int sum = 0;
  
  for (unsigned int valueNum = 0; valueNum < numOfValues; valueNum++)
  {
    sum += values[valueNum];
  }

  return sum;
}

//Add the latest sensor reading to the reading history
void addToSensorReadingHistory(unsigned int sensorSum)
{
  shiftBackReadingHistory();
  readingHistory[0] = sensorSum;
}

//Shift all recent sensor readings back one place in the reading history
void shiftBackReadingHistory()
{
  for (unsigned int reading = READING_HISTORY_MAX; reading > 0; reading--)
  {
    readingHistory[reading] = readingHistory[reading - 1];
  }
}

//Based on the recent history of sensor readings, decide if we are over a "signal" section of track
//and capture the signal if we are
void checkForSignal()
{
  if (sum(readingHistory, READING_HISTORY_LENGTH) < WHITE_THRESHOLD)
  {
    handleSignal(true);
  }
  else
  {
    handleSignal(false);
  }
}

//Handle the receipt of a signal.  There are two cases:
//  - We enter a white section of track (signal start)
//  - We leave a white section of track (signal end)
void handleSignal(boolean isWhite)
{
    //We have transitioned from black to white (signal start)
    if (!onWhite && isWhite)
    {
      updateMessage();
      resetMessageTimer();
      onWhite = true;

      if (inDebugMode)
      {
        signalBeep();
        displayLastMessage();
      }
    }

    //We have transitioned from white to black (signal end)
    if (onWhite && !isWhite)
    {
      onWhite = false;
    }
}

//A message will be a count of signals received within the message window
void updateMessage()
{
  if (messageIsBeingReceived)
  {
    message++;
  }
  else
  {
    messageIsBeingReceived = true;
    message = 1;
  }
}

//If a period of time greater than MESSAGE_WINDOW_TIMEOUT elapses between two
//signals being received, we consider it to be the end of the message.
//In that case, a further signal would be treated as the start of a new message
boolean messageReceived()
{
  if (messageIsBeingReceived && millis() > messageWindowExpiryTime)
  {
    messageIsBeingReceived = false;
    lastMessage = message;
    return true;
  }

  return false;
}

//Whenever a signal is received, we reset the message timer.
//In this way, a message may consist of any number of signals.
void resetMessageTimer()
{
  messageWindowExpiryTime = millis() + MESSAGE_WINDOW_TIMEOUT;
}

//Plays a beep to indicate receipt of a signal
void signalBeep()
{
  OrangutanBuzzer::playFromProgramSpace(signalTune);
}

//Plays a beep to indicate receipt of a message
void messageBeep()
{
//  for (unsigned int beepNum = 0; beepNum < lastMessage; beepNum++)
//  {
    OrangutanBuzzer::playFromProgramSpace(messageTune);
//    while(OrangutanBuzzer::isPlaying());  //allow one beep to complete before playing the next
//  }
}

//Displays the current (partial) "message" (number of signals within the message window) on the LCD
void displayPartialMessage()
{
  OrangutanLCD::clear();
  OrangutanLCD::print("Message: ");
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::print(message);
}

//Displays the last complete message received on the LCD
void displayLastMessage()
{
  OrangutanLCD::clear();
  OrangutanLCD::print("Last Msg: ");
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::print(lastMessage);
}

//Displays the current state when in debug mode
//TODO: have the enum store brief state string for each state and use that within this function
void displayState(char *briefState)
{
  if (inDebugMode && stateHasChanged())
  {
    OrangutanLCD::clear();
    OrangutanLCD::print("State: ");
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print(briefState);
  }
}

//Checks if the state has changed since the last time it was called
boolean stateHasChanged()
{
  if (state != previousState)
  {
    previousState = state;
    return true;
  }

  return false;
}


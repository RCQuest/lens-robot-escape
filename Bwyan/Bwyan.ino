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
//Use the other flags to skip 'niceties' for rapid testing
boolean inDebugMode = true;
boolean skipWelcome = true;
boolean skipCalibration = false;

Pololu3pi robot;

const unsigned int NUM_SENSORS = 5;
unsigned int sensors[NUM_SENSORS];

//PID (Proportional, Integral, Differential) control parameters
unsigned int lastProportional = 0;
long integral = 0;

const unsigned int TURBO_MAX_SPEED = 200;
const unsigned int FAST_MAX_SPEED = 200;
const unsigned int NORMAL_MAX_SPEED = 100;
const unsigned int SLOW_MAX_SPEED = 50;
const unsigned int CAREFUL_MAX_SPEED = 30;
const unsigned int STOP = 0;
unsigned int targetSpeed = FAST_MAX_SPEED;

//To enable accurate verification of whether the sensors are over signal or black,
//we check several sensor readings over a short period.
const unsigned int READING_HISTORY_LENGTH = 5;
unsigned int readingHistory[READING_HISTORY_LENGTH];
unsigned int nextReading = 0;

//Set the threshold below which a sensor reading will be assumed to indicate a "white" section of track
//(Sensor readings are between 0 and 1000)
unsigned int WHITE_THRESHOLD = 200;

//If SIGNAL_THRESHOLD out of READING_HISTORY_LENGTH sensor readings appear to be signals, we
//must be over a signal section of line
const unsigned int SIGNAL_TOLERANCE = 2;
const unsigned int SIGNAL_THRESHOLD = READING_HISTORY_LENGTH - SIGNAL_TOLERANCE;
boolean onSignal = false;

//Maximum 'distance' between two signals such that they are counted as part of the same message
//This 'distance' is in undefined units, but we want it to remain constant, regardless of speed
//A value of 30 is calculated on the basis that we wish the max time between signals to be 300ms at "100" (normal) speed
//time = distance / speed   [0.3 = MAX_MESSAGE_DISTANCE / 100]
const unsigned int MAX_MESSAGE_DISTANCE = 30;
unsigned long messageWindowExpiryTime;

enum MessageCode  {
                    SET_SPEED = 1, 
                    SET_SPEED_REVERSE = 1,
                    SET_SPEED_CAREFUL = 2,
                    SET_SPEED_SLOW = 3,
                    SET_SPEED_NORMAL = 4,
                    SET_SPEED_FAST = 5,
                    SET_SPEED_TURBO = 6,
                    STOP_REVERSING = 1,

                    CORNER_AHEAD = 2,
                    LEFT_TURN = 1,
                    RIGHT_TURN = 2,
                    END_OF_CORNER = 1,

                    OBSTACLE_AHEAD = 3,
                    ZIP_WIRE = 1,
                    END_OF_OBSTACLE = 1                                        
                  };
                
enum MessageState { 
                    READY_TO_RECEIVE,

                    AWAITING_SPEED_INFO,
                    REVERSING,
                    
                    AWAITING_CORNER_INFO,
                    TURNING_LEFT,
                    TURNING_RIGHT,

                    AWAITING_OBSTACLE_INFO,
                    ON_ZIP_WIRE
                  };

const unsigned int MSG_SLOW = 2;
const unsigned int MSG_TURBO = 3;
const unsigned int MSG_RETURN_TO_WORK = 5;
boolean messageIsBeingReceived = false;
unsigned int message = 0; //the message currently being received
unsigned int lastMessage = 0; //the last complete message received
MessageState messageState = READY_TO_RECEIVE;

//All possible states
//Set state and previousState to "TEST" for rapid development using the test() function
enum State  {
              TEST,
              START_WORK, 
              FOLLOW_LINE, 
              GET_BORED, 
              CHECK_FOR_THE_BOSS, 
              GO_OFF_ROAD, 
              ENTER_CABLE_CAR, 
              BALANCE_ON_BEAM, 
              FOLLOW_LINE_REVERSE, 
              LOOP_THE_LOOP, 
              BARREL_ROLL, 
              RETURN_TO_WORK,
              SHUTDOWN
            };

// change this to initial state
/*State state = TEST;
State previousState = TEST;*/
State state = FOLLOW_LINE_REVERSE;
State previousState = FOLLOW_LINE_REVERSE;
State nextState;

//Time-based state transition control
const unsigned long NEVER = 0;
const unsigned long MORNING_DURATION = 6000;
const unsigned long BORED_DURATION = 3000;
const unsigned long CHECK_FOR_BOSS_PAUSE = 1000;
unsigned long nextTransitionTime = NEVER;

// Introductory messages.  The "PROGMEM" identifier causes the data to
// go into program space.
const char WELCOME_LINE_1[] PROGMEM = "  I'm";
const char WELCOME_LINE_2[] PROGMEM = " Bwyan";
const char WELCOME_LINE_3[] PROGMEM = "I follow";
const char WELCOME_LINE_4[] PROGMEM = " lines";

// A couple of simple tunes, stored in program space.
const char WELCOME_TUNE[] PROGMEM = ">g32>>c32";
const char GO_TUNE[] PROGMEM = "L16 cdegreg4";
const char SIGNAL_TUNE[] PROGMEM = "L16 c";
const char MESSAGE_TUNE[] PROGMEM = "L16 ad";

// Data for generating the characters used in load_custom_characters
// and display_readings.  By reading levels[] starting at various
// offsets, we can generate all of the 7 extra characters needed for a
// bargraph.  This is also stored in program space.
const char LEVELS[] PROGMEM = {
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


///////////////SETUP PROCEDURE//////////////////

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.  This function is automatically called
// by the Arduino framework at the start of program execution.
void setup()
{
  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  robot.init(2000);
  
  loadCustomCharacters(); // load the custom characters

  clearSensorReadingHistory();

  OrangutanBuzzer::playFromProgramSpace(WELCOME_TUNE);
  
  displayWelcomeMessage();
  displayBatteryVoltage();
  calibrateSensors();
  setTargetSpeed();

  OrangutanLCD::clear();
  OrangutanLCD::print("Go!");    

  // Play music and wait for it to finish before we start driving.
  OrangutanBuzzer::playFromProgramSpace(GO_TUNE);
  while(OrangutanBuzzer::isPlaying());
}

// This function loads custom characters into the LCD.  Up to 8
// characters can be loaded; we use them for 7 levels of a bar graph.
void loadCustomCharacters()
{
  OrangutanLCD::loadCustomCharacter(LEVELS + 0, 0); // no offset, e.g. one bar
  OrangutanLCD::loadCustomCharacter(LEVELS + 1, 1); // two bars
  OrangutanLCD::loadCustomCharacter(LEVELS + 2, 2); // etc...
  OrangutanLCD::loadCustomCharacter(LEVELS + 3, 3);
  OrangutanLCD::loadCustomCharacter(LEVELS + 4, 4);
  OrangutanLCD::loadCustomCharacter(LEVELS + 5, 5);
  OrangutanLCD::loadCustomCharacter(LEVELS + 6, 6);
  OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect
}

//Display welcome message on LCD at startup.  For fast start during rapid testing, this can be
//skipped by setting skipWelcome to true
void displayWelcomeMessage()
{
  if (!skipWelcome)
  {
    OrangutanLCD::printFromProgramSpace(WELCOME_LINE_1);
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::printFromProgramSpace(WELCOME_LINE_2);
    delay(1000);

    OrangutanLCD::clear();
    OrangutanLCD::printFromProgramSpace(WELCOME_LINE_3);
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::printFromProgramSpace(WELCOME_LINE_4);
    delay(1000);
  }
}

//Display battery voltage on LCD until a button is pressed
void displayBatteryVoltage()
{
  // Display battery voltage and wait for button press
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    int batteryVoltage = OrangutanAnalog::readBatteryMillivolts();

    OrangutanLCD::clear();
    OrangutanLCD::print(batteryVoltage);
    OrangutanLCD::print("mV");
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print("Press B");

    delay(100);
  }
}

//Adjust sensor sensitivity based on reflectance of the track surface.
//Can be skipped during rapid testing by setting the skipCalibration parameter to true.
void calibrateSensors()
{
  if (!skipCalibration)
  {
    unsigned int counter; // used as a simple timer

    // Always wait for the button to be released so that 3pi doesn't
    // start moving until your hand is away from it.
    OrangutanPushbuttons::waitForRelease(BUTTON_B);
    delay(1000);
    
    // Auto-calibration: turn right and left while calibrating the
    // sensors.
    for (counter = 0; counter < 80; counter++)
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
      // 80 * 20 = 1600 ms.
      delay(20);
    }

    OrangutanMotors::setSpeeds(0, 0);
  
    // Display calibrated values as a bar graph
    while (!OrangutanPushbuttons::isPressed(ANY_BUTTON))
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
      displayReadings(sensors);
  
      delay(100);
    }
  }
}

//Allows the user to specify whether to start at a slow, normal or fast speed
//depending on which button is pressed post-calibration
void setTargetSpeed()
{
  if (OrangutanPushbuttons::isPressed(BUTTON_A))
  {
    targetSpeed = SLOW_MAX_SPEED;
    OrangutanPushbuttons::waitForRelease(BUTTON_A);
  }
  else if (OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    targetSpeed = NORMAL_MAX_SPEED;
    OrangutanPushbuttons::waitForRelease(BUTTON_B);
  }
  else if (OrangutanPushbuttons::isPressed(BUTTON_C))
  {
    targetSpeed = FAST_MAX_SPEED;
    OrangutanPushbuttons::waitForRelease(BUTTON_C);
  }
}

// This function displays the sensor readings using a bar graph
void displayReadings(const unsigned int *calibrated_values)
{
  unsigned char sensorNum;

  for (sensorNum = 0; sensorNum < NUM_SENSORS; sensorNum++)
  {
    // Initialize the array of characters that we will use for the
    // graph.  Using the space, an extra copy of the one-bar
    // character, and character 255 (a full black box), we get 10
    // characters in the array.
    const char DISPLAY_CHARACTERS[10] = { ' ', 0, 0, 1, 2, 3, 4, 5, 6, 255 };

    // The variable c will have values from 0 to 9, since
    // calibrated values are in the range of 0 to 1000, and
    // 1000/101 is 9 with integer math.
    char character = DISPLAY_CHARACTERS[calibrated_values[sensorNum] / 101];

    // Display the bar graph character.
    OrangutanLCD::print(character);
  }
}



///////////////MAIN LOOP//////////////////

// The main function.  This function is repeatedly called by
// the Arduino framework.
void loop()
{
  processTimerActions();
  processMessages();

  switch(state)
  {
    case TEST: test(); break;
    case START_WORK: startWork(); break;
    case FOLLOW_LINE: followLine(); break;
    case GET_BORED: getBored(); break;
    case CHECK_FOR_THE_BOSS: checkForTheBoss(); break;
    case GO_OFF_ROAD: goOffRoad(); break;
    case ENTER_CABLE_CAR: break;
    case BALANCE_ON_BEAM: break; // followLine or PIDfollowline() ?
    case FOLLOW_LINE_REVERSE: followLineInReverse();
    case LOOP_THE_LOOP: break;
    case BARREL_ROLL: break;
    case RETURN_TO_WORK: returnToWork(); break;
    case SHUTDOWN: finish(); break;
    default: break;
  }
}

//Performs a state transition if a state transition timer has expired
void processTimerActions()
{
  if (nextTransitionTime != NEVER && millis() > nextTransitionTime)
  {
    state = nextState;
    nextTransitionTime = NEVER;
  }
}

//Decodes messages as they are received and takes action (changes state) accordingly
void processMessages()
{
  if (messageReceived())
  {
    if (inDebugMode)
    {
      messageBeep();
//      displayLastMessage();
    }

    switch(messageState)
    {
      case READY_TO_RECEIVE:
      {
        switch(lastMessage)
        {
          case SET_SPEED: displayMessageState("SetSpeed"); messageState = AWAITING_SPEED_INFO; break;
          case CORNER_AHEAD: displayMessageState("Corner"); messageState = AWAITING_CORNER_INFO; break;
          case OBSTACLE_AHEAD: displayMessageState("Obstacle"); messageState = AWAITING_CORNER_INFO; break;
          default: break;
        }
      }; break;

///SPEED CONTROL
      case AWAITING_SPEED_INFO:
      {
        switch(lastMessage)
        {
          case SET_SPEED_REVERSE: displayMessageState("Reverse"); messageState = REVERSING; break;
          case SET_SPEED_CAREFUL: displayMessageState("Careful"); messageState = READY_TO_RECEIVE; break;
          case SET_SPEED_SLOW: displayMessageState("Slow"); messageState = READY_TO_RECEIVE; break;
          case SET_SPEED_NORMAL: displayMessageState("Normal"); messageState = READY_TO_RECEIVE; break;
          case SET_SPEED_FAST: displayMessageState("Fast"); messageState = READY_TO_RECEIVE; break;
          case SET_SPEED_TURBO: displayMessageState("Turbo"); messageState = READY_TO_RECEIVE; break;
          default: break;
        }
      }; break;

      case REVERSING:
      {
        switch(lastMessage)
        {
          case STOP_REVERSING: displayMessageState("EndRevrs"); messageState = READY_TO_RECEIVE; break;
          default: displayMessageState("UNEXPCTD"); state = SHUTDOWN; break;   //Unexpected message - STOP!
        }
      }; break;


///CORNERING
      case AWAITING_CORNER_INFO:
      {
        switch(lastMessage)
        {
          case LEFT_TURN: displayMessageState("Left"); messageState = TURNING_LEFT; break;
          case RIGHT_TURN: displayMessageState("Right"); messageState = TURNING_RIGHT; break;
          default: break;
        }
      }; break;

      case TURNING_LEFT:
      case TURNING_RIGHT:
      {
        switch(lastMessage)
        {
          case END_OF_CORNER: displayMessageState("EndCornr"); messageState = READY_TO_RECEIVE; break;
          default: displayMessageState("UNEXPCTD"); state = SHUTDOWN; break;   //Unexpected message - STOP!
        }
      }; break;


///OBSTACLE-SPECIFIC DYNAMIC CONFIGURATION
      case AWAITING_OBSTACLE_INFO:
      {
        switch(lastMessage)
        {
          case ZIP_WIRE: displayMessageState("ZipWire"); messageState = ON_ZIP_WIRE; break;
          default: displayMessageState("UNEXPCTD"); state = SHUTDOWN; break;   //Unexpected message - STOP!
        }
      }; break;

      case ON_ZIP_WIRE:
      {
        switch(lastMessage)
        {
          case END_OF_OBSTACLE: displayMessageState("EndObstl"); messageState = READY_TO_RECEIVE; break;
          default: displayMessageState("UNEXPCTD"); state = SHUTDOWN; break;   //Unexpected message - STOP!
        }
      }; break;

      default: break;
    }
  }
}


///////////////////////STATES////////////////////////

//Until lunchtime, we will follow the line.
void startWork()
{
  displayState("StartWrk");
  
  state = FOLLOW_LINE;
  nextState = GET_BORED;
  transitionAfter(MORNING_DURATION);
}

//Tracks a black line on a light background and monitors for
//signals (which trigger state changes)
void followLine() {
  displayState("FollowLn");
  
  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

  if (readingIndicatesSignal())
  {
    addToSensorReadingHistory(1);
  }
  else
  {
    addToSensorReadingHistory(0);
  }

  checkForSignal();

  // PID line follower
  // The "proportional" term should be 0 when we are on the line.
  int proportional = (int)position - 2000;

  // Compute the derivative (change) and integral (sum) of the
  // position.
  int derivative = proportional - lastProportional;
  integral += proportional;

  // Remember the last position.
  lastProportional = proportional;

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
  const int maximum = targetSpeed;
  
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

//'Looks' left and right, in a furtive fashion
void checkForTheBoss() {
  displayState("ChkBoss");

  unsigned int counter;
  
  for (counter = 0; counter < 80; counter++) {
    if (counter < 20 || counter >= 60)
    {
      OrangutanMotors::setSpeeds(-20, 20);
    }
    else
    {
      OrangutanMotors::setSpeeds(20, -20);
    }

    delay(20); //80 * 20 = 1600ms
  } 
  
  OrangutanMotors::setSpeeds(0, 0);

  delay (CHECK_FOR_BOSS_PAUSE);  //dramatic pause

  state = GO_OFF_ROAD;
}

//Turns then drives at high speed in a straight line (unmarked) until it finds a new line
void goOffRoad()
{
  displayState("GoOffRd");

  OrangutanMotors::setSpeeds(30, -30);

  //TODO: implement high-speed escape, and everything thereafter

  delay(300);

  state = RETURN_TO_WORK;
}

void followLineInReverse() {
  displayState("Reversing");

  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

  if (readingIndicatesSignal())
  {
    addToSensorReadingHistory(1);
  }
  else
  {
    addToSensorReadingHistory(0);
  }

  checkForSignal();

  // PID line follower
  // The "proportional" term should be 0 when we are on the line.
  int proportional = (int)position - 2000;

  // Compute the derivative (change) and integral (sum) of the
  // position.
  int derivative = proportional - lastProportional;
  integral += proportional;

  // Remember the last position.
  lastProportional = proportional;

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.  You can adjust the constants by which
  // the proportional, integral, and derivative terms are multiplied to
  // improve performance.
  int power_difference = proportional/20; // + integral/10000 + derivative*3/2;

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  const int maximum = targetSpeed;
  
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)  
    power_difference = -maximum;

  if (position == 0 || position == 4000)
  {
    //If we see no line at all, just go straight
    OrangutanMotors::setSpeeds(-maximum, -maximum);
  }
  else
  {
    if (power_difference < 0)
      OrangutanMotors::setSpeeds(-1 * (maximum + power_difference), -1 * maximum);
    else
      OrangutanMotors::setSpeeds(-1 * maximum, -1 * (maximum - power_difference));
  }
}

void returnToWork()
{
  displayState("Rtn2Wrk");

  //TODO: go back to boring line-following mode
  state = SHUTDOWN;
}

void finish()
{
  OrangutanMotors::setSpeeds(0, 0);
}


//Function used to quickly test functionality
void test()
{
  displayState("Test");
  
  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

  if (readingIndicatesSignal())
  {
    addToSensorReadingHistory(1);
  }
  else
  {
    addToSensorReadingHistory(0);
  }

  checkForSignal();

  if (!onSignal)
  {
  // PID line follower
  // The "proportional" term should be 0 when we are on the line.
  int proportional = (int)position - 2000;

  // Compute the derivative (change) and integral (sum) of the
  // position.
  int derivative = proportional - lastProportional;
  integral += proportional;

  // Remember the last position.
  lastProportional = proportional;

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.  You can adjust the constants by which
  // the proportional, integral, and derivative terms are multiplied to
  // improve performance.
  int power_difference = proportional/20; // + integral/10000 + derivative*3/2;

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  const int maximum = targetSpeed;
  
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
}


/////////////////UTILITIES/////////////////////

//Set time at which next transition will happen
void transitionAfter(unsigned long duration)
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

//Return true if the sensor is above a "white" section of track
boolean isWhite(unsigned int sensorReading)
{
  return sensorReading < WHITE_THRESHOLD;
}

//Return true if the sensor is above a "black" section of track
boolean isBlack(unsigned int sensorReading)
{
  return !isWhite(sensorReading);
}

//If the sensor reading looks like "BWWWB", this may be a signal section
//Any of these readings could indicate a potential signal condition:
//  BWWxB
//  BxWWB
boolean readingIndicatesSignal()
{

  //Check for BWWxB
  if (isBlack(sensors[0]) && isWhite(sensors[1]) && isWhite(sensors[2]) && isBlack(sensors[4]))
  {
    return true;
  }

  //Check for BxWWB
  if (isBlack(sensors[0]) && isWhite(sensors[2]) && isWhite(sensors[3]) && isBlack(sensors[4]))
  {
    return true;
  }

  return false;
}

//Add the latest sensor reading to the reading history
void addToSensorReadingHistory(unsigned int reading)
{
  readingHistory[nextReading] = reading;
  nextReading = (nextReading + 1) % READING_HISTORY_LENGTH;
}

void clearSensorReadingHistory()
{
  for (unsigned int readingNum = 0; readingNum < READING_HISTORY_LENGTH; readingNum++)
  {
    readingHistory[readingNum] = 0;
  }

  nextReading = 0;
}

//Based on the recent history of sensor readings, decide if we are over a "signal" section of track
//and capture the signal if we are
void checkForSignal()
{
  if (sum(readingHistory, READING_HISTORY_LENGTH) >= SIGNAL_THRESHOLD)
  {
    handleSignalling(true);
  }
  else
  {
    handleSignalling(false);
  }
}

//Handle the receipt of a signal.  There are two cases:
//  - We enter a signal section of track (signal start)
//  - We leave a signal section of track (signal end)
void handleSignalling(boolean isSignal)
{
    //We have transitioned from black to signal (signal start)
    if (!onSignal && isSignal)
    {
      messageIsBeingReceived = true;
      message++;
      resetMessageTimer();
      onSignal = true;

      if (inDebugMode)
      {
        signalBeep();
        displayPartialMessage();
      }
    }

    //We have transitioned from signal to black (signal end)
    if (onSignal && !isSignal)
    {
      onSignal = false;
      clearSensorReadingHistory();
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
    message = 0;
    return true;
  }

  return false;
}

//Whenever a signal is received, we reset the message timer.
//In this way, a message may consist of any number of signals.
void resetMessageTimer()
{
  unsigned int messageTimeoutPeriod = MAX_MESSAGE_DISTANCE / targetSpeed;
  messageWindowExpiryTime = millis() + messageTimeoutPeriod;
}

//Plays a beep to indicate receipt of a signal
void signalBeep()
{
  OrangutanBuzzer::playFromProgramSpace(SIGNAL_TUNE);
}

//Plays a beep to indicate receipt of a message
void messageBeep()
{
  OrangutanBuzzer::playFromProgramSpace(MESSAGE_TUNE);
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
    OrangutanLCD::print(briefState);
  }
}

//Displays the message state when in debug mode
void displayMessageState(char *briefState)
{
  if (inDebugMode)
  {
    OrangutanLCD::clear();
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

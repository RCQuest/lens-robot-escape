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
int lastProportional = 0;
long integral = 0;

const int TURBO_MAX_SPEED = 250;
const int FAST_MAX_SPEED = 180;
const int NORMAL_MAX_SPEED = 100;
const int MAZE_MAX_SPEED = 60;
const int TIGHTROPE_MAX_SPEED = 60;
const int SLOW_MAX_SPEED = 50;
const int CAREFUL_MAX_SPEED = 30;
const int FURTIVE_MAX_SPEED = 20;
const int REVERSING_MAX_SPEED = 50;
const int STOP = 0;
int targetSpeed = NORMAL_MAX_SPEED;

//Spinning on the spot
const int SPIN_SPEED = 40;
const int FAST_SPIN_SPEED = 60;
const int MAZE_TURN_SPEED = 80;
const unsigned long TIME_TO_TURN_AROUND = 800; //ms

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
//It is calculated on the basis that we wish the max time between signals to be 250ms at "100" (normal) speed
//  time = distance / speed   [250ms = MAX_MESSAGE_DISTANCE / 100]
//Re-arranging the equation:
//  distance = time * speed   [MAX_MESSAGE_DISTANCE = 250 * 100]
const unsigned int MAX_MESSAGE_DISTANCE = 25000;
unsigned long messageWindowExpiryTime;

/*
 * Message Code Card
 *   1. Set Speed Normal
 *   2. Set Speed Careful
 *   3. Set Speed Slow
 *   4. Set Speed Fast
 *   5. Set Speed Turbo
 *   6. Reverse (stop, spin around and reverse at normal speed)
 *      1. Stop reversing and return to normal speed forwards
 *   7. Special
 *      1. End of course (return to work)
 */

enum MessageCode  {
                    SET_SPEED_NORMAL = 1,
                    SET_SPEED_CAREFUL = 2,
                    SET_SPEED_SLOW = 3,
                    SET_SPEED_FAST = 4,
                    SET_SPEED_TURBO = 5,
                    SET_SPEED_REVERSE = 6,
                    STOP_REVERSING = 1,
                    SPECIAL = 7,
                    END_OF_COURSE = 1,
                    MAZE = 2,
                    TIGHTROPE = 3
                  };
                
enum MessageState { 
                    READY_TO_RECEIVE,
                    REVERSING,
                    AWAITING_ADDITIONAL_INFO
                  };

boolean messageIsBeingReceived = false;
unsigned int message = 0; //the message currently being received
unsigned int lastMessage = 0; //the last complete message received
MessageState messageState = READY_TO_RECEIVE;

//All possible states
//Set state and previousState to "TEST" for rapid development using the test() function
enum State  {
              TEST,
              START_WORK,
              FOLLOW_LINE_SIMPLE, 
              FOLLOW_LINE, 
              GET_BORED, 
              CHECK_FOR_THE_BOSS, 
              GO_OFF_ROAD,
              NAVIGATE_MAZE,
              CROSS_TIGHTROPE,
              FOLLOW_WHITE_LINE, 
              TURN_AROUND,
              REVERSE, 
              RETURN_TO_WORK,
              SHUTDOWN
            };
              
State state = FOLLOW_LINE;
State previousState = FOLLOW_LINE;
State nextState;

//Time-based state transition control (milliseconds)
const unsigned long NEVER = 0;
const unsigned long MORNING_DURATION = 5000;
const unsigned long BORED_DURATION = 3900;
const unsigned long CHECK_FOR_BOSS_PAUSE1 = 1000;
const unsigned long CHECK_FOR_BOSS_PAUSE2 = 500;
const unsigned long CHECK_FOR_BOSS_PAUSE3 = 1000;
const unsigned long GO_OFF_ROAD_PAUSE = 300;
const unsigned long TRAVEL_TO_OBSTACLE_COURSE_TIME = 300;
const unsigned long CROSS_TIGHTROPE_TIME = 5800;
const unsigned long TRAVEL_BACK_TO_WORK_TIME = 200;
unsigned long nextTransitionTime = NEVER;

char path[100] = "RRSLRLLRLRLRLSLLRSSR";
unsigned char pathLength = 20; // the length of the path

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
        OrangutanMotors::setSpeeds(SPIN_SPEED, -SPIN_SPEED);
      else
        OrangutanMotors::setSpeeds(-SPIN_SPEED, SPIN_SPEED);
  
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
    case FOLLOW_LINE_SIMPLE: followLineSimpleMode(); break;
    case FOLLOW_LINE: followLine(); break;
    case GET_BORED: getBored(); break;
    case CHECK_FOR_THE_BOSS: checkForTheBoss(); break;
    case GO_OFF_ROAD: goOffRoad(); break;
    case NAVIGATE_MAZE: navigateSolvedMaze(); break;
    case CROSS_TIGHTROPE: crossTightrope(); break;
    case FOLLOW_WHITE_LINE: followWhiteLine(); break;
    case TURN_AROUND: turnAround(); break;
    case REVERSE: followLineInReverse(); break;
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
          case SET_SPEED_NORMAL: displayMessageState("Normal"); targetSpeed = NORMAL_MAX_SPEED; break;
          case SET_SPEED_REVERSE: displayMessageState("Reverse"); state = TURN_AROUND; nextState = REVERSE; messageState = REVERSING; break;
          case SET_SPEED_CAREFUL: displayMessageState("Careful"); targetSpeed = CAREFUL_MAX_SPEED; break;
          case SET_SPEED_SLOW: displayMessageState("Slow"); targetSpeed = SLOW_MAX_SPEED; break;
          case SET_SPEED_FAST: displayMessageState("Fast"); targetSpeed = FAST_MAX_SPEED; break;
          case SET_SPEED_TURBO: displayMessageState("Turbo"); targetSpeed = TURBO_MAX_SPEED; break;
          case SPECIAL: displayMessageState("Special"); messageState = AWAITING_ADDITIONAL_INFO; break;
          default: displayMessageState("UNEXPCTD"); state = SHUTDOWN; break;   //Unexpected message - STOP!
        }
      }; break;

      case REVERSING:
      {
        switch(lastMessage)
        {
          case STOP_REVERSING: displayMessageState("EndRevrs"); state = TURN_AROUND; nextState = FOLLOW_LINE; messageState = READY_TO_RECEIVE; break;
          default: displayMessageState("UNEXPCTD"); state = SHUTDOWN; break;   //Unexpected message - STOP!
        }
      }; break;

      case AWAITING_ADDITIONAL_INFO:
      {
        switch(lastMessage)
        {
          case END_OF_COURSE: displayMessageState("RtnWork"); state = RETURN_TO_WORK; messageState = READY_TO_RECEIVE; break;
          case MAZE: displayMessageState("Maze"); state = NAVIGATE_MAZE; messageState = READY_TO_RECEIVE; break;
          case TIGHTROPE: displayMessageState("TigtRope"); state = CROSS_TIGHTROPE; messageState = READY_TO_RECEIVE; break;
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

  targetSpeed = SLOW_MAX_SPEED;
  state = FOLLOW_LINE;
  nextState = GET_BORED;
  transitionAfter(MORNING_DURATION);
}

//Tracks a black line on a light background and monitors for
//signals (which trigger state changes)
//Simple mode (gives impression of immaturity of line-following mechanism)
void followLineSimpleMode() {
  displayState("SimpleLn");
  
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

  //If we're on a signal section, just drive straight
  if (!onSignal)
  {
    if (position < 1000)
    {
      // We are far to the right of the line: turn left.
  
      // Set the right motor to 100 and the left motor to zero,
      // to do a sharp turn to the left.  Note that the maximum
      // value of either motor speed is 255, so we are driving
      // it at just about 40% of the max.
      OrangutanMotors::setSpeeds(0, targetSpeed);
    }
    else if (position < 3000)
    {
      // We are somewhat close to being centered on the line:
      // drive straight.
      OrangutanMotors::setSpeeds(targetSpeed, targetSpeed);
    }
    else
    {
      // We are far to the left of the line: turn right.
      OrangutanMotors::setSpeeds(targetSpeed, 0);
    }
  }
  else
  {
    //If we're on a signal section, just drive straight
    OrangutanMotors::setSpeeds(targetSpeed, targetSpeed);
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

  stopAndWait(CHECK_FOR_BOSS_PAUSE1);

  for (unsigned int counter = 0; counter < 80; counter++)
  {
    if (counter < 20 || counter >= 60)
    {
      OrangutanMotors::setSpeeds(-FURTIVE_MAX_SPEED, FURTIVE_MAX_SPEED);
    }
    else
    {
      OrangutanMotors::setSpeeds(FURTIVE_MAX_SPEED, -FURTIVE_MAX_SPEED);
    }

    delay(30); //80 * 30 = 2400ms

    //Dramatic pauses at each extremity (taking a close look)
    if (counter == 20 || counter == 60)
    {
      stopAndWait(CHECK_FOR_BOSS_PAUSE2);  //dramatic pause
    }
  } 

  stopAndWait(CHECK_FOR_BOSS_PAUSE3);  //dramatic pause

  state = GO_OFF_ROAD;
}

//Turns then drives at high speed in a straight line (unmarked) until it finds a new line
void goOffRoad()
{
  displayState("GoOffRd");

  OrangutanMotors::setSpeeds(FAST_SPIN_SPEED, -FAST_SPIN_SPEED);
  delay(150);

  stopAndWait(GO_OFF_ROAD_PAUSE);

  OrangutanMotors::setSpeeds(FAST_MAX_SPEED, FAST_MAX_SPEED);
  delay(TRAVEL_TO_OBSTACLE_COURSE_TIME);

  targetSpeed = FAST_MAX_SPEED;
  state = FOLLOW_LINE;
}

//Tracks a black line on a light background and monitors for
//signals (which trigger state changes)
//Proportional-Derivative Mode (smoother and more accurate)
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

  //If we're on a signal section, just drive straight
  if (!onSignal)
  {
    // PID line follower
    // The "proportional" term should be 0 when we are on the line.
    int proportional = (int)position - 2000;
  
    // Compute the derivative (change) and integral (sum) of the
    // position.
    int derivative = proportional - lastProportional;
//    integral += proportional;
  
    // Remember the last position.
    lastProportional = proportional;
  
    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the right.  If it is a negative number, the robot will
    // turn to the left, and the magnitude of the number determines
    // the sharpness of the turn.  You can adjust the constants by which
    // the proportional, integral, and derivative terms are multiplied to
    // improve performance.
    int powerDifference = proportional / 7 + derivative * 4; // + integral/10000 
  
    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    const int maximum = targetSpeed;
    
    if (powerDifference > maximum)
      powerDifference = maximum;
    if (powerDifference < -maximum)
      powerDifference = -maximum;
  
    if (position == 0 || position == 4000)
    {
      //If we see no line at all, just go straight
      OrangutanMotors::setSpeeds(maximum, maximum);
    }
    else
    {
      if (powerDifference < 0)
        OrangutanMotors::setSpeeds(maximum + powerDifference, maximum);
      else
        OrangutanMotors::setSpeeds(maximum, maximum - powerDifference);
    }
  }
  else
  {
    //If we're on a signal section, just drive straight
    OrangutanMotors::setSpeeds(targetSpeed, targetSpeed);
  }
}

void navigateSolvedMaze()
{
  displayState("NavMaze");

  // Re-run the maze.  It's not necessary to identify the
  // intersections, so this loop is really simple.
  for (unsigned int segmentNum = 0; segmentNum < pathLength; segmentNum++)
  {
    followSegment();

    // Drive straight while slowing down, as before.
    OrangutanMotors::setSpeeds(50, 50);
    delay(50);
    OrangutanMotors::setSpeeds(40, 40);
    delay(200);

    // Make a turn according to the instruction stored in
    // path[i].
    turn(path[segmentNum]);
  }

  state = FOLLOW_LINE;
}

void crossTightrope()
{
  displayState("Tightrope");
  targetSpeed = TIGHTROPE_MAX_SPEED;
  state = FOLLOW_WHITE_LINE;
  nextState = FOLLOW_LINE;
  transitionAfter(CROSS_TIGHTROPE_TIME);
}

//Tracks a white line on a black background
//Proportional-Derivative Mode (smoother and more accurate)
void followWhiteLine() {
  displayState("WhiteLn");
  
  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

  // PID line follower (white on black)
  int proportional = (int)(2000 - position);
  int derivative = proportional - lastProportional;
  lastProportional = proportional;

  int powerDifference = proportional / 7 + derivative * 4; // + integral/10000 

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  const int maximum = targetSpeed;
  
  if (powerDifference > maximum)
    powerDifference = maximum;
  if (powerDifference < -maximum)
    powerDifference = -maximum;

  if (position == 0 || position == 4000)
  {
    //If we see no line at all, just go straight
    OrangutanMotors::setSpeeds(maximum, maximum);
  }
  else
  {
    if (powerDifference < 0)
      OrangutanMotors::setSpeeds(maximum + powerDifference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - powerDifference);
  }
}

void turnAround()
{
  displayState("Turning");

  stopGently();

  OrangutanMotors::setSpeeds(SPIN_SPEED, -SPIN_SPEED);
  delay(TIME_TO_TURN_AROUND);

  OrangutanMotors::setSpeeds(0, 0);

  if (nextState == REVERSE)
  {
    targetSpeed = REVERSING_MAX_SPEED;
  }
  else
  {
    targetSpeed = NORMAL_MAX_SPEED;
  }
  
  state = nextState;
}

void followLineInReverse()
{
  displayState("Reverse");

  //Comment/uncomment the following lines to use different reversing strategies
  followLineInReverseProportionalDifferential();
//  followLineInReverseProportional();
}

void followLineInReverseProportionalDifferential()
{
  int position = robot.readLine(sensors, IR_EMITTERS_ON);

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
    int proportional = position - 2000;
    int derivative = proportional - lastProportional;
    lastProportional = proportional;
  
    float powerReduction = 1 - ((float)(abs(proportional)) / 30000) + ((float)derivative / 500);

    if (proportional < 0)
    {
      OrangutanLCD::clear();
      OrangutanLCD::print(" < < < ");

      OrangutanMotors::setSpeeds(-(targetSpeed * powerReduction), -targetSpeed);
    }
    else // (proportional >= 0)
    {
      OrangutanLCD::clear();
      OrangutanLCD::print(" > > > ");
  
      OrangutanMotors::setSpeeds(-targetSpeed, -(targetSpeed * powerReduction));
    }
  }
  else
  {
    //If we're on a signal section, just drive straight
    OrangutanMotors::setSpeeds(-targetSpeed, -targetSpeed);
  }
}
  
void followLineInReverseProportional()
{
  int position = robot.readLine(sensors, IR_EMITTERS_ON);

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
    int proportional = position - 2000;

    if (proportional == -2000)
    {
      OrangutanLCD::clear();
      OrangutanLCD::print(" < < < ");
  
      OrangutanMotors::setSpeeds(-(targetSpeed * 0.9), -targetSpeed);
      delay(targetSpeed / 2.5);
    }
    else if (proportional < -1000)
    {
      OrangutanLCD::clear();
      OrangutanLCD::print("  < <  ");
  
      OrangutanMotors::setSpeeds(-(targetSpeed * 0.8), -targetSpeed);
      delay(targetSpeed / 3);
    }
    else if (proportional < 0)
    {
      OrangutanLCD::clear();
      OrangutanLCD::print("   <   ");
  
      OrangutanMotors::setSpeeds(-(targetSpeed * 0.95), -targetSpeed);
      delay(targetSpeed / 5);
    }
    else if (proportional < 1000)
    {
      OrangutanLCD::clear();
      OrangutanLCD::print("   >   ");
  
      OrangutanMotors::setSpeeds(-targetSpeed, -(targetSpeed * 0.95));
      delay(targetSpeed / 5);
    }
    else if (proportional < 2000)
    {
      OrangutanLCD::clear();
      OrangutanLCD::print("  > >  ");
  
      OrangutanMotors::setSpeeds(-targetSpeed, -(targetSpeed * 0.8));
      delay(targetSpeed / 3);
    }
    else //(proportional == 2000)
    {
      OrangutanLCD::clear();
      OrangutanLCD::print(" > > > ");
  
      OrangutanMotors::setSpeeds(-targetSpeed, -(targetSpeed * 0.9));
      delay(targetSpeed / 2.5);
    }
  }
  else
  {
    //If we're on a signal section, just drive straight
    OrangutanMotors::setSpeeds(-targetSpeed, -targetSpeed);
  }
}

void returnToWork()
{
  displayState("Rtn2Wrk");

  //Get off the obstacle course line and just drive straight
  changeSpeedSmoothly(NORMAL_MAX_SPEED);
  delay(TRAVEL_BACK_TO_WORK_TIME);

  targetSpeed = SLOW_MAX_SPEED;
  state = FOLLOW_LINE;
}

void finish()
{
  stopGently();
}

//Function used to quickly test functionality
void test()
{
//  targetSpeed = 100;
//
//  OrangutanMotors::setSpeeds(targetSpeed, targetSpeed);
//  delay (1000);
//
//  changeSpeedSmoothly(SLOW_MAX_SPEED);
//  delay (1000);
//
//  changeSpeedSmoothly(121);
//  delay (1000);

//navigateSolvedMaze();

//  state = CROSS_TIGHTROPE;
  state = FOLLOW_LINE;
//  nextState = RETURN_TO_WORK;
//  transitionAfter(2000);
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

void stopAndWait(unsigned long waitTime)
{
  OrangutanMotors::setSpeeds(0, 0);
  delay(waitTime);
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

void stopGently()
{
  changeSpeedSmoothly(STOP);
}

void changeSpeedSmoothly(int newSpeed)
{
  //If we're already going at the target speed, we don't need to do anything
  if (newSpeed != targetSpeed)
  {
    const int RATE_OF_CHANGE = 5;
  
    int currentSpeed = targetSpeed;
    int speedDifference = abs(newSpeed - currentSpeed);
    int accelerationMultiplier = (newSpeed < currentSpeed) ? -1 : 1;
  
    //Make sure that we will reach a complete stop by ensuring targetSpeed divides exactly by RATE_OF_CHANGE
    currentSpeed += (speedDifference % RATE_OF_CHANGE) * accelerationMultiplier;
    
    //Slow down gradually to a stop
    do
    {
      currentSpeed += (RATE_OF_CHANGE * accelerationMultiplier);
  
      OrangutanLCD::clear();
      OrangutanLCD::print(currentSpeed);
  
      OrangutanMotors::setSpeeds(currentSpeed, currentSpeed);
      delay(50);
    }
    while (currentSpeed != newSpeed);
  
    targetSpeed = newSpeed;
  }
}

// This function, causes the 3pi to follow a segment of the maze until
// it detects an intersection, a dead end, or the finish.
void followSegment()
{
  while(1)
  {
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

    int proportional = ((int)position) - 2000;
    int derivative = proportional - lastProportional;
    lastProportional = proportional;

    int powerDifference = proportional / 7 + derivative * 4;

    const int maximum = MAZE_MAX_SPEED;
    
    if (powerDifference > maximum)
      powerDifference = maximum;
    if (powerDifference < -maximum)
      powerDifference = -maximum;

    if (powerDifference < 0)
      OrangutanMotors::setSpeeds(maximum + powerDifference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - powerDifference);

    // We use the inner three sensors (1, 2, and 3) for
    // determining whether there is a line straight ahead, and the
    // sensors 0 and 4 for detecting lines going to the left and
    // right.

    if (sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100)
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }
    else if (sensors[0] > 200 || sensors[4] > 200)
    {
      // Found an intersection.
      return;
    }
  }
}

// Code to perform various types of turns according to the parameter dir,
// which should be 'L' (left), 'R' (right), 'S' (straight), or 'B' (back).
// The delays here had to be calibrated for the 3pi's motors.
void turn(unsigned char dir)
{
  switch(dir)
  {
    case 'L':
      // Turn left.
      OrangutanMotors::setSpeeds(-MAZE_TURN_SPEED, MAZE_TURN_SPEED);
      delay(200);
      break;
    case 'R':
      // Turn right.
      OrangutanMotors::setSpeeds(MAZE_TURN_SPEED, -MAZE_TURN_SPEED);
      delay(200);
      break;
    case 'B':
      // Turn around.
      OrangutanMotors::setSpeeds(MAZE_TURN_SPEED, -MAZE_TURN_SPEED);
      delay(390);
      break;
    case 'S':
      // Don't do anything!
      break;
  }
}

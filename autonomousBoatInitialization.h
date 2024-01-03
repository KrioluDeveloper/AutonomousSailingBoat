//////////////////////////////////////////////////////////////////////////////

// Compass navigation
int targetHeading;              // where we want to go to reach current waypoint
int currentHeading;             // where we are actually facing now
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

#define UPWIND_TOLERANCE 50
#define WAYPOINT_DIST_TOLERANE 10 

// float currentLat,
//       currentLong,
//       targetLat,
//       targetLong;



int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoint when we started navigating to it
    



#define RUDDER_HIGH_PULSE 2000  //nominal 2000
#define RUDDER_LOW_PULSE 1000   //nominal 1000
#define SAIL_HIGH_PULSE 2000   //nominal 2000
#define SAIL_LOW_PULSE 980    //nominal 1000
#define WIND_HIGH_PULSE 1023   //nominal 1023


double currentLat,
      currentLong,
      targetLat,
      targetLong;

// Pin assignments
const int ROBOSAIL_PIN_WIND = 7; 
//input pins from receiver
const int ROBOSAIL_PIN_RUDDER_RC = 5;//CH 1
const int ROBOSAIL_PIN_SAIL_RC = 6;//CH 3
const int ROBOSAIL_PIN_MODE_RC = 4;//CH 6
// Output pins to the servos
const int ROBOSAIL_PIN_RUDDER_SERVO = 8;
const int ROBOSAIL_PIN_SAIL_SERVO = 9;
// variables to hold input and output values
int rudderPulseWidth;
int rudderServoOut;
int RCRudderServoOut;
int sailPulseWidth;
int modePulseWidth;
int sailServoOut;
int RCSailServoOut;
int sailPosition = 30;
int rudderPosition;
int rudderRCPosition;
int sailRCPosition;
//variables for WindSensor
int windAngle = 0;
int windPulseWidth = 0;

//variables for GPS
// this will be false until GPS fix is found and starting position saved
bool start_pos_found = false;
int GPSfix;
int GPSqual;
int GPSsat;
// once GPS fix is found, these variables will be updated
float startPositionX = 0;
float startPositionY = 0;
float relPositionX = 0;
float relPositionY = 0;
float pos[2];
float angleFromStart = 0;



float roll, pitch, yaw, heading, robosailHeading, robosailRoll;
// Source: http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
float declination = -14.0563;
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

//Turn Direction

// Steering/turning 
#define TURN_LEFT -60
#define TURN_RIGHT 60
#define TURN_STRAIGHT 0
enum directions {left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT} ;
directions turnDirection = straight;
int smoothTurnMagnitude = 0;

// These values will need to be adjusted based on your particular compass.
// Use compassCalibration (in the Orientation library) to determine the correct hard iron calibration.

//  Hard iron calibration for X: 7.18 for Y: -1.82 for Z: 16.94
float hardiron_x = 7.18;
float hardiron_y = 1.55;
float hardiron_z = 10.41;

int position = 0;

//create servo objects
Servo rudderServo;
Servo sailServo;

//Smooth Wind initialization
const int numReadings = 10;
int windReadings[numReadings];
int readIndex = 0;
int total = 0;
int windAverage;
int trueWind;

//Ultrasonic Initialization
unsigned char data_buffer[4] = {0};
int distance = 0;
unsigned char CS;

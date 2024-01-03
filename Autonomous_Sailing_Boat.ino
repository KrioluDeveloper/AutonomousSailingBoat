//Necessary Packages
#include <Servo.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <UsefulCalcs.h>
#include <wayPointClass.h>         // custom class to manage GPS waypoints
#include <Wire.h>
#include <math.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include "autonomousBoatInitialization.h"
#include "wayPoints.h"

boolean verbose = true;  //true calls function for values to be printed to monitor
//#define GPSSerial Serial

Adafruit_GPS GPS(&Serial1);
// initialize utility that will convert lat/lon to (x,y) positions in meters
UsefulCalcs calc(false);

void setup() {

  //Assign serial ports and baud rates
  Serial.begin(115200);
  GPS.begin(9600);
  Serial3.begin(9600);
 
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //This dummy waypoint must be in Set up or the code will break!!!!
  targetLat = 41.6106;
  targetLong = -70.9251;
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  Serial.println("\nCommunity Boating Center Autonomous Sailing Drone 8/12/2023");  //write program name/rev here

  //Declares Pin Types for RC and attaches servo
  declarePins();
 

  //////////////////////////////////////
  Serial.println("DEBUG");
  //Set Up GPS and wait for fix on position
  GPS.begin(9600);  //default baud rate for Adafruit MTK GPS's
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);  //setting for minimum recommended data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  //update rate is 1 Hz
  enableInterrupt(); // activate TIMER0 interrupt, goes off every 1 msec
 /* while (start_pos_found == false)  //loop code will not start until GPS is ready
  { readGPS();}*/
  
  // Set up Compass and check that it is connected
  mag.enableAutoRange(true);
    if(!mag.begin() || !accel.begin()) //Initialize the sensor
    {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("No LSM303 Compass detected ... Check your wiring!");
    while(1); }
}

void loop() {

  //Read in data from the RC receiver and sensors 
  //Read the command pulse from the RC receiver
  readReceiver();
  

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 //Track Mode if Switch SWB is set to 2
  if (1900 < modePulseWidth && modePulseWidth < 2000){

    Serial.print("Mode Pulse: ");
    Serial.println(modePulseWidth);

    Serial.println("Mode: Track");

    //delay(5000);

  }
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 //Manual RC Mode if Switch SWB is set to 1
  else if (900 < modePulseWidth && modePulseWidth < 1000){
 
    if (verbose) {printToMonitorManual();}

    driveSailServo(sailRCPosition);
    driveRudderServo(rudderRCPosition);
    delay(1000);

  }
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //Fully Autonomous Mode if Controller is Off
  
  else if (modePulseWidth == 0){
  
    // Servo Commands/ Reading Wind
  

    //Read and Smooth Wind

    readWind();// Calculates a wind angle from the input

    smoothWind();// Gives an average wind value after 10 points are stored

    //Read position from the GPS
    readGPS();  //puts values in pos array
  
    //Read heading and tilt from the Compass
    readCompass();

    if (GPS.newNMEAreceived()){ // check for updated GPS information

          if(GPS.parse(GPS.lastNMEA()) )      // if we successfully parse it, update our data fields
           processGPS();   
    } 


    //Dummy Waypoint When there is no fix
    /////////////////////////////////
    if (GPSfix == 0){

      currentLat = 41.61;
      currentLong = -70.92;
    }
  /////////////////////////////////////
  //Gets current and target latitudes and longitudes
  ////////////////////////////////////
    else{
     currentLat = GPS.latitudeDegrees;
     currentLong = GPS.longitudeDegrees;
     targetLat = waypointList[waypointNumber].getLong();
     targetLong = waypointList[waypointNumber].getLat();
   }
  ///////////////////////////////////////

  //Print to monitor must be here or the code runs into memory errors
  // Therefore, it is wise to not move this line in context with everything else
  if (verbose) {printToMonitor();}


    // Navigation Functions 
   targetHeading = courseToWaypoint();
   // currentHeading = heading; // get our current heading
   calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      

   checkDistance(); // Checks distance of objects directly in front of drone

   collisionAvoidance(); //Avoids collision by sailing to leeward if an object is within a given distance tolerance 

   calcTrueWind();
   //trueWind = 180;

   //Autonomous Steering
   autonomousSteer();

   //Autonomous Sail Trim

   autonomousTrim();

   sailUpWind(); // Sails in a scripted maneuver if the boat must sail upwind to approach a waypoint


   //AntiBroach
   //antiBroach();


   delay(100);
  }
 
  
} //end of loop()

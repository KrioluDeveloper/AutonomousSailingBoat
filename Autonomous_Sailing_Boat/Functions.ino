

// Function prints out all values for debugging in Autonomous Mode.
void printToMonitor(){
  Serial.println("------------------------------------------------");
  Serial.println("");
  Serial.print("Mode Pulse: ");
  Serial.println(modePulseWidth);
  Serial.println("Mode: Autonomous");
  Serial.print("Wind Angle: ");
  Serial.println(windAngle);
  Serial.print("\n"); // Print a new line
  Serial.print("Fix: "); Serial.print(GPSfix);
  Serial.print(" quality: "); Serial.print(GPSqual);
  Serial.print(" satellites: "); Serial.println(GPSsat);
  Serial.print("x = "); Serial.print(relPositionX);
  Serial.print("   y = "); Serial.print(relPositionY);
  Serial.print(" startPositionX = "); Serial.print(startPositionX);
  Serial.print(" startPositionY = "); Serial.print(startPositionY);
  Serial.print("  angle from start = "); Serial.println(angleFromStart);
      
  Serial.print("Roll: "); Serial.print(robosailRoll);
  Serial.println();
  Serial.print("Current Latitude= ");
 
  Serial.println(currentLat, 5);
  Serial.print("Current Longitude= ");
  Serial.println(currentLong, 5);
  Serial.print("Target Latitude= ");
  Serial.println(targetLat, 5);
  Serial.print("Target Longitude= ");
  Serial.println(targetLong, 5);

  Serial.print("Distance to Waypoint= ");
  Serial.println(distanceToTarget);

 

  Serial.print("Waypoint Number= ");
  Serial.println(waypointNumber);

  Serial.print("Heading: "); Serial.println(heading);

 Serial.print("Course to Waypoint= ");
  Serial.println(targetHeading);

  Serial.print("Heading Error= ");
  Serial.println(targetHeading - heading);

  Serial.print(" Ruddle angle out: ");




  if (abs(headingError) <= HEADING_TOLERANCE){      // if within tolerance, don't turn
    Serial.println(turnDirection); 
  }

  else if((turnDirection == right) && (abs(headingError) < 90)){
    Serial.println(smoothTurnMagnitude);
  }

   else if((turnDirection == left) && (-abs(headingError) > -90)){
    Serial.println(smoothTurnMagnitude);
  }

  else if((turnDirection == left) && (-abs(headingError) < -90)){
    Serial.println(turnDirection);
  }

  else if((turnDirection == right) && (abs(headingError) > 90)){
    Serial.println(turnDirection);
}









  Serial.print(" Turn Direction: ");

  if (turnDirection == right){      // if within tolerance, don't turn
    Serial.println("right"); 
  }

  else if(turnDirection == left){
    Serial.println("left");
  }

   else if(turnDirection == straight){
    Serial.println("straight");
  }

  
  

  Serial.print(" Sail angle out: ");
  Serial.println(sailPosition);
 
  Serial.print("Average Wind Angle: ");
  Serial.println(windAverage); 

  Serial.print("True Wind: ");
  Serial.println(trueWind); 
}




// Function prints out all values for debugging in Manual Mode.
void printToMonitorManual(){

  Serial.print("Mode Pulse: ");
  Serial.println(modePulseWidth);
  Serial.print("Sail Pulse: ");
  Serial.println(sailPulseWidth);
  Serial.print("Rudder Pulse: ");
  Serial.println(rudderPulseWidth);
  Serial.print("Rudder Position: ");
  Serial.println(rudderRCPosition);
  Serial.print("Sail Position: ");
  Serial.println(sailRCPosition);
  Serial.print("rudderServoOut: ");
  Serial.println(rudderServoOut);
  Serial.print("sailServoOut: ");
  Serial.println(sailServoOut);
  Serial.println("Mode: Manual");
  Serial.println("");

}





// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect){

  GPS.read(); // reads char (if available) into internal buffer in GPS object
}

// function to enable TIMER0 interrupt for GPS
void enableInterrupt(){

  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

//gets GPS data, pareses it, and returns (x,y) position in meters in array called pos[]

void readGPS(){

if (GPS.newNMEAreceived())
  {
    char* LastNMEA; // declare pointer to GPS data
    LastNMEA = GPS.lastNMEA(); // read the string and set the newNMEAreceived() flag to false
    if (!GPS.parse(LastNMEA)) 
    {
      return; // failed to parse a sentence (was likely incomplete) so just wait for another
    }
    Serial.println("\nNew data from GPS");
    GPSfix = GPS.fix;  //put parsed data in variables for printing
    GPSqual = GPS.fixquality;
    GPSsat = GPS.satellites;
    if (GPS.fix)
    {
      if (start_pos_found)
      {
        // take in lat/lon degree values and return (x,y) in meters in pos array
        calc.latLonToUTM(GPS.latitudeDegrees, GPS.longitudeDegrees, pos);
        
        // calculate the boat position relative to where it was started
        relPositionX = pos[0] - startPositionX;
        relPositionY = pos[1] - startPositionY;
        angleFromStart = atan2(relPositionY, relPositionX) * 180 / 3.14;
        while (angleFromStart < 0){ angleFromStart += 360; }
        while (angleFromStart > 360){ angleFromStart -= 360; }

      }
      else // starting position not yet found but there is a fix
      { 
        // take in lat/lon degree values and return (x,y) in meters in pos array
        calc.latLonToUTM(GPS.latitudeDegrees, GPS.longitudeDegrees, pos);
        startPositionX = pos[0];
        startPositionY = pos[1];
        
        Serial.println("Starting position found!");
        Serial.print("x = "); Serial.print(startPositionX);
        Serial.print("   y = "); Serial.println(startPositionY);
        Serial.println();
        
        start_pos_found = true;
      }
    }
  }
}

//reads Compass to get heading and tilt
void readCompass(){

  float ax, ay, az, mx, my, mz;
  float my_adj, mx_adj;
  float Pi = 3.1415926;

   /* Get a new sensor event */
  sensors_event_t accel_event;
  accel.getEvent(&accel_event);
  sensors_event_t mag_event;
  mag.getEvent(&mag_event);

  /* Invert X so that when when X, Y, or Z is pointed down, it has a positive reading. */
  ax = -accel_event.acceleration.x;
  ay = accel_event.acceleration.y;
  az = accel_event.acceleration.z;
  
  /* Adjust for hard iron effects */
  mx = mag_event.magnetic.x - hardiron_x;
  my = mag_event.magnetic.y - hardiron_y;
  mz = mag_event.magnetic.z - hardiron_z;

  /* Invert Y and Z axis so that when X, Y, or Z is pointed towards Magnetic North they get a positive reading. */
  my = -my;
  mz = -mz;
  
  roll = atan2(ay,az);
  pitch = atan(-ax/sqrt(pow(ay,2)+pow(az,2)));
  
  my_adj = mz*sin(roll) - my*cos(roll);
  mx_adj = (mx*cos(pitch) + my*sin(pitch)*sin(roll) + mz*sin(pitch)*cos(roll));
  
  yaw = atan2(my_adj,mx_adj);
  
  roll = roll * 180/Pi;
  pitch =  pitch * 180/Pi;
  yaw = yaw * 180/Pi;
  
  heading = yaw + declination;
  
  if (heading >= 360) {
    heading -= 360;
  } else if (heading < 0) {
    heading += 360;
  }


  if (heading >= 360) {heading -= 360;}
  
  //define roll for RoboSail as rolling to Port side is positive, rolling to Starboard is negative
  robosailRoll  = -1 * roll; 
//
 
//
}


// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// copied from TinyGPS library
int courseToWaypoint(){

  float dlon = radians(targetLong-currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  //targetHeading = degrees(a2) ;
 targetHeading = degrees(a2);
 if (targetHeading >= 360){
   targetHeading -= 360;
 }

  return targetHeading;
}  


void nextWaypoint(void){

  waypointNumber = waypointNumber + 1;
  targetLat = waypointList[waypointNumber].getLat();
  targetLong = waypointList[waypointNumber].getLong();
  
  
  /*if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
    {
      driveMotor->run(RELEASE);    // make sure we stop
      turnMotor->run(RELEASE);  
      lcd.clear();
      lcd.println(F("* LAST WAYPOINT *"));
      loopForever();
    }
    */
   processGPS();
   distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
   courseToWaypoint();
   
}  


// converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
double convertDegMinToDecDeg (float degMin){

  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}

// returns distance in meters between two positions, both specified 
// as signed decimal-degrees latitude and longitude. Uses great-circle 
// distance computation for hypothetical sphere of radius 6372795 meters.
// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
// copied from TinyGPS library
int distanceToWaypoint(){
  
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  distanceToTarget =  delta * 6372795; 
   
  // check to see if we have reached the current waypoint
  if (distanceToTarget <= WAYPOINT_DIST_TOLERANE)
    nextWaypoint();
    
    
  return distanceToTarget;
}  // distanceToWaypoint()


//
// Called after new GPS data is received; updates our position and course/distance to waypoint
void processGPS(void){

  currentLat = convertDegMinToDecDeg(GPS.latitude);
  currentLong = convertDegMinToDecDeg(GPS.longitude);
             
  if (GPS.lat == 'S')            // make them signed
    currentLat = -currentLat;
  if (GPS.lon = 'W')  
    currentLong = -currentLong; 
             
  // update the course and distance to waypoint based on our new position
  distanceToWaypoint();
  courseToWaypoint();         
  
}   // processGPS(void)


//Capsize avoidance (with ballast system WIP)
void antiBroach(void){
  
      if (abs(robosailRoll) > 60){
        sailPosition = 125;
      }

}


// Edited from source for turn efficiency and to fit with sail boat logic
void calcDesiredTurn(void){

    // calculate where we need to turn to head to destination
    headingError = targetHeading - heading;
    
    // adjust for compass wrap
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
  
    // calculate which way to turn to intercept the targetHeading
    if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
      turnDirection = straight;  
    else if (headingError < 0)
      turnDirection = left;
    else if (headingError > 0)
      turnDirection = right;
    else
      turnDirection = straight;

      //Optimizing Turn Direction
    if (abs(headingError) > 180){
      if(turnDirection == right)
        turnDirection = left;
      else if(turnDirection == left)
        turnDirection = right;


    }      
 
}  


void autonomousSteer(void){

  if (abs(headingError) <= HEADING_TOLERANCE){      // if within tolerance, don't turn
      driveRudderServo(turnDirection); 
  }

  else if((turnDirection == right) && (abs(headingError) < 90)){
    smoothTurnMagnitude = map(abs(headingError), 10, 90, 15, 60);
    driveRudderServo(smoothTurnMagnitude);
  }

   else if((turnDirection == left) && (-abs(headingError) > -90)){
    smoothTurnMagnitude = map(-abs(headingError), -10, -90, -15, -60);
    driveRudderServo(smoothTurnMagnitude);
  }

  else if((turnDirection == left) && (-abs(headingError) < -90)){
    driveRudderServo(turnDirection);
  }

  else if((turnDirection == right) && (abs(headingError) > 90)){
    driveRudderServo(turnDirection);
  }

}

void autonomousTrim(void){

  //sailPosition = (-(abs(windAverage) * 2) / 3) + 120;//Auto Trim Servo Calculation and Command
    //sailPosition = (90 - ((90/180)*(abs(windAverage))));


   sailPosition = map(abs(windAverage), 0, 180, 0, 90);

  sailServo.write(sailPosition);//Auto Trim

}





void calcTrueWind(){

//Calculate True Wind
  trueWind = heading + windAverage;

  // adjust for compass wrap
  if (trueWind < 0 ){
    trueWind += 360;
  }
   if (trueWind > 360 ){
    trueWind -= 360;
  }

}

// This function calls for the boat to make a specific maneuver if it has to sail up wind... Zig-Zag method
void sailUpWind(){

  
  //Conditions for Tacking Maneuver
  if (  ((heading < (trueWind + UPWIND_TOLERANCE)) && (heading > (trueWind - UPWIND_TOLERANCE))) &&     
     ((targetHeading < (trueWind + UPWIND_TOLERANCE)) && (targetHeading > (trueWind - UPWIND_TOLERANCE)))  ) {
    // if heading is in no go zone and if the waypoint is in the no go zone
      Serial.println("Tacking Manuver!!");

      //Tack Right
      if (turnDirection == 60){
        //if we are going right
        turnDirection = -30;
        Serial.println("Tacking to Starboard");
        driveRudderServo(turnDirection);
        Serial.println("Going Left!!");
        //Turn Slightly left to power sail
        driveSailServo(0);
        Serial.println("Pulling in the Sails!!");
         delay(3000);
        // Pull in the sail all the way
        turnDirection = 0;
        driveRudderServo(turnDirection);
        Serial.println("Going Straight!!");
        delay(3000);
        //Go Straight and gain some speed
        turnDirection = 60;
        driveRudderServo(turnDirection);
        Serial.println("Tacking Starboard!!");
        delay(4000);
        // Tack into the wind going right
      }

      //Tack Left
      if (turnDirection == -60){
        //if we are going right
        turnDirection = 30;
        Serial.println("Tacking to Port");
        driveRudderServo(turnDirection);
        Serial.println("Going Right!!");
        //Turn Slightly left to power sail
        driveSailServo(0);
        Serial.println("Pulling in the Sails!!");
        delay(3000);
        // Pull in the sail all the way
        turnDirection = 0;
        driveRudderServo(turnDirection);
        Serial.println("Going Straight!!");
        delay(3000);
        //Go Straight and gain some speed
        turnDirection = -60;
        driveRudderServo(turnDirection);
        Serial.println("Tacking Port!!");
        delay(4000);
        // Tack into the wind going right
      }
  }
}


// Read values from the WindSensor
void readWind(){
  
  windPulseWidth = pulseIn(ROBOSAIL_PIN_WIND, HIGH, 25000);
  // Convert the wind angle to degrees from PWM.  Range -180 to +180
  windAngle = map(windPulseWidth, 0, 1023, 180, -180);
  windAngle = constrain(windAngle, -180, 180);
}


 // Wind Angle Smoothing
// This method averages the wind data
void smoothWind(){

  total -= windReadings[readIndex];
  //total = total - windReadings[readIndex];

  windReadings[readIndex] = windAngle;

  total += windReadings[readIndex];
  //total = total + windReadings[readIndex];

  readIndex ++;
  //readIndex = readIndex + 1;

  if (readIndex >= numReadings){
    readIndex = 0;
  }

  windAverage = total / numReadings;
}


// Takes in the PWM signals from the RC Receiver and translates
 // translates them to the servo positions in degrees.
 void readReceiver(){

  // Read the command pulse from the RC receiver
  rudderPulseWidth = pulseIn(ROBOSAIL_PIN_RUDDER_RC, HIGH);
  sailPulseWidth = pulseIn(ROBOSAIL_PIN_SAIL_RC, HIGH);
  modePulseWidth = pulseIn(ROBOSAIL_PIN_MODE_RC, HIGH);
  // Calculate the servo position in degrees.
  rudderRCPosition = map(rudderPulseWidth, RUDDER_LOW_PULSE, RUDDER_HIGH_PULSE, -60, 60);
  sailRCPosition = map(sailPulseWidth, SAIL_LOW_PULSE, SAIL_HIGH_PULSE, 0, 90);
  
 }


// Declares Pin Types and attaches servos
 void declarePins(){

   //RC input pins
   pinMode(ROBOSAIL_PIN_RUDDER_RC, INPUT);
  pinMode(ROBOSAIL_PIN_SAIL_RC, INPUT);
  pinMode(ROBOSAIL_PIN_MODE_RC, INPUT);

  // attach the servos to the proper pins
  rudderServo.attach(ROBOSAIL_PIN_RUDDER_SERVO);
  sailServo.attach(ROBOSAIL_PIN_SAIL_SERVO);
}


// Maps servo degrees from -90 to 90  to the rudder servo motor's range from 0 to 180 with 90 deg in the center
//then writes to the servo the appropriate value
void driveRudderServo(int rudderPos){

  if ((rudderPos >= -60) && (rudderPos <= 60)){
    rudderServoOut = map(rudderPos, -90, 90, 0, 180);
    rudderServo.write(rudderServoOut);
  }

  else {
    Serial.print("ERROR - rudder position out of range: ");
    Serial.println(rudderPos);
    }
}


// Maps servo degrees from 0 to 90  to the rudder servo motor's range from 55 to 125
// the Sailwinch servo is at ~55 deg when full-in, or 0 deg,
// and ~125 deg when full out, or 90 deg
void driveSailServo(int sailPos){

  if ((sailPos >= 0) && (sailPos <= 90)){
    sailServoOut = map(sailPos, 0, 90, 55, 125);
    sailServo.write(sailServoOut);
  }

  else {
    Serial.print("ERROR - sail position out of range: ");
    Serial.println(sailPos);
  } 
}

// Takes a distance from the TX pin of the sensor by sending the hexidecimal value 0x55
//to the RX pin. Assigns a distance value in mm. Adapted from DroneBot Workshop Code
void checkDistance(){

  if (Serial.available() > 0) {
    
    Serial3.write(0x55); // Send hexadecimal 55 character
      
      // Read and process data response
    while (Serial3.available()) {
      delay(4);
        
      if (Serial3.read() == 0xff) { // data header
        data_buffer[0] = 0xff;
          
        for (int i = 1; i < 4; i++) {
          data_buffer[i] = Serial3.read();
        }
          
        CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
          
        if (data_buffer[3] == CS) {
            distance = (data_buffer[1] << 8) + data_buffer[2];
            Serial.print("distance: ");
            Serial.print(distance);
            Serial.println(" mm");
        }
      }
    }
    
  }
}


//Based on the distance measured and the current direction the boat is going,
//a new course will be made for the boat
void collisionAvoidance(){

  //If going right and there is an obstacle, turn left to avoid it
  if ((distance < 3000) && (turnDirection == 60) && (distance > 1000)){
    Serial.print("Obstacle Detected. Turning Left!");
    turnDirection = -45;
    driveRudderServo(turnDirection);
    delay(4000);
  }

  //If going Straight and there is an obstacle, turn Right to avoid it
  else if ((distance < 3000) && (turnDirection == 0) && (distance > 1000)){
    Serial.print("Obstacle Detected. Turning Right!");
    turnDirection = 45;
    driveRudderServo(turnDirection);
    delay(4000);
  }

  //If going left and there is an obstacle, turn right to avoid it
  else if ((distance < 3000) && (turnDirection == -60) && (distance > 1000)){
    Serial.print("Obstacle Detected. Turning Right!");
    turnDirection = 45;
    driveRudderServo(turnDirection);
    delay(4000);
  }

}
 

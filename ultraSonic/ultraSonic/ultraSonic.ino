
/*
  JSN-SR04T-V3.0 Ultrasonic Sensor - Mode 0 Demo
  srt04-mode0.ino
  Uses JSN-SR04T-V3.0 Ultrasonic Sensor
  Displays on Serial Monitor
 
  Mode 0 is default mode with no jumpers or resistors (emulates HC-SR04)
 
  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/
 /*
// Define connections to sensor
#define TRIGPIN 11
#define ECHOPIN 10
 
// Floats to calculate distance
float duration, distance;
 
void setup() {
  // Set up serial monitor
  Serial.begin(115200);
 
  // Set pinmodes for sensor connections
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);
}
 
void loop() {
 
  // Set the trigger pin LOW for 2uS
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
 
  // Set the trigger pin HIGH for 20us to send pulse
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(20);
 
  // Return the trigger pin to LOW
  digitalWrite(TRIGPIN, LOW);
 
  // Measure the width of the incoming pulse
  duration = pulseIn(ECHOPIN, HIGH);
 
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  // Divide by 1000 as we want millimeters
 
  distance = (duration / 2) * 0.343;
 
  // Print result to serial monitor
  Serial.print("distance: ");
  Serial.print(distance);
  Serial.println(" mm");
 
  // Delay before repeating measurement
  delay(100);
}



*/

/*
  JSN-SR04T-V3.0 Ultrasonic Sensor - Mode 1 Demo
  srt04-mode1.ino
  Uses JSN-SR04T-V3.0 Ultrasonic Sensor
  Displays on Serial Monitor

  Mode 1 is set by bridging "M1" pads on board

  Also works with A02YYUW Ultrasonic Sensor

  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/

// Include the Software Serial library
//#include <SoftwareSerial.h>

// Define connections to sensor
//int pinRX = 10;
//int pinTX = 11;

// Array to store incoming serial data

/*
unsigned char data_buffer[4] = {0};

// Integer to store distance
int distance = 0;

// Variable to hold checksum
unsigned char CS;

// Object to represent software serial port
//SoftwareSerial mySerial(pinRX, pinTX);

void setup() {
  // Set up serial monitor
  Serial.begin(115200);
  // Set up software serial port
  Serial3.begin(9600);
}

void loop() {

  // Run if data available
  if (Serial3.available() > 0) {

    delay(4);

    // Check for packet header character 0xff
    if (Serial3.read() == 0xff) {
      // Insert header into array
      data_buffer[0] = 0xff;
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = Serial3.read();
      }

      //Compute checksum
      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
      // If checksum is valid compose distance from data
      if (data_buffer[3] == CS) {
        distance = (data_buffer[1] << 8) + data_buffer[2];
        // Print to serial monitor
        Serial.print("distance: ");
        Serial.print(distance);
        Serial.println(" mm");
      }
    }
  }
}

*/

/*
// Array to store incoming serial data
unsigned char data_buffer[4] = {0};

// Integer to store distance
int distance = 0;

// Variable to hold checksum
unsigned char CS;

void setup() {
  // Set up serial monitor
  Serial.begin(115200);

  // Set up the second hardware serial port
  Serial3.begin(9600);

  // Enter Mode 2 by sending the data request
  Serial3.write(0x55); // Send hexadecimal 55 character
}

void loop() {

  // Run if data available
  if (Serial3.available() > 0) {

    delay(4);
    //Serial3.write(0x55); // Send hexadecimal 55 character

    // Check for packet header character 0xff
    if (Serial3.read() == 0xff) {
      // Insert header into array
      data_buffer[0] = 0xff;
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = Serial3.read();
      }

      // Compute checksum
      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
      // If checksum is valid, compose distance from data
      if (data_buffer[3] == CS) {
        distance = (data_buffer[1] << 8) + data_buffer[2];
        // Print to serial monitor
        Serial.print("distance: ");
        Serial.print(distance);
        Serial.println(" mm");
        //Serial3.write(0x55); // Send hexadecimal 55 character
        

      }
             
    }
  }
}
*/







unsigned char data_buffer[4] = {0};
int distance = 0;
unsigned char CS;



void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char inputChar = Serial.read(); // Read the input character
    
    if (inputChar == 'R' || inputChar == 'r') {
      // Send data request when 'R' or 'r' is received
      Serial3.write(0x55); // Send hexadecimal 55 character
      
      // Read and process data response
      while (Serial3.available()) {
        delay(4);
        
        if (Serial3.read() == 0xff) {
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
}



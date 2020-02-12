/*
  Wireless Servo Control, with ESP as Access Point

  Usage: 
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1. 
    A webpage with four buttons should appear. Click them to move the robot.

  Installation: 
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP

  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon

    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets
    
    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See 
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system

  Hardware: 
  * NodeMCU Amica DevKit Board (ESP8266 chip)
  * Motorshield for NodeMCU 
  * 2 continuous rotation servos plugged into motorshield pins D1, D2
  * Ultra-thin power bank 
  * Paper chassis

*/

#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#include <Servo.h>

#include "debug.h"
#include "file.h"
#include "server.h"


#include <Wire.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define SDA_PORT 14
#define SCL_PORT 12



#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PORT 14
#define SCL_PORT 12
VL53L0X sensor;
VL53L0X sensor2;


const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;

int16_t max_mx = -1000;
int16_t min_mx = 1000;
int16_t max_my = -1000;
int16_t min_my = 1000;

// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";

// WiFi STA parameters
char* sta_ssid = 
  "...";
char* sta_password = 
  "...";

char* mDNS_name = "paperbot";

String html;
String css;


void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void setup() {
    setupPins();

    sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);



          Wire.begin(SDA_PORT,SCL_PORT);
  Serial.begin(115200);

  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  delay(500);
  Wire.begin(SDA_PORT,SCL_PORT);

  Serial.begin (115200);

  digitalWrite(D3, HIGH);
  delay(150);
  Serial.println("00");
  
  sensor.init(true);
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);

  digitalWrite(D4, HIGH);
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("addresses set");
  
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  delay(3000);



  
    }
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password);
    LED_OFF;

    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);

    setupHTTP();
    setupWS(webSocketEvent);
    //setupMDNS(mDNS_name);

    stop();
}
long int cpt=0;

void loop() {
    wsLoop();
    httpLoop();


    // _______________
  // ::: Counter :::
  
  // Display data counter
  Serial.print (cpt++,DEC);
  Serial.print ("\t");
  
  // _____________________
  // :::  Magnetometer ::: 

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx=(Mag[1]<<8 | Mag[0]);
  int16_t my=(Mag[3]<<8 | Mag[2]);
  //int16_t mz=(Mag[5]<<8 | Mag[4]);

  if (max_mx < mx)
    max_mx = mx;
  if (min_mx > mx)
    min_mx = mx;

  if (max_my < my)
    max_my = my;
  if (min_my > my)
    min_my = my;

  // subtracts the offsets
  int16_t mx_calib = (mx - (max_mx + min_mx)/2);
  int16_t my_calib = (my - (max_my + min_my)/2);
 
  
  // calculate the scaling
  int16_t avg_mx = (max_mx - min_mx)/2;
  int16_t avg_my = (max_my - min_my)/2;
  float avg_delta = (avg_mx + avg_my) / 2.0;
  float scale_x = avg_delta / avg_mx;
  float scale_y = avg_delta / avg_my;  
  mx_calib = mx_calib * scale_x;
  my_calib = my_calib * scale_y; 

  Serial.print(max_mx);
  Serial.print("\t");
  Serial.print(min_mx);
  Serial.print("\t");
  Serial.print(max_my);
  Serial.print("\t");
  Serial.print(min_my);
  Serial.print("\t");
  Serial.print(scale_x);
  Serial.print("\t");
  Serial.print(scale_y);
  Serial.print("\t");
  Serial.print(mx_calib);
  Serial.print("\t");
  Serial.println(my_calib);

  // swap the values (due to hardware mismatch)
  float heading = atan2(-mx_calib, my_calib);

  // Once you have your heading, you must then add your 'Declination Angle',
  // which is the 'Error' of the magnetic field in your location. Mine is 0.0404 
  // Find yours here: http://www.magnetic-declination.com/
  
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
//  float declinationAngle = 0.0404;
//  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/PI; 
//
//  Serial.print("\rHeading:\t");
//  Serial.print(heading);
//  Serial.print(" Radians   \t");
//  Serial.print(headingDegrees);
//  Serial.print(",");
//  Serial.println(" Degrees   \t");

  Serial.print ("Magnetometer readings:"); 
  Serial.print ("\tMx:");
  Serial.print (mx); 
  Serial.print(",");
  Serial.print ("\tMy:");
  Serial.print (my);
//  Serial.print ("\tMz:");
//  Serial.print (mz);  
  Serial.println("\tHeading:");
  Serial.print(heading);
  Serial.println("\tHeading Degrees:");
  Serial.print(headingDegrees);
  Serial.println ("\t");
  Serial.print("\n");
  
  
  // End of line
  delay(100); 


// sensors

  Serial.print("Lidar 1 range(mm): ");
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.print("  Lidar 2 range(mm): ");
  Serial.println(sensor2.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }

  
//self drive
/*if(sensor.readRangeSingleMillimeters()>260){

backward();
  
}

if((sensor.readRangeSingleMillimeters()<=260)&&(sensor2.readRangeSingleMillimeters()<=160)){

do{
  right();
}
  while(mx>-180&&my<60);

  backward();
}

*/



}


//
// Movement Functions //
//

void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

void forward() {
  DEBUG("forward");
  drive(0, 180);
}

void backward() {
  DEBUG("backward");
  drive(180, 0);
}

void left() {
  DEBUG("left");
  drive(180, 180);
}

void right() {
  DEBUG("right");
  drive(0, 0);
}



//
// Setup //
//

void setupPins() {
    // setup Serial, LEDs and Motors
    Serial.begin(115200);
    DEBUG("Started serial.");

    pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    DEBUG("Setup motor pins");
}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);

            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            break;
        }
        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);

            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);

        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);

           
            //our code
            //wsSend(1, sensor.readRangeSingleMillimeters())
           // wsSend(2, sensor2.readRangeSingleMillimeters())
            //wsSend(id, "Hello world!")
            
            if (payload[0] == '#') {
                if(payload[1] == 'C') {
                  LED_ON;
                  //string stringl1 ;
                  //char* l1 = (char*)sensor.readRangeSingleMillimeters();
                  wsSend(id, "hi");
                  
                  
                }
                else if(payload[1] == 'F') 
                  backward();
                  //Serial.print("ours:");
                 // Serial.print( sensor.readRangeSingleMillimeters());
                else if(payload[1] == 'B') 
                  forward();
                else if(payload[1] == 'L') 
                  right();
                else if(payload[1] == 'R') 
                  left();
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') 
                    servo_left_ctr -= 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr += 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    servo_left_ctr += 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr -= 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else 
                  stop();
            }

            break;
    }
}


#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);


float baselinePressure;
const int chipSelect = 10; //true ss pin is 10
int piezoPin = 22; //true piezo pin in 22
int servoPin = 20;  //true servo pin is 20
int ledPin = 9; //true led pin is 9
float MAX_HEIGHT = 238;
int heightRound = 0;
float minFallingRate = 6; //initial falling rate that activates chute
float timeMulti = .5;  //time between height resets for calculating fallingRate
Servo myservo;  

boolean isStartUp = true;  //boolean true if calibrating
float fixRoll = 0;  //values for adjusting to calibration 
float fixPitch = 0;
float fixHeading = 0;

float lastHeight = -2;  //vlaues for calulating fallingRate
float lastTime = 0;

float adjTime = 0; //resets to make time appear as 0 in log when launched
boolean hasLaunched = false;

File dataFile;

void setup(){
  noTone(piezoPin);
  
  pinMode(SS, OUTPUT);
  pinMode(ledPin, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    //failed, did not find chip
    error();
  }
  tone(piezoPin, 349);
  // Open up the file we're going to log to!
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  
  if (! dataFile) {
    // Wait forever since we cant write data
    error();
  }
  
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    error2();
  }
  delay(200);
  tone(piezoPin, 392);
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    error2();
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    error2();
  }
  
  sensors_event_t bmp_event;
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure){
    baselinePressure = bmp_event.pressure;
  }
  
  delay(200); //success intiation sound
  tone(piezoPin, 440);
  myservo.attach(servoPin);
  delay(200);
  noTone(piezoPin);
  myservo.write(150);
  delay(6000);//wait for spring to be compressed
  myservo.write(90);
}

void loop(){
  
  noTone(piezoPin);

  float height = 0;
  long time = (millis())/1000; //change to sec
  
  // make a string for assembling the data to log:
  String dataString = "";
  
  sensors_event_t bmp_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;
  
  if(isStartUp){
      bmp.getEvent(&bmp_event);
      if (bmp_event.pressure){
        baselinePressure = bmp_event.pressure;
        //set baseling pressure again if reset
      }
      if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)){
        /* 'orientation' should have valid .roll and .pitch fields */
        //set calibrated values so appears as 0 in log
        float fixRoll = orientation.roll;
        float fixPitch = orientation.pitch;
        float fixHeading = orientation.heading;
      }     
      isStartUp = false; 
      //do not run again
  }
 
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure){
    /*atmospheric pressure in hPa */
    /*ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    
    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    height = bmp.pressureToAltitude(baselinePressure, bmp_event.pressure, temperature); 
    //height in meters
    
    /*changes min falling rate that activates the chute at different heights so that 
    it ignores pressure spikes during setup and is sensitive at apogee*/
    if(height > 100){
        heightRound = 100;
        minFallingRate = 1.5; 
    }else if(height > 30){
        heightRound = 30;
        minFallingRate = 4;  
    }else{
        minFallingRate = 10; 
        heightRound = 0; 
    }
  }
  
  
  if(height >= MAX_HEIGHT){
    myservo.write(150);
    tone(piezoPin, 440);
    dataFile.println("max height reached");
    //deploy chute when max height is reached
    
  }
  if((lastHeight - height) > minFallingRate){
    //if falling at a certing rate (minFallingRate) deploy chute
    myservo.write(150);
    tone(piezoPin, 440);
    float diff = (lastHeight - height)*(1/timeMulti);  //finds diff and m/s
    dataFile.print("Falling m/s ");
    dataFile.println(diff);
  }
  
  float heightInFeet = height * 3.2808;
  float syncTime = time - adjTime;
  dataString = dataString + syncTime + " seconds, height: " + height + "m, " + heightInFeet + "ft. roll: ";
  
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)){
    /* 'orientation' should have valid .roll and .pitch fields */
    float adjRoll = orientation.roll - fixRoll;
    float adjPitch = orientation.pitch - fixPitch;
    float adjHeading = orientation.heading - fixHeading;
    //adjust all values with calibration values
    
    dataString = dataString + adjRoll + " pitch: " + adjPitch + " heading: "
    + adjHeading + " degrees. ";
    
  }
  
  if(accel_event.acceleration.y && accel_event.acceleration.x && accel_event.acceleration.z){
    float y = accel_event.acceleration.y;
    float x = accel_event.acceleration.x;
    float z = accel_event.acceleration.z;;

    float accel = sqrt(y*y+x*x+z*z);
    
    //find final vector of acceleration w/o direction and combining x,y,z
    dataString = dataString + "accel: " + accel;
    
    if(!hasLaunched && (accel > 20)){
        hasLaunched = true;
        adjTime = time;
    }
  }
  
  dataFile.println(dataString);

  // The following line will 'save' the file to the SD card after every
  // line of data - this will use more power and slow down how much data
  // you can read but it's safer! 
  // If you want to speed up the system, remove the call to flush() and it
  // will save the file only every 512 bytes - every time a sector on the 
  // SD card is filled with data.
  dataFile.flush();
  
  if((time - lastTime) > timeMulti){
    lastHeight = height;
    lastTime = time;
    //every .5 seconds (or whatever timeMulti is) reset height and time in order to calulate falling rate
  }
  
  // Take 1 measurement every 100 milliseconds aka 10 times per second
  analogWrite(ledPin, 0);//led indicator for sucess
  delay(10);  //time between writes (each write takes about 50-25 millisec)
  analogWrite(ledPin, 5);
}

void error(){
   while(1){
      tone(piezoPin, 400);
      analogWrite(ledPin,0);
      delay(600);
      noTone(piezoPin);
      analogWrite(ledPin, 255);
      delay(600);
   } 
}

void error2(){
   while(1){
      tone(piezoPin, 400);
      analogWrite(ledPin,0);
      delay(600);
      tone(piezoPin, 500);
      analogWrite(ledPin, 255);
      delay(600);
      
   } 
}

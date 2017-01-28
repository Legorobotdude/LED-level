//LED Bubble Level
//By Aditya Bawankule
//Simulates a (reversed) bubble level using a NeoPixel NeoMatrix and an Arduino 101

#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#ifndef PSTR
 #define PSTR // Make Arduino Due happy
#endif

#define PIN 6//This is the pin the NeoMatrix is connected to

int width = 5;
int height = 8;

int degMax = 30;//This is the degree range the display will show lines between. If the Arduino is turned past the range, nothing will show. (30 means it goes between -30 degrees and 30 degrees)


Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(width, height, PIN,//NeoMatrix setup
  NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB            + NEO_KHZ800);

Madgwick filter;//Setup for Madgwick
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;


const uint16_t colors[] = {//Predefine RGB colors, adjust to your liking for different display colors
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(0, 0, 255) };
  
void setup() {
  //Serial.begin(9600);

  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();


  matrix.begin();//More NeoMatrix setup
  matrix.setTextWrap(false);
  matrix.setBrightness(10);//Set your preffered brightness here
  
}
void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    //Unomment the following 2 lines if you want to inverse the movement for a more traditional bubble level
    //roll = -roll;
    //pitch = -pitch;
    
    matrix.fillScreen(0);//Clear the screen

    if (width % 2) { //if x is odd, only use a single line
      
      int xPos = map(-roll,-degMax,degMax,0,width-1);
      matrix.drawFastVLine(xPos, 0, height, colors[2]);//draw a line
      }
    else{//else use a double wide line since we need an absolute center

     int xPos = map(-roll,-degMax,degMax,1,width-2);
      matrix.drawFastVLine(xPos, 0, height, colors[2]);//draw two lines
      matrix.drawFastVLine(xPos+1, 0, height, colors[2]);
    }
    if (height % 2) { //if y is odd, only use a single line
      
      
      int yPos = map(pitch,-degMax,degMax,0,height-1);
      matrix.drawFastHLine(0, yPos, height, colors[1]);//draw a line
      }
    else{//else use a double wide line since we need an absolute center

     
     int yPos = map(pitch,-degMax,degMax,1,height-2);
      matrix.drawFastHLine(0, yPos, height, colors[1]);//draw two lines
      matrix.drawFastHLine(0, yPos+1, height, colors[1]);
    }
   
       

    
    //Serial.print(roll);
    //Serial.print(heading);//uncomment these lines if your level is not working correctly to debug the headings
   
    
    matrix.show();//draw changes to the matrix
    
     // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }

delay(10);
}


float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

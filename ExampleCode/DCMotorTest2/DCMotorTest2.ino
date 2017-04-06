/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Braccio.h>
#include <Servo.h>
#include <math.h>

class color {
  public:
  int r;
  int g;
  int b;
  String nam;
  color(){}
  color(int red, int green, int blue, String n){
    r = red;
    g = green;
    b = blue;
    nam = n;
  }
  
};

  Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;
#define S0 31
#define S1 33
#define S2 35
#define S3 37
#define sensorOut 39

#define rMap 250
#define gMap 300//420
#define bMap 300//350
#define NUM_SUPPORTED_COLORS 4

int frequency = 0;
long gCounter = 0;
long rCounter = 0;
long bCounter = 0;
long bAcc = 0;
long rAcc = 0;
long gAcc = 0;
long rMin = 100000;
long gMin = 100000;
long bMin = 100000;
long rMax = 0;
long gMax = 0;
long bMax = 0;
color colorList[NUM_SUPPORTED_COLORS];

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  color green = color(110, 170, 90, "GREEN");
  color red = color(220, 30, 60, "RED");
  color blue = color(80, 150, 185, "BLUE");
  color table = color(200, 190, 180, "TABLE");
  colorList[0] = green;
  colorList[1] = red;
  colorList[2] = blue;

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

   AFMS.begin();  // create with the default frequency 1.6KHz

  Braccio.begin();

  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor1->setSpeed(150);  
  myMotor2->setSpeed(150);
  myMotor3->setSpeed(150);
  myMotor4->setSpeed(150);

//  myMotor->run(FORWARD);
  // turn on motor
//  myMotor->run(RELEASE);
}

int mapColor(int r, int g, int b){
  String color;
//  if(r > 200 && g < 100 && b < 100){
//    color = "RED";
//  }
//  if(r > 100 && r < 125 && g > 150 && g < 180 && b > 80 && b < 110){
//    color = "GREEN";
//  }
  int closestColor;
  double minDiff = 1000000000;
  for(int i = 0; i < NUM_SUPPORTED_COLORS; i++){
    int red = colorList[i].r;
    int green = colorList[i].g;
    int blue = colorList[i].b;
    double redDiff = (red - r)*(red - r);
    double greenDiff = (green - g)*(green-g);
    double blueDiff = (blue - b)*(blue-b);
    double diff = sqrt(redDiff + greenDiff + blueDiff);
    if(diff < minDiff){
      minDiff = diff;
      closestColor = i;
    }
  }
  return closestColor;
  

}

void loop() {

   digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  long redrgb = map(frequency, 40,rMap,255,0);

  if(frequency > rMax){
    rMax = frequency;
  }
  if(frequency < rMin){
    rMin = frequency;
  }


  rAcc += frequency;
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  // Printing the value on the serial monitor
//  Serial.print("R= ");//printing name
//  Serial.print(frequency);//printing RED color frequency
//  Serial.print("  ");
//  delay(100);
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  long greenrgb = map(frequency, 50,gMap,255,0);
  if(frequency > gMax){
    gMax = frequency;
  }
  if(frequency < gMin){
    gMin = frequency;
  }
  gAcc += frequency;
  //Remaping the value of the frequency to the RGB Model of 0 to 255
//  frequency = map(frequency, 30,90,255,0);
  // Printing the value on the serial monitor
//  Serial.print("G= ");//printing name
//  Serial.print(frequency);//printing RED color frequency
//  Serial.print("  ");
//  delay(100);
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  long bluergb = map(frequency, 40,bMap,255,0);
  if(frequency > bMax){
    bMax = frequency;
  }
  if(frequency < bMin){
    bMin = frequency;
  }
  bAcc += frequency;
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  // Printing the value on the serial monitor
//  Serial.print("B= ");//printing name
//  Serial.print(frequency);//printing RED color frequency
//  Serial.println("  ");
//  delay(100);
  gCounter++;
  bCounter++;
  rCounter++;
  int colorNum = mapColor(redrgb, greenrgb, bluergb);
  if(colorNum == 0){//if red color
    
  }else if(colorNum == 1){//if green color
    
  }else if(colorNum == 2){//if blue color
    
  } else {//if table color
    
  }
//  if(gCounter == 10){
//    Serial.print("Ravg = ");
//    Serial.print((rAcc/rCounter));
//    Serial.print(" max=");
//    Serial.print(rMax);
//    Serial.print(" min=");
//    Serial.println(rMin);
//    Serial.println(redrgb);
//
//     Serial.print("Gavg = ");
//    Serial.print((gAcc/gCounter));
//    Serial.print(" max=");
//    Serial.print(gMax);
//    Serial.print(" min=");
//    Serial.println(gMin);
//    Serial.println(greenrgb);
//
//
//    Serial.print("Bavg = ");
//    Serial.print((bAcc/bCounter));
//    Serial.print(" max=");
//    Serial.print(bMax);
//    Serial.print(" min=");
//    Serial.println(bMin);
//    Serial.println(bluergb);
//    Serial.println(mapColor(redrgb, greenrgb, bluergb));
//  }

//  myMotor1->run(FORWARD);
//  myMotor2->run(FORWARD);
//  myMotor3->run(FORWARD);
//  myMotor4->run(FORWARD);

// the arm is aligned upwards  and the gripper is closed
                     //(step delay, M1, M2, M3, M4, M5, M6);
//  Braccio.ServoMovement(20,           0,  15, 180, 170, 0,  73);  
//
//  //Wait 1 second
////  delay(1000);
//
//  Braccio.ServoMovement(20,           180,  165, 0, 0, 180,  10);  

  //Wait 1 second
//  delay(1000);
}

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

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define sensorOut 6

#define rMap 23
#define gMap 21
#define bMap 15
int frequency = 0;
long gCounter = 0;
long rCounter = 0;
long bCounter = 0;
long bAcc = 0;
long rAcc = 0;
long gAcc = 0;
long rMin = 10000;
long gMin = 10000;
long bMin = 10000;
long rMax = 0;
long gMax = 0;
long bMax = 0;

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

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);

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

void loop() {

   digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  long redrgb = map(frequency, 2,rMap,255,0);

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
  long greenrgb = map(frequency, 2,gMap,255,0);
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
  long bluergb = map(frequency, 2,bMap,255,0);
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

  if(gCounter == 1){
    Serial.print("Ravg = ");
    Serial.print((rAcc/rCounter));
    Serial.print(" max=");
    Serial.print(rMax);
    Serial.print(" min=");
    Serial.println(rMin);
    Serial.println(redrgb);

     Serial.print("Gavg = ");
    Serial.print((gAcc/gCounter));
    Serial.print(" max=");
    Serial.print(gMax);
    Serial.print(" min=");
    Serial.println(gMin);
    Serial.println(greenrgb);


    Serial.print("Bavg = ");
    Serial.print((bAcc/bCounter));
    Serial.print(" max=");
    Serial.print(bMax);
    Serial.print(" min=");
    Serial.println(bMin);
    Serial.println(bluergb);

  }

  Serial.print("tech");
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(FORWARD);

  Serial.println("Hi");
  // the arm is aligned upwards  and the gripper is closed
                     //(step delay, M1, M2, M3, M4, M5, M6);
  Braccio.ServoMovement(20,         90, 90, 90, 90, 90,  10);  
  delay(3000);
  Braccio.ServoMovement(20,         90, 90, 90, 90, 90,  73);  
  delay(3000);
}

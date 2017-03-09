//#include <LiquidCrystal.h>
//#define PI 3.14159265
#define led 30
#define red 32
#define green 31
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
double IRsensor[3][7], Angles[3],speeds,Distance[3],gasSensor[3][7];

Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);
boolean found; //1 of the side got the source
boolean found2;
void setup(){
  Serial.begin(9600);
  AFMS.begin();
//  pinMode(0,OUTPUT); //gas Left
//  pinMode(1,OUTPUT); //gas Mid
//  pinMode(2,OUTPUT); //gas Right
//  pinMode(led,OUTPUT);
//  pinMode(green,OUTPUT);
//  pinMode(red,OUTPUT);
  pinMode(10,OUTPUT); //IR left 
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  Angles[0] = PI/4;
  Angles[1] = 0;
  Angles[2] = -PI/4;
  speeds=150;
  found=false;
  found2=false;
}
void backG(){
  for( int i=0;i<3;i++){
    for(int j=6;j>0;j--)
      gasSensor[i][j]=gasSensor[i][j-1];
  }
}
void back(){
  for( int i=0;i<3;i++){
    for(int j=6;j>0;j--)
      IRsensor[i][j]=IRsensor[i][j-1];
  }
}
void average(double a[3][7]){
  for(int i=0;i<3;i++){
    Distance[i]=0;
    for(int j=0;j<7;j++){
      Distance[i]+=a[i][j];
    }
    Distance[i]/=7;
  }
}

void lite(){
  double v, F, kmid, kside, Fx, Fy, theta,left, right, speedL, speedR;
  int i,x;
  
  kmid = .2; // hooke's law constant for one middle sensors
  kside = 0.18; // hooke's law constant two outside sensors
  Fx = 100.0; // attractive goal force in x direction (forward)
  Fy = 0.0; // robot does not move sideways (without turning)
  back();
  for(int i=10;i<13;i++){
    IRsensor[i-10][0]=analogRead(i);
    //Serial.print("IR:");
//    Serial.println(IRsensor[i-10][0]);
  }
  average(IRsensor);
  
  for (i = 0; i < 3; i++){
    if (Distance[i] >= 110 && Distance[i]<=600) x = Distance[i]; // object detected
    else if(Distance[i]>600) x=600;
    else x = 0; // nothing seen
    if (i == 1) F = kmid*x; // compute force from two inside sensors
    else F = kside*x; // compute force from two outside sensors 
    Angles[0];
    double thing;
    if(i == 0){
      thing = PI/4;
    } else if(i == 1){
      thing = 0;
    } else {
      thing = -(PI/4);
    }
//    double thing = Angles[i];
//    The 2 following lines of code were in the original code. 
//    However, a compiler bug existed for accessing the ith position of Angles
//    so these lines had to be scrapped
//    Fx = Fx - F*cos(Angles[i]); // Repulsive x component
//    Fy = Fy - F*sin(Angles[i]); // Repulsive y component

    Fx = Fx - F*cos(thing); // Repulsive x component
    Fy = Fy - F*sin(thing); // Repulsive y component

  }
  
  v = sqrt((Fx*Fx) + (Fy*Fy)); // v = magnitude of force vector
//  Serial.print("Vector");
//  Serial.println(v);
//  
  theta = atan2(Fy, Fx); // robot moves in 1st or 4th quadrant
  left=(v - (v*theta)*2/PI);
  right=(v + (v*theta)*2/PI);
  left/=100;right/=100;
  if (left > 1 || right > 1){ // if > 100% power output requested, cap the motor power.
	if (right >= left) { 
	  left = left/right; 
	  right = 1.0; 
	}
	else { 
	  right = right/left; 
	  left = 1.0; 
	}
  }
  if(left<0){
      left=0;
    }
    if(right<0){
      right=0;
    }
    Serial.print("left: ");
    Serial.println(left);
    Serial.print("right: ");
    Serial.println(right);
    M1->setSpeed(255*right);  // 10 rpm   
    M2->setSpeed(255*left);
    M3->setSpeed(255*left);
    M4->setSpeed(255*right);

}    

void loop(){
//    M1->setSpeed(60);  // 10 rpm   
//    M2->setSpeed(60);
//    M3->setSpeed(60);
//    M4->setSpeed(60);
  lite();
  M1->run(FORWARD);
  M2->run(FORWARD);
  M3->run(FORWARD);
  M4->run(FORWARD);

  delay(30);
  
}





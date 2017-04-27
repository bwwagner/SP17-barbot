/*     
 *  XFM-VM Source code
 *  Program that executes on the Arduino controller
 *  to perform item delivery.
 *  v 1.1
 *  
 *  Includes code from
 *  Arduino Color Sensing Tutorial      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
 
//INCLUDES
#include <Wire.h>
#include <QTRSensors.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Braccio.h>
#include <Servo.h>
#include <math.h>

//CONSTANTS
//PINS
#define S0 31
#define S1 33
#define S2 35
#define S3 37
#define sensorOut 39
//MOTOR CONSTANTS
#define MOVE_SPEED 100
//QTR LINE FOLLOWER CONSTANTS
#define KP 0.005 //experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define KD 0.0060 //experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define M1_minumum_speed 85  //minimum speed of the Motor1
#define M2_minumum_speed 85  //minimum speed of the Motor2
#define M1_maksimum_speed 120 //max. speed of the Motor1
#define M2_maksimum_speed 120 //max. speed of the Motor2
#define MIDDLE_SENSOR 4       //number of middle sensor used
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN 24 // emitter is controlled by digital pin 24

//COLOR MAPPING
#define R_MAP_LOW 7
#define G_MAP_LOW 13
#define B_MAP_LOW 9

#define R_MAP_HIGH 40
#define G_MAP_HIGH 58
#define B_MAP_HIGH 50

//COLOR LIST
#define NUM_SUPPORTED_COLORS 4

/*
 * DATA:
 * =========================================================================================================================================================
 * |    R    |    G    |    B    | ERROR |                                                   DESC                                                          |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  10-16  |  50-58  |  42-50  |   8   | Raw readings on Red paper, 20% Frequency scaling.                                                               |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  36-37  |  25-32  |  44-45  |   7   | Raw readings on green paper, 20% Frequency scaling.                                                             |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  25-31  |  36-42  |  43-44  |   6   | Raw readings on black electrical tape, 20% Frequency scaling.                                                   |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   7-11  |  13-14  |   9-13  |   4   | Raw readings on back of coloured paper (white), 20% Frequency scaling.                                          |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------
 * |    9    |    22   |  25-32  |   7   | Raw readings on Orange post-it note, 20% Frequency scaling.                                                     |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  12-13  |  10-15  |  27-28  |   5   | Raw readings on yellow-ish green post-it note, 20% Frequency scaling.                                           |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   8-9   |  29-35  |   22    |   6   | Raw readings on pinkish post-it note, 20% frequency scaling.                                                    |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   0-2   |   0-2   |   0-2   |   2   | Raw readings where I shone a light into the sensor, 20% frequency scaling.                                      |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  11-19  |  17-24  | 17- 24  |   8   | Raw readings of the table we are using, 20% frequency scaling. Took samples from several locations on the table.|
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------
 */

 //STUCRTURES
 enum Color_Enum {
  RED = 0,
  GREEN = 1,
  BLUE = 2,
  TABLE = 3 //error value meant to be selected when previous enumerations do not work.
};

struct color {
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

//MOTORS
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
//WHEELS
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);
//BRACCIO ARM
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;
//LINE FOLLOWER
QTRSensorsRC qtrrc((unsigned char[]) {26, 28, 30, 32, 34, 36, 38, 40}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

//GLOBALS
//DATA FLAGS
bool displayingFreq = false;
bool displayingRGB = false;

//GLOBAL OPERATION
bool movingForward = true;
//COLOR SENSOR
int frequency = 0;
color * colorList = new color[NUM_SUPPORTED_COLORS];
Color_Enum lastColor = 2; //set to blue for reasons.
unsigned int sensorValues[NUM_SENSORS];
//LINE FOLLOWER
int lastError = 0;
int last_proportional = 0;
int integral = 0;
//LOOP
unsigned int startTime;

//PERFORMANCE METRICS
long deliveryCount;
long greenReadCount;

//function prototypes
void manual_calibration();
long getRedRGB();
long getGreenRGB();
long getBlueRGB();
Color_Enum getColor(long r, long g, long b);
void moveForward();
void set_motors(int leftMotorSpeed, int rightMotorSpeed);
void moveBackward();
void switchDirection();
void waitASecond();
void dispatch(Color_Enum reading);
void dropMint();
int getError();
//=======================================================================================================================================
//---------------------------------------------------------------------------------------------------------------------------------------
//=======================================================================================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Beginning setup.");
  
  manual_calibration();
  Serial.print(".");

  color red = color(213, 60, 47, "RED");
  color green = color(89, 173, 69, "GREEN");
  color blue = color(66, 173, 184, "BLUE");
  color table = color(201, 216, 184, "TABLE");
  Serial.print(".");
  
  colorList[0] = red;
  colorList[1] = green;
  colorList[2] = blue;
  colorList[3] = table;
  Serial.print(".");

  deliveryCount = 0;
  greenReadCount = 0;
  startTime = millis();
  Serial.print(".");
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);
  Serial.print(".");
  
  Braccio.begin();
  Braccio.ServoMovement(20,         135,   45, 180,  45,  0, 73);
  Braccio.ServoMovement(20, 180, 45, 180, 45, 180, 73);
  Serial.print(".");
  
  AFMS.begin();
  Serial.println(".");
  
  delay(100);
  Serial.println("Setup complete!");
  
  moveForward(); //initial movement
}

void loop() {
  unsigned int currentTime = millis();
  
  long redrgb = getRedRGB();
  long greenrgb = getGreenRGB();
  long bluergb = getBlueRGB();

  if (currentTime - startTime >= 100){
    Color_Enum color = getColor(redrgb, greenrgb, bluergb);
    dispatch(color);
    startTime = currentTime;
  }

  if (movingForward) {
    moveForward();
  } else {
    moveBackward();
  }
}
//=======================================================================================================================================
//---------------------------------------------------------------------------------------------------------------------------------------
//=======================================================================================================================================
void manual_calibration() {
  int i;
  
  for (i = 0; i < 50; i++)
  {
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
  }
}

long getRedRGB(){
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  //Map and print
  if (displayingFreq) {
    Serial.print ("RF : ");
    Serial.println(frequency);
  }
  
  return map(frequency, R_MAP_LOW,R_MAP_HIGH,255,0);
}

long getGreenRGB () {
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  //Map and print
  if (displayingFreq) {
    Serial.print ("GF : ");
    Serial.println(frequency);
  }

  return map(frequency, G_MAP_LOW,G_MAP_HIGH,255,0);  
}

long getBlueRGB() {
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  //Map and print.
  if (displayingFreq) {
    Serial.print ("BF : ");
    Serial.println(frequency);
  }
  
  return map(frequency, B_MAP_LOW,B_MAP_HIGH,255,0);
}

void dispatch(Color_Enum color){
  if (color == RED) {
    greenReadCount = 0;
    deliveryCount = 0;
    if (lastColor != RED){
      Serial.println("Red");
      switchDirections();
      lastColor = RED;
    }
  } 
  else if (color == GREEN) {
    greenReadCount++;
    if (lastColor != GREEN) {
      deliveryCount++;
      Serial.println("Green");
      unloadMint(); 
      lastColor = GREEN;
    }
  }
  else if (color == BLUE) {
    Serial.println("Blue");
    lastColor = BLUE;
  }
  else {
    Serial.println("Table");
    lastColor = TABLE;
  }
}//end dispatch

Color_Enum getColor(long r, long g, long b){
  Color_Enum output;

  if (displayingRGB) {
    Serial.print(r);
    Serial.print(", ");
    Serial.print(g);
    Serial.print(", ");
    Serial.println(b);
  }

  String color;
  int closestColor = 4;
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

  output = closestColor;
    
  return output;
}

void switchDirections() {
  movingForward = !movingForward; //flip flag to tell the logic which way to go.
}

void moveForward() {
  int error = getError();

  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  
  int leftMotorSpeed = M1_minumum_speed + motorSpeed;
  int rightMotorSpeed = M2_minumum_speed - motorSpeed;
  set_motors(leftMotorSpeed, rightMotorSpeed);
}

void set_motors(int leftMotorSpeed, int rightMotorSpeed){
  if (leftMotorSpeed > M1_maksimum_speed ) leftMotorSpeed = M1_maksimum_speed;
  if (rightMotorSpeed > M2_maksimum_speed ) rightMotorSpeed = M2_maksimum_speed;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; 
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; 
  M1->setSpeed(rightMotorSpeed); 
  M2->setSpeed(leftMotorSpeed);
  M1->run(FORWARD); 
  M2->run(FORWARD);
  M3->setSpeed(leftMotorSpeed); 
  M4->setSpeed(rightMotorSpeed);
  M3->run(FORWARD); 
  M4->run(FORWARD);
}

void moveBackward(){
  M1->setSpeed(MOVE_SPEED);
  M2->setSpeed(MOVE_SPEED);
  M3->setSpeed(MOVE_SPEED);
  M4->setSpeed(MOVE_SPEED);

  M1->run(BACKWARD);
  M2->run(BACKWARD);
  M3->run(BACKWARD);
  M4->run(BACKWARD);
}

void unloadMint() {
  M1->setSpeed(0);
  M2->setSpeed(0);
  M3->setSpeed(0);
  M4->setSpeed(0);

  dropMint();//this is where we will use thearm logic to deliver our mint from the robot.

  //continue on our path.
  M1->setSpeed(MOVE_SPEED);
  M2->setSpeed(MOVE_SPEED);
  M3->setSpeed(MOVE_SPEED);
  M4->setSpeed(MOVE_SPEED);
}

void dropMint(){
  delay(500);
    //Starting position
                      //(step delay  M1 , M2 , M3 , M4 , M5 , M6);
    Braccio.ServoMovement(20,           128,  45, 180, 180,  180,  10);
  
    //Wait 1 second
    delay(1000);

    //The braccio moves to the sponge. Only the M2 servo will moves
    Braccio.ServoMovement(20,           128,  78, 180, 180,  180,   10);

    //Close the gripper to take the sponge. Only the M6 servo will moves
    Braccio.ServoMovement(10,           128,  78, 180, 180,  180,  70 );//166
    Braccio.ServoMovement(10,           128,  45, 180, 180,  180,  70 );//166

    //Brings the sponge upwards.
    Braccio.ServoMovement(20,         180,   45, 180,  45,  180, 73);

    //Show the sponge. Only the M1 servo will moves
    Braccio.ServoMovement(20,         180,  45, 180,   45,   90,  73);

    //Return to the start position.
    Braccio.ServoMovement(20,         180,   120, 145,  180,  90, 73);

    //Open the gripper
    Braccio.ServoMovement(20,         180,   140, 145,  180,  90, 10 );
    Braccio.ServoMovement(20,         180,   100, 145,  180,  90, 10 );
}

int getError(){
  int position = qtrrc.readLine(sensorValues); //get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  Serial.println(position);
  delay(10);

  return position - 3500;
}


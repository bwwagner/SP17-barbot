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

#include <Adafruit_MotorShield.h>

//MOTORS
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);

//CONSTANTS
//PINS
#define S0 31
#define S1 33
#define S2 35
#define S3 37
#define sensorOut 39
//MOTOR CONSTANTS
#define MOVE_SPEED 100

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

//GLOBALS
//DATA FLAGS
bool displayingFreq = false;
bool displayingRGB = false;

//GLOBAL OPERATION
int frequency = 0;
bool movingForward = true;

enum Color_Enum {
  RED = 0,
  GREEN = 1,
  BLUE = 2,
  TABLE = 3 //error value meant to be selected when previous enumerations do not work.
};

//function prototypes
long getRedRGB();
long getGreenRGB();
long getBlueRGB();
Color_Enum getColor(long r, long g, long b);
void moveForward();
void moveBackward();
void switchDirection();
void waitASecond();
void dispatch(Color_Enum reading);

//color list instantiation
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

color * colorList = new color[NUM_SUPPORTED_COLORS];

//---------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.println("Beginning setup...");

  color red = color(220, 30, 60, "RED");
  color green = color(110, 170, 90, "GREEN");
  color blue = color(80, 150, 185, "BLUE");
  color table = color(200, 190, 180, "TABLE");
  
  colorList[0] = red;
  colorList[1] = green;
  colorList[2] = blue;
  colorList[3] = table;
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);

  AFMS.begin();
  
  Serial.begin(9600);
  moveForward(); //initial movement
}

void loop() {  
  long redrgb = getRedRGB();
  long greenrgb = getGreenRGB();
  long bluergb = getBlueRGB();
  
  Color_Enum color = getColor(redrgb, greenrgb, bluergb);

  dispatch(color);

  delay(1000);
}
//---------------------------------------------------------------------------------------------------------------------------------------

void dispatch(Color_Enum color){
  int count = 0; // count of orders delivered on this pass
  if (color == RED) {
    Serial.println("Red");
	Serial.println(count);
	count = 0;
    switchDirections();
  } 
  else if (color == GREEN) {
    Serial.println("Green");
	count++;
    unloadMint(); 
  }
  else if (color == BLUE) {
    Serial.println("Blue");
  }
  else {
    Serial.println("Table");
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

Color_Enum getColor(long r, long g, long b){
  Color_Enum output;

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

void moveForward() {
  M1->setSpeed(MOVE_SPEED);
  M2->setSpeed(MOVE_SPEED);
  M3->setSpeed(MOVE_SPEED);
  M4->setSpeed(MOVE_SPEED);

  M1->run(FORWARD);
  M2->run(FORWARD);
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

void switchDirections() {
  if (movingForward){
    moveBackward();
  } else {
    moveForward();
  }

  movingForward = !movingForward; //flip flag to tell the logic which way to go.
}

void unloadMint() {
  M1->setSpeed(0);
  M2->setSpeed(0);
  M3->setSpeed(0);
  M4->setSpeed(0);

  delay (1000);//this is where we will use thearm logic to deliver our mint from the robot.

  //continue on our path.
  M1->setSpeed(MOVE_SPEED);
  M2->setSpeed(MOVE_SPEED);
  M3->setSpeed(MOVE_SPEED);
  M4->setSpeed(MOVE_SPEED);
}


#include <QTRSensors.h> //Pololu QTR Sensor Library. First you must download and install QTRSensors library
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
  
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);
  
#define KP 0.005 //experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define KD 0.0060 //experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define M1_minumum_speed 85  //minimum speed of the Motor1
#define M2_minumum_speed 85  //minimum speed of the Motor2
#define M1_maksimum_speed 120 //max. speed of the Motor1
#define M2_maksimum_speed 120 //max. speed of the Motor2
#define MIDDLE_SENSOR 4       //number of middle sensor used
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   24     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {26, 28, 30, 32, 34, 36, 38, 40},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
  
void setup()
{
  delay(500);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
  AFMS.begin();  // create with the default frequency 1.6KHz


//delay(1500);
manual_calibration();
//  qtrrc.calibrate(QTR_EMITTERS_ON);
//set_motors(0,0);
}
  
int lastError = 0;
int last_proportional = 0;
int integral = 0;
  
void loop()
{
//  unsigned int sensors[NUM_SENSORS];
  int position = qtrrc.readLine(sensorValues); //get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  Serial.println(position);
  delay(10);

  qtrrc.read(sensorValues);
  
//  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
//  // 2500 means minimum reflectance
//  for (unsigned char i = 0; i < NUM_SENSORS; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
//  }
//  Serial.println();
//  
//  delay(250);

  int error = position - 3500;
//  Serial.print("Error: ");
//  Serial.println(error);
  
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
//  Serial.println(motorSpeed);
  
  int leftMotorSpeed = M1_minumum_speed + motorSpeed;
  int rightMotorSpeed = M2_minumum_speed - motorSpeed;

  Serial.print("Left: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" Right: ");
  Serial.println(rightMotorSpeed);
//  delay(1000);
  //if(motorSpeed > 10 || motorSpeed < -10){
  // set motor speeds using the two motor speed variables above
    set_motors(leftMotorSpeed, rightMotorSpeed);
  //}
}
  
void set_motors(int leftMotorSpeed, int rightMotorSpeed)
{
if (leftMotorSpeed > M1_maksimum_speed ) leftMotorSpeed = M1_maksimum_speed;
if (rightMotorSpeed > M2_maksimum_speed ) rightMotorSpeed = M2_maksimum_speed;
if (leftMotorSpeed < 0) leftMotorSpeed = 0; 
if (rightMotorSpeed < 0) rightMotorSpeed = 0; 
myMotor1->setSpeed(rightMotorSpeed); 
myMotor2->setSpeed(leftMotorSpeed);
myMotor1->run(FORWARD); 
myMotor2->run(FORWARD);
myMotor3->setSpeed(leftMotorSpeed); 
myMotor4->setSpeed(rightMotorSpeed);
myMotor3->run(FORWARD); 
myMotor4->run(FORWARD);
}

//calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
void manual_calibration() {
  
int i;
for (i = 0; i < 50; i++)
{
qtrrc.calibrate(QTR_EMITTERS_ON);
delay(20);
}
  
//if (DEBUG) {
//Serial.begin(9600);
//for (int i = 0; i < NUM_SENSORS; i++)
//{
//Serial.print(qtrrc.calibratedMinimumOn[i]);
//Serial.print(' ');
//}
//Serial.println();
//  
//for (int i = 0; i < NUM_SENSORS; i++)
//{
//Serial.print(qtrrc.calibratedMaximumOn[i]);
//Serial.print(' ');
//}
//Serial.println();
//Serial.println();
//}
}




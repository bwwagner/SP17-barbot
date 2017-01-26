#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include <MeMCore.h>

MeDCMotor motor_9(9);
MeDCMotor motor_10(10);
void move(int direction, int speed)
{
      int leftSpeed = 0;
      int rightSpeed = 0;
      if(direction == 1){
          leftSpeed = speed;
          rightSpeed = speed;
      }else if(direction == 2){
          leftSpeed = -speed;
          rightSpeed = -speed;
      }else if(direction == 3){
          leftSpeed = -speed;
          rightSpeed = speed;
      }else if(direction == 4){
          leftSpeed = speed;
          rightSpeed = -speed;
      }
      motor_9.run((9)==M1?-(leftSpeed):(leftSpeed));
      motor_10.run((10)==M1?-(rightSpeed):(rightSpeed));
}
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;
double linef;
double distance;
MeLineFollower linefollower_2(2);
MeUltrasonicSensor ultrasonic_3(3);

void setup(){
}

void loop(){
    linef = linefollower_2.readSensors();
    if(((linef)==(2))){
        distance = ultrasonic_3.distanceCm();
        if((distance) > ( 10 )){
            move(1,100);
            _delay(0.1);
        }else{
            move(2,62);
            _delay(0.4);
            move(4,62);
            _delay(1);
        }
    }else if(((linef)==(1))){
      distance = ultrasonic_3.distanceCm();
        if((distance) > ( 10 )){
            move(1,100);
            _delay(0.1);
        }else{
            move(2,62);
            _delay(0.4);
            move(3,62);
            _delay(1);
        }

      
    }else if(((linef)==(3))){
    
        distance = ultrasonic_3.distanceCm();
        if((distance) > ( 10 )){
            move(1,100);
            _delay(0.1);
        }else{
            move(2,62);
            _delay(0.4);
            move(3,62);
            _delay(1);
        }
    }
    _loop();
}

void _delay(float seconds){
    long endTime = millis() + seconds * 1000;
    while(millis() < endTime)_loop();
}

void _loop(){
}

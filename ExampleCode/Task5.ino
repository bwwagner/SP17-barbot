#include <Encoder.h>
#include <Wire.h>
#include <L3G.h>
#include <Motors.h>

#define L 0
#define R 1

#define MOVE 0
#define TURN 1

L3G gyro;

#define X 0
#define Y 1
#define Z 2

int R0[3];

void turn(float);

// cm traveled per state change = 
// Wheel Circumference / State changes per sec
#define CM_PER_ST ( (3.14159*6.5) / (1000/3) )

//Encoder leftEnc(19, 18);
Encoder leftEnc(19, 16);
//Encoder rightEnc(20, 21);
Encoder rightEnc(18, 17);

Motors m0;

char state = MOVE;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  
  m0.forward( CHAN_A | CHAN_B );
  m0.setDuty( CHAN_A | CHAN_B, 200);
  m0.stop( CHAN_A | CHAN_B );
  
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
  collect_zero();

}

long oldPos[]  = {-999, -999};
boolean go = false;

void loop() {
  boolean updated = false;
  
  long newPos[2];
  newPos[L] = leftEnc.read();
  newPos[R] = rightEnc.read();
  
  if (newPos[L] != oldPos[L]) {
    oldPos[L] = newPos[L];
    updated = true;
  }
  if (newPos[R] != oldPos[R]) {
    oldPos[R] = newPos[R];
    updated = true;
  }
  
  if(updated) {
    Serial.print(newPos[L] * CM_PER_ST); Serial.print("cm ");
    Serial.print("\t");
    Serial.print(newPos[R] * CM_PER_ST); Serial.println("cm ");
  }
  
  int dist;
  switch(state) {
    case MOVE:
      m0.forward( CHAN_A | CHAN_B );
      m0.start( CHAN_A | CHAN_B );
      dist = (CM_PER_ST * (newPos[L] + newPos[R])/2);
      Serial.println();
      Serial.println(dist);
      Serial.println();
      if( dist > 100.0 ) {
        m0.stop( CHAN_A | CHAN_B );
        state = TURN;
      }
      break;
    case TURN:
      turn(90);
      state = MOVE;
      leftEnc.write(0);
      rightEnc.write(0);
      break;
    default:
      break;
  }
  
  if(Serial.available()) {
    char cmd = Serial.read();
    leftEnc.write(0);
    rightEnc.write(0);
    
    if( cmd == 'g' ) {
      go = !go;
      if(go) m0.start( CHAN_A | CHAN_B );
      else m0.stop( CHAN_A | CHAN_B );
    }
    
    if( cmd == 't' ) {
      turn( PI / 2 );
    }
    
    if( cmd >= '0' && cmd <= '9' ) {
      float scale = ((int)(cmd - '0') + 1 ) / 10.0;
      m0.setDuty( CHAN_A | CHAN_B, (int)(200*scale));
    }
    
  }
}

void turn(float angle) {
  float delt_z = 0;
  
  m0.forward( CHAN_A );
  m0.backward( CHAN_B );
  m0.setDuty( CHAN_A | CHAN_B, 200);
  m0.start( CHAN_A | CHAN_B );
  
  do {
    int Rg[3];
    gyro.read();
    
    Rg[X] = ((int)gyro.g.x - R0[X]) * 0.0073;
    Rg[Y] = ((int)gyro.g.y - R0[Y]) * 0.0073;
    Rg[Z] = ((int)gyro.g.z - R0[Z]) * 0.0073;
    
    delt_z += Rg[Z] * 0.01;
    Serial.println(delt_z);
  } while( abs(delt_z) < angle );
  
  m0.stop( CHAN_A | CHAN_B );
}

void collect_zero() {
  int R_tmp[] = {0,0,0};
  
  for(int i = 0; i < 100; i++) {
    gyro.read();
    R_tmp[X] += (int)gyro.g.x;
    R_tmp[Y] += (int)gyro.g.y;
    R_tmp[Z] += (int)gyro.g.z;
  }
  R0[X] = (int)(R_tmp[X] / 100);
  R0[Y] = (int)(R_tmp[Y] / 100);
  R0[Z] = (int)(R_tmp[Z] / 100);
}

/*     Arduino Color Sensing Tutorial
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
 
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define sensorOut 6

#define rMap 20
#define gMap 18
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
void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);
  
  Serial.begin(9600);
}
void loop() {
  // Setting red filtered photodiodes to be read
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

  if(gCounter == 10000){
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
  
}

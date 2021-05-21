#include<Arduino.h>
const int encoderPinA = 3;
const int encoderPinB = 2;
const int directionPin11 = 7;
const int directionPin12 = 8;
const int PinPWM1 = 6;
const int directionPin21 = 13;
const int directionPin22 = 12;
const int PinPWM2 = 5;

int currentTick1 = 0;
int currentTick2 = 0;
int prevticks1 = 0;
int prevticks2 = 0;
double ticksPerSec1, ticksPerSec2;

unsigned int SampleTime =10;
unsigned int oldTime=0;

void doEncoderA();
void doEncoderB();
void setup() {
Serial.begin(921600);
pinMode( directionPin11, OUTPUT);  
pinMode( directionPin21, OUTPUT);
pinMode( PinPWM1, OUTPUT);
pinMode( directionPin12, OUTPUT);  
pinMode( directionPin22, OUTPUT);
pinMode( PinPWM2, OUTPUT);

pinMode(encoderPinA, INPUT); //sorties encodeur
pinMode(encoderPinB, INPUT);
digitalWrite(encoderPinA, HIGH); // Resistance interne arduino ON
digitalWrite(encoderPinB, HIGH); // Resistance interne arduino ON

attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);


}
void Run(int power);
void Run2(int power);
void speed(int prevnbTicks1, int prevnbTicks2,double* diffpos1,double* diffpos2);


void loop() {

unsigned int newTime = millis();

if ((newTime-oldTime) >= SampleTime){
//attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
//attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);



speed(prevticks1, prevticks2, &ticksPerSec1, &ticksPerSec2);


Run(255);
Run2(255);
Serial.print(currentTick1/2); 
Serial.print("   "); 

Serial.print(currentTick2/2); 
Serial.print("   "); 

Serial.print(ticksPerSec1); 
Serial.print("   "); 
Serial.println(ticksPerSec2); 

prevticks1 = currentTick1;
prevticks2 = currentTick2;
oldTime = newTime;  

}
}


void doEncoderA() {
currentTick1 += 1;
}
void doEncoderB()
{
currentTick2 +=1 ;   
}

void Run( int power ){
 if (power > 0) {
   digitalWrite( directionPin11, LOW );
   digitalWrite( directionPin12, HIGH );
   analogWrite(PinPWM1, power);
 }
 else { 
   digitalWrite( directionPin11, HIGH );
   digitalWrite( directionPin12, LOW );
   analogWrite(PinPWM1, -power);
 }
}
void Run2( int power ){
 if (power > 0) {
   digitalWrite( directionPin21, LOW );
   digitalWrite( directionPin22, HIGH );
   analogWrite(PinPWM2, power);
 }
 else {
   digitalWrite( directionPin21, HIGH );
   digitalWrite( directionPin22, LOW );
   analogWrite(PinPWM2, -power);
 }
}
void speed(int prevnbTicks1, int prevnbTicks2,double* diffpos1,double* diffpos2){

    *diffpos1 = 1000*(currentTick1 - prevnbTicks1)/(2*SampleTime);
    *diffpos2 = 1000*(currentTick2 - prevnbTicks2)/(2*SampleTime);
   
}

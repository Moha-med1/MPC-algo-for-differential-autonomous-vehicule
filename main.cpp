#include <Arduino.h>
#include"MPC.h"
#include "Wire.h"
#include<MPU6050_light.h>
#include <PID_v1.h> 

// left motor = 1
// right motor = 2
int i =0;
int max_iter = 1000; 

// Gyro parameters 
float w_gyro_old = 0 ;
float sampleTimeGyro = 10 ;
float old_angleX = 0;
//

double Q[3]={25, 25 ,1}; 
double R[2]= {1, 1};

const int encoderPinA = 2;
const int encoderPinB = 3;
const int directionPin11 = 8;
const int directionPin12 = 7;
const int PinPWM1 = 6;
const int directionPin21 = 13;
const int directionPin22 = 12;
const int PinPWM2 = 5;

int currentTick1 = 0;
int currentTick2 = 0;
int prevticks1 = 0;
int prevticks2 = 0;
double ticksPerSec1, ticksPerSec2;

unsigned int SampleTime =100;
unsigned int PIDSampleTime = 10;
unsigned int oldTime=0;
double d= 4.46e-4; //Ticks into meters
//double c= 0.1; //TicksPerSec into m/s

MPU6050 mpu(Wire);

//initial position
double x = 0;  //initialise x
double y = 0;  //initialise y

//generate reference
double** Xref = cercleRef(max_iter, SampleTime, 0.02, 2, 0, 0, M_PI_2);
// double** Xref = rightRef(max_iter, SampleTime, 1);



double error1 = 0;
double command1 =0;
double error2 = 0;
double command2 =0;

//Specify the links and initial tuning parameters
double Kp1=2000, Ki1=0.5, Kd1=0;
double Kp2=2000, Ki2=0.5, Kd2=0;

double Vl,Vr;
double Vlopt, Vropt;

PID myPID1(&Vl, &command1, &Vlopt, Kp1, Ki1, Kd1, DIRECT);
PID myPID2(&Vr, &command2, &Vropt, Kp2, Ki2, Kd2, DIRECT); 



void doEncoderA();
void doEncoderB();

void setup() {
Serial.begin(921600);

// setting gyro

Wire.begin();  
byte status = mpu.begin();
while(status!=0){ } // stop everything if could not connect to MPU6050
delay(1000);
mpu.calcOffsets(true,true); // gyro and accelero

// setting motors and encoders

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

attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, RISING);
attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, RISING);

myPID1.SetOutputLimits(0,255);
myPID1.SetSampleTime(PIDSampleTime);
myPID1.SetMode(AUTOMATIC);

myPID2.SetOutputLimits(0,255);
myPID2.SetSampleTime(PIDSampleTime);
myPID2.SetMode(AUTOMATIC);

}
void Run1(int power);
void Run2(int power);
float angleX();
void speed(int prevnbTicks1, int prevnbTicks2,double* diffpos1,double* diffpos2);
void loop() {
    while(i <= max_iter){
  

        unsigned int newTime = millis();
        mpu.update();
        if ((newTime-oldTime) >= SampleTime){
            

            double w_gyro_new = mpu.getGyroX();
            double new_angleX = (sampleTimeGyro*w_gyro_new)/1000+old_angleX ;
            double mapped_angle = map(new_angleX,-100,100,-180,180);
            Serial.println("ANGLE X                  ");
            Serial.println(mapped_angle);
            double theta = (mapped_angle*M_PI)/180 ;
            double dleft = (currentTick1-prevticks1) * d;
            double dright = (currentTick2-prevticks1) * d;
            double deltaX = (1/2)*(dleft + dright)*cos(theta);
            double deltaY = (1/2)*(dleft + dright)*sin(theta);         
            x = x + deltaX;
            y = y + deltaY;
            double X[3] = {x, y, theta};

            double* Vopt = MPC(Xref, X, 20, 150, Q, R, SampleTime ,i, 0.0325, 0.445);

            speed(prevticks1, prevticks2, &Vl, &Vr);
            Vlopt = Vopt[0];
            Vropt = Vopt[1];

            myPID1.Compute();
            myPID2.Compute();

            Run1(command1);
            Run2(command2);            


            oldTime = newTime;
            i++;  
        }
    }




}

void doEncoderA() {
currentTick1 += 1;
}
void doEncoderB()
{
currentTick2 +=1 ;   
}

void Run1( int power ){
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

    *diffpos1 = d*1000*(currentTick1 - prevnbTicks1)/(SampleTime);
    *diffpos2 = d*1000*(currentTick2 - prevnbTicks2)/(SampleTime);
   
}

// float angleX(int sampleTime,float old_angleX){
//   float w_gyro_new = mpu.getGyroX();
//   float new_angleX = (sampleTimeGyro*w_gyro_new)/1000 + old_angleX ;
//   float mapped_angle = map(new_angleX,-100,100,-180,180);
//   old_angleX = new_angleX ;
//   return mapped_angle ; 
// }
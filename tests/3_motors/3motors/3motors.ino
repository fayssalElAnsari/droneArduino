#include <Wire.h>
#include <Servo.h>

Servo prop1, prop2, prop3;

float step;
////////DIFFERENT FUNCITIONING MODES/////////
//MODE 1 : FIXED SPEED AT 1150
//MODE 2 : VARYING SPEED BETWEEN 1150 & 1100
int mode = 1;

double throttle;
float desired_angle_lr = 0; 
float desired_angle_fb = 0;

void setup() {
  throttle = 0;
  step = 50;
  Serial.begin(250000);
  prop1.attach(3, 1000, 2000);
  prop2.attach(5, 1000, 2000);
  prop3.attach(7, 1000, 2000);
  prop1.writeMicroseconds(throttle);
  prop2.writeMicroseconds(throttle);
  prop3.writeMicroseconds(throttle);
  Serial.println("waiting 7s");
  if(mode == 1){
        Serial.println("Running in fixed mode at: 1150");
          throttle = 1000;
  }
    if(mode == 2){
    Serial.println("Running in varying mode between 1150 & 1200");
      throttle = 1130;
   }
  delay(7000); 
}

void loop() {
  if(mode == 1){
    throttle = 1150;
  }
  if(mode == 2){
    if(throttle>1190){
    step = -10;
   }
   if(throttle<1160){
    step = 10;
   }
   throttle += step;
   Serial.println(throttle);
  }
   prop1.writeMicroseconds(throttle);
   prop2.writeMicroseconds(throttle);
   prop3.writeMicroseconds(throttle);
   delay(2000);
}

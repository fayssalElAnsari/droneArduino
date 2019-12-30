#include <Wire.h>
#include <Servo.h>

Servo prop;

double throttle;
float desired_angle_lr = 0; 
float desired_angle_fb = 0;

void setup() {
  throttle = 0;
  prop.writeMicroseconds(throttle);
  Serial.begin(250000);
  prop.attach(5, 1000, 2000);
  Serial.println("waiting 7s");
  delay(7000); 
  throttle = 1000;
}

void loop() {
   prop.writeMicroseconds(throttle);
   Serial.println("waiting 2s");
   delay(2000);
   throttle =+ 50;
}

#include <Wire.h>
#include <Servo.h>

Servo prop;

double throttle;
float desired_angle_lr = 0; 
float desired_angle_fb = 0;

void setup() {
  throttle = 0;
  Serial.begin(250000);
  prop.attach(3, 1000, 2000);
  prop.writeMicroseconds(throttle);
  Serial.println("waiting 7s");
  delay(7000); 
  throttle = 1000;
}

void loop() {
   prop.writeMicroseconds(throttle);
   Serial.println(throttle);
   delay(2000);
   throttle += 50;
}

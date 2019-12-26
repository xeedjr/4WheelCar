/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
#include <HCSR04.h>
#include "motor.h"
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
// Initialize sensor that uses digital pins 13 and 12.
int triggerPin = 31;
int echoPin = 30;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);
 
// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
  Serial.begin(115200);
  Serial.println("Hello world");
  delay(2000);// Give reader a chance to see the output.
}
 
// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second
//  Serial.println("Hello");
  loop_motor();
  double distance = distanceSensor.measureDistanceCm();
  Serial.println(distance);
}

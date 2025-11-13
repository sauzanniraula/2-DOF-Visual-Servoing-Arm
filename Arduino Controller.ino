#include <Servo.h>

// Create Servo objects
Servo servoBase;  // Servo B (Pin 9)
Servo servoElbow; // Servo A (Pin 10)

void setup() {
  // Attach servos to their pins
  servoBase.attach(9);
  servoElbow.attach(10);
  
  // Start Serial connection to listen to Python
  Serial.begin(9600);
  
  // Set initial "home" position
  servoBase.write(90);
  servoElbow.write(90);
}

void loop() {
  // Check if Python sent any data
  if (Serial.available() > 0) {
    // Read the command prefix (B or E)
    char command = Serial.read();

    // 'B' = Base Servo, 'E' = Elbow Servo
    if (command == 'B') {
      int angle = Serial.parseInt();
      servoBase.write(angle);
    } 
    else if (command == 'E') {
      int angle = Serial.parseInt();
      servoElbow.write(angle);
    }
  }
  
  // A small delay to stabilize the serial buffer
  delay(5);
}
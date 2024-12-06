#include <ServoTimer2.h>
#include<Pixy2.h>

// Initialize servo objects
ServoTimer2 leftServo;
ServoTimer2 rightServo;
Pixy2 pixy;

// Define servo pins
const int leftServoPin = 7;
const int rightServoPin = 8;

// Servo positions \
// const int servoStop = 0;      // Neutral position to stop servos
// const int servoForward = 1700;  // Forward position (adjust as needed)

void setup() {
  //Serial.begin(115200);
  pixy.init();
  // Attach servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);

  // Stop the servos
  leftServo.write(1500);
  rightServo.write(1500);

  pinMode(5, OUTPUT);
}

void loop() {
  pixy.ccc.getBlocks();
  if(pixy.ccc.numBlocks){
    digitalWrite(5, HIGH);
    moveForward();
    delay(1000);
  }
  else{
    digitalWrite(5, LOW);
    stopRobot();
    delay(100);
  }
  // moveForward();
  // delay(1000);
  // stopRobot();
  // delay(1000);
}
void moveForward() {
  leftServo.write(1700);
  rightServo.write(1300);
}

void stopRobot() {
  leftServo.write(1500);
  rightServo.write(1500);
}

#include <Servo.h>
#include<Pixy2.h>
#include<PID_v1.h>

// Initialize servo objects
Servo leftServo;
Servo rightServo;
Pixy2 pixy;

double Kp = 1.0, Ki = .1, Kd = .05;
double Setpoint = 160; //center of frame = target
double Input, Output; //for pid

PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //direct is how output responds to error

// Define servo pins
const int leftServoPin = 7;
const int rightServoPin = 8;

void setup() {
  Serial.begin(9600);

  pixy.init();
  
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-300, 300);

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
    Input = pixy.ccc.blocks[0].m_x; //gets x coords of block
    //Serial.println(Input);
    pid.Compute();
    setMotorSpeeds(Output);
    Serial.println("GOOOOO");
    delay(10);
  }
  else{
    digitalWrite(5, LOW);
    Serial.println("no obj stop");
    stopRobot();
    delay(10);
  }
}

void setMotorSpeeds(double output){
  int leftServoSpeed = constrain(1700 - output, 1500, 1700);
  int rightServoSpeed = constrain(1500 - output, 1300, 1500);


  // Serial.print(leftServoSpeed);
  // Serial.print(" | ");
  // Serial.println(rightServoSpeed);

  leftServo.write(leftServoSpeed);
  rightServo.write(rightServoSpeed);
}


void stopRobot() {
  leftServo.write(1500);
  rightServo.write(1500);
}


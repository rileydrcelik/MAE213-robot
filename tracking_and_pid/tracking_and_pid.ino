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

// Servo positions \
// const int servoStop = 0;      // Neutral position to stop servos
// const int servoForward = 1700;  // Forward position (adjust as needed)

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

  pinMode(9, INPUT);  pinMode(4, OUTPUT);   // Left IR LED & Receiver
  pinMode(3, INPUT);  pinMode(2, OUTPUT);   // Left IR LED & Receiver
}

void loop() {
  int irLeft = irDetect(4, 9, 44000);       // Check for object
  int irRight = irDetect(2, 3, 44000);       // Check for object

  irLeft = 1-irLeft;
  irRight = 1-irRight;

  // Serial.print(irLeft);
  // Serial.print(" | ");
  // Serial.println(irRight);

  if (irLeft == 1 && irRight == 1){
    digitalWrite(5, HIGH);
    Serial.println("TOO CLOSE");
    stopRobot();
  }
  else{
    digitalWrite(5, LOW);

    pixy.ccc.getBlocks();
    if(pixy.ccc.numBlocks){
      Input = pixy.ccc.blocks[0].m_x; //gets x coords of block
      //Serial.println(Input);
      pid.Compute();
      setMotorSpeeds(Output);
      Serial.println("GOOOOO");
      delay(10);
    }
    else{
      Serial.println("no obj stop");
      stopRobot();
      delay(10);
    }
  }
  delay(10);
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

int irDetect(int irLedPin, int irReceiverPin, long frequency){
  tone(irLedPin, frequency, 8);              // IRLED 38 kHz for at least 1 ms
  delay(1);                                  // Wait 1 ms
  int ir = digitalRead(irReceiverPin);       // IR receiver -> ir variable
  delay(1);                                  // Down time before recheck
  return ir;                                 // Return 1 no detect, 0 detect
}  


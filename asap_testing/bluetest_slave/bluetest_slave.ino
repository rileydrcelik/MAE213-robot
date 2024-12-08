//bluetest slave

#include<SoftwareSerial.h>

SoftwareSerial BTSerial (0, 1);

void setup(){
  Serial.begin(9600);
  BTSerial.begin(9600);
}
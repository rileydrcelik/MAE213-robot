#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial BTSErial(0, 1);

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  lcd.being();
  lcd.backlight();
  lcd.setCursor(0, 0);
}

int choiceMade = 0;

void loop() {
  lcd.print("Hello, World!");

  // while (choiceMade == 0) {
  //   //code to get user input
  //   BTSerial.println(input);
  //   choiceMade = 1;
  // }
  //switch choice?
}
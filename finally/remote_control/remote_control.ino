#include <LiquidCrystal_I2C.h>

const int VRxPin = A0;
const int VRyPin = A1;
const int SWPin = 2;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);

  pinMode(SWPin, INPUT_PULLUP);

  Serial.begin(9600);
}

int page = 1;
int pressed = 0;

void loop() {
  lcd.clear();

  int x = analogRead(VRxPin);
  int y = analogRead(VRyPin);
  int button = digitalRead(SWPin);

  lcd.clear();
  if (((button == LOW) || (page == 1)) && pressed == 0){
    if (y > 600) {
      page += 1;
      if (page == 3) {
        page = 0;
      }
    } else if (y < 400) {
      page -= 1;
      if (page == -1) {
        page = 2;
      }
    }
    switch (page) {
      case 1:
        lcd.print("Item to Track:");
        lcd.setCursor(0, 1);
        lcd.print("<-- Select -->");
        delay(200);
        break;
      case 2:
        lcd.print("Yellow Puck");
        delay(200);
        break;
      case 0:
        lcd.print("red ball");
        delay(200);
        break;
    }
  }
  else if((button == HIGH) || pressed == 1){
    pressed = 1;
    lcd.setCursor(0, 0);
    lcd.print("Item Selected: ");
    switch (page){
      case 2:
        lcd.setCursor(0, 1);
        lcd.print("yellow puck");
        Serial.println(2);
        delay(200);
        break;
      case 0:
        lcd.setCursor(0, 1);
        lcd.print("red ball");
        Serial.println(0);
        delay(200);
        break;
    }
  }
}

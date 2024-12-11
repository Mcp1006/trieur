#include <Arduino.h>
#include "rgb_lcd.h"

rgb_lcd lcd;
int BP0=0, BP1=2, BP2=12, POT=33;
int ValBP0=0, ValBP1=0, ValBP2=0, ValPOT=0;
void setup() {
  // Initialise la liaison avec le terminal
  Serial.begin(9600);
  pinMode(BP0,INPUT_PULLUP);
  pinMode(BP1,INPUT_PULLUP);
  pinMode(BP2,INPUT_PULLUP);
  pinMode(POT,INPUT);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(8, 2, LCD_5x8DOTS, Wire1);
  lcd.setColor(WHITE);
}

void loop() {
ValBP0=digitalRead(BP0);
ValBP1=digitalRead(BP1);
ValBP2=digitalRead(BP2);
ValPOT=analogRead(POT);

lcd.setCursor(0,0);
lcd.printf("BP0:%dBP1:%dBP2:%d",ValBP0,ValBP1,ValBP2);
lcd.setCursor(0,1);
lcd.printf("%4d",ValPOT);

}

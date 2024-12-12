#include <Arduino.h>
#include "rgb_lcd.h"
#include <ESP32Encoder.h>


#define CLK 23 // A ENCODER 
#define DT 19 // B ENCODER 


rgb_lcd lcd;
ESP32Encoder encoder;
int BP0=0, BP1=2, BP2=12, POT=33, PWM=27, DIRECTION=26, a, CNY70=36;
int ValBP0=0, ValBP1=0, ValBP2=0, ValPOT=0,ValCNY70=0;
void setup() {
  // Initialise la liaison avec le terminal
  Serial.begin(115200);
  pinMode(BP0,INPUT_PULLUP);
  pinMode(BP1,INPUT_PULLUP);
  pinMode(BP2,INPUT_PULLUP);
  pinMode(POT,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(DIRECTION,OUTPUT);
  pinMode(CNY70,INPUT);
  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(8, 2, LCD_5x8DOTS, Wire1);
  lcd.setColor(WHITE);
  // Initialise PWM
  ledcSetup(0, 500, 10);
  ledcAttachPin(PWM, 0);
  encoder.attachHalfQuad ( DT, CLK );
  encoder.setCount ( 0 );
  while(ValCNY70<4000)
  {
    ValCNY70=analogRead(CNY70);
    ledcWrite(0, 250);
  }
    encoder.setCount ( 0 );
}

void loop() {

//lecture et affichage des boutons
ValBP0=digitalRead(BP0);
ValBP1=digitalRead(BP1);
ValBP2=digitalRead(BP2);
ValCNY70=analogRead(CNY70);


lcd.setCursor(0,0);
lcd.printf("VAL:%4d",ValCNY70);
//Serial.printf("BP0:%dBP1:%dBP2:%d\n",ValBP0,ValBP1,ValBP2);


//lecture et affichage du potetiometre
ValPOT=analogRead(POT);
//lcd.setCursor(0,1);
//lcd.printf("%4d",ValPOT);
//Serial.printf("%4d\n",ValPOT);


//commande pwm
if (ValBP1 == 0)
{
  digitalWrite(DIRECTION, HIGH);
}
if (ValBP1 == 1)
{
  digitalWrite(DIRECTION, LOW);
}
ledcWrite(0, ValPOT/4);
  long newPosition = encoder.getCount();
  Serial.println(newPosition);
  lcd.setCursor(0,1);
  lcd.printf("nbTour:%3d",newPosition/420);
}


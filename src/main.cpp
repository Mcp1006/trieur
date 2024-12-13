#include <Arduino.h>
#include "rgb_lcd.h"
#include <ESP32Encoder.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>

#define CLK 23 // A ENCODER 
#define DT 19 // B ENCODER 
rgb_lcd lcd;
ESP32Encoder encoder;
int BP0=0, BP1=2, BP2=12, POT=33, PWM=27, DIRECTION=26, a, CNY70=36, ETAT = 0, lux, i=0;
int ValBP0=0, ValBP1=0, ValBP2=0, ValPOT=0,ValCNY70=0;
long newPosition;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void position (long pos)
{
  lcd.setCursor(0,1);
  lcd.printf("nbTour:%3d",pos);
  if (newPosition < (pos*51))
  {
    digitalWrite(DIRECTION, HIGH);
    ledcWrite(0, 250);
  }
 if (newPosition > (pos*51))
  {
    digitalWrite(DIRECTION, LOW);
    ledcWrite(0, 250);
  }
  if (newPosition == (pos*51))
  {
    ledcWrite(0, 0);
  }
}

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
    ledcWrite(0, 230);
  }
    encoder.setCount ( 0 );
    ledcWrite(0, 0);
    
    
  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

}

void loop() {

//lecture et affichage des boutons
ValBP0=digitalRead(BP0);
ValBP1=digitalRead(BP1);
ValBP2=digitalRead(BP2);
ValCNY70=analogRead(CNY70);


lcd.setCursor(0,0);
//lcd.printf("VAL:%4d",ValCNY70);
//Serial.printf("BP0:%dBP1:%dBP2:%d\n",ValBP0,ValBP1,ValBP2);


//lecture et affichage du potetiometre
ValPOT=analogRead(POT);
//lcd.setCursor(0,1);
//lcd.printf("%4d",ValPOT);
//Serial.printf("%4d\n",ValPOT);


//commande pwm

//ledcWrite(0, ValPOT/4);
  if (ValCNY70 > 4094) 
  {
    encoder.setCount(0);
  }
  switch (ETAT)
  {
    case 0:
    {
      if (ValBP0 == 0)
      {
        position(3);
        ETAT = 1;
      }
      if (ValBP1 == 0)
      {
        ETAT = 1;
        position(4);
      }
    }

    case 1:
    {
      if(newPosition == 208)   //position 4
      {
        for (i=0;i<8;i++)
      }
      if(newPosition == 156)   //position 3
      {

      }
    }
  }
  
  newPosition = encoder.getCount();
  Serial.println(newPosition);
float red, green, blue;
  
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));

//  Serial.print("\t");
//  Serial.print((int)red, HEX); Serial.print((int)green, HEX); Serial.print((int)blue, HEX);
  Serial.print("\n");
lcd.setRGB(red, green, blue);

}


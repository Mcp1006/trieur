#include <Arduino.h>
#include "rgb_lcd.h"
#include <ESP32Encoder.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


#define CLK 23 // A ENCODER
#define DT 19  // B ENCODER

rgb_lcd lcd;
ESP32Encoder encoder;
int BP0 = 0, BP1 = 2, BP2 = 12, POT = 33, PWM = 27, DIRECTION = 26, csg, CNY70 = 36, etat = 0, lux, i = 0, vts = 0, erreur = 0, valPWM = 250, somme = 0, tour, SERVO = 13, newValPOT=0;
int ValBP0 = 0, ValBP1 = 0, ValBP2 = 0, ValPOT = 0, ValCNY70 = 0;
long newPosition;

#define Kp 100
#define Ki 10

void vTaskPeriodic(void *pvParameters)
{
  int64_t a, aprecedent = 0;
  TickType_t xLastWakeTime;
  // Lecture du nombre de ticks quand la tâche commence
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    a = encoder.getCount();
    vts = a - aprecedent;
    aprecedent = a;
    erreur = csg - vts;
    somme = somme + erreur;
    if(somme < -200) somme = -200;
    if(somme > 200) somme = 200;
    valPWM = Kp * erreur + Ki * somme;

    if (valPWM > 0)
    {
      if (valPWM > 2047)
      {
        valPWM = 2047;
      }
      digitalWrite(DIRECTION, LOW);
      ledcWrite(0, valPWM);
    }
    else
    {
      if (valPWM < -2047)
      {
        valPWM = -2047;
      }
      digitalWrite(DIRECTION, HIGH);
      ledcWrite(0, -valPWM);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);

void position(long pos)
{
  if (newPosition < ((pos-1) * 103 + 70))
  {
    csg = 1;
  }
  if (newPosition > ((pos-1) * 103 + 70))
  {
    csg = -1;
  }
  if((newPosition < ((pos-1) * 103 + 75)) && (newPosition > ((pos-1) * 103 + 65)))
  {
    csg = 0;
  }

}

void setup()
{
  // Initialise la liaison avec le terminal
  Serial.begin(115200);
  pinMode(BP0, INPUT_PULLUP);
  pinMode(BP1, INPUT_PULLUP);
  pinMode(BP2, INPUT_PULLUP);
  pinMode(POT, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(SERVO, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(CNY70, INPUT);
  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(8, 2, LCD_5x8DOTS, Wire1);
  lcd.setColor(WHITE);
  // Initialise PWM
  ledcSetup(0, 500, 10);
  ledcAttachPin(PWM, 0);
  
  ledcSetup(2, 50, 16);
  ledcAttachPin(SERVO, 2);
  encoder.attachFullQuad(DT, CLK);
  encoder.setCount(0);

  while (ValCNY70 < 4000)
  {
    ValCNY70 = analogRead(CNY70);
    ledcWrite(0, 230);
  }
  encoder.setCount(0);
  ledcWrite(0, 0);

  if (tcs.begin())
  {
    // Serial.println("Found sensor");
  }
  else
  {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ; // halt!
  }

  // Création de la tâche périodique
  xTaskCreate(vTaskPeriodic, "vTaskPeriodic", 10000, NULL, 2, NULL);
}

void loop()
{

  // lecture et affichage des boutons
  ValBP0 = digitalRead(BP0);
  ValBP1 = digitalRead(BP1);
  ValBP2 = digitalRead(BP2);
  ValCNY70 = analogRead(CNY70);

  lcd.setCursor(0, 0);
  // lcd.printf("VAL:%4d",ValCNY70);
  // Serial.printf("BP0:%dBP1:%dBP2:%d\n",ValBP0,ValBP1,ValBP2);

  // lecture et affichage du potetiometre
  ValPOT = analogRead(POT);
  // lcd.setCursor(0,1);
  // lcd.printf("%4d",ValPOT);
  // Serial.printf("%4d\n",ValPOT);

  // commande pwm

  // ledcWrite(0, ValPOT/4);
   tour = encoder.getCount()/825;
   newPosition = encoder.getCount() - 825*tour;
   
  
  float red, green, blue;

  tcs.setInterrupt(false); // turn on LED

  // delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);

  tcs.setInterrupt(true); // turn off LED
/*
   Serial.print("R:\t"); Serial.print(int(red));
   Serial.print("\tG:\t"); Serial.print(int(green));
   Serial.print("\tB:\t"); Serial.print(int(blue));
   Serial.print("\t");
   Serial.print((int)red, HEX); Serial.print((int)green, HEX); Serial.print((int)blue, HEX);
   Serial.print("\n");
*/
   lcd.setRGB(red, green, blue);

newValPOT = map(ValPOT, 0, 4095, 0,  65000);

switch (etat){
  case 0 : 
  {
    if(ValBP0 == 0) {etat=1;}
    break;
  }
  case 1 : 
  {
    position(6);
    break;
  }
}
  lcd.printf("%8d", newValPOT);
  ledcWrite(2, newValPOT);
  delay(100);
}


//
//    FILE: SRF05_demo.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo distance sensor
//     URL: https://github.com/RobTillaart/SRF05


#include "SRF05.h"

const int trigger1 = 13;
const int echo1   = 12;
const int trigger2 = 8;
const int echo2  = 7;
const int trigger3 = 2;
const int echo3  = 4;

unsigned long previousMillis = 0;
const long interval = 1000; // 1 detik

SRF05 SRF1(trigger1, echo1);
SRF05 SRF2(trigger2, echo2);
SRF05 SRF3(trigger3, echo3);

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println();

  SRF1.setCorrectionFactor(1.035);
  SRF2.setCorrectionFactor(1.035);
  SRF3.setCorrectionFactor(1.035);
}


void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Toggle kedua LED
    Serial.print("Jarak Depan : ");
    Serial.println(SRF1.getCentimeter(), 2);
    Serial.print("Jarak Belakang : ");
    Serial.println(SRF2.getCentimeter(), 2);
    Serial.print("Jarak Kanan : ");
    Serial.println(SRF3.getCentimeter(), 2);

  }
  
}
//  -- END OF FILE --


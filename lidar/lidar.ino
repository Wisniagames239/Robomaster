#include <Wire.h>

#define LIDAR_ADDR 0x62

void setup() {
  Wire.begin();
  Wire.setClock(100000); // Stabilne 100kHz
  Serial.begin(115200);
  
  // Resetuj czujnik na starcie
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  
  delay(500);
  Serial.println("Lidar Lite v3 gotowy do pracy!");
}

void loop() {
  // 1. Wyślij komendę pomiaru
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x00);
  Wire.write(0x04);
  if (Wire.endTransmission() != 0) {
    // Jeśli tu jest błąd, spróbuj ponownie za chwilę
    delay(100);
    return;
  }

  // 2. Sztywne opóźnienie (zamiast pętli busy, która wieszała program)
  // 20ms wystarczy na standardowy pomiar
  delay(20);

  // 3. Odczyt dystansu
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x8f); 
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom(LIDAR_ADDR, 2);
    if (Wire.available() >= 2) {
      byte highByte = Wire.read();
      byte lowByte = Wire.read();
      uint16_t distance = (highByte << 8) | lowByte;

      if (distance > 0 && distance < 4500) {
        Serial.print("Dystans: ");
        Serial.print(distance);
        Serial.println(" cm");
      }
    }
  }

  delay(30); // Prędkość pomiarów ok. 20-25 na sekundę
}
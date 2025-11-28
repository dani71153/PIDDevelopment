#include <Arduino.h>
#include <Wire.h>

// Pines I2C del ESP32 (SDA, SCL)
const int SDA_PIN = 21;
const int SCL_PIN = 22;

void scanI2C() {
  Serial.println("Escaneando bus I2C...");
  byte devicesFound = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(" - Encontrado dispositivo en 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devicesFound++;
    }
    delay(2);  // Pequeña pausa para evitar saturar el bus
  }

  if (devicesFound == 0) {
    Serial.println("No se detectaron dispositivos I2C.");
  } else {
    Serial.print("Total de dispositivos encontrados: ");
    Serial.println(devicesFound);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(1000);  // Evita bloqueos si el bus se queda tomado
  Wire.setClock(400000);  // Frecuencia estándar alta para ESP32

  Serial.println("Listo para escanear I2C (ESP32)");
}

void loop() {
  scanI2C();
  delay(2000);  // Escanea cada 2 segundos
}

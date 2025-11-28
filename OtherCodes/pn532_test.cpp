#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>

// Pines I2C ESP32
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// Pines de control del PN532 (modo I2C)
const int PN532_IRQ_PIN = 32;
const int PN532_RESET_PIN = 33;

Adafruit_PN532 pn532(PN532_IRQ_PIN, PN532_RESET_PIN);

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(1000);
  Wire.setClock(400000);

  Serial.println("Iniciando PN532 (Adafruit)...");
  if (!pn532.begin()) {
    Serial.println("No se encontro PN532. Verifica conexiones y direccion I2C (0x24/0x48).");
    while (true) delay(1000);
  }

  uint32_t versiondata = pn532.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("PN532 no responde a getFirmwareVersion()");
    while (true) delay(1000);
  }

  Serial.print("PN532 detectado. Chip: 0x");
  Serial.print((versiondata >> 24) & 0xFF, HEX);
  Serial.print("  Vers.: ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print(".");
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  pn532.SAMConfig();  // Habilita lectura pasiva
  pn532.setPassiveActivationRetries(0xFF);

  Serial.println("Acerque una tarjeta MIFARE/ISO14443A...");
}

void loop() {
  uint8_t uid[7];
  uint8_t uidLength = 0;

  bool success = pn532.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 1000);
  if (success) {
    Serial.print("Tarjeta detectada (UID ");
    Serial.print(uidLength);
    Serial.print(" bytes): ");
    for (uint8_t i = 0; i < uidLength; i++) {
      if (uid[i] < 0x10) Serial.print("0");
      Serial.print(uid[i], HEX);
      if (i < uidLength - 1) Serial.print(":");
    }
    Serial.println();
    delay(500);  // Pequeña pausa para evitar lecturas múltiples inmediatas
  } else {
    // Timeout: no se detecto tarjeta
    delay(100);
  }
}

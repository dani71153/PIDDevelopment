#include <Arduino.h>
#include <Adafruit_PN532.h>

// UART pins for ESP32
const uint8_t PN532_RX_PIN = 21;  // PN532 TX -> ESP32 RX
const uint8_t PN532_TX_PIN = 22;  // PN532 RX <- ESP32 TX
const uint32_t PN532_BAUD = 115200;
const uint8_t PN532_RESET_PIN = 23;

const uint16_t PN_READ_TIMEOUT_MS = 250;  // Timeout corregido

HardwareSerial PN532Serial(2);         
Adafruit_PN532 pn532(PN532_RESET_PIN,
                     &PN532Serial);    

enum ReaderSelector { PN = 0 };

void setup() {
  Serial.begin(115200);
  PN532Serial.begin(PN532_BAUD, SERIAL_8N1, PN532_RX_PIN, PN532_TX_PIN);

  Serial.println("Inicializando PN532 por Serial (HSU)...");

  if (!pn532.begin()) {
    Serial.println("No se encontro PN532. Revisa conexion UART y pines RX/TX.");
    while (true) delay(500);
  }

  uint32_t versiondata = pn532.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("PN532 no responde a getFirmwareVersion()");
    while (true) delay(500);
  }

  Serial.print("PN532 detectado. Chip: 0x");
  Serial.print((versiondata >> 24) & 0xFF, HEX);
  Serial.print("  Vers.: ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print(".");
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  pn532.SAMConfig();                 
  pn532.setPassiveActivationRetries(0xFF);

  Serial.println("Acerque una tarjeta MIFARE/ISO14443A...");
}

int PN532Handler(ReaderSelector selector = PN) {
  static uint32_t cardCount = 0;
  uint8_t uid[7] = {0};
  uint8_t uidLength = 0;

  switch (selector) {
    case PN: {

      // === Limpia el buffer UART del PN532 ===
      while (PN532Serial.available()) PN532Serial.read();

      bool success = pn532.readPassiveTargetID(
          PN532_MIFARE_ISO14443A,
          uid,
          &uidLength,
          PN_READ_TIMEOUT_MS);

      if (success) {
        cardCount++;
        Serial.print("[");
        Serial.print(cardCount);
        Serial.print("] Tarjeta detectada (UID ");
        Serial.print(uidLength);
        Serial.print(" bytes): ");

        for (uint8_t i = 0; i < uidLength; i++) {
          if (uid[i] < 0x10) Serial.print("0");
          Serial.print(uid[i], HEX);
          if (i < uidLength - 1) Serial.print(":");
        }
        Serial.println();
        return 1;  // tarjeta detectada
      }

      return 0;  // no tarjeta
    }

    default:
      return 0;
  }
}

void loop() {
  int tarjetaPresente = PN532Handler(PN);

  if (!tarjetaPresente) {
    delay(50);  // Polling estable (no saturar al PN532)
  }
}

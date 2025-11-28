#include <Arduino.h>

// Pines del ESP32
const int PIN_VOLT_5V   = 32;  // Sensor que mira ~5V
const int PIN_VOLT_14V  = 33;  // Sensor que mira ~14V

// Divisor del FZ0430
const float R1 = 30000.0;     // ohm
const float R2 = 7500.0;      // ohm
const float DIV_RATIO = (R1 + R2) / R2;  // ≈ 5.0

// Calibración global (de 3.16V a 4.513V)
const float K_CAL = 4.513f / 3.16f;      // ≈ 1.43

float readVoltageRaw(int pin) {
  const int samples = 50;
  uint32_t total = 0;

  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delayMicroseconds(500);
  }

  float adc = total / (float)samples;
  float vADC = (adc / 4095.0f) * 3.3f;     // tensión en el pin del ESP32
  float vReal = vADC * DIV_RATIO;          // antes del divisor (línea medida)

  return vReal;
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // hasta ~3.6V en el pin

  Serial.println("Lectura FZ0430 en D32 (~5V) y D33 (~14V) con calibracion.");
}

void loop() {
  float v5_raw   = readVoltageRaw(PIN_VOLT_5V);
  float v14_raw  = readVoltageRaw(PIN_VOLT_14V);

  // Aplicar calibración
  float v5   = v5_raw  * K_CAL;
  float v14  = v14_raw * K_CAL;

  Serial.print("D32 (~5V): ");
  Serial.print(v5, 3);
  Serial.print(" V   |   D33 (~14V): ");
  Serial.print(v14, 3);
  Serial.println(" V");

  delay(500);
}

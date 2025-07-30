#include <Arduino.h>
void calcularOffset();
// Definición de pines
#define RPWM 15     // PWM giro horario
#define LPWM 2      // PWM giro antihorario
#define R_EN 4      // Habilitación sentido horario
#define L_EN 13     // Habilitación sentido antihorario
#define L_R 32      // ADC corriente sentido antihorario
#define R_R 33      // ADC corriente sentido horario

#define PWM_FREQ 5000   // Frecuencia PWM recomendada (5 kHz)
#define PWM_RES 8       // Resolución PWM (8 bits = 0-255)

// Parámetros de medición de corriente (ajustar según módulo HW-039)
const float R_IS = 2200.0;    // Resistencia de sensado (1 kΩ por defecto)
const float K_ILIS = 150;  // Factor del BTS7960 (ajustar con calibración)
const int MUESTRAS = 1;      // Muestras para filtro de media móvil

// Canales PWM
const int PWM_CHANNEL_RPWM = 0;
const int PWM_CHANNEL_LPWM = 1;

void setup() {
  Serial.begin(115200);

  // Configurar pines de control
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);

  // Habilitar driver
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  // Configurar PWM
  ledcSetup(PWM_CHANNEL_RPWM, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_LPWM, PWM_FREQ, PWM_RES);
  ledcAttachPin(RPWM, PWM_CHANNEL_RPWM);
  ledcAttachPin(LPWM, PWM_CHANNEL_LPWM);
}

// Función para mover el motor
void moverMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, 255); // Limita rango de 0 a 255 (solo giro horario)

  ledcWrite(PWM_CHANNEL_RPWM, velocidad);
  ledcWrite(PWM_CHANNEL_LPWM, 0);
}

// Offset de los pines ADC (medir en reposo)
const int OFFSET_L_R = 0;  // Ejemplo: medir L_R en reposo y promediar
const int OFFSET_R_R = 0;  // Ejemplo: medir R_R en reposo y promediar

float leerCorrienteFiltrada(int pin, int offset) {  // Añade offset como parámetro
  const int muestras = 1;
  float suma = 0;
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(pin) - offset;  // Resta el offset
    delayMicroseconds(200);
  }
  float voltaje = (suma / muestras) * 3.3 / 4095.0;
  if (abs(voltaje) < 0.01) return 0.0;
  return (voltaje * K_ILIS) / R_IS;
}

void loop() {
  // Mover motor en giro horario a 255 PWM
  moverMotor(255);
// Uso en loop():
float corrienteHorario = leerCorrienteFiltrada(R_R, OFFSET_R_R);
float corrienteAntihorario = leerCorrienteFiltrada(L_R, OFFSET_L_R);
  // Enviar datos en el formato requerido
  Serial.print(">");
  Serial.print("corrienteHorario:");
  Serial.print(corrienteHorario, 4);  // Enviar con 4 decimales
  Serial.print(",");
  Serial.print("corrienteAntihorario:");
  Serial.print(corrienteAntihorario, 4);  // Enviar con 4 decimales
  Serial.println("\r\n");

  delay(1000);  // Espera 1 segundo antes de la siguiente lectura

//  calcularOffset();
}


void calcularOffset() {
  const int muestras = 100;
  int suma_L = 0, suma_R = 0;
  for (int i=0; i<muestras; i++) {
    suma_L += analogRead(L_R);
    suma_R += analogRead(R_R);
    delay(10);
  }
  Serial.print("Offset L_R: "); Serial.println(suma_L / muestras);
  Serial.print("Offset R_R: "); Serial.println(suma_R / muestras);
}
#include <Arduino.h>

// Definición de pines
#define RPWM 15     // PWM giro horario
#define LPWM 2      // PWM giro antihorario
#define R_EN 4      // Habilitación sentido horario
#define L_EN 13     // Habilitación sentido antihorario
#define L_R 32      // ADC corriente sentido antihorario
#define R_R 33      // ADC corriente sentido horario

#define PWM_FREQ 5000   // Frecuencia PWM recomendada (5 kHz)
#define PWM_RES 8       // Resolución PWM (8 bits = 0-255)

// Canales PWM
const int PWM_CHANNEL_RPWM = 0;
const int PWM_CHANNEL_LPWM = 1;

bool motorEncendido = false;  // Estado del motor
unsigned long tiempoInicio = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(L_R, INPUT);
  pinMode(R_R, INPUT);
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
  analogSetPinAttenuation(L_R, ADC_0db);
  analogSetPinAttenuation(R_R, ADC_6db);


  tiempoInicio = millis();  // Iniciar el tiempo
}

void moverMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  if (velocidad > 0) {
      ledcWrite(PWM_CHANNEL_RPWM, velocidad);
      ledcWrite(PWM_CHANNEL_LPWM, 0);
  } else {
      ledcWrite(PWM_CHANNEL_RPWM, 0);
      ledcWrite(PWM_CHANNEL_LPWM, 0);
  }
  motorEncendido = (velocidad > 0);
}


void loop() {
  unsigned long tiempoActual = millis();
  
  // Control del motor: encendido por 5s, apagado por 5s
  if ((tiempoActual - tiempoInicio) < 5000) {
    moverMotor(0);  // Encender motor
  } else if ((tiempoActual - tiempoInicio) < 10000) {
    moverMotor(0);    // Apagar motor
  } else {
    tiempoInicio = millis();  // Reiniciar el ciclo
  }

  // Leer valores ADC en crudo
  int adc_L_R = analogRead(L_R);
  int adc_R_R = analogRead(R_R);

  // Mostrar valores ADC en el monitor serial
  Serial.print("Tiempo: ");
  Serial.print(tiempoActual / 1000.0, 2);
  Serial.print("s, Estado: ");
  Serial.print(motorEncendido ? "Motor ENCENDIDO" : "Motor APAGADO");
  Serial.print(", ADC L_R: ");
  Serial.print(adc_L_R);
  Serial.print(", ADC R_R: ");
  Serial.println(adc_R_R);

  delay(200);  // Pequeña espera para evitar saturación de datos
}

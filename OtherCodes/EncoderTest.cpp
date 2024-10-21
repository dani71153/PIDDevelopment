#include <Arduino.h>
#include <ESP32Encoder.h>


/*Este codigo lo que hace es mover el motor una cantidad de tiempo indefinida hasta que llegue a la cantidad de pulsos del encoder asignadas en pulsos por vuelta?

En esta configuracion para el motor 3. Tenemos que da una revolucion, con 


(270 * 64)/4 = 4320 pulsos del encoder



*/
// Definición de pines Pines del motor 3. 
const int pinPWM = 21;
const int pinDir1 = 23;
const int pinDir2 = 22;
const int pinEncoderA = 19;
const int pinEncoderB = 18;

// Constantes
const int PULSOS_POR_REVOLUCION_MOTOR = 64;
const int RELACION_ENGRANAJES = 270;
const float PULSOS_POR_VUELTA_RUEDA = (PULSOS_POR_REVOLUCION_MOTOR * RELACION_ENGRANAJES)/4;

// Variables
ESP32Encoder encoder;
unsigned long startTime;

void setup() {
  // Configuración de pines de control del motor
  pinMode(pinPWM, OUTPUT);
  pinMode(pinDir1, OUTPUT);
  pinMode(pinDir2, OUTPUT);

  // Configuración del encoder
  encoder.attachSingleEdge(pinEncoderA, pinEncoderB);
  encoder.clearCount();

  // Inicializar la comunicación serial
  Serial.begin(115200);

  // Configurar motor para girar a máxima velocidad
  digitalWrite(pinDir1, HIGH);   // Configurar dirección
  digitalWrite(pinDir2, LOW);
  analogWrite(pinPWM, 255);      // PWM a máxima velocidad

  // Registrar el tiempo de inicio
  startTime = millis();
}

void loop() {
  // Verificar si se alcanzó el número de pulsos por vuelta de la rueda
  if (encoder.getCount() >= PULSOS_POR_VUELTA_RUEDA) {
    // Calcular el tiempo transcurrido
    unsigned long endTime = millis();
    unsigned long elapsedTime = endTime - startTime;

    // Detener el motor inmediatamente invirtiendo la dirección brevemente para frenar
    analogWrite(pinPWM, 0);
    digitalWrite(pinDir1, LOW);
    digitalWrite(pinDir2, HIGH);
    analogWrite(pinPWM, 255); // Frenado activo con un breve impulso
    delay(70);               // Breve impulso para frenar
    analogWrite(pinPWM, 0);   // Apagar el motor después del frenado

    // Imprimir el tiempo transcurrido y la cantidad de pulsos del encoder
    Serial.print("Tiempo para una vuelta completa de la rueda: ");
    Serial.print(elapsedTime);
    Serial.println(" ms");
    Serial.print("Pulsos del encoder: ");
    Serial.println(encoder.getCount());

    // Detener el código para evitar reintentos continuos
    while (true) {
      delay(1000);
    }
  }
}
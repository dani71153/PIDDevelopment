#include <Arduino.h>
#include <ESP32Encoder.h>

// Pines y configuración igual que antes...

#define R_EN3 27
#define L_EN3 26
#define RPWM3 25
#define LPWM3 23
#define R_EN4 14
#define L_EN4 13
#define RPWM4 4
#define LPWM4 5

#define ENC3_A 18
#define ENC3_B 19
#define ENC4_A 35
#define ENC4_B 34

#define RPWM3_CH 0
#define LPWM3_CH 1
#define RPWM4_CH 2
#define LPWM4_CH 3
#define PWM_FREQ 20000
#define PWM_RES 8

ESP32Encoder encoder3;
ESP32Encoder encoder4;

enum State {IDLE, MOTOR3, MOTOR4, ENCODER3, ENCODER4};
State state = IDLE;

// Variables para el control no bloqueante
unsigned long actionStart = 0, actionDur = 0;
int actionPWM = 0, actionSentido = 1, actionAbsPWM = 0;
int64_t encStart = 0;
String respToSend = "";

void stopMotor3() {
  ledcWrite(RPWM3_CH, 0); ledcWrite(LPWM3_CH, 0);
  digitalWrite(R_EN3, LOW); digitalWrite(L_EN3, LOW);
}

void stopMotor4() {
  ledcWrite(RPWM4_CH, 0); ledcWrite(LPWM4_CH, 0);
  digitalWrite(R_EN4, LOW); digitalWrite(L_EN4, LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(R_EN3, OUTPUT); pinMode(L_EN3, OUTPUT);
  ledcAttachPin(RPWM3, RPWM3_CH); ledcAttachPin(LPWM3, LPWM3_CH);
  ledcSetup(RPWM3_CH, PWM_FREQ, PWM_RES); ledcSetup(LPWM3_CH, PWM_FREQ, PWM_RES);

  pinMode(R_EN4, OUTPUT); pinMode(L_EN4, OUTPUT);
  ledcAttachPin(RPWM4, RPWM4_CH); ledcAttachPin(LPWM4, LPWM4_CH);
  ledcSetup(RPWM4_CH, PWM_FREQ, PWM_RES); ledcSetup(LPWM4_CH, PWM_FREQ, PWM_RES);

  encoder3.attachFullQuad(ENC3_A, ENC3_B); encoder3.clearCount();
  encoder4.attachFullQuad(ENC4_A, ENC4_B); encoder4.clearCount();

  stopMotor3(); stopMotor4();
  delay(400);
}

void loop() {
  // Si está inactivo, procesa nuevos comandos serial
  if (state == IDLE && Serial.available()) {
    String entrada = Serial.readStringUntil('\n');
    entrada.trim();
    int args[3], argCount = 0;
    while (entrada.length() > 0 && argCount < 3) {
      int spaceIdx = entrada.indexOf(' ');
      String part;
      if (spaceIdx == -1) { part = entrada; entrada = ""; }
      else { part = entrada.substring(0, spaceIdx); entrada = entrada.substring(spaceIdx + 1); }
      args[argCount++] = part.toInt();
    }

    if (argCount == 3) {
      actionPWM = args[1];
      actionSentido = actionPWM >= 0 ? 1 : 0;
      actionAbsPWM = abs(actionPWM);
      actionDur = args[2];
      if (args[0] == 1) { // Motor 3
        stopMotor3();
        digitalWrite(R_EN3, HIGH); digitalWrite(L_EN3, HIGH);
        ledcWrite(RPWM3_CH, actionSentido ? actionAbsPWM : 0);
        ledcWrite(LPWM3_CH, actionSentido ? 0 : actionAbsPWM);
        actionStart = millis();
        state = MOTOR3;
        respToSend = "OK";
      } else if (args[0] == 2) { // Encoder 3
        stopMotor3();
        encoder3.clearCount();
        digitalWrite(R_EN3, HIGH); digitalWrite(L_EN3, HIGH);
        ledcWrite(RPWM3_CH, actionSentido ? actionAbsPWM : 0);
        ledcWrite(LPWM3_CH, actionSentido ? 0 : actionAbsPWM);
        delay(80); // Breve estabilización
        encStart = encoder3.getCount();
        actionStart = millis();
        state = ENCODER3;
      } else if (args[0] == 3) { // Motor 4
        stopMotor4();
        digitalWrite(R_EN4, HIGH); digitalWrite(L_EN4, HIGH);
        ledcWrite(RPWM4_CH, actionSentido ? actionAbsPWM : 0);
        ledcWrite(LPWM4_CH, actionSentido ? 0 : actionAbsPWM);
        actionStart = millis();
        state = MOTOR4;
        respToSend = "OK";
      } else if (args[0] == 4) { // Encoder 4
        stopMotor4();
        encoder4.clearCount();
        digitalWrite(R_EN4, HIGH); digitalWrite(L_EN4, HIGH);
        ledcWrite(RPWM4_CH, actionSentido ? actionAbsPWM : 0);
        ledcWrite(LPWM4_CH, actionSentido ? 0 : actionAbsPWM);
        delay(80);
        encStart = encoder4.getCount();
        actionStart = millis();
        state = ENCODER4;
      } else {
        Serial.println("ERROR");
      }
    } else {
      Serial.println("ERROR");
    }
    while (Serial.available()) Serial.read(); // limpia
  }

  // Procesamiento no bloqueante para cada estado
  if (state == MOTOR3 && (millis() - actionStart >= actionDur)) {
    stopMotor3();
    Serial.println(respToSend);
    state = IDLE;
  }
  else if (state == MOTOR4 && (millis() - actionStart >= actionDur)) {
    stopMotor4();
    Serial.println(respToSend);
    state = IDLE;
  }
  else if (state == ENCODER3 && (millis() - actionStart >= actionDur)) {
    stopMotor3();
    int64_t encEnd = encoder3.getCount();
    Serial.print("PULSOS:");
    Serial.println((long)(encEnd - encStart));
    state = IDLE;
  }
  else if (state == ENCODER4 && (millis() - actionStart >= actionDur)) {
    stopMotor4();
    int64_t encEnd = encoder4.getCount();
    Serial.print("PULSOS:");
    Serial.println((long)(encEnd - encStart));
    state = IDLE;
  }
}

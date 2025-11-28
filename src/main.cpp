#include "MotorControlPIDV1.cpp"
#include <ESP32Encoder.h>
#include <ACS712.h>
#include <Arduino.h>

// =====================================
// PINES DE VOLTAJE (FZ0430)
// =====================================
const int PIN_VOLT_5V   = 32;  // Sensor ~5V
const int PIN_VOLT_14V  = 33;  // Sensor ~14V

// Divisor FZ0430
const float R1 = 30000.0;
const float R2 = 7500.0;
const float DIV_RATIO = (R1 + R2) / R2;   // ≈5.0

// Calibración
const float K_CAL5V = 1.57;   // ≈1.43
const float K_CAL14V = 1.05;       // ≈1.43

// Lectura ADC promediada
float readVoltageRaw(int pin) {
  const int samples = 50;
  uint32_t total = 0;

  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delayMicroseconds(300);
  }

  float adc = total / (float)samples;
  float vADC = (adc / 4095.0f) * 3.3f;      // Voltaje en pin
  float vReal = vADC * DIV_RATIO;           // Voltaje real antes del divisor

  return vReal;
}

float readVoltage5V() {
  return readVoltageRaw(PIN_VOLT_5V) * K_CAL5V;
}

float readVoltage14V() {
  return readVoltageRaw(PIN_VOLT_14V) * K_CAL14V;
}



// =====================================
// PINES MOTORES 3 Y 4
// =====================================
#define RPWM3     25
#define LPWM3     23
#define R_EN3     27
#define L_EN3     26
#define ENC3_A    18
#define ENC3_B    19
#define RPWM3_CH  0
#define LPWM3_CH  1

#define RPWM4     4
#define LPWM4     5
#define R_EN4     14
#define L_EN4     13
#define ENC4_A    35
#define ENC4_B    34
#define RPWM4_CH  2
#define LPWM4_CH  3

// Motores
Motor motor3(RPWM3, LPWM3, R_EN3, L_EN3, ENC3_A, ENC3_B,
             0.12, 0.0857, 0.001, 10, RPWM3_CH, LPWM3_CH);

Motor motor4(RPWM4, LPWM4, R_EN4, L_EN4, ENC4_A, ENC4_B,
             0.12, 0.09, 0.001, 10, RPWM4_CH, LPWM4_CH);

// ACS712
ACS712 myACS(36, 3.3, 4095, 185);

// Variables
String inputCommand = "";
bool usarPID = true;
unsigned long lastCommandTime = 0;
const unsigned long timeout = 3000;
bool enControlDePosicion = false;
const float PULSOS_POR_REVOLUCION = 270.0 * 64.0;
long targetPosMotor3 = 0, targetPosMotor4 = 0;
const float Kp_posicion = 0.05;
const int POSICION_TOLERANCIA = 50;

// Prototipo
void processCommand(String command);


// =====================================
// SETUP
// =====================================
void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando ESP32...");

  motor3.inicializar();
  motor4.inicializar();
  motor3.resetEncoderValues();
  motor4.resetEncoderValues();

  myACS.autoMidPointDC(1000);
  myACS.setNoisemV(50.88);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  lastCommandTime = millis();
}



// =====================================
// LOOP PRINCIPAL
// =====================================
void loop() {

  // Recepción de comandos
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '<') {
      inputCommand = "";
    } else if (receivedChar == '>') {
      processCommand(inputCommand);
      inputCommand = "";
      lastCommandTime = millis();
    } else {
      inputCommand += receivedChar;
    }
  }

  // Control PID o posición
  if (usarPID){
    motor3.actualizar();
    motor4.actualizar();
  }

  // Timeout de seguridad
  if (millis() - lastCommandTime > timeout) {
    motor3.controlarMotor(0);
    motor4.controlarMotor(0);
    motor3.desactivarMotor();
    motor4.desactivarMotor();
    usarPID = false;
    enControlDePosicion = false;
  }
}



// =====================================
// PARSER DE COMANDOS
// =====================================
void processCommand(String command) {
  if (command.length() == 0) return;

  switch (command[0]) {

    // ================================
    // VELOCIDAD EN RPS
    // ================================
    case 'm': {
      enControlDePosicion = false;

      if (command.length() < 3 || command[1] != ' ') {
        Serial.println("<Error: Formato invalido. Debe ser <m valor3 valor4>>");
        break;
      }

      command.remove(0, 2);
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex == -1) { Serial.println("<Error: Formato invalido>"); break; }

      float velMotor3RPS = command.substring(0, spaceIndex).toFloat();
      float velMotor4RPS = command.substring(spaceIndex + 1).toFloat();

      motor3.setReferenciaVelocidadRPS(velMotor3RPS);
      motor4.setReferenciaVelocidadRPS(velMotor4RPS);
      motor3.actualizar();
      motor4.actualizar();
      usarPID = true;
      motor3.sincronizarRampa();
      motor4.sincronizarRampa();

      if (velMotor3RPS == 0 && velMotor4RPS == 0) {
        motor3.controlarMotor(0);
        motor4.controlarMotor(0);
        usarPID = false;
        motor3.desactivarMotor();
        motor4.desactivarMotor();
      }
      break;
    }


    // ================================
    // POSICIÓN
    // ================================
    case 'p': {
      if (command.length() < 3 || command[1] != ' ') {
        Serial.println("<Error: Formato invalido. Debe ser <p vueltas3 vueltas4>>");
        break;
      }

      command.remove(0, 2);
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex == -1) { Serial.println("<Error: Formato invalido>"); break; }

      float vueltasMotor3 = command.substring(0, spaceIndex).toFloat();
      float vueltasMotor4 = command.substring(spaceIndex + 1).toFloat();
      targetPosMotor3 = motor3.leerEncoder() + (long)(vueltasMotor3 * PULSOS_POR_REVOLUCION);
      targetPosMotor4 = motor4.leerEncoder() + (long)(vueltasMotor4 * PULSOS_POR_REVOLUCION);

      Serial.print("<Moviendo a posicion (ticks): ");
      Serial.print(targetPosMotor3); Serial.print(", "); Serial.print(targetPosMotor4);
      Serial.println(">");

      enControlDePosicion = true;
      usarPID = true;
      break;
    }


    // ================================
    // CONTROL PWM DIRECTO
    // ================================
    case 'o': {
      enControlDePosicion = false;
      command.remove(0, 1);
      int spaceIndex = command.indexOf(' ');
      int pwmMotor3Value = command.substring(0, spaceIndex).toInt();
      int pwmMotor4Value = command.substring(spaceIndex + 1).toInt();

      motor3.controlarMotor(pwmMotor3Value);
      motor4.controlarMotor(pwmMotor4Value);
      usarPID = false;

      Serial.println("<Control PWM directo activado>");
      break;
    }


    // ================================
    // BAUDRATE
    // ================================
    case 'b':
      Serial.print("<Baudrate:115200>");
      break;


    // ================================
    // ENCÓDERS
    // ================================
    case 'e': {
      Serial.print("<");
      Serial.print(motor3.leerEncoder()); Serial.print(",");
      Serial.print(motor4.leerEncoder());
      Serial.println(">");
      break;
    }


    case 'r': {
      motor3.resetEncoderValues();
      motor4.resetEncoderValues();
      Serial.println("<Encoders reseteados>");
      break;
    }


    // ================================
    // VELOCIDAD ACTUAL
    // ================================
    case 'v': {
      Serial.print("<");
      Serial.print(motor3.getVelocidadRPS()); Serial.print(",");
      Serial.print(motor4.getVelocidadRPS());
      Serial.println(">");
      break;
    }


    // ================================
    // CORRIENTE
    // ================================
    case 'c': {
      float current_mA = myACS.mA_DC(60);
      Serial.print("<");
      Serial.print(current_mA / 1000);
      Serial.println(">");
      break;
    }


    // ================================
    // VOLTAJES INTEGRADOS
    // ================================
    case 'V': {
      if (command == "V1") {
        float v5 = readVoltage5V();
        Serial.print("<"); Serial.print(v5, 3); Serial.println(">");
      }
      else if (command == "V2") {
        float v14 = readVoltage14V();
        Serial.print("<"); Serial.print(v14, 3); Serial.println(">");
      }
      else {
        Serial.println("<Error: comando V invalido>");
      }
      break;
    }


    case 'i': {
      Serial.println("<OK>");
      break;
    }


    default:
      Serial.println("<Comando invalido>");
      break;
  }
}

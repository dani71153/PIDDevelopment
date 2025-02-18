#include <MotorControlPIDV1.cpp>
#include <ACS712.h>


// === CONFIGURACIÓN DEL SENSOR ACS712 ===
ACS712 myACS(25, 5.0, 1023, 200);

// Instanciar motores
// Motor(int enable, int in1, int in2, int encoderA, int encoderB, float kp, float ki, float kd, unsigned long muestreo)
Motor motor3(21, 23, 22, 19, 18, 0.1, 0.15, 0.08, 1);
Motor motor4(15, 2, 4, 34, 35, 0.1, 0.15, 0.08, 1);

String inputCommand = ""; // Variable para almacenar el comando recibido
void processCommand(String command);
bool usarPID = true; // Variable para controlar si se usa PID o no
unsigned long lastCommandTime = 0; // Variable para almacenar el tiempo del último comando recibido
const unsigned long timeout = 1000; // Tiempo de espera (1 segundo)

void setup() {
  Serial.begin(115200);
  while (!Serial); // Esperamos que el serial esté habilitado.

  Serial.println("Inicializando el ESP32");

  // Inicializar motores
  motor3.inicializar();
  motor4.inicializar();
  motor3.configurarPWM(490, 8);
  motor4.configurarPWM(490, 8);

  //Definimos el Pin de 25 como pulldown
  pinMode(25, INPUT_PULLDOWN);
  // Calibrar OFFSET del ACS712 en DC
  myACS.autoMidPointDC(10000); // 50 lecturas => Ajusta a tu gusto

  // Definir el ruido
  myACS.setNoisemV(5.88);
}

void loop() {
  // Verificar si hay datos disponibles en el Serial
  while (Serial.available() > 0) {
    char receivedChar = Serial.read(); // Leer el carácter entrante

    // Verificar delimitadores
    if (receivedChar == '<') {
      inputCommand = ""; // Iniciar un nuevo comando
    } else if (receivedChar == '>') {
      processCommand(inputCommand); // Procesar comando completo
      inputCommand = ""; // Limpiar la variable del comando
      lastCommandTime = millis(); // Actualizar el tiempo del último comando recibido
    } else {
      inputCommand += receivedChar; // Agregar carácter al comando actual
    }
  }

  // Actualizar los motores si se está usando PID
  if (usarPID) {
    motor3.actualizar();
    motor4.actualizar();
  }

  // Verificar si ha pasado el tiempo de espera sin recibir comandos
  if (millis() - lastCommandTime > timeout) {
    // Detener los motores y deshabilitar el controlador
    motor3.controlarMotor(0);
    motor4.controlarMotor(0);
    motor3.desactivarMotor();
    motor4.desactivarMotor();
    usarPID = false;
  }
}

void processCommand(String command) {
  switch (command.charAt(0)) {
    case 'm': {
      // Verificar si el segundo carácter es un espacio
      if (command.length() < 3 || command.charAt(1) != ' ') {
        Serial.println("<Error: Formato inválido. Debe ser <m valor1 valor2>>");
        break;
      }

      // Eliminar el prefijo "m " (incluyendo el espacio)
      command.remove(0, 2);

      // Verificar si hay exactamente un espacio separando los valores
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex == -1 || spaceIndex == 0 || spaceIndex == command.length() - 1) {
        Serial.println("<Error: Formato de comando inválido>");
        break;
      }

      // Separar las velocidades
      String velocidadMotor1 = command.substring(0, spaceIndex);
      String velocidadMotor2 = command.substring(spaceIndex + 1);

      // Convertir a float
      float velMotor1RPS = velocidadMotor1.toFloat();
      float velMotor2RPS = velocidadMotor2.toFloat();

      // Configurar velocidades de los motores
      motor3.setReferenciaVelocidadRPS(velMotor1RPS);
      motor4.setReferenciaVelocidadRPS(velMotor2RPS);

      usarPID = true;

      // Detener motores si ambas velocidades son 0
      if (velMotor1RPS == 0 && velMotor2RPS == 0) {
        motor3.controlarMotor(0);
        motor4.controlarMotor(0);
        usarPID = false;
        motor3.desactivarMotor();
        motor4.desactivarMotor();
      }
      break;
    }


    case 'o': {
      // Comando para configurar PWM de los motores directamente (sin PID)
      command.remove(0, 1); // Eliminar el prefijo "o"

      int spaceIndex = command.indexOf(' ');
      String pwmMotor1 = command.substring(0, spaceIndex);
      String pwmMotor2 = command.substring(spaceIndex + 1);

      int pwmMotor1Value = pwmMotor1.toInt();
      int pwmMotor2Value = pwmMotor2.toInt();

      motor3.controlarMotor(pwmMotor1Value);
      motor4.controlarMotor(pwmMotor2Value);

      usarPID = false;

      Serial.println("<Control PWM directo activado>");
      Serial.print("<Motor 3 (PWM): ");
      Serial.print(pwmMotor1Value);
      Serial.println(">");
      Serial.print("<Motor 4 (PWM): ");
      Serial.print(pwmMotor2Value);
      Serial.println(">");
      break;
    }

    case 'b': {
      // Comando para devolver el baudrate
      Serial.print("<Baudrate actual: ");
      Serial.print(Serial.baudRate());
      Serial.println(">");
      break;
    }

    case 'e': {
      // Comando para devolver los valores de los encoders
      Serial.print("<");
      Serial.print(motor3.leerEncoder());
      Serial.print(",");
      Serial.print(motor4.leerEncoder());
      Serial.println(">");
      break;
    }

    case 'r': {
      // Comando para resetear los encoders
      motor3.resetEncodersValues();
      motor4.resetEncodersValues();
      Serial.println("<Encoders reseteados>");
      break;
    }

    case 'i': {
      Serial.println("<OK>");
      break;
    }

    case 'v': {
      // Comando para devolver la velocidad actual de los motores en RPS
      Serial.print("<");
      Serial.print(motor3.getVelocidadRPS());
      Serial.print(",");
      Serial.println(motor4.getVelocidadRPS());
      Serial.println(">");
      break;
    }
    case 'c': {
      float current_mA = myACS.mA_DC(60); // Leer corriente
      float umbral = 50.0;

      if (current_mA > umbral) {
        // Corriente positiva
      } else if (current_mA < -umbral) {
        // Corriente negativa
      } else {
        current_mA = 0.0;
      }

      Serial.print("<Corriente (mA): ");
      Serial.print(current_mA / 1000);
      Serial.println(">");
      break;
    }
    default: {
      // Comando inválido
      Serial.println("<Comando inválido>");
      break;
    }
  }
}

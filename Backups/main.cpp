
#include <MotorControlPIDV1.cpp>
// Instanciar motores
//Motor(int enable, int in1, int in2, int encoderA, int encoderB, float kp, float ki, float kd, unsigned long muestreo) 
Motor motor3(21, 23, 22, 19, 18, 0.1, 0.15, 0.08, 10);
Motor motor4(15, 2, 4, 34, 35, 0.1, 0.15, 0.08, 10);

String inputCommand = "";  // Variable para almacenar el comando recibido
void processCommand(String command);
bool usarPID = true;  // Variable para controlar si se usa PID o no

void setup() {
  Serial.begin(115200);

  // Inicializar motores
  motor3.inicializar();
  motor4.inicializar();

  // Imprimir mensaje inicial
 // Serial.println("Sistema de comandos iniciado. Ingrese comandos.");
}

void loop() {
  // Verificar si hay datos disponibles en el Serial
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();  // Leer el carácter entrante
    inputCommand += receivedChar;       // Agregar carácter al comando actual

    // Procesar el comando al recibir un salto de línea (fin de comando)
    if (receivedChar == '\n') {
      inputCommand.trim();  // Eliminar espacios en blanco y caracteres no visibles
      processCommand(inputCommand);  // Procesar el comando
      inputCommand = "";  // Limpiar la variable del comando para recibir el siguiente
    }
  }

  // Actualizar los motores en el loop principal, solo si se está usando PID
  if (usarPID) {
    motor3.actualizar();
    motor4.actualizar();
  }
}

void processCommand(String command) {
  if (command.startsWith("m ")) {
    // Comando para configurar velocidades de los motores usando PID
    command.remove(0, 2);  // Eliminar el prefijo "m "
    
    // Dividir los valores de velocidad
    int spaceIndex = command.indexOf(' ');
    String velocidadMotor1 = command.substring(0, spaceIndex);
    String velocidadMotor2 = command.substring(spaceIndex + 1);

    // Convertir a float
    float velMotor1RPS = velocidadMotor1.toFloat();
    float velMotor2RPS = velocidadMotor2.toFloat();

    // Configurar las velocidades de los motores usando PID
    motor3.setReferenciaVelocidadRPS(velMotor1RPS);
    motor4.setReferenciaVelocidadRPS(velMotor2RPS);

    usarPID = true;  // Asegurarse de que el PID esté activado
    //Serial.println("Velocidades establecidas (PID activado):");
    //Serial.print("Motor 1 (RPS): ");
    //Serial.println(velMotor1RPS);
    //Serial.print("Motor 2 (RPS): ");
    //Serial.println(velMotor2RPS);
  } 
  else if (command.startsWith("o ")) {
    // Comando para configurar PWM de los motores directamente (sin PID)
    command.remove(0, 2);  // Eliminar el prefijo "o "
    
    // Dividir los valores de PWM
    int spaceIndex = command.indexOf(' ');
    String pwmMotor1 = command.substring(0, spaceIndex);
    String pwmMotor2 = command.substring(spaceIndex + 1);

    // Convertir a entero
    int pwmMotor1Value = pwmMotor1.toInt();
    int pwmMotor2Value = pwmMotor2.toInt();

    // Controlar los motores directamente con PWM (sin PID)
    motor3.controlarMotor(pwmMotor1Value);
    motor4.controlarMotor(pwmMotor2Value);

    usarPID = false;  // Desactivar el uso de PID
    Serial.println("Control PWM directo activado:");
    Serial.print("Motor 1 (PWM): ");
    Serial.println(pwmMotor1Value);
    Serial.print("Motor 2 (PWM): ");
    Serial.println(pwmMotor2Value);
  } 
  else if (command == "b") {
    // Comando para devolver el baudrate
    Serial.print("Baudrate actual: ");
    Serial.println(Serial.baudRate());
  } 
  else if (command == "e") {
    // Comando para devolver los valores de los encoders
    Serial.print(motor3.leerEncoder());
    Serial.print(",");
    Serial.println(motor4.leerEncoder());
  } 
  else if (command == "r") {
    // Comando para resetear los encoders
    motor3.inicializar();  // Esto reinicia el encoder del motor 1
    motor4.inicializar();  // Esto reinicia el encoder del motor 2
    Serial.println("Encoders reiniciados.");
  } 
  else {
    // Comando inválido
    Serial.println("Comando inválido.");
  }
}

#include <Arduino.h>
#include <ESP32Encoder.h>
/**
 * Características y Funcionalidades del Código (Antes de los Cambios)
 *
 * 1. Configuración del Motor y Encoder:
 *    - Configura los pines necesarios para controlar el motor (pines de dirección, habilitación y encoder).
 *    - Utiliza la librería ESP32Encoder para manejar la lectura del encoder en modo cuadratura.
 *
 * 2. Control PID:
 *    - Implementa un controlador PID para regular la velocidad del motor en función de una referencia.
 *    - Calcula los términos proporcional, integral y derivativo para ajustar el valor PWM que controla la velocidad del motor.
 *    - Incluye saturación del valor de salida del PID al rango permitido por el PWM (de -255 a 255).
 *
 * 3. Control del Motor:
 *    - Controla el sentido de giro del motor según el valor del PID.
 *    - Genera señales PWM utilizando un canal configurado para el ESP32 con la función ledcWrite.
 *
 * 4. Velocidades en Diferentes Unidades:
 *    - Permite establecer la velocidad de referencia en ticks por segundo, RPS (revoluciones por segundo) o RPM (revoluciones por minuto).
 *    - Proporciona métodos para obtener la velocidad actual en estas mismas unidades.
 *
 * 5. Frecuencia y Resolución del PWM:
 *    - Configura la frecuencia y resolución del PWM mediante métodos dedicados.
 *    - Es posible cambiar estos parámetros durante la ejecución.
 *
 * 6. Lectura del Encoder y Cálculo de Velocidad:
 *    - Calcula la velocidad actual del motor en función de los pulsos del encoder y el tiempo transcurrido.
 *    - La velocidad se mide en ticks por segundo.
 *
 * 7. Actualización periódica:
 *    - El método `actualizar` verifica si ha transcurrido el intervalo de muestreo para realizar las operaciones de lectura del encoder, cálculo del PID y actualización del motor.
 *
 * Agregados hoy 13 de Enero:
 *
 * 1. Parada Activa:
 *    - Se implementó un mecanismo de frenado activo en el método controlarMotor. Esto asegura que el motor se detenga de manera rápida y precisa, reduciendo el deslizamiento.
 *    - La parada activa consiste en poner ambos pines de dirección del motor en HIGH cuando el valor del PID es 0.
 *
 * 2. Anti-Windup:
 *    - Se añadió una limitación a la acumulación de errores en el cálculo del PID (término integral), para evitar el windup.
 *    - Esto asegura que el término integral no crezca indefinidamente, lo que mejora la respuesta del sistema y evita comportamientos inestables.
 * 
 * Nota: Esto funciona para cuando la velocidad maxima es 0.6. SOlucion sencilla. Trabajar con 0.3RPS. Jajaja
 */

class Motor {
  private:
    int pinEnable;
    int pinIN1;
    int pinIN2;
    int pinEncoderA;
    int pinEncoderB;
    float kp, ki, kd;
    float referenciaVelocidad;
    float errorActual, errorPrevio, sumaErrores, derivadaError;
    unsigned long tiempoPrevio;
    unsigned long intervaloMuestreo;
    ESP32Encoder encoder;
    long posicionEncoder;
    float velocidadActual;
    const float pulsosPorRevolucion = 4320.0 * 2; // Pulsos del encoder por revolución
    float valorPWM; // Nueva variable para almacenar el valor actual del PWM
    float ajuste = 1;

    int pwmChannel = 0; // Canal de PWM
    int pwmResolution = 8; // Resolución del PWM
    float pwmFrequency = 1000; // Frecuencia del PWM por defecto en Hz

  public:
    Motor(int enable, int in1, int in2, int encoderA, int encoderB, float kp, float ki, float kd, unsigned long muestreo) 
      : pinEnable(enable), pinIN1(in1), pinIN2(in2), pinEncoderA(encoderA), pinEncoderB(encoderB), kp(kp), ki(ki), kd(kd), 
        intervaloMuestreo(muestreo), errorActual(0), errorPrevio(0), sumaErrores(0), derivadaError(0), referenciaVelocidad(0), 
        tiempoPrevio(0), posicionEncoder(0), velocidadActual(0), valorPWM(0) {
    }

    void inicializar() {
      // Configuración de los pines de motor
      pinMode(pinIN1, OUTPUT);
      pinMode(pinIN2, OUTPUT);

      // Configuración del encoder
      encoder.attachHalfQuad(pinEncoderA, pinEncoderB);
      encoder.clearCount();

      // Configuración inicial del PWM
      ledcSetup(pwmChannel, pwmFrequency, pwmResolution); // Configurar canal de PWM
      ledcAttachPin(pinEnable, pwmChannel); // Asociar pinEnable al canal de PWM

      // Inicializar tiempo
      tiempoPrevio = millis();
    }

    // Configurar frecuencia del PWM
    void configurarPWM(float frecuencia, int resolucion = 8) {
      pwmFrequency = frecuencia;
      pwmResolution = resolucion;
      ledcSetup(pwmChannel, pwmFrequency, pwmResolution); // Actualizar configuración del canal PWM
    }

    void setReferenciaVelocidad(float referencia) {
      referenciaVelocidad = referencia;
    }

    void setReferenciaVelocidadRPS(float rps) {
      referenciaVelocidad = (rps * pulsosPorRevolucion) / ajuste; 
    }

    void setReferenciaVelocidadRPM(float rpm) {
      float rps = rpm / 60.0;
      referenciaVelocidad = (rps * pulsosPorRevolucion) / ajuste; 
    }

    void actualizar() {
      unsigned long tiempoActual = millis();
      if (tiempoActual - tiempoPrevio >= intervaloMuestreo) {
        long posicionAnterior = posicionEncoder;
        posicionEncoder = leerEncoder();

        velocidadActual = calcularVelocidad(posicionEncoder, posicionAnterior, tiempoPrevio);
        tiempoPrevio = tiempoActual;

        valorPWM = calcularPID(referenciaVelocidad, velocidadActual); 
        controlarMotor(valorPWM);
      }
    }

    long leerEncoder() {
      return encoder.getCount();
    }

    float calcularVelocidad(long posicionActual, long posicionAnterior, unsigned long tiempoAnterior) {
      long deltaPosicion = posicionActual - posicionAnterior;
      unsigned long deltaTiempo = millis() - tiempoAnterior;
      float velocidad = (deltaPosicion / (float)deltaTiempo) * 1000;
      return velocidad;
    }

/* Version ORIGINAL. No tiene el ajuste dinamico.*/
    float calcularPID(float referencia, float actual) {
      errorActual = referencia - actual;
      sumaErrores += errorActual;

      // Agregamos una proteccion atraves de la suma de los errores.  Para ponerle un limite.

      if (sumaErrores > 1000) sumaErrores = 1000; // Ajusta según tus necesidades
      if (sumaErrores < -1000) sumaErrores = -1000;

      derivadaError = errorActual - errorPrevio;

      float salida = (kp * errorActual) + (ki * sumaErrores) + (kd * derivadaError);
      errorPrevio = errorActual;

      if (salida > 255) salida = 255;
      if (salida < -255) salida = -255;

      return salida;
    }

    void controlarMotor(float valorPID) {
      if (valorPID > 0) {
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);
        ledcWrite(pwmChannel, abs(valorPID));
      } else if (valorPID < 0) {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);
        ledcWrite(pwmChannel, abs(valorPID));
      } else {
      //Agregamos un freno activo.
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, HIGH); // Freno activo
        ledcWrite(pwmChannel, 0);
      }
    }

    void desactivarMotor() {
      ledcWrite(pwmChannel, 0);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
    }

    float getVelocidadTicksPorSegundo() {
      return velocidadActual;
    }

    float getVelocidadRPS() {
      return (velocidadActual / pulsosPorRevolucion);
    }

    float getVelocidadRPM() {
      return ((velocidadActual / pulsosPorRevolucion) * 60.0);
    }

    float getValorPWM() {
      return valorPWM;
    }
};


#include <Arduino.h>
#include <ESP32Encoder.h>
#include "RampaVelocidad.h"

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
    // Pines de control (BTS7960)
    int pinRPWM, pinLPWM, pinR_EN, pinL_EN;
    int pinEncoderA, pinEncoderB;

    // PID y control
    float kp, ki, kd;
    float referenciaVelocidad, errorActual, errorPrevio, sumaErrores, derivadaError;
    unsigned long tiempoPrevio, intervaloMuestreo;
    float velocidadActual, valorPWM;
    float currentReferenciaVelocidad = 0;
    float maxAceleracion = 10000;

    // Encoder
    ESP32Encoder encoder;
    long posicionEncoder;

    // PWM (ESP32)
    int channelRPWM, channelLPWM;
    int pwmResolution = 8;  // 8 bits: 0-255
    float pwmFrequency = 1000; // 1 kHz

    // Motor/encoder
    const float pulsosPorRevolucion = 270 * 64;
    RampaVelocidad rampa;

public:
    // Constructor
    Motor(int rpwm, int lpwm, int r_en, int l_en, int encoderA, int encoderB, float Kp, float Ki, float Kd, unsigned long muestreo, float aceleracionMax = 85000.0f,
          int canalRPWM = 0, int canalLPWM = 1, int resolucion = 8, float frecuencia = 1000)
        : pinRPWM(rpwm), pinLPWM(lpwm), pinR_EN(r_en), pinL_EN(l_en),
          pinEncoderA(encoderA), pinEncoderB(encoderB),
          kp(Kp), ki(Ki), kd(Kd),
          intervaloMuestreo(muestreo),
          rampa(aceleracionMax),
          channelRPWM(canalRPWM), channelLPWM(canalLPWM),
          pwmResolution(resolucion), pwmFrequency(frecuencia)
    {
        referenciaVelocidad = 0;
        errorActual = errorPrevio = sumaErrores = derivadaError = 0;
        posicionEncoder = 0;
        velocidadActual = valorPWM = 0;
    }

    void inicializar() {
        pinMode(pinR_EN, OUTPUT); digitalWrite(pinR_EN, LOW);
        pinMode(pinL_EN, OUTPUT); digitalWrite(pinL_EN, LOW);

        // PWM: asociar pines a canales y configurar canales
        ledcSetup(channelRPWM, pwmFrequency, pwmResolution);
        ledcSetup(channelLPWM, pwmFrequency, pwmResolution);
        ledcAttachPin(pinRPWM, channelRPWM);
        ledcAttachPin(pinLPWM, channelLPWM);

        // Apagar PWM
        ledcWrite(channelRPWM, 0);
        ledcWrite(channelLPWM, 0);

        // Encoder
        encoder.attachHalfQuad(pinEncoderA, pinEncoderB);
        encoder.clearCount();
        posicionEncoder = encoder.getCount();
        tiempoPrevio = millis();
    }

    void setReferenciaVelocidad(float referencia) { referenciaVelocidad = referencia; }
    void setReferenciaVelocidadRPS(float rps) { referenciaVelocidad = rps * pulsosPorRevolucion; }
    void setReferenciaVelocidadRPM(float rpm) { referenciaVelocidad = (rpm / 60.0f) * pulsosPorRevolucion; }

    void actualizar() {
        unsigned long tiempoActual = millis();
        if (tiempoActual - tiempoPrevio >= intervaloMuestreo) {
            long posicionAnterior = posicionEncoder;
            posicionEncoder = leerEncoder();
            velocidadActual = calcularVelocidad(posicionEncoder, posicionAnterior, tiempoPrevio);
            tiempoPrevio = tiempoActual;
            float referenciaSuavizada = rampa.actualizar(referenciaVelocidad);
            valorPWM = calcularPID(referenciaSuavizada, velocidadActual);
            controlarMotor(valorPWM);
        }
    }

    long leerEncoder() { return encoder.getCount(); }

    float calcularVelocidad(long posActual, long posAnterior, unsigned long tiempoAnterior) {
        long deltaPosicion = posActual - posAnterior;
        unsigned long deltaTiempo = millis() - tiempoAnterior;
        if (deltaTiempo == 0) return 0;
        return (deltaPosicion / (float)deltaTiempo) * 1000.0f;
    }

    float calcularPID(float referencia, float actual) {
        if (referencia == 0) {
            errorActual = 0; sumaErrores = 0;
        } else {
            errorActual = referencia - actual;
            sumaErrores += errorActual;
        }
        // Anti-windup
        sumaErrores = constrain(sumaErrores, -2000, 2000);
        derivadaError = errorActual - errorPrevio;
        errorPrevio = errorActual;
        float salida = (kp * errorActual) + (ki * sumaErrores) + (kd * derivadaError);
        return constrain(salida, -255.0, 255.0);
    }

    void controlarMotor(float valorPID) {
        digitalWrite(pinR_EN, HIGH);
        digitalWrite(pinL_EN, HIGH);
        int pwmValue = (int)abs(valorPID);

        if (valorPID > 1.0) { // Adelante
            ledcWrite(channelLPWM, 0);
            ledcWrite(channelRPWM, pwmValue);
        } else if (valorPID < -1.0) { // Atrás
            ledcWrite(channelRPWM, 0);
            ledcWrite(channelLPWM, pwmValue);
        } else { // Freno
            ledcWrite(channelRPWM, 0);
            ledcWrite(channelLPWM, 0);
        }
    }

    void desactivarMotor() {
        ledcWrite(channelRPWM, 0);
        ledcWrite(channelLPWM, 0);
        digitalWrite(pinR_EN, LOW);
        digitalWrite(pinL_EN, LOW);
    }

    void resetEncoderValues() {
        encoder.clearCount();
        posicionEncoder = 0;
    }

    // Getters
    float getVelocidadTicksPorSegundo() { return velocidadActual; }
    float getVelocidadRPS() { return velocidadActual / pulsosPorRevolucion; }
    float getVelocidadRPM() { return (velocidadActual / pulsosPorRevolucion) * 60.0f; }
    float getValorPWM() { return valorPWM; }

    void sincronizarRampa() { rampa.setVelocidadActual(velocidadActual);}
};

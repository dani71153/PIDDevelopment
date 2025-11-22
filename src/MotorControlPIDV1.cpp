#include <Arduino.h>
#include <ESP32Encoder.h>
#include "RampaVelocidad.cpp"

// Parámetros globales para PWM (ajusta según el proyecto)
#define PWM_FREQ 20000
#define PWM_RES 8   // 8 bits = 0-255
#define RPWM_CH 0   // Canal para RPWM
#define LPWM_CH 1   // Canal para LPWM

class Motor {
private:
    int pinRPWM, pinLPWM, pinR_EN, pinL_EN;
    int pinEncoderA, pinEncoderB;

    float kp, ki, kd;
    float referenciaVelocidad, errorActual, errorPrevio, sumaErrores, derivadaError;
    unsigned long tiempoPrevio, intervaloMuestreo;
    ESP32Encoder encoder;
    int64_t posicionEncoder;
    float velocidadActual, valorPWM;
    const float pulsosPorRevolucion = 270 * 64;
    RampaVelocidad rampa;
    int rpwm_ch, lpwm_ch;

public:
    Motor(int rpwm, int lpwm, int r_en, int l_en, int encoderA, int encoderB,
          float Kp, float Ki, float Kd, unsigned long muestreo,
          int rpwm_ch, int lpwm_ch)
        : pinRPWM(rpwm), pinLPWM(lpwm), pinR_EN(r_en), pinL_EN(l_en),
          pinEncoderA(encoderA), pinEncoderB(encoderB),
          kp(Kp), ki(Ki), kd(Kd), intervaloMuestreo(muestreo),
          rampa(40000.0f), rpwm_ch(rpwm_ch), lpwm_ch(lpwm_ch)
    {
        referenciaVelocidad = errorActual = errorPrevio = sumaErrores = derivadaError = 0;
        posicionEncoder = 0; velocidadActual = 0; valorPWM = 0;
    }

    void inicializar() {
        pinMode(pinRPWM, OUTPUT); pinMode(pinLPWM, OUTPUT);
        pinMode(pinR_EN, OUTPUT); pinMode(pinL_EN, OUTPUT);
        digitalWrite(pinR_EN, LOW); digitalWrite(pinL_EN, LOW);

        // Configuración de PWM hardware ESP32
        ledcSetup(rpwm_ch, PWM_FREQ, PWM_RES);
        ledcSetup(lpwm_ch, PWM_FREQ, PWM_RES);
        ledcAttachPin(pinRPWM, rpwm_ch);
        ledcAttachPin(pinLPWM, lpwm_ch);
        ledcWrite(rpwm_ch, 0); ledcWrite(lpwm_ch, 0);

        // Encoder ESP32
        encoder.attachFullQuad(pinEncoderA, pinEncoderB);
        encoder.clearCount();
        posicionEncoder = encoder.getCount();
        tiempoPrevio = millis();
    }

    void setReferenciaVelocidad(float referencia) { referenciaVelocidad = referencia; }
    void setReferenciaVelocidadRPS(float rps) { referenciaVelocidad = rps * pulsosPorRevolucion; }
    void setReferenciaVelocidadRPM(float rpm) { referenciaVelocidad = (rpm / 60.0) * pulsosPorRevolucion; }

    void actualizar() {
        unsigned long tiempoActual = millis();
        if (tiempoActual - tiempoPrevio >= intervaloMuestreo) {
            int64_t posicionAnterior = posicionEncoder;
            posicionEncoder = leerEncoder();
            velocidadActual = calcularVelocidad(posicionEncoder, posicionAnterior, tiempoPrevio);
            tiempoPrevio = tiempoActual;
            float referenciaSuavizada = rampa.actualizar(referenciaVelocidad);
            valorPWM = calcularPID(referenciaSuavizada, velocidadActual);
            controlarMotor(valorPWM);
        }
    }

    int64_t leerEncoder() { return encoder.getCount(); }

    float calcularVelocidad(int64_t posActual, int64_t posAnterior, unsigned long tiempoAnterior) {
        int64_t deltaPos = posActual - posAnterior;
        unsigned long deltaT = millis() - tiempoAnterior;
        if (deltaT == 0) return 0;
        return (deltaPos / (float)deltaT) * 1000.0f;
    }

    float calcularPID(float referencia, float actual) {
        if (referencia == 0) { errorActual = 0; sumaErrores = 0; }
        else { errorActual = referencia - actual; sumaErrores += errorActual; }
        sumaErrores = constrain(sumaErrores, -2000, 2000);
        derivadaError = errorActual - errorPrevio; errorPrevio = errorActual;
        float salida = (kp * errorActual) + (ki * sumaErrores) + (kd * derivadaError);
        return constrain(salida, -255.0, 255.0);
    }

    void controlarMotor(float valorPID) {
        digitalWrite(pinR_EN, HIGH); digitalWrite(pinL_EN, HIGH);
        int pwmValue = (int)abs(valorPID);
        if (valorPID > 1.0) {
            ledcWrite(lpwm_ch, 0); ledcWrite(rpwm_ch, pwmValue);
        } else if (valorPID < -1.0) {
            ledcWrite(rpwm_ch, 0); ledcWrite(lpwm_ch, pwmValue);
        } else {
            ledcWrite(rpwm_ch, 0); ledcWrite(lpwm_ch, 0);
        }
    }

    void desactivarMotor() {
        ledcWrite(rpwm_ch, 0); ledcWrite(lpwm_ch, 0);
        digitalWrite(pinR_EN, LOW); digitalWrite(pinL_EN, LOW);
    }

    void resetEncoderValues() {
        encoder.clearCount();
        posicionEncoder = 0;
    }

    float getVelocidadTicksPorSegundo() { return velocidadActual; }
    float getVelocidadRPS() { return velocidadActual / pulsosPorRevolucion; }
    float getVelocidadRPM() { return (velocidadActual / pulsosPorRevolucion) * 60.0f; }
    float getValorPWM() { return valorPWM; }
    void sincronizarRampa() { rampa.setVelocidadActual(velocidadActual);}
};

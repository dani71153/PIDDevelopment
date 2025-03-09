#include <Arduino.h>
#include <ESP32Encoder.h>
#include <QuickPID.h>

/**
 * Adaptación del control de motor original usando la librería QuickPID
 * 
 * Esta versión mantiene la misma estructura y funcionalidad, pero reemplaza
 * la implementación manual del PID con la librería QuickPID de Arduino.
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
    unsigned long tiempoPrevio;
    unsigned long intervaloMuestreo;
    ESP32Encoder encoder;
    long posicionEncoder;
    float velocidadActual;
    const float pulsosPorRevolucion = 4320.0 * 2; // Pulsos del encoder por revolución
    float valorPWM; // Variable para almacenar el valor actual del PWM
    float ajuste = 1;
    int pwmChannel = 0; // Canal de PWM
    int pwmResolution = 8; // Resolución del PWM
    float pwmFrequency = 1000; // Frecuencia del PWM por defecto en Hz
    float referenciaAnterior = 0.0; // Variable para almacenar la referencia anterior
    
    // Variables para QuickPID
    float pidInput = 0;
    float pidOutput = 0;
    float pidSetpoint = 0;
    QuickPID* pid; // Puntero al objeto QuickPID

  public:
    Motor(int enable, int in1, int in2, int encoderA, int encoderB, float kp, float ki, float kd, unsigned long muestreo) 
      : pinEnable(enable), pinIN1(in1), pinIN2(in2), pinEncoderA(encoderA), pinEncoderB(encoderB), kp(kp), ki(ki), kd(kd), 
        intervaloMuestreo(muestreo), referenciaVelocidad(0), tiempoPrevio(0), posicionEncoder(0), 
        velocidadActual(0), valorPWM(0) {
      
      // Crear instancia de QuickPID
      // El constructor correcto requiere 7 argumentos, incluyendo Action::direct
      pid = new QuickPID(&pidInput, &pidOutput, &pidSetpoint, kp, ki, kd, QuickPID::Action::direct);
    }

    ~Motor() {
      // Liberar memoria
      if(pid) delete pid;
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

      // Configuración del QuickPID
      pid->SetOutputLimits(-255, 255);
      pid->SetSampleTimeUs(intervaloMuestreo/1000); // Convertir ms a µs
      pid->SetMode(QuickPID::Control::manual);
      pid->SetProportionalMode(QuickPID::pMode::pOnError);
      pid->SetAntiWindupMode(QuickPID::iAwMode::iAwClamp); // Activar anti-windup
      pid->SetDerivativeMode(QuickPID::dMode::dOnMeas); // Derivada sobre medición, más estable que sobre error
      pid->SetControllerDirection(QuickPID::Action::direct); // Establecer la dirección del controlador a directa
      // Inicializar tiempo
      tiempoPrevio = millis();
    }

    // Configurar frecuencia del PWM
    void configurarPWM(float frecuencia, int resolucion = 8) {
      pwmFrequency = frecuencia;
      pwmResolution = resolucion;
      ledcSetup(pwmChannel, pwmFrequency, pwmResolution); // Actualizar configuración del canal PWM
    }

    // Configuración de velocidad por ticks por segundo
    void setReferenciaVelocidad(float referencia) {
      referenciaVelocidad = referencia;
      pidSetpoint = referencia;
      referenciaAnterior = referencia;
    }

    // Configuración de velocidad por revoluciones por segundo (RPS)
    void setReferenciaVelocidadRPS(float rps) {
      float nuevaReferencia = (rps * pulsosPorRevolucion) / ajuste;
      
      // Si hay un cambio significativo en la referencia, reiniciar el control PID
      // para evitar sobre-disparos debido a la acumulación de error
      if (abs(nuevaReferencia - referenciaVelocidad) > (referenciaVelocidad * 0.2)) { // 20% de cambio
        // Otra forma de reiniciar el PID es cambiar el modo a manual y luego de nuevo a automático
        pid->SetMode(QuickPID::Control::manual);
        // Esperamos un ciclo
        pid->Compute();
        // Y volvemos al modo automático
        pid->SetMode(QuickPID::Control::automatic);
      }
      
      referenciaVelocidad = nuevaReferencia;
      pidSetpoint = referenciaVelocidad;
      referenciaAnterior = rps;
    }

    // Configuración de velocidad por revoluciones por minuto (RPM)
    void setReferenciaVelocidadRPM(float rpm) {
      float rps = rpm / 60.0;
      float nuevaReferencia = (rps * pulsosPorRevolucion) / ajuste;
      
      // Si hay un cambio significativo en la referencia, reiniciar el control PID
      if (abs(nuevaReferencia - referenciaVelocidad) > (referenciaVelocidad * 0.2)) {
        pid->SetMode(QuickPID::Control::manual);
        pid->Compute();
        pid->SetMode(QuickPID::Control::automatic);
      }
      
      referenciaVelocidad = nuevaReferencia;
      pidSetpoint = referenciaVelocidad;
      referenciaAnterior = rps;
    }

    void actualizar() {
      unsigned long tiempoActual = millis();
      if (tiempoActual - tiempoPrevio >= intervaloMuestreo) {
        long posicionAnterior = posicionEncoder;
        posicionEncoder = leerEncoder();

        // Cálculo más robusto de la velocidad
        unsigned long deltaTiempo = tiempoActual - tiempoPrevio;
        long deltaPosicion = posicionEncoder - posicionAnterior;
        
        // Verificar que el delta de tiempo no sea cero para evitar divisiones por cero
        if (deltaTiempo > 0) {
          velocidadActual = (deltaPosicion / (float)deltaTiempo) * 1000.0;
        }
        
        tiempoPrevio = tiempoActual;

        // Filtro simple para suavizar cambios bruscos en la velocidad
        static float velocidadFiltrada = 0;
        float alpha = 0.7; // Factor de filtrado (0-1), más cercano a 1 = menos filtrado
        velocidadFiltrada = alpha * velocidadActual + (1-alpha) * velocidadFiltrada;
        
        // Actualizar input del PID con la velocidad filtrada
        pidInput = velocidadFiltrada;
        
        // Verifica si tenemos una referencia muy baja (cerca de cero)
        // En ese caso, desactivamos el motor directamente en lugar de usar el PID
        if (abs(referenciaVelocidad) < 0.01) {
          // Referencia muy pequeña, tratamos como cero
          desactivarMotor();
          return;
        }
        
        // Verificar que el setpoint no sea cero si la referencia tiene algún valor
        if (referenciaVelocidad != 0 && pidSetpoint == 0) {
          pidSetpoint = referenciaVelocidad;
        }
        
        // Calcular PID usando QuickPID
        pid->Compute();
        
        // Guardar el valor calculado
        valorPWM = pidOutput;
        
        // Si la salida es muy pequeña y la referencia no es cero, aseguramos un mínimo
        // Esto ayuda a vencer la fricción estática del motor
        float minPWM = 35.0; // Valor mínimo para vencer la fricción estática
        if (abs(valorPWM) > 0 && abs(valorPWM) < minPWM && referenciaVelocidad != 0) {
          valorPWM = (valorPWM > 0) ? minPWM : -minPWM;
        }
        
        // Aplicar el control al motor
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
        // Freno activo
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, HIGH);
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
      return (velocidadActual * ajuste) / pulsosPorRevolucion;
    }

    float getVelocidadRPM() {
      return getVelocidadRPS() * 60.0;
    }

    float getValorPWM() {
      return valorPWM;
    }
    
    // Métodos adicionales para ajustar parámetros del PID en tiempo de ejecución
    void setParametrosPID(float _kp, float _ki, float _kd) {
      kp = _kp;
      ki = _ki;
      kd = _kd;
      pid->SetTunings(kp, ki, kd);
    }
    
    // Agregamos el método resetEncodersValues a la clase Motor
    void resetEncodersValues() {
      encoder.clearCount();
      posicionEncoder = 0;
    }

    // Método para diagnóstico PID
    void getPIDTerms(float& proporcional, float& integral, float& derivativo) {
      // Los métodos Get devuelven el valor directamente, no aceptan parámetros por referencia
      proporcional = pid->GetPterm();
      integral = pid->GetIterm();
      derivativo = pid->GetDterm();
    }
}; 
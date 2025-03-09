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
 * 
 * Cambios Realizados 7 de Marzo de 2025 al 9 de Marzo de 2025
 * Variables Nuevas y Modificadas:
 * Se añadió float referenciaAnterior = 0.0; para almacenar la referencia anterior.
 * Método setReferenciaVelocidad:
 * Se añadió lógica para resetear errorActual y sumaErrores si hay un cambio de referencia y la nueva referencia no es cero.
 * Se actualiza referenciaAnterior con la nueva referencia.
 * Método setReferenciaVelocidadRPS:
 * Se añadió lógica similar a setReferenciaVelocidad para manejar cambios de referencia.
 * Se convierte RPS a ticks por segundo y se actualiza referenciaAnterior.
 * Método calcularPID:
 * Se eliminó la lógica que reseteaba errorActual y sumaErrores cuando la referencia era cero.
 * Se ajustaron los límites de sumaErrores de ±1000 a ±2000 para el anti-windup.
 * Parámetros del Motor:
 * En main.cpp, se ajustaron los parámetros PID para motor4:
 * k
 * i
 * ki se cambió de 0.15 a 0.30.
 * kd se cambió de 0.08 a 0.0.
 */


// ==================
// Modificación de la clase Motor
// ==================
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
    float valorPWM; // Variable para almacenar el valor actual del PWM
    float ajuste = 1;
    int pwmChannel;  // Se elimina la asignación fija y se asigna en el constructor
    int pwmResolution = 8; // Resolución del PWM
    float pwmFrequency = 1000; // Frecuencia del PWM por defecto en Hz
    float referenciaAnterior = 0.0; // Variable para almacenar la referencia anterior
    static const int numLecturas = 1; // Número de lecturas para el filtro de media
    long lecturasEncoder[numLecturas]; // Array para almacenar las lecturas
    int indiceLectura; // Índice para las lecturas
    float valorPIDAnterior = 0.0; // Valor anterior del PID
    float maxCambioRampa = 2;  // Cambio máximo permitido por iteración
    static const int numLecturasFiltro = 5; // Número de lecturas para el filtro de media móvil
    long bufferLecturas[numLecturasFiltro]; // Buffer para almacenar las lecturas del encoder
    int indiceFiltro; // Índice para las lecturas del filtro
    long sumaLecturas; // Suma de las lecturas para el filtro
    float salidaIIR; // Variable para almacenar la salida del filtro IIR
    float alpha = 0.5; // Coeficiente de suavizado para el filtro IIR

  public:
    // Se agrega el parámetro pwmChannel al constructor
    Motor(int enable, int in1, int in2, int encoderA, int encoderB, float kp, float ki, float kd, unsigned long muestreo, int pwmChannel)
      : pinEnable(enable), pinIN1(in1), pinIN2(in2), pinEncoderA(encoderA), pinEncoderB(encoderB), 
        kp(kp), ki(ki), kd(kd), intervaloMuestreo(muestreo), errorActual(0), errorPrevio(0), sumaErrores(0), 
        derivadaError(0), referenciaVelocidad(0), tiempoPrevio(0), posicionEncoder(0), velocidadActual(0), 
        valorPWM(0), pwmChannel(pwmChannel) { // Asignación del canal PWM
    }

    void inicializar() {
      // Configuración de los pines de motor
      pinMode(pinIN1, OUTPUT);
      pinMode(pinIN2, OUTPUT);

      // Configuración del encoder
      encoder.attachHalfQuad(pinEncoderA, pinEncoderB);
      encoder.clearCount();

      // Configuración inicial del PWM usando el canal asignado
      ledcSetup(pwmChannel, pwmFrequency, pwmResolution); // Configurar canal de PWM
      ledcAttachPin(pinEnable, pwmChannel); // Asociar pinEnable al canal de PWM

      // Inicializar tiempo y lecturas del encoder
      tiempoPrevio = millis();
      indiceLectura = 0; 
      memset(lecturasEncoder, 0, sizeof(lecturasEncoder));
      indiceFiltro = 0;
      sumaLecturas = 0;
      memset(bufferLecturas, 0, sizeof(bufferLecturas));
      salidaIIR = 0; // Inicializar la salida del filtro IIR
    }

    // Configurar frecuencia del PWM
    void configurarPWM(float frecuencia, int resolucion = 8) {
      pwmFrequency = frecuencia;
      pwmResolution = resolucion;
      ledcSetup(pwmChannel, pwmFrequency, pwmResolution); // Actualizar configuración del canal PWM
    }
    
    // Configuración de velocidad por ticks por segundo
    void setReferenciaVelocidad(float referencia) {
        // Verificar si hay un cambio de referencia
        if (referencia != referenciaAnterior && referencia != 0) {
            errorActual = 0; // Resetear el error actual
            sumaErrores = 0; // Resetear la suma de errores
        }
        referenciaAnterior = referencia; // Actualizar la referencia anterior
        referenciaVelocidad = referencia; // Asignar la nueva referencia
    }

    // Configuración de velocidad por RPS (Revoluciones por segundo)
    void setReferenciaVelocidadRPS(float rps) {
      float nuevaReferencia = (rps * pulsosPorRevolucion) / ajuste; // Convertir RPS a ticks por segundo

      // Verificar si hay un cambio de referencia
      if (nuevaReferencia != referenciaAnterior && nuevaReferencia != 0) {
          errorActual = 0; // Resetear el error actual
          sumaErrores = 0; // Resetear la suma de errores
      }
      referenciaAnterior = nuevaReferencia + nuevaReferencia*0.25 ; // Actualizar la referencia anterior
      referenciaVelocidad = nuevaReferencia ; // Asignar la nueva referencia
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
      }
      controlarMotor(valorPWM);
    }

    long leerEncoder() {
      // Leer el valor actual del encoder
      long lecturaActual = encoder.getCount();

      // Actualizar el buffer y la suma para el filtro de media móvil
      sumaLecturas -= bufferLecturas[indiceFiltro];
      bufferLecturas[indiceFiltro] = lecturaActual;
      sumaLecturas += lecturaActual;

      // Avanzar el índice del buffer
      indiceFiltro = (indiceFiltro + 1) % numLecturasFiltro;

      // Salida del filtro de media móvil
      long salidaMediaMovil = sumaLecturas / numLecturasFiltro;

      // Aplicar el filtro IIR en cascada
      salidaIIR = alpha * salidaMediaMovil + (1 - alpha) * salidaIIR;

      // Retornar el valor filtrado por el IIR
      return salidaIIR;
    }

    float calcularVelocidad(long posicionActual, long posicionAnterior, unsigned long tiempoAnterior) {
      // Usar el valor filtrado del encoder
      long deltaPosicion = posicionActual - posicionAnterior;
      unsigned long deltaTiempo = millis() - tiempoAnterior;

      // Evitar división por cero
      if (deltaTiempo == 0) {
          return 0;
      }

      // Calcular la velocidad en ticks por segundo
      float velocidad = (deltaPosicion / (float)deltaTiempo) * 1000;

      // Aplicar un filtro adicional si es necesario
      // Por ejemplo, un filtro de media móvil o un filtro de Kalman

      return velocidad;
    }

    float calcularPID(float referencia, float actual) {
      errorActual = referencia - actual;
      sumaErrores += errorActual;
      
      // Agregamos una proteccion atraves de la suma de los errores.  Para ponerle un limite.
      if (sumaErrores > 65536) sumaErrores = 65536;
      if (sumaErrores < -65536) sumaErrores = -65536;

      // Cambiar la derivada del error a la derivada de la salida medida
      // derivadaError = actual - velocidadActual; // Anti-derivative kick

  // Calcular la derivada del error correctamente
    derivadaError = errorActual - errorPrevio; // Cambiar a la diferencia de errores

      float salidaSinLimitar = (kp * errorActual) + (ki * sumaErrores) + (kd * derivadaError);
      errorPrevio = errorActual;

      // Saturación normal
      if (salidaSinLimitar > 255) salidaSinLimitar = 255;
      if (salidaSinLimitar < -255) salidaSinLimitar = -255;
      
      // Aplicar limitador de rampa
      float cambio = salidaSinLimitar - valorPIDAnterior;
      
      // Limitar la tasa de cambio
      if (cambio > maxCambioRampa)
        cambio = maxCambioRampa;
      else if (cambio < -maxCambioRampa)
        cambio = -maxCambioRampa;
      
      float salidaLimitada = valorPIDAnterior + cambio;
      valorPIDAnterior = salidaLimitada;
      
      return salidaLimitada;
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
        // Modificar el comportamiento del freno activo
        // Aplicar freno solo si el motor debe detenerse completamente
        if (referenciaVelocidad == 0) {
            digitalWrite(pinIN1, HIGH);
            digitalWrite(pinIN2, HIGH); // Freno activo
            ledcWrite(pwmChannel, 0);
        } else {
            // Mantener el último estado del motor
            ledcWrite(pwmChannel, 0);
        }
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

void resetEncodersValues(){

  encoder.clearCount();
}

};

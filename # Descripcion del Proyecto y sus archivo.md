# Descripcion del Proyecto y sus archivos

Este proyecto implementa un sistema de control de motores utilizando un controlador PID en un ESP32. El sistema permite controlar la velocidad de dos motores y leer la corriente consumida por ellos utilizando un sensor ACS712.

## Características

1. **Control PID**: Implementa un controlador PID para regular la velocidad de los motores.
2. **Lectura de Corriente**: Utiliza el sensor ACS712 para medir la corriente consumida por los motores.
3. **Comandos Seriales**: Permite el control y monitoreo del sistema a través de comandos enviados por el puerto serial.
4. **Freno Activo**: Implementa un mecanismo de frenado activo para detener los motores rápidamente.
5. **Anti-Windup**: Incluye una limitación en la acumulación de errores en el cálculo del PID para evitar el windup.

## Archivos

- `MotorControlPIDV1.h`: Declaraciones de la clase `Motor`.
- `MotorControlPIDV1.cpp`: Definiciones de la clase `Motor`.
- `main.cpp`: Código principal que inicializa el sistema y procesa los comandos seriales.

## Clase `Motor`

### Métodos

- `Motor(int enable, int in1, int in2, int encoderA, int encoderB, float kp, float ki, float kd, unsigned long muestreo, int pwmChannel)`: Constructor de la clase `Motor`.
- `void inicializar()`: Inicializa los pines del motor y el encoder.
- `void configurarPWM(float frecuencia, int resolucion = 8)`: Configura la frecuencia y resolución del PWM.
- `void setReferenciaVelocidad(float referencia)`: Establece la velocidad de referencia en ticks por segundo.
- `void setReferenciaVelocidadRPS(float rps)`: Establece la velocidad de referencia en revoluciones por segundo.
- `void setReferenciaVelocidadRPM(float rpm)`: Establece la velocidad de referencia en revoluciones por minuto.
- `void actualizar()`: Actualiza la velocidad del motor y calcula el valor del PID.
- `long leerEncoder()`: Lee la posición actual del encoder.
- `float calcularVelocidad(long posicionActual, long posicionAnterior, unsigned long tiempoAnterior)`: Calcula la velocidad actual del motor.
- `float calcularPID(float referencia, float actual)`: Calcula el valor del PID.
- `void controlarMotor(float valorPID)`: Controla el motor según el valor del PID.
- `void desactivarMotor()`: Desactiva el motor.
- `float getVelocidadTicksPorSegundo()`: Obtiene la velocidad actual en ticks por segundo.
- `float getVelocidadRPS()`: Obtiene la velocidad actual en revoluciones por segundo.
- `float getVelocidadRPM()`: Obtiene la velocidad actual en revoluciones por minuto.
- `float getValorPWM()`: Obtiene el valor actual del PWM.
- `void resetEncodersValues()`: Resetea los valores del encoder.

## Flujo de Trabajo

1. **Inicialización**:
   - En el método `setup()`, se inicializan los motores y el sensor ACS712.
   - Se configuran los pines y el PWM para los motores.
   - Se calibra el sensor ACS712.

2. **Bucle Principal**:
   - En el método `loop()`, se verifica si hay datos disponibles en el puerto serial.
   - Si se recibe un comando, se procesa en la función `processCommand(String command)`.
   - Si se está utilizando el PID, se actualizan los motores.
   - Si no se recibe un comando en un tiempo determinado, se detienen los motores.

3. **Procesamiento de Comandos**:
   - Los comandos se procesan en la función `processCommand(String command)`.
   - Los comandos disponibles son:
     - `m <valor1> <valor2>`: Configura las velocidades de los motores en RPS.
     - `o <valor1> <valor2>`: Configura el PWM de los motores directamente.
     - `b`: Devuelve el baudrate actual.
     - `e`: Devuelve los valores de los encoders.
     - `r`: Resetea los valores de los encoders.
     - `i`: Devuelve un mensaje de OK.
     - `v`: Devuelve la velocidad actual de los motores en RPS.
     - `c`: Devuelve la corriente consumida por los motores.

## Ejecución

Para ejecutar el proyecto, sigue estos pasos:

1. Conecta el ESP32 a tu computadora.
2. Abre el proyecto en PlatformIO.
3. Compila y sube el código al ESP32.
4. Abre el monitor serial para enviar comandos y ver los resultados.

## Notas

- Asegúrate de ajustar los pines y parámetros del motor según tu configuración de hardware.
- Puedes modificar los valores de `kp`, `ki` y `kd` para ajustar el comportamiento del controlador PID.

¡Disfruta controlando tus motores con este proyecto!
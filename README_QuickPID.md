# Implementación de Control de Motores con QuickPID

Este proyecto implementa un sistema de control PID para motores con encoder usando la librería QuickPID para ESP32.

## Requisitos

- PlatformIO IDE (VS Code + extensión PlatformIO)
- Placa ESP32
- Motores DC con encoders
- Módulo ACS712 (opcional, para monitoreo de corriente)

## Instalación de la librería QuickPID

Para usar este código, es necesario instalar la librería QuickPID:

1. En PlatformIO, abre el archivo `platformio.ini`
2. Añade la siguiente línea en la sección `lib_deps`:
   ```
   QuickPID
   ```
3. O alternativamente, instala la librería manualmente desde el PlatformIO Library Manager

## Archivos del proyecto

- `MotorControlPIDQuickPID.cpp`: Contiene la clase `Motor` modificada para usar QuickPID
- `main_QuickPID.cpp`: Versión adaptada del archivo principal que usa la implementación con QuickPID

## Cómo usar

1. Renombra `main_QuickPID.cpp` a `main.cpp` o cambia el archivo principal en `platformio.ini`
2. Compila y carga el proyecto a tu ESP32
3. Conecta motores y encoders según la configuración definida en `main_QuickPID.cpp`

## Ventajas de QuickPID sobre la implementación manual

- Implementación más robusta y optimizada del algoritmo PID
- Mejores opciones de configuración (antiwindup, modos de operación)
- Mejor rendimiento en la ejecución del controlador
- Mantenimiento más sencillo del código

## Comandos disponibles

Los comandos se envían a través del puerto serie con el formato `<comando>`:

- `<m valor1 valor2>`: Configura la velocidad de los motores en RPS
- `<o pwm1 pwm2>`: Control directo por PWM (sin PID)
- `<b>`: Devuelve el baudrate actual
- `<e>`: Devuelve los valores de los encoders
- `<p motor kp ki kd>`: Configura los parámetros PID para un motor específico (nuevo en esta versión)
- `<r>`: Resetea los encoders
- `<i>`: Comando de prueba (devuelve OK)
- `<v>`: Devuelve la velocidad actual de los motores en RPS
- `<c>`: Devuelve la lectura de corriente del sensor ACS712

## Diferencias con la implementación original

- Reemplazo del algoritmo PID manual por QuickPID
- Ajuste dinámico de parámetros PID en tiempo de ejecución con el comando `<p>`
- Mejor manejo del anti-windup
- Configuración más avanzada del controlador PID

## Personalización del PID

Puedes ajustar los parámetros del PID en tiempo de ejecución utilizando el nuevo comando `<p>`:

```
<p 3 0.1 0.2 0.05>  // Configura kp=0.1, ki=0.2, kd=0.05 para el motor 3
```

O modificar la configuración inicial en el constructor de cada motor. 
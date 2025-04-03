# Contenido
1. [Materiales](#materiales)
2. [Esquema de Conexión Motores con L298N](#esquema-de-conexión-motores-con-l298n)
3. [Código Funcionamiento Motores con L298N](#esquema-de-conexión-motores-con-l298n)
4. [Esquema de Conexión MPU6500](#esquema-de-conexión-mpu6500)
5. [Código con Arduino Usando MPU6500](#-ejemplo-de-código-básico-mpu6500)

## Materiales
- Arduino UNO R3
- Cable Largo USB para Arduino
- Cables Dupont (20) M-H.
- Cables Dupont (10) M-M.
- Sensor SR04.
- Sensor RGB.
- Sensor MPU6500.
- Driver L298N.
- Protoboard 170 puntos.
- Chasis (placa acrílico) básico con dos ruedas y rueda loca.
- Piezas para fijación de motor (2).
- Bateria 9V recargable.
- Broche Bateria 9V.
- Motores DC Amarillos (2).
- Servomotor SG90.
- Kit de tornillos.

## Esquema de Conexión Motores con L298N
![Esquema de L298N](./esquema1.png)

### Conectando Arduino UNO + L298N + Motor DC
- Conectar los Pines IN1 a pin 9 (Arduino UNO), IN2 pin 10 (Arduino UNO), INA a pin 5 (PWM).
- GND es conectar al GND del Arduino.
- Vlógico es conectar 5V del Arduino.
- Salida Motor A, conectar el motor A que sería el motor derecho.
- Salida Motor B, conectar el motor B que sería el motor izquierdo.
- En Vin requiere de una fuente de alimentación externa, el cual puede ser una batería de 9 Voltios. 

## Esquema de conexión
+-------------------+           +---------------------+           +---------------+
|    Arduino Uno    |           |      L298N         |           |   Motor DC    |
|                   |           |                   |           |               |
|   Pin 9 --------> | IN1       | OUT1 ------------> +          +  Terminal 1   |
|  Pin 10 --------> | IN2       | OUT2 ------------> +          +  Terminal 2   |
|   Pin 5 --------> | ENA       |                   |           |               |
|   GND --------->  | GND       |                   |           |               |
|   5V --------->   | 5V        |                   |           |               |
+-------------------+           +---------------------+           +---------------+


## 🔌 Ejemplo de código usando L298N
```arduino
# Código de ejemplo en Python (simulado, normalmente usarías Arduino C++)
# En Arduino sería algo así:
void setup() {
  pinMode(9, OUTPUT); // IN1
  pinMode(10, OUTPUT); // IN2
  pinMode(5, OUTPUT); // ENA
}

void loop() {
  digitalWrite(9, HIGH);
  digitalWrite(10, LOW);
  analogWrite(5, 200); // Velocidad (0-255)
  delay(2000);

  digitalWrite(9, LOW);
  digitalWrite(10, HIGH);
  analogWrite(5, 200);
  delay(2000);
}
```
## Esquema de Conexión MPU6500

![Esquema de L298N](./esquema2.png)

## Esquema de Conexión MPU6500

| Arduino Uno | MPU-6500 | Descripción          |
|:----------:|:--------:|:--------------------:|
| 5V        | VCC      | Alimentación (+5V)   |
| GND       | GND      | Tierra               |
| A4        | SDA      | Datos I2C            |
| A5        | SCL      | Reloj I2C            |

### Explicación rápida
- - SDA (A4): Línea de datos.
- - SCL (A5): Línea de reloj.
- - Ambas líneas deben llevar una resistencia pull-up (normalmente ya vienen integradas en el módulo).
- - El GND debe estar común con el GND de Arduino.

## 🔌 Ejemplo de código básico MPU6500
```arduino 
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("MPU6500 no encontrado!");
    while (1);
  }
  Serial.println("MPU6500 listo.");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Aceleración X: "); Serial.print(a.acceleration.x); Serial.print(" m/s² ");
  Serial.print(" Y: "); Serial.print(a.acceleration.y);
  Serial.print(" Z: "); Serial.println(a.acceleration.z);

  Serial.print("Giroscopio X: "); Serial.print(g.gyro.x);
  Serial.print(" Y: "); Serial.print(g.gyro.y);
  Serial.print(" Z: "); Serial.println(g.gyro.z);

  delay(500);
}
```

## 🔌 Ejemplo de código usando MPU6500 y Filtrando la señal
```arduino
#include <Wire.h>
#include <MPU6050_light.h> // Compatible con MPU6500

MPU6050 mpu(Wire);

// Parámetros del filtro
#define N 10 // Número de muestras
float ax_buffer[N]; // Buffer para almacenar las últimas N lecturas
int index = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  mpu.begin();
  mpu.calcOffsets(); // Calibración
  
  // Inicializar buffer a cero
  for (int i = 0; i < N; i++) {
    ax_buffer[i] = 0;
  }
}

float filterAx(float new_value) {
  // Guardar nueva lectura
  ax_buffer[index] = new_value;
  index = (index + 1) % N;

  // Calcular promedio
  float sum = 0;
  for (int i = 0; i < N; i++) {
    sum += ax_buffer[i];
  }
  return sum / N;
}

void loop() {
  mpu.update();
  
  // Leer aceleración en X
  float ax = mpu.getAccX() * 9.81; // Convertir a m/s²
  
  // Aplicar filtro de media móvil
  float ax_filtrado = filterAx(ax);
  
  // Mostrar datos
  Serial.print("Aceleración cruda: ");
  Serial.print(ax, 2);
  Serial.print(" m/s²   |   Filtrada: ");
  Serial.print(ax_filtrado, 2);
  Serial.println(" m/s²");
  
  delay(50);
}
```

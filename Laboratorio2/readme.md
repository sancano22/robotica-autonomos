# Contenido
1. [Esquema de Conexión HC-SR04](#esquema-de-conexión-hc-sr04)
2. [Esquema de Conexión Sensor RGB (TCS34725)](#esquema-de-conexión-sensor-rgb)
3. [Algoritmo básico detección de Color](#algoritmo-básico-de-detección-color)
4. [Algoritmo detección de Obstáculos](#algoritmo-de-obstáculo-con-umbral)


## Esquema de Conexión HC-SR04

| HC-SR04 | Arduino UNO |
| ------- | ----------- |
| VCC     | 5V          |
| GND     | GND         |
| TRIG    | D9          |
| ECHO    | D10         |


Pines del HC-SR04:

    VCC: 5V
    GND: GND
    TRIG: Pin digital para enviar pulso
    ECHO: Pin digital para recibir el pulso
----
## Esquema de Conexión Sensor RGB (TCS34725)
| Pin del TCS34725   | ESP32 (Wemos D1 R32)     | Arduino UNO | Función                 |
| ------------------ | ------------------------ | ----------- | ----------------------- |
| **VIN**            | 3.3V o 5V (según módulo) | 5V          | Alimentación            |
| **GND**            | GND                      | GND         | Tierra común            |
| **SCL**            | GPIO 22 (D22)            | A5          | I2C reloj               |
| **SDA**            | GPIO 21 (D21)            | A4          | I2C datos               |
| **INT** (opcional) | No conectar              | No conectar | Interrupción (opcional) |

La mayoría de los módulos TCS34725 tienen un regulador, así que puedes conectarlo directamente a 5V.

**Librería TCS34725**
Desde el IDE de Arduino:

    1. Ve a Sketch > Include Library > Manage Libraries...

    2. Busca "TCS34725"

    3. Instala Adafruit TCS34725
----
## Algoritmo detección color
```c
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Crear objeto del sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

void setup() {
  Serial.begin(9600);
  
  if (tcs.begin()) {
    Serial.println("Sensor TCS34725 detectado.");
  } else {
    Serial.println("No se encontró el sensor TCS34725.");
    while (1); // Se detiene
  }
}

void loop() {
  uint16_t r, g, b, c;
  
  tcs.getRawData(&r, &g, &b, &c);
  
  // Calcula el valor de luminosidad y color corregido
  uint16_t colorTemp = tcs.calculateColorTemperature(r, g, b);
  uint16_t lux = tcs.calculateLux(r, g, b);

  Serial.print("Rojo: "); Serial.print(r);
  Serial.print(" Verde: "); Serial.print(g);
  Serial.print(" Azul: "); Serial.print(b);
  Serial.print(" Clear: "); Serial.print(c);
  Serial.print(" Lux: "); Serial.print(lux);
  Serial.print(" Temp Color: "); Serial.print(colorTemp);
  Serial.println(" K");

  delay(1000);
}
```
**Calibrar el Sensor de Color**
Compensar las lecturas del sensor para que los valores crudos (rojo, verde, azul) correspondan mejor a los colores reales o esperados.

*A tener en cuenta:* 
1. Normalizar los valores RGB.
2. Compensar luz ambiental.
3. Detectar colores estándar (rojo, verde, azul, blanco, negro).
4. Ajustar umbrales o aplicar transformaciones.

**1. Preparar el entorno de calibración**
1. Preparar el entorno de calibración
2. Usa muestras de colores conocidas (papel rojo, verde, azul, blanco, negro).
3. Mantén la distancia constante entre sensor y muestra (~1–2 cm).
4. Evita luz ambiental variable (mejor en interior con luz controlada).
5. Activa el LED blanco del sensor (puedes conectarlo a GND para que quede siempre encendido).

**2. Leer datos crudos del sensor**
```c
#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725();

void setup() {
  Serial.begin(9600);
  if (tcs.begin()) {
    Serial.println("Sensor listo");
  } else {
    Serial.println("No se detecta el TCS34725");
    while (1);
  }
}

void loop() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  Serial.print("R: "); Serial.print(r);
  Serial.print(" G: "); Serial.print(g);
  Serial.print(" B: "); Serial.print(b);
  Serial.print(" C: "); Serial.println(c);

  delay(1000);
}
```
**3.Calcular valores normalizados**
*Usa un umbral aproximado (puedes ajustarlo según tus pruebas)*
```c
float fr = (float)r / c;
float fg = (float)g / c;
float fb = (float)b / c;
```
**4. Identificar colores**
```c
if (fr > 0.4 && fg < 0.3 && fb < 0.3) {
  Serial.println("Rojo");
} else if (fr < 0.3 && fg > 0.4 && fb < 0.3) {
  Serial.println("Verde");
} else if (fr < 0.3 && fg < 0.3 && fb > 0.4) {
  Serial.println("Azul");
} else if (c < 100) {
  Serial.println("Negro");
} else if (fr > 0.3 && fg > 0.3 && fb > 0.3) {
  Serial.println("Blanco");
}

```
**A tener en cuenta para tener una buena Calibración**
1. Mide varias veces cada color conocido.
2. Anota los valores promedio de RGB y clear.
3. Ajusta los umbrales del if anterior según tu entorno y colores reales.
4. Guarda esos valores como referencia para futuras lecturas.


**Umbrales en Sensor de Color**
Son rangos de valores normalizados o proporciones de los componentes RGB que se usan para decidir qué color estás viendo. Por ejemplo:
- - Si el rojo es alto y los otros bajos → probablemente sea rojo.
- - Si todos los valores son altos → probablemente sea blanco.
- - Si todos son bajos → puede ser negro o muy oscuro.

**Paso 1: Normalización**
Normaliza los valores crudos del sensor para que los colores no dependan de la iluminación total:
```c
float fr = (float)r / c;
float fg = (float)g / c;
float fb = (float)b / c;
```
**Paso 2: Definir umbrales x Color**
*Rojo*
```c
(fr > 0.4) && (fg < 0.3) && (fb < 0.3)
```
*Verde*
```c
(fr < 0.3) && (fg > 0.4) && (fb < 0.3)
```
*Azul*
```c
(fr < 0.3) && (fg < 0.3) && (fb > 0.4)
```
*Blanco*
```c
(fr > 0.3) && (fg > 0.3) && (fb > 0.3) && (c > 1500)
```
*Negro*
```c
(c < 200)  // baja luz reflejada total
```

**Puedes ajustar c según tus condiciones de luz (más fuerte o más débil)**

**EJEMPLO**
```c
String detectarColor(float fr, float fg, float fb, uint16_t c) {
  if (c < 200) {
    return "Negro";
  } else if (fr > 0.4 && fg < 0.3 && fb < 0.3) {
    return "Rojo";
  } else if (fr < 0.3 && fg > 0.4 && fb < 0.3) {
    return "Verde";
  } else if (fr < 0.3 && fg < 0.3 && fb > 0.4) {
    return "Azul";
  } else if (fr > 0.3 && fg > 0.3 && fb > 0.3 && c > 1500) {
    return "Blanco";
  } else {
    return "Desconocido";
  }
}
```
**Recomendación**
1. Mide varias veces cada color real en tu entorno.
2. Promedia los valores r, g, b, c.
3. Normaliza y anota fr, fg, fb.
4. Ajusta los umbrales anteriores con tus datos reales.
----
## Algoritmo de obstáculo (HC-SR04)
**¿Qué es un Umbral?**
Es un valor límite que se usa para tomar decisiones en un algoritmo. En palabras simples *Es el número a partir del cual algo ocurre*.

- - considerando un umbral de 20, entonces podemos decir **Si la distancia es menor a 20 cm (el umbral), entonces el robot detecta un obstáculo**

```c
const int trigPin = 9;
const int echoPin = 10;
const int ledPin = 13;

const int umbral = 20; // cm

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Emitir pulso
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Medir duración del eco
  long duracion = pulseIn(echoPin, HIGH);
  float distancia = duracion * 0.034 / 2;

  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.println(" cm");

  // Detección de obstáculo
  if (distancia <= umbral) {
    Serial.println("¡Obstáculo detectado!");
    digitalWrite(ledPin, HIGH); // Encender LED
    // Aquí podrías detener motores o girar
  } else {
    digitalWrite(ledPin, LOW); // Apagar LED
  }

  delay(200);
}
```

**¿Qué es Calibrar?**
Ajustar el sistema para que el valor medido sea lo más cercano al valor real.

**Paso a Paso**

    1. Coloca el sensor fijo apuntando a una pared.
    2. Ubica una regla o mide con una cinta métrica distancias exactas (por ejemplo: 10 cm, 20 cm, ..., 100 cm).
    3. Mide con el sensor la distancia a la pared en esos puntos.
    4. Registra en una tabla: distancia real vs medida.

*si nota que el sensor siempre mide 1 cm* una alternativa es:
```c
float distancia = duracion * 0.034 / 2;
distancia -= 1.0; // corrección empírica
```
Otra alternativ es **factor de calibración**
```c
float distancia = duracion * 0.034 / 2;
distancia *= 0.95; // ajuste si el sensor mide un poco de más
```
**Lectura Promedio con Calibración**

```c 
const int trigPin = 9;
const int echoPin = 10;
const int numLecturas = 10;   // Número de lecturas a promediar
const float correccion = -1.0; // Corrección empírica (puedes ajustarla)

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  float suma = 0;

  for (int i = 0; i < numLecturas; i++) {
    suma += medirDistancia();
    delay(50); // pequeña pausa entre lecturas
  }

  float promedio = suma / numLecturas;
  promedio += correccion; // aplicar corrección empírica

  Serial.print("Distancia promedio: ");
  Serial.print(promedio);
  Serial.println(" cm");

  delay(1000); // espera 1 segundo antes del siguiente ciclo
}

// Función para medir distancia puntual
float medirDistancia() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH);
  float distancia = duracion * 0.034 / 2.0;
  return distancia;
}
```

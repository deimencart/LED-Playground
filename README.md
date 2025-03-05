# LED-Playground
This repository explains the code and the usage for the project of a touchless lamp with a measurement sensor and circle of leds.

# Control de LEDs con Sensor VL53L0X y Dimmer en Arduino

## 📌 Introducción
Este código permite controlar un aro de LEDs WS2812 usando un **sensor de distancia VL53L0X** en un **Arduino**.  
- Los LEDs **se encienden** cuando la mano se acerca a menos de 50 mm.
- Su **brillo varía gradualmente** según la distancia de la mano.
- Si la mano se mantiene **menos de 50 mm** después de encendidos, **se apagan**.
- Hay un **tiempo de espera** de **5 segundos** antes de volver a detectar después de apagarse.
- Si la mano se mantiene **5 segundos en la misma posición**, se **mantiene el brillo** por otros **5 segundos** sin actualizar la lectura.
- Se usa el **LED incorporado de la placa** (usualmente en el **pin 13**) para indicar que los LEDs están activos.

---

## 🚀 Materiales Necesarios
- **Arduino Uno / Mega / Nano**
- **Sensor VL53L0X** (ToF)
- **Aro de LEDs WS2812** (Neopixel)
- **Cables de conexión**
- **Fuente de alimentación 5V (si el consumo de LEDs es alto)**

---

## 🛠 Conexiones
| Componente     | Arduino |
|---------------|---------|
| VL53L0X VCC   | 5V      |
| VL53L0X GND   | GND     |
| VL53L0X SDA   | A4 (SDA)|
| VL53L0X SCL   | A5 (SCL)|
| LEDs VCC      | 5V      |
| LEDs GND      | GND     |
| LEDs DATA     | D6      |

**⚠ Nota:** Algunos Arduino Nano usan **SDA en A4 y SCL en A5**. En otros modelos pueden estar en **pins específicos de I2C**.

---

## 📜 Código
# 📖 Explicación del Código

## 1️⃣ Inicialización y Configuración  
- Se incluyen las librerías necesarias (`Wire.h`, `FastLED.h`, `Adafruit_VL53L0X.h`).  
- Se define el **pin del LED integrado** (pin 13).  
- Se configuran los **LEDs y el sensor VL53L0X**.  

## 2️⃣ Variables Clave  
- **isOn**: Indica si los LEDs están encendidos.  
- **lastOffTime**: Guarda el tiempo en que los LEDs se apagaron.  
- **holdStartTime**: Guarda el tiempo en que los LEDs comenzaron a brillar.  
- **holdingBrightness**: Indica si el brillo se mantiene fijo.  

## 3️⃣ Control de Encendido y Apagado  
- Si la distancia es menor a `MIN_DIST` (**50 mm**), **se encienden los LEDs** y el **LED del Arduino**.  
- Si se mantiene la mano en la misma posición **por 5 segundos**, **se congela el brillo**.  
- Si la distancia baja de nuevo a `MIN_DIST`, **los LEDs se apagan**.  
- Después de apagarse, **espera 5 segundos antes de volver a detectar**.  

## 4️⃣ Control del LED Incorporado en el Arduino  
- Cuando los LEDs **se encienden**, el **LED del Arduino (pin 13) también se enciende**.  
- Cuando los LEDs **se apagan**, el **LED del Arduino se apaga**.  

---

# 🔥 Paso a Paso para Implementarlo  
1. **Conecta el VL53L0X y el aro de LEDs** según la tabla de conexiones.  
2. **Carga el código en el Arduino** usando el IDE de Arduino.  
3. **Abre el Monitor Serial** para ver la distancia detectada (**opcional**).  
4. **Acerca la mano a menos de 50 mm** para encender los LEDs.  
5. **Mueve la mano** para cambiar el brillo o **mantenla** para fijar la intensidad.  
6. **Si bajas la mano nuevamente a menos de 50 mm, los LEDs se apagan**.  
7. **Después de apagarse, espera 5 segundos antes de volver a encender**.  

---

# ✅ Conclusión  
Este código permite encender LEDs con **control de brillo progresivo**, un **tiempo de espera tras apagarse**, y un **LED en la placa para indicar cuando están encendidos**.  
Es ideal para **control sin contacto** en lámparas o efectos de iluminación.  

🚀✨ ¡Espero que te sirva!  

```cpp
#include <Wire.h>
#include <FastLED.h>
#include <Adafruit_VL53L0X.h>

#define LED_PIN     6      // Pin del aro de LEDs
#define NUM_LEDS    16     // Número de LEDs en el aro
#define LED_TYPE    WS2812 // Tipo de LED
#define COLOR_ORDER GRB    // Orden de colores
#define MIN_BRIGHT  2      // Brillo mínimo
#define MAX_BRIGHT  255    // Brillo máximo
#define MIN_DIST    50     // Distancia mínima en mm para encender
#define MAX_DIST    300    // Distancia máxima en mm para brillo máximo

#define WAIT_TIME   5000   // Tiempo de espera antes de nueva detección (5s)
#define HOLD_TIME   5000   // Tiempo para mantener brillo fijo (5s)
#define BUILTIN_LED 13     // LED incorporado en Arduino (pin 13)

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
CRGB leds[NUM_LEDS];

bool isOn = false;         // Estado de los LEDs
unsigned long lastOffTime = 0;
unsigned long holdStartTime = 0;
bool holdingBrightness = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!lox.begin()) {
    Serial.println("¡Error al iniciar VL53L0X!");
    while (1);
  }

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW); // Apagar LED de Arduino al inicio
}

void loop() {
  if (!isOn && millis() - lastOffTime < WAIT_TIME) {
    return;  // Espera antes de volver a detectar
  }

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) { // Si la lectura es válida
    int distance = measure.RangeMilliMeter;
    Serial.print("Distancia: "); Serial.println(distance);

    if (!isOn && distance <= MIN_DIST) {
      isOn = true;
      FastLED.setBrightness(MIN_BRIGHT);
      setWhite();
      digitalWrite(BUILTIN_LED, HIGH);
      holdStartTime = millis(); 
      holdingBrightness = false;
    }

    if (isOn) {
      if (distance <= MIN_DIST) {
        isOn = false;
        FastLED.clear();
        FastLED.show();
        digitalWrite(BUILTIN_LED, LOW);
        lastOffTime = millis(); // Marca tiempo de apagado
      } else {
        int brightness = map(distance, MIN_DIST, MAX_DIST, MIN_BRIGHT, MAX_BRIGHT);
        brightness = constrain(brightness, MIN_BRIGHT, MAX_BRIGHT);

        if (!holdingBrightness) {
          FastLED.setBrightness(brightness);
          FastLED.show();

          if (millis() - holdStartTime >= HOLD_TIME) {
            holdingBrightness = true;
            lastOffTime = millis(); // Bloquea lecturas por 5s
          }
        }
      }
    }
  } else {
    Serial.println("Error en la medición");
  }

  delay(100); // Pequeño delay para estabilidad
}

void setWhite() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 255, 255);
  }
  FastLED.show();
}

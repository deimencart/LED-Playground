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
#define MIN_BRIGHT  2      // Brillo mínimo (casi apagado)
#define MAX_BRIGHT  255    // Brillo máximo (cuando estés lejos)
#define MIN_DIST    50     // Distancia mínima para encender (mm)
#define MAX_DIST    300    // Distancia máxima en mm para brillo máximo
#define OFF_HOLD_TIME 2000 // Tiempo en ms que la mano debe mantenerse cerca para apagar
#define RESTART_DELAY 5000 // Tiempo en ms antes de volver a detectar después de apagar
#define STABLE_HOLD_TIME 5000 // Tiempo en ms que debe mantenerse en una posición para fijar el brillo
#define IGNORE_TIME 5000  // Tiempo en ms que se ignora la lectura después de fijar el brillo

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
CRGB leds[NUM_LEDS]; // Arreglo de LEDs

bool isOn = false;   // Estado de los LEDs
bool brightnessLocked = false; // Indica si el brillo está bloqueado
int lockedBrightness = 0; // Brillo bloqueado
int lastDistance = 0; // Última distancia registrada
unsigned long holdStartTime = 0; // Momento en que la mano se acercó para apagarlos
unsigned long offTime = 0; // Tiempo en que los LEDs se apagaron
unsigned long stableStartTime = 0; // Momento en que la distancia fue estable
unsigned long ignoreUntil = 0; // Tiempo hasta el cual se ignoran lecturas

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!lox.begin()) {
    Serial.println("¡Error al iniciar VL53L0X!");
    while (1);
  }

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  
  // Iniciar con LEDs apagados
  FastLED.clear(); 
  FastLED.show();
}

void loop() {
  unsigned long currentMillis = millis();

  // Si los LEDs están apagados, esperar 5 segundos antes de volver a leer
  if (!isOn && offTime != 0 && currentMillis - offTime < RESTART_DELAY) {
    return;
  }

  // Si el brillo está bloqueado, ignorar lecturas por IGNORE_TIME
  if (brightnessLocked && currentMillis < ignoreUntil) {
    return;
  }

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) { // Si la lectura es válida
    int distance = measure.RangeMilliMeter;
    Serial.print("Distancia: "); Serial.println(distance);

    if (!isOn && distance < MIN_DIST) {
      // Si los LEDs están apagados y la mano está cerca, encenderlos
      isOn = true;
      brightnessLocked = false; // Resetear bloqueo
      Serial.println("💡 Encendiendo LEDs");
    }

    if (isOn) {
      if (distance < MIN_DIST) {
        if (holdStartTime == 0) {
          holdStartTime = currentMillis; // Empieza a contar el tiempo
        } else if (currentMillis - holdStartTime >= OFF_HOLD_TIME) {
          // Si la mano se mantiene cerca por el tiempo definido, apagar LEDs
          isOn = false;
          holdStartTime = 0; // Resetear el contador
          offTime = currentMillis; // Guardar el tiempo de apagado
          Serial.println("💤 Apagando LEDs");
          FastLED.clear();
          FastLED.show();
          return;
        }
      } else {
        holdStartTime = 0; // Resetear el tiempo si la mano se aleja

        // Si la distancia cambia significativamente, resetear estabilidad
        if (abs(distance - lastDistance) > 5) {
          stableStartTime = currentMillis;
        }

        // Si la mano se mantiene estable por STABLE_HOLD_TIME, fijar el brillo
        if (!brightnessLocked && currentMillis - stableStartTime >= STABLE_HOLD_TIME) {
          brightnessLocked = true;
          lockedBrightness = FastLED.getBrightness();
          ignoreUntil = currentMillis + IGNORE_TIME; // Ignorar lecturas por IGNORE_TIME
          Serial.println("🔒 Brillo fijado, ignorando lecturas por 5 segundos");
        }

        if (!brightnessLocked) {
          // Ajustar brillo según la distancia
          int brightness = map(distance, MIN_DIST, MAX_DIST, MIN_BRIGHT, MAX_BRIGHT);
          brightness = constrain(brightness, MIN_BRIGHT, MAX_BRIGHT);
          FastLED.setBrightness(brightness);
        } else {
          // Mantener el brillo fijo
          FastLED.setBrightness(lockedBrightness);
        }

        setWhite();
        FastLED.show();
        lastDistance = distance; // Guardar última distancia
      }
    }
  } else {
    Serial.println("Error en la medición");
  }

  delay(100); // Pequeño delay para estabilidad
}

void setWhite() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 255, 255); // Blanco
  }
}


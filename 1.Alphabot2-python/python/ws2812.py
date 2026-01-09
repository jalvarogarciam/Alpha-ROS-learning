#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# import time
# Importamos la librería específica para controlar estos chips LED
from rpi_ws281x import Adafruit_NeoPixel, Color

# --- CONFIGURACIÓN TÉCNICA (Hardware) ---
# No tocar estos valores salvo que se sepa lo que se hace
LED_COUNT      = 4       # Tenemos 4 bombillitas en el robot
LED_PIN        = 18      # Conectadas al Pin 18 (Soporta PWM)
LED_FREQ_HZ    = 800000  # Frecuencia de datos (800khz es el estándar)
LED_DMA        = 10      # Canal DMA (Memoria directa) para no saturar la CPU
LED_BRIGHTNESS = 255     # Brillo máximo (0 es apagado, 255 es máximo)
LED_INVERT     = False   # No invertir la señal
LED_CHANNEL    = 0       # Canal PWM

# Inicializamos el objeto 'strip' (la tira de LEDs)
strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)

# Arrancamos la comunicación con los LEDs
strip.begin()

print("Encendiendo luces de prueba...")

# --- FASE 1: PREPARAR COLORES ---
# La función setPixelColor guarda en memoria qué color queremos,
# pero NO lo muestra todavía.
# Formato: Color(Rojo, Verde, Azul) donde 255 es el máximo.

strip.setPixelColor(0, Color(255, 0, 0))   # LED 0 -> Rojo Puro
strip.setPixelColor(1, Color(0, 255, 0))   # LED 1 -> Verde Puro
strip.setPixelColor(2, Color(0, 0, 255))   # LED 2 -> Azul Puro
strip.setPixelColor(3, Color(255, 255, 0)) # LED 3 -> Mezcla Rojo+Verde = Amarillo

# --- FASE 2: ENVIAR A LOS LEDs ---
# Ahora sí, empujamos los datos al cable. ¡Se hace la luz!
strip.show()

# Esperamos 2 segundos para admirar el resultado
time.sleep(2)

print("Apagando...")

# --- FASE 3: APAGADO ---
# Para apagar, simplemente pintamos de "Negro" (0, 0, 0)
strip.setPixelColor(0, Color(0, 0, 0))
strip.setPixelColor(1, Color(0, 0, 0))
strip.setPixelColor(2, Color(0, 0, 0))
strip.setPixelColor(3, Color(0, 0, 0))

# Importante: Volver a enviar los datos para que el cambio surta efecto
strip.show()
#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
from AlphaBot2 import AlphaBot2
from rpi_ws281x import Adafruit_NeoPixel, Color  # Librería para controlar luces LED inteligentes
from TRSensors import TRSensor
import time

Button = 7

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Button, GPIO.IN, GPIO.PUD_UP)

# --- CONFIGURACIÓN DE LOS LEDs (NeoPixel/WS2812B) ---
LED_COUNT      = 4       # Número de luces en el robot
LED_PIN        = 18      # Pin GPIO conectado a las luces (Soporta PWM)
LED_FREQ_HZ    = 800000  # Frecuencia de señal (Estándar 800khz)
LED_DMA        = 5       # Canal DMA para generar señal sin cargar la CPU
LED_BRIGHTNESS = 255     # Brillo (0 a 255)
LED_INVERT     = False   

# --- VARIABLES DEL CONTROL PID ---
maximum = 35             # Velocidad máxima de las ruedas
integral = 0             # Error acumulado (Memoria)
last_proportional = 0    # Error anterior (Para saber si mejora o empeora)
j = 0                    # Variable para la animación de colores

def Wheel(pos):
    """
    Función auxiliar para generar colores del Arcoíris.
    Convierte una posición (0-255) en un color RGB.
    """
    if pos < 85:
        return Color(pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return Color(255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return Color(0, pos * 3, 255 - pos * 3)

# Inicializamos la tira de LEDs
strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)
strip.begin()

# Mostramos colores de prueba al arrancar para ver que funcionan
strip.setPixelColor(0, Color(100, 0, 0))       # Rojo
strip.setPixelColor(1, Color(0, 100, 0))       # Azul
strip.setPixelColor(2, Color(0, 0, 100))       # Verde
strip.setPixelColor(3, Color(100, 100, 0))     # Amarillo
strip.show()

# Inicializamos Sensores y Motor
TR = TRSensor()
Ab = AlphaBot2()
Ab.stop()

print("Iniciando Ejemplo de Siguelíneas PID con LEDs")
time.sleep(0.5)

# --- FASE 1: CALIBRACIÓN ---
# El robot se mueve lado a lado para "aprender" qué es blanco y qué es negro
for i in range(0,100):
    if(i<25 or i>= 75):
        Ab.right(); Ab.setPWMA(30); Ab.setPWMB(30)
    else:
        Ab.left();  Ab.setPWMA(30); Ab.setPWMB(30)
    TR.calibrate() # Lee los valores extremos
Ab.stop()

print("Mínimos detectados:", TR.calibratedMin)
print("Máximos detectados:", TR.calibratedMax)
print("PULSA EL BOTÓN PARA EMPEZAR")

# --- FASE 2: ESPERA ---
while (GPIO.input(Button) != 0):
    position,Sensors = TR.readLine()
    # Imprimimos la posición (0 a 4000) donde 2000 es el centro
    print(position, Sensors)
    time.sleep(0.05)

Ab.forward() # ¡Arrancamos!

# --- FASE 3: BUCLE PRINCIPAL (PID + ANIMACIÓN) ---
while True:
    try:
        # Leemos la posición de la línea (0 a 4000)
        position,Sensors = TR.readLine()
        
        # SEGURIDAD: Si todos los sensores ven negro (>900) o levantamos el robot...
        if(Sensors[0] >900 and Sensors[1] >900 and Sensors[2] >900 and Sensors[3] >900 and Sensors[4] >900):
            Ab.setPWMA(0) # Parar motores
            Ab.setPWMB(0)
        else:
            # --- CÁLCULO PID ---
            # 1. Proporcional: ¿Cuánto me he alejado del centro (2000)?
            proportional = position - 2000
            
            # 2. Derivativo: ¿Cómo de rápido me estoy alejando? (Diferencia con el anterior)
            derivative = proportional - last_proportional
            
            # 3. Integral: La suma de todos los errores pasados (corrige desviaciones pequeñas constantes)
            integral += proportional
            
            # Guardamos la posición actual para la siguiente vuelta
            last_proportional = proportional

            # FÓRMULA MAESTRA:
            # power_difference es cuánto debemos frenar una rueda y acelerar la otra.
            # Los números (30, 10000, 2) son las "constantes" que ajustan la sensibilidad.
            power_difference = proportional/30  + integral/10000 + derivative*2  

            # Limitamos el valor para no exceder la velocidad máxima
            if (power_difference > maximum):
                power_difference = maximum
            if (power_difference < - maximum):
                power_difference = - maximum
            
            # Aplicamos la corrección a los motores
            # Si power_difference es negativo, giramos a un lado; si es positivo, al otro.
            if (power_difference < 0):
                Ab.setPWMA(maximum + power_difference)
                Ab.setPWMB(maximum)
            else:
                Ab.setPWMA(maximum)
                Ab.setPWMB(maximum - power_difference)
            
        # --- ANIMACIÓN DE LEDs ---
        # Recorremos cada LED y le asignamos un color basado en la variable 'j'
        for i in range(0, strip.numPixels()):
            # Lógica matemática para crear el efecto arcoíris rotatorio
            color_val = Wheel((int(i * 256 / strip.numPixels()) + j) & 255)
            strip.setPixelColor(i, color_val)
        strip.show() # Actualizamos las luces
        
        # Avanzamos el contador de la animación
        j += 1
        if(j > 256*4): 
            j= 0
            
    except KeyboardInterrupt:
        break # Salir si pulsamos Ctrl+C
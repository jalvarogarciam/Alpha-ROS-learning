#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO  # Para leer los sensores
import time              # Para controlar los tiempos de giro
from AlphaBot2 import AlphaBot2 # Importamos nuestro driver de motores

# Creamos el objeto robot para poder moverlo
Ab = AlphaBot2()

# Definición de Pines (Mapeado físico)
DR = 16  # Sensor Derecho (Digital Right)
DL = 19  # Sensor Izquierdo (Digital Left)

# Configuración de los pines GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Configuración de ENTRADA (Input) con resistencia PULL-UP
# ¿Qué significa PUD_UP?
# Por defecto, si el sensor no dice nada, el pin leerá un '1' (3.3V).
# Cuando el sensor detecta un obstáculo, conecta el pin a tierra (GND) y leemos un '0'.
GPIO.setup(DR, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(DL, GPIO.IN, GPIO.PUD_UP)

try:
    print("Iniciando evasión de obstáculos. Pulsa Ctrl+C para salir.")
    while True:
        # 1. LEER EL ENTORNO
        # Leemos el estado actual de los sensores (0 = Obstáculo, 1 = Libre)
        DR_status = GPIO.input(DR)
        DL_status = GPIO.input(DL)

        # 2. TOMAR DECISIÓN
        # Si el izquierdo (DL) O el derecho (DR) ven algo (valor 0)...
        if((DL_status == 0) or (DR_status == 0)):
            
            # ... ¡MANIOBRA DE EVASIÓN!
            Ab.left()        # Giramos a la izquierda
            time.sleep(0.002) # Durante un tiempo minúsculo
            Ab.stop()        # Y paramos
            
            # Nota: Al hacer esto dentro de un bucle while muy rápido,
            # el efecto visual es que el robot gira hasta que deja de ver el obstáculo.
            
        else:
            # 3. CAMINO LIBRE
            # Si no hay nada delante, seguimos avanzando
            Ab.forward()

except KeyboardInterrupt:
    # Si el usuario para el programa, limpiamos la configuración de pines
    GPIO.cleanup()
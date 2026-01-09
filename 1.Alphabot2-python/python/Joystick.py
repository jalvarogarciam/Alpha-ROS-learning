#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time
from AlphaBot2 import AlphaBot2  # Importamos el driver de motores

Ab = AlphaBot2()

# --- CONFIGURACIÓN DE PINES ---
# Mapeado del Joystick en la placa AlphaBot2
CTR = 7   # Botón Central (Pulsar hacia adentro)
A = 8     # Arriba
B = 9     # Derecha
C = 10    # Izquierda
D = 11    # Abajo
BUZ = 4   # Zumbador (Buzzer)

# Funciones auxiliares para el sonido
def beep_on():
    GPIO.output(BUZ, GPIO.HIGH) # Encender sonido

def beep_off():
    GPIO.output(BUZ, GPIO.LOW)  # Apagar sonido

# Configuración inicial de GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Configuramos los botones como ENTRADA con resistencia PULL-UP.
# PULL-UP significa: "Si nadie toca el botón, lee un 1. Si lo tocas, lee un 0".
GPIO.setup(CTR, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(A, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(B, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(C, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(D, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(BUZ, GPIO.OUT)

print("Iniciando control por Joystick. Pulsa el botón central (GPIO 7) para parar.")

try:
    while True:
        # Preguntamos: ¿Está pulsado el botón Central? (Valor 0)
        if GPIO.input(CTR) == 0:
            beep_on()       # Pita
            Ab.stop()       # PARA el robot
            print("Centro - STOP")
            
            # Bucle de espera: Mientras el dedo siga pulsando el botón...
            while GPIO.input(CTR) == 0:
                time.sleep(0.01) # ... no hacemos nada (evita rebotes)
                
        # ¿Está pulsado Arriba?
        elif GPIO.input(A) == 0:
            beep_on()
            Ab.forward()    # Avanza
            print("Arriba - ADELANTE")
            while GPIO.input(A) == 0:
                time.sleep(0.01)
                
        # ¿Está pulsado Derecha?
        elif GPIO.input(B) == 0:
            beep_on()
            Ab.right()      # Gira Derecha
            print("Derecha")
            while GPIO.input(B) == 0:
                time.sleep(0.01)
                
        # ¿Está pulsado Izquierda?
        elif GPIO.input(C) == 0:
            beep_on()
            Ab.left()       # Gira Izquierda
            print("Izquierda")
            while GPIO.input(C) == 0:
                time.sleep(0.01)
                
        # ¿Está pulsado Abajo?
        elif GPIO.input(D) == 0:
            beep_on()
            Ab.backward()   # Retrocede
            print("Abajo - ATRÁS")
            while GPIO.input(D) == 0:
                time.sleep(0.01)
                
        # Si NO hay ningún botón pulsado...
        else:
            beep_off() # Apagamos el pitido
            # NOTA: No paramos el robot aquí. 
            # El robot seguirá haciendo la última acción hasta que pulsemos otra cosa.

except KeyboardInterrupt:
    GPIO.cleanup()
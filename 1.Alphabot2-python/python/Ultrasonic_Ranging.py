#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time

# --- CONFIGURACIÓN DE PINES ---
TRIG = 22  # Pin que envía el sonido
ECHO = 27  # Pin que recibe el eco

# Configuración básica de la Raspberry
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# TRIG es Salida (nosotros mandamos), ECHO es Entrada (nosotros escuchamos)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

def dist():
    """
    Función de medición física.
    Rango efectivo del sensor HC-SR04: 2cm a 400cm.
    Si el objeto está a menos de 2cm, la física falla y da errores.
    """
    # 1. ENVIAR PULSO (PING)
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.000015) # Pulso de 15 microsegundos
    GPIO.output(TRIG, GPIO.LOW)

    # 2. ESPERAR RESPUESTA
    # Esperamos a que el sensor ponga el pin ECHO en ALTO (Inicio de recepción)
    while not GPIO.input(ECHO):
        pass
    t1 = time.time() # Guardamos la hora de inicio

    # Esperamos a que el sensor ponga el pin ECHO en BAJO (Fin de recepción)
    while GPIO.input(ECHO):
        pass
    t2 = time.time() # Guardamos la hora de fin

    # 3. CÁLCULO
    # (Tiempo * Velocidad Sonido cm/s) / 2
    return (t2-t1)*34000/2

print("Iniciando prueba de sensor. Pulsa Ctrl+C para salir.")

try:
    while True:
        # Llamamos a la función y guardamos el dato
        distancia_actual = dist()
        
        # Imprimimos formateado: %.2f significa "número decimal con 2 cifras"
        print("Distancia detectada: %0.2f cm" % distancia_actual)
        
        # Medimos cada segundo. 
        # Es una herramienta de lectura humana, no necesitamos velocidad extrema.
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
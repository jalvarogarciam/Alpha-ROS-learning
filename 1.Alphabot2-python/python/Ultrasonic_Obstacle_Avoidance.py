#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time
from AlphaBot2 import AlphaBot2 # Importamos el conductor

# Definición de Pines del Sensor HC-SR04
TRIG = 22  # Disparador (Trigger - Altavoz)
ECHO = 27  # Receptor (Echo - Micrófono)

Ab = AlphaBot2()

# Configuración GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Configuración de pines:
# TRIG es SALIDA (nosotros enviamos la señal para que dispare)
# ECHO es ENTRADA (esperamos recibir el voltaje de vuelta)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

def Distance():
    """
    Función que mide la distancia física en centímetros
    usando el principio de vuelo del sonido.
    """
    # 1. Enviamos un pulso de sonido muy breve (15 microsegundos)
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TRIG, GPIO.LOW)
    
    # 2. Esperamos a que el sensor empiece a recibir (ECHO se pone en HIGH)
    while not GPIO.input(ECHO):
        pass # Bucle de espera activo
    
    # Guardamos el tiempo exacto de inicio del rebote
    t1 = time.time()
    
    # 3. Esperamos a que termine el rebote (ECHO vuelve a LOW)
    while GPIO.input(ECHO):
        pass # Bucle de espera activo
    
    # Guardamos el tiempo exacto de fin
    t2 = time.time()
    
    # 4. Cálculo Matemático:
    # Tiempo de vuelo = t2 - t1
    # Velocidad del sonido = 340 m/s = 34000 cm/s
    # Distancia = (Tiempo * Velocidad) / 2
    # (Dividimos por 2 porque el sonido ha ido Y vuelto)
    return (t2-t1)*34000/2
    
print("Iniciando Evasión por Ultrasonidos")

try:
    while True:
        # Llamamos a nuestra función física
        Dist = Distance()
        
        # Mostramos el dato en pantalla con 2 decimales
        print("Distancia = %0.2f cm" % Dist)
        
        # --- LÓGICA DE CONTROL ---
        
        # Si el objeto está a 20 cm o menos... ¡PELIGRO!
        if Dist <= 20:
            # Maniobra de esquiva (Girar a la derecha)
            # Nota: Si el sensor mira a una pared lisa en ángulo, 
            # a veces el sonido rebota y no vuelve, leyendo distancias falsas.
            Ab.right()
        else:
            # Camino despejado -> Avanzar
            Ab.forward()
            
        # Esperamos un poco antes de la siguiente medición
        # para no saturar el procesador ni el sensor.
        time.sleep(0.02)

except KeyboardInterrupt:
    GPIO.cleanup()
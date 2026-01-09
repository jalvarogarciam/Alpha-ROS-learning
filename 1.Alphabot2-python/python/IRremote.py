#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time
from AlphaBot2 import AlphaBot2

Ab = AlphaBot2()

# --- CONFIGURACIÓN ---
IR = 17      # Pin donde está conectado el receptor IR
PWM = 50     # Velocidad inicial (0-100)
n = 0        # Contador de seguridad

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(IR, GPIO.IN) # Configuramos el pin como ENTRADA para leer datos

def getkey():
    """
    Función encargada de leer el Protocolo NEC.
    Mide el tiempo de los parpadeos de luz para saber qué tecla se pulsó.
    """
    if GPIO.input(IR) == 0: # Si detectamos que empieza una señal...
        count = 0
        # 1. Esperamos el pulso de inicio (9ms)
        while GPIO.input(IR) == 0 and count < 200:
            count += 1
            time.sleep(0.00006)
        if(count < 10): # Si fue ruido (muy corto), ignoramos
            return
        
        # 2. Esperamos el espacio (4.5ms)
        count = 0
        while GPIO.input(IR) == 1 and count < 80:
            count += 1
            time.sleep(0.00006)

        # 3. Leemos los 32 bits de datos (4 bytes)
        idx = 0
        cnt = 0
        data = [0,0,0,0] # Aquí guardaremos los 4 números recibidos
        for i in range(0,32):
            # Medimos cuánto dura el estado bajo
            count = 0
            while GPIO.input(IR) == 0 and count < 15:
                count += 1
                time.sleep(0.00006)
            
            # Medimos cuánto dura el estado alto (aquí está la información)
            count = 0
            while GPIO.input(IR) == 1 and count < 40:
                count += 1
                time.sleep(0.00006)
            
            # Si el pulso alto fue largo (>7 ciclos), es un '1'. Si no, es un '0'.
            if count > 7:
                data[idx] |= 1<<cnt
            
            # Organizamos los bits en bytes
            if cnt == 7:
                cnt = 0
                idx += 1
            else:
                cnt += 1

        # 4. Verificación de integridad (Checksum)
        # El mando envía el comando y su inverso para asegurar que no hay errores
        if data[0]+data[1] == 0xFF and data[2]+data[3] == 0xFF:
            return data[2] # Devolvemos solo el byte del comando (la tecla)
        else:
            return "repeat" # Código de tecla mantenida pulsada

print('Iniciando Receptor IR. Apunta con el mando...')
Ab.stop()

try:
    while True:
        # Esperamos a recibir una tecla
        key = getkey()
        
        if(key != None):
            n = 0 # Reseteamos contador de seguridad
            
            # --- MAPEO DE TECLAS (Códigos Hexadecimales) ---
            # Si usas otro mando, imprime 'key' para saber sus códigos
            
            if key == 0x18: # Botón ARRIBA (2)
                Ab.forward()
                print("Adelante")
                
            if key == 0x08: # Botón IZQUIERDA (4)
                Ab.left()
                print("Izquierda")
                
            if key == 0x1c: # Botón OK / 5
                Ab.stop()
                print("Stop")
                
            if key == 0x5a: # Botón DERECHA (6)
                Ab.right()
                print("Derecha")
                
            if key == 0x52: # Botón ABAJO (8)
                Ab.backward()       
                print("Atras")
                
            if key == 0x15: # Botón + (Subir Velocidad)
                if(PWM + 10 < 101):
                    PWM = PWM + 10
                    Ab.setPWMA(PWM)
                    Ab.setPWMB(PWM)
                    print("Velocidad:", PWM)
                    
            if key == 0x07: # Botón - (Bajar Velocidad)
                if(PWM - 10 > -1):
                    PWM = PWM - 10
                    Ab.setPWMA(PWM)
                    Ab.setPWMB(PWM)
                    print("Velocidad:", PWM)
        else:
            # --- SISTEMA DE SEGURIDAD (TIMEOUT) ---
            # Si dejamos de recibir señal durante un rato, paramos el robot
            # para que no se quede andando "loco" infinitamente.
            n += 1
            if n > 20000:
                n = 0
                Ab.stop()   

except KeyboardInterrupt:
    GPIO.cleanup()
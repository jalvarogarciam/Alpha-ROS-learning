#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO  # Librería para controlar los pines físicos de la Raspberry Pi
import time              # Librería para gestionar pausas y tiempos

class AlphaBot2(object):
    """
    Clase principal para el control de motores.
    Actúa como el 'driver' o 'controlador' de bajo nivel.
    """
    
    def __init__(self, ain1=12, ain2=13, ena=6, bin1=20, bin2=21, enb=26):
        # Configuración de los PINES GPIO (Mapeado físico)
        # AIN1, AIN2: Dirección del Motor Izquierdo (A)
        # BIN1, BIN2: Dirección del Motor Derecho (B)
        # ENA, ENB: Pines de Velocidad (PWM) para A y B
        self.AIN1 = ain1
        self.AIN2 = ain2
        self.BIN1 = bin1
        self.BIN2 = bin2
        self.ENA = ena
        self.ENB = enb
        
        # Velocidad inicial (Duty Cycle): 50% de potencia
        self.PA  = 50
        self.PB  = 50

        # Configuración inicial de la Raspberry Pi
        GPIO.setmode(GPIO.BCM)       # Usamos la numeración estándar Broadcom
        GPIO.setwarnings(False)      # Desactivamos alertas de pines ocupados
        
        # Le decimos a la Raspberry que estos pines enviarán corriente (SALIDA)
        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        
        # Inicializamos el PWM (Modulación por Ancho de Pulsos)
        # 500 Hz es la frecuencia eléctrica de actualización
        self.PWMA = GPIO.PWM(self.ENA, 500)
        self.PWMB = GPIO.PWM(self.ENB, 500)
        
        # Arrancamos los motores a la velocidad definida (PA y PB)
        self.PWMA.start(self.PA)
        self.PWMB.start(self.PB)
        self.stop()  # Pero los frenamos inmediatamente para que no salga corriendo

    def forward(self):
        """ Mueve el robot hacia ADELANTE """
        # Aplicamos la velocidad configurada
        self.PWMA.ChangeDutyCycle(self.PA)
        self.PWMB.ChangeDutyCycle(self.PB)
        
        # Configuración lógica para avanzar:
        # Motor A (Izquierda): Pin 1 LOW, Pin 2 HIGH
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH)
        # Motor B (Derecha): Pin 1 LOW, Pin 2 HIGH
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH)

    def stop(self):
        """ Detiene el robot en seco """
        # Ponemos la velocidad a 0 (Corte de energía)
        self.PWMA.ChangeDutyCycle(0)
        self.PWMB.ChangeDutyCycle(0)
        
        # Apagamos todos los pines de dirección por seguridad
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.LOW)

    def backward(self):
        """ Mueve el robot hacia ATRÁS """
        self.PWMA.ChangeDutyCycle(self.PA)
        self.PWMB.ChangeDutyCycle(self.PB)
        
        # Invertimos la lógica de 'forward':
        # Lo que antes era LOW ahora es HIGH y viceversa
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.HIGH)
        GPIO.output(self.BIN2, GPIO.LOW)

    def left(self):
        """ 
        Giro sobre su propio eje hacia la IZQUIERDA.
        El motor izquierdo retrocede y el derecho avanza.
        """
        # Fijamos velocidad de giro a 30 (Lento para mayor precisión)
        self.PWMA.ChangeDutyCycle(30)
        self.PWMB.ChangeDutyCycle(30)
        
        GPIO.output(self.AIN1, GPIO.HIGH) # Izquierda Atrás
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)  # Derecha Adelante
        GPIO.output(self.BIN2, GPIO.HIGH)

    def right(self):
        """ 
        Giro sobre su propio eje hacia la DERECHA.
        El motor izquierdo avanza y el derecho retrocede.
        """
        self.PWMA.ChangeDutyCycle(30)
        self.PWMB.ChangeDutyCycle(30)
        
        GPIO.output(self.AIN1, GPIO.LOW)  # Izquierda Adelante
        GPIO.output(self.AIN2, GPIO.HIGH)
        GPIO.output(self.BIN1, GPIO.HIGH) # Derecha Atrás
        GPIO.output(self.BIN2, GPIO.LOW)
        
    def setPWMA(self, value):
        """ Cambia la velocidad del Motor Izquierdo (0-100) """
        self.PA = value
        self.PWMA.ChangeDutyCycle(self.PA)

    def setPWMB(self, value):
        """ Cambia la velocidad del Motor Derecho (0-100) """
        self.PB = value
        self.PWMB.ChangeDutyCycle(self.PB)    
        
    def setMotor(self, left, right):
        """ 
        Control dual avanzado.
        Permite pasar valores negativos para ir hacia atrás.
        Ejemplo: setMotor(50, -50) haría un giro.
        """
        # Lógica para el motor DERECHO (Right)
        if((right >= 0) and (right <= 100)):
            # Velocidad positiva -> Avanzar
            GPIO.output(self.AIN1, GPIO.HIGH)
            GPIO.output(self.AIN2, GPIO.LOW)
            self.PWMA.ChangeDutyCycle(right)
        elif((right < 0) and (right >= -100)):
            # Velocidad negativa -> Retroceder
            GPIO.output(self.AIN1, GPIO.LOW)
            GPIO.output(self.AIN2, GPIO.HIGH)
            self.PWMA.ChangeDutyCycle(0 - right) # Convertimos a positivo para el PWM
            
        # Lógica para el motor IZQUIERDO (Left)
        if((left >= 0) and (left <= 100)):
            GPIO.output(self.BIN1, GPIO.HIGH)
            GPIO.output(self.BIN2, GPIO.LOW)
            self.PWMB.ChangeDutyCycle(left)
        elif((left < 0) and (left >= -100)):
            GPIO.output(self.BIN1, GPIO.LOW)
            GPIO.output(self.BIN2, GPIO.HIGH)
            self.PWMB.ChangeDutyCycle(0 - left)

# --- ZONA DE PRUEBAS ---
# Este bloque solo se ejecuta si lanzas el archivo directamente.
# Si lo importas desde otro programa, esto se ignora.
if __name__ == '__main__':
    Ab = AlphaBot2()
    print("Probando motores... Avanzando.")
    Ab.forward()
    try:
        # Mantiene el robot andando hasta que lo pares
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # Al pulsar Ctrl+C, limpiamos los pines para no dejarlos con corriente
        GPIO.cleanup()
        print("Motores detenidos.")
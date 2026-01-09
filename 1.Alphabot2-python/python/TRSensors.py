#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time

# Configuración de Pines GPIO para hablar con el chip TLC1543
CS = 5         # Chip Select (Habilita la comunicación)
Clock = 25     # Reloj (Marca el ritmo de lectura)
Address = 24   # Dirección (Le dice al chip qué sensor queremos leer)
DataOut = 23   # Datos (Por aquí nos llegan los números)
Button = 7     # Botón para calibrar

class TRSensor(object):
    def __init__(self, numSensors = 5):
        self.numSensors = numSensors
        # Arrays para guardar los valores máximos y mínimos (Calibración)
        self.calibratedMin = [0] * self.numSensors
        self.calibratedMax = [1023] * self.numSensors
        self.last_value = 0
        
        # Configuración de hardware
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(Clock, GPIO.OUT)
        GPIO.setup(Address, GPIO.OUT)
        GPIO.setup(CS, GPIO.OUT)
        GPIO.setup(DataOut, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(Button, GPIO.IN, GPIO.PUD_UP)
        
    def AnalogRead(self):
        """
        Lee los sensores a nivel físico.
        Implementa el protocolo de comunicación serie "bit a bit" con el chip.
        Devuelve una lista con 5 valores (0 = Blanco Puro, 1023 = Negro Puro).
        """
        value = [0]*(self.numSensors+1)
        # Bucle para leer cada canal (sensor)
        for j in range(0, self.numSensors+1):
            GPIO.output(CS, GPIO.LOW) # Iniciamos transmisión
            
            # Enviamos al chip qué sensor queremos leer (Address)
            for i in range(0,8):
                if i<4:
                    if(((j) >> (3 - i)) & 0x01):
                        GPIO.output(Address, GPIO.HIGH)
                    else:
                        GPIO.output(Address, GPIO.LOW)
                else:
                    GPIO.output(Address, GPIO.LOW)        
                
                # Leemos los datos que nos devuelve el chip
                value[j] <<= 1
                if(GPIO.input(DataOut)):
                    value[j] |= 0x01
                # "Tick" de reloj
                GPIO.output(Clock, GPIO.HIGH)
                GPIO.output(Clock, GPIO.LOW)
            
            # (El resto del bucle sigue leyendo los bits restantes...)
            for i in range(0,4):
                value[j] <<= 1
                if(GPIO.input(DataOut)):
                    value[j] |= 0x01
                GPIO.output(Clock, GPIO.HIGH)
                GPIO.output(Clock, GPIO.LOW)
            time.sleep(0.0001)
            GPIO.output(CS, GPIO.HIGH) # Fin transmisión
            
        for i in range(0,6):
            value[i] >>= 2    
        
        # Devolvemos la lista de lecturas limpias
        return value[1:]
        
    def calibrate(self):
        """
        Fase de Aprendizaje.
        Lee los sensores muchas veces para saber qué valores
        consideramos 'Blanco' y 'Negro' en este suelo concreto.
        """
        max_sensor_values = [0]*self.numSensors
        min_sensor_values = [0]*self.numSensors
        
        # Realizamos 10 lecturas rápidas
        for j in range(0,10):
            sensor_values = self.AnalogRead()
            for i in range(0,self.numSensors):
                # Buscamos los picos máximos y mínimos
                if((j == 0) or max_sensor_values[i] < sensor_values[i]):
                    max_sensor_values[i] = sensor_values[i]
                if((j == 0) or min_sensor_values[i] > sensor_values[i]):
                    min_sensor_values[i] = sensor_values[i]

        # Guardamos los resultados para usarlos luego
        for i in range(0,self.numSensors):
            if(min_sensor_values[i] > self.calibratedMin[i]):
                self.calibratedMin[i] = min_sensor_values[i]
            if(max_sensor_values[i] < self.calibratedMax[i]):
                self.calibratedMax[i] = max_sensor_values[i]

    def readCalibrated(self):
        """
        Lee los sensores y ajusta el valor de 0 a 1000
        basándose en la calibración anterior.
        0 = Totalmente Blanco
        1000 = Totalmente Negro
        """
        value = 0
        sensor_values = self.AnalogRead()

        for i in range (0,self.numSensors):
            denominator = self.calibratedMax[i] - self.calibratedMin[i]
            if(denominator != 0):
                value = (sensor_values[i] - self.calibratedMin[i])* 1000 / denominator
            if(value < 0): value = 0
            elif(value > 1000): value = 1000
            sensor_values[i] = value
        
        return sensor_values
            
    def readLine(self, white_line = 0):
        """
        LA FUNCIÓN MATEMÁTICA PRINCIPAL.
        Calcula la "Posición Ponderada" de la línea.
        
        No devuelve "izquierda o derecha", sino un número preciso:
        0    -> Línea bajo sensor 0 (Extrema Izquierda)
        1000 -> Línea bajo sensor 1
        2000 -> Línea bajo sensor 2 (Centro perfecto)
        3000 -> Línea bajo sensor 3
        4000 -> Línea bajo sensor 4 (Extrema Derecha)
        
        Ejemplo: Si devuelve 2500, sabemos que la línea está
        exactamente entre el sensor central y el de su derecha.
        """
        sensor_values = self.readCalibrated()
        avg = 0
        sum = 0
        on_line = 0
        
        for i in range(0,self.numSensors):
            value = sensor_values[i]
            if(white_line): value = 1000-value
            
            # Detectamos si estamos viendo la línea
            if(value > 200): on_line = 1
                
            # Fórmula de la Media Ponderada:
            # (Valor * Posición) / Suma_Valores
            if(value > 50): # Filtro de ruido
                avg += value * (i * 1000)
                sum += value

        if(on_line != 1):
            # Si hemos perdido la línea, recordamos dónde la vimos por última vez
            # para saber hacia dónde girar desesperadamente.
            if(self.last_value < (self.numSensors - 1)*1000/2):
                self.last_value = 0 # Estaba a la izquierda
            else:
                self.last_value = (self.numSensors - 1)*1000 # Estaba a la derecha
        else:
            self.last_value = avg/sum # Posición calculada
        
        return self.last_value, sensor_values

# Prueba unitaria si ejecutamos el archivo solo
if __name__ == '__main__':
    TR = TRSensor()
    print("Iniciando prueba de sensores...")
    while True:
        try:
            print(TR.AnalogRead()) # Imprime valores brutos
            time.sleep(0.2)
        except KeyboardInterrupt:
            break
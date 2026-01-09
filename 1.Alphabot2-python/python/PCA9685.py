#!/usr/bin/python
# -*- coding:utf-8 -*-
import time
import math
import smbus # Librería estándar para comunicaciones I2C (System Management Bus)

# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo Driver
# ============================================================================

class PCA9685:

  # --- REGISTROS INTERNOS DEL CHIP ---
  # Son las "direcciones de memoria" dentro del chip donde escribimos la configuración.
  # No es necesario que el alumno las memorice, solo que sepa que existen.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00 # Registro de configuración de modo
  __PRESCALE           = 0xFE # Registro para configurar la velocidad del reloj (Frecuencia)
  
  # Registros para encender/apagar el LED0 (Canal 0)
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  
  # Registros para controlar TODOS los canales a la vez
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD

  def __init__(self, address=0x40, debug=False):
    """ Inicializa la conexión I2C con el chip """
    # 1 = bus I2C número 1 de la Raspberry Pi
    self.bus = smbus.SMBus(1)
    # 0x40 es la dirección de fábrica del PCA9685
    self.address = address
    self.debug = debug
    if (self.debug):
      print("Reiniciando PCA9685")
    # Configuramos el Modo 1 inicial (reinicio)
    self.write(self.__MODE1, 0x00)
	
  def write(self, reg, value):
    """ Función auxiliar para escribir datos en el chip """
    self.bus.write_byte_data(self.address, reg, value)
    if (self.debug):
      print("I2C: Escribiendo 0x%02X en registro 0x%02X" % (value, reg))
	  
  def read(self, reg):
    """ Función auxiliar para leer datos del chip """
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Leído 0x%02X del registro 0x%02X" % (result, reg))
    return result
	
  def setPWMFreq(self, freq):
    """
    Configura la frecuencia de los pulsos.
    Para servos, ESTO DEBE SER 50Hz SIEMPRE.
    """
    prescaleval = 25000000.0    # El reloj interno del chip va a 25MHz
    prescaleval /= 4096.0       # El contador es de 12 bits (4096 pasos)
    prescaleval /= float(freq)  # Dividimos por la frecuencia deseada (50Hz)
    prescaleval -= 1.0
    
    if (self.debug):
      print("Ajustando frecuencia PWM a %d Hz" % freq)
      print("Pre-escala estimada: %d" % prescaleval)
      
    prescale = math.floor(prescaleval + 0.5) # Redondeamos al entero más cercano

    # Secuencia técnica para cambiar el reloj (requiere dormir el chip)
    oldmode = self.read(self.__MODE1);
    newmode = (oldmode & 0x7F) | 0x10        # Mode Sleep (Dormir)
    self.write(self.__MODE1, newmode)        # Mandamos a dormir
    self.write(self.__PRESCALE, int(math.floor(prescale))) # Cambiamos la velocidad
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005) # Esperamos 5ms a que el oscilador se estabilice
    self.write(self.__MODE1, oldmode | 0x80) # Despertamos y reiniciamos

  def setPWM(self, channel, on, off):
    """ 
    Configura un canal individual.
    channel: 0-15
    on: momento en el ciclo (0-4096) para encender
    off: momento en el ciclo (0-4096) para apagar
    """
    self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H+4*channel, on >> 8)
    self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H+4*channel, off >> 8)
	  
  def setServoPulse(self, channel, pulse):
    """
    Función amigable para humanos.
    Convierte 'tiempo de pulso' (microsegundos) a 'valores del chip'.
    pulse: duración en microsegundos (ej: 1500us = centro)
    """
    # Matemática:
    # Frecuencia = 50Hz -> Periodo = 20ms = 20000us
    # Resolución = 4096 pasos
    # Pasos = (Pulso_us * 4096) / 20000
    pulse = int(pulse*4096/20000)
    self.setPWM(channel, 0, pulse)

# --- ZONA DE PRUEBAS ---
# Si ejecutas este archivo, hará un barrido con el servo 0
if __name__=='__main__':
 
  # 1. Inicializamos conexión (Debug activado para ver qué pasa)
  pwm = PCA9685(0x40, debug=True)
  
  # 2. Ponemos la frecuencia a 50Hz (Estándar para servos analógicos)
  pwm.setPWMFreq(50)
  
  print("Iniciando prueba de barrido en Canal 0...")
  while True:
    # Mover de un lado al otro (500us a 2500us)
    for i in range(500,2500,10):  
      pwm.setServoPulse(0,i)   
      time.sleep(0.02)     
    
    # Mover de vuelta
    for i in range(2500,500,-10):
      pwm.setServoPulse(0,i) 
      time.sleep(0.02)
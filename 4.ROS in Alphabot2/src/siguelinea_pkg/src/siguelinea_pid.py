#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import RPi.GPIO as GPIO
from AlphaBot2 import AlphaBot2
from TRSensors import TRSensor
from std_msgs.msg import Bool
import time

# --- FUNCIONES DE LIMPIEZA ---
def parada_emergencia():
    rospy.loginfo("Deteniendo...")
    try:
        Ab.stop()
        GPIO.cleanup()
    except: pass

rospy.init_node('siguelinea_pid_node', anonymous=True)
rospy.on_shutdown(parada_emergencia)

# --- CONFIGURACIÓN DE PINES ---
Button = 7
DR = 16  # Sensor Obstáculo Derecha
DL = 19  # Sensor Obstáculo Izquierda

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Button, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(DR, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(DL, GPIO.IN, GPIO.PUD_UP)

# Publicador del chivato
pub_obstaculo = rospy.Publisher('/aviso_obstaculo', Bool, queue_size=1)

# Objetos del robot
maximum = 35
integral = 0
last_proportional = 0

TR = TRSensor()
Ab = AlphaBot2()
Ab.stop()

# Variable para controlar el estado anterior
estaba_bloqueado = False

rospy.loginfo("--- ALPHABOT LISTO ---")
rospy.loginfo("Calibrando...")
time.sleep(2)

# Calibración
for i in range(0,100):
    if rospy.is_shutdown(): break
    if(i<25 or i>= 75):
        Ab.right(); Ab.setPWMA(30); Ab.setPWMB(30)
    else:
        Ab.left(); Ab.setPWMA(30); Ab.setPWMB(30)
    TR.calibrate()
Ab.stop()

rospy.loginfo("Pulsa el boton para empezar...")

#Espera un poco para acabar con la inercia
time.sleep(0.5)

rospy.loginfo("--- EN MARCHA ---")
Ab.forward()

while not rospy.is_shutdown():
    try:
        # LEER OBSTÁCULOS (0 = Detecta, 1 = Libre)
        dr_status = GPIO.input(DR)
        dl_status = GPIO.input(DL)

        if dr_status == 0 or dl_status == 0:
            # --- OBSTÁCULO DETECTADO ---
            if not estaba_bloqueado:
                rospy.logwarn("¡Obstaculo! Parando.")
                Ab.stop()
                pub_obstaculo.publish(True)
                estaba_bloqueado = True
            
            # Reset del PID para cuando arranque
            integral = 0
            last_proportional = 0
            time.sleep(0.05) # Espera breve
            
        else:
            # --- CAMINO LIBRE ---
            
            # 1. LOGICA DE REARRANQUE (El Empujón)
            if estaba_bloqueado:
                rospy.loginfo("Obstaculo quitado. Reanudando marcha...")
                pub_obstaculo.publish(False)
                
                # ¡AQUI ESTÁ EL TRUCO!
                # Le damos un empujón fuerte (45) durante un instante para vencer la fricción
                Ab.forward()
                Ab.setPWMA(35) 
                Ab.setPWMB(35)
                
                estaba_bloqueado = False # Ya no estamos bloqueados
            
            # Publicamos false continuamente por seguridad
            pub_obstaculo.publish(False)

            # 2. RUTINA PID NORMAL
            position,Sensors = TR.readLine()
            
            proportional = position - 2000
            derivative = proportional - last_proportional
            integral += proportional
            last_proportional = proportional

            power_difference = proportional/30 + integral/10000 + derivative*2

            if (power_difference > maximum): power_difference = maximum
            if (power_difference < - maximum): power_difference = - maximum
            
            if (power_difference < 0):
                Ab.setPWMA(maximum + power_difference)
                Ab.setPWMB(maximum)
            else:
                Ab.setPWMA(maximum)
                Ab.setPWMB(maximum - power_difference)

    except Exception as e:
        rospy.logerr(e)
        break

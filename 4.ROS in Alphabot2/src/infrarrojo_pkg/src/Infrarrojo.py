#!/usr/bin/env python

#Bibliotecas
import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
from std_msgs.msg import String


DR = 16 #PIN (BCM) del DR del sensor infrarrojo
DL = 19 #PIN (BCM) del DL del sensor infrarrojo
gpio.setmode(gpio.BCM) #configura la numeracion de pines gpio en modo BCM
gpio.setwarnings(False) #desactiva los mesajes de advertancia que se muestran cuando se reconfiguran los pines gpio ya configurados
gpio.setup(DR,gpio.IN,gpio.PUD_UP) #DR = entrada con resistancia de pull up
gpio.setup(DL,gpio.IN,gpio.PUD_UP) #DL = entrada con resistancia de pull up


# Funcion para quitar el programa -> ctrl + c
def signal_handler(signal, frame):
    print('Presionaste Ctrl+C !')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class Infrarrojo():
    #Funcion que empeza el nodo y dice que este nodo publica, a una frecuencia de 15Hz, en el topic /infrarrojo_obstaculo
    def __init__(self):
        rospy.init_node('infrarrojo', anonymous=True)
        self.infra_publisher = rospy.Publisher('/infrarrojo_obstaculo',String, queue_size=1) #creer le publisher infra qui publie du String
        self.r = rospy.Rate(15)
    #Funcion que publica el valor sobre el topic     
    def infra_mesaje(self,mesaje):
        mesaje_infra = String() #publicar un tipo string
        mesaje_infra.data = mesaje #escribe la valor de mesaje_infra en el mesaje
        self.infra_publisher.publish(mesaje_infra) #publica en el topic

sensor = Infrarrojo()


print ''                   
print '          -----Nodo infrarrojo empeza-----'
print ''
print '   Presione Ctrl+C para detener el nodo !'
print ''


#hace un bucle infinito hasta que se presione ctrl+c
try:
    while True:
        DR_status = gpio.input(DR) #lectura del PIN DR
        DL_status = gpio.input(DL) #lectura del PIN DL
        if((DL_status == 0) or (DR_status == 0)): #si hay un obstaculo
            print ('Obstaculo')
            obstaculo = "si"
            sensor.infra_mesaje(obstaculo) #dar el mesaje a la funcion que despues lo enviara en el Topic
            sensor.r.sleep() #hacer un descanso para que el bucle funciona a 15Hz
        else: #si no hay obstaculo
            print ('No obstaculo')
            obstaculo = "no"
            sensor.infra_mesaje(obstaculo) #dar el mesaje a la funcion que despues lo enviara en el Topic
            sensor.r.sleep() #hacer un descanso para que el bucle funciona a 15Hz
             

except (KeyboardInterrupt, SystemExit):
    gpio.cleanup() #retablece el estado de los pines gpio
    sys.exit(0)
except: #para asegurarse de retablecer los pines incluso con un apago en modo de error
    gpio.cleanup() #retablece el estado de los pines gpio

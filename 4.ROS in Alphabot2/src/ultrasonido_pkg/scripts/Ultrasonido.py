#!/usr/bin/env python

#Bibliotecas
import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
from std_msgs.msg import Float32


trig = 22 #PIN (BCM) del trigger del sensor ultrasonido
echo = 27 #PIN (BCM) del echo del sensor ultrasonido
gpio.setmode(gpio.BCM) #configura la numeracion de pines gpio en modo BCM
gpio.setwarnings(False) #desactiva los mesajes de advertancia que se muestran cuando se reconfiguran los pines gpio ya configurados
gpio.setup(trig, gpio.OUT) #configura trig como una salida
gpio.setup(echo, gpio.IN) #configura echo como una entrada


# Funcion para quitar el programa -> ctrl + c 
def signal_handler(signal, frame):
    print('Presionaste Ctrl+C !')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class Ultrasonido():
    #Funcion que empeza el nodo y dice que este nodo publica, a una frecuencia de 15Hz, en el topic /ultrasonido_distancia
    def __init__(self):
        rospy.init_node('ultrasonido', anonymous=True)
        self.distance_publisher = rospy.Publisher('/ultrasonido_distancia',Float32, queue_size=1)
        self.r = rospy.Rate(15)
    #Funcion que publica el valor sobre el topic 
    def dist_sensor(self,dist):
        data_dist = Float32() #publicar un tipo float
        data_dist.data = dist #escribe la valor de dist en el mesaje
        self.distance_publisher.publish(data_dist) #publica en el topic
        
sensor = Ultrasonido()


print ''                   
print '          -----Nodo ultrasonido empeza-----'
print ''
print '   El ultrasonido esta calculando la distancia'
print '   Presione Ctrl+C para detener el nodo !'
print ''


#hace un bucle infinito hasta que se presione ctrl+c
try :
    while True :
        #las siguientes lineas sirven para medir la distancia con el ultrasonido
        gpio.output(trig, False) #poner trig en el estado FALSE
        time.sleep(0.1) #esperar 0.1s
        gpio.output(trig, True)
        time.sleep(0.00001)
        gpio.output(trig, False)
        while gpio.input(echo) == 0 :
            pulse_start = time.time()
        while gpio.input(echo) == 1 :
            pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17000
        if pulse_duration >=0.01746:
            #print('time out')
            continue
        elif distance > 300 or distance==0:
            #print('out of range')
            continue
        distance = round(distance, 3)
        print 'Distancia = ', int(distance), ' cm'
        sensor.dist_sensor(distance) #dar el valor a la funcion que despues lo enviara en el Topic
        sensor.r.sleep() #hacer un descanso para que el bucle funciona a 15Hz
        
except (KeyboardInterrupt, SystemExit):
    gpio.cleanup() #retablece el estado de los pines gpio
    sys.exit(0)
except: #para asegurarse de retablecer los pines incluso con un apago en modo de error
    gpio.cleanup() #retablece el estado de los pines gpio


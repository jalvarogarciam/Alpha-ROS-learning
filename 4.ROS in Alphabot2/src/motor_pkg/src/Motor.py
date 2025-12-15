#!/usr/bin/env python

#Bibliotecas
import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
from std_msgs.msg import Float32, Int32
from AlphaBot2 import AlphaBot2

Ab = AlphaBot2()
PWM = 50
remoto = 0
gpio.setmode(gpio.BCM) #configura la numeracion de pines gpio en modo BCM
gpio.setwarnings(False) #desactiva los mesajes de advertancia que se muestran cuando se reconfiguran los pines gpio ya configurados


# Funcion para quitar el programa -> ctrl + c
def signal_handler(signal, frame): 
    print 'Presionaste Ctrl+C !'
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


# Funcion que cambia la direccion del robot dependiendo del valor del ultrasonido
def motor_callback1(msg):
    distancia = msg.data
    global PWM
    global remoto
    if distancia <= 20: #si el obstaculo es a una distancia inferior a 20cm --> mostrar la distancia y detener el robot
        print 'Obstaculo --> STOP       Distancia = ', int(distancia)
        Ab.stop()
    else: #sino dirigir segun el valor del remoto
        if remoto == 0x18: #BOTON 2 del control remoto
            Ab.forward()
            print("Avanzar")
        if remoto == 0x08: #BOTON 4 del control remoto
            Ab.left()
            print("Izquierda")
        if remoto == 0x1c: #BOTON 5 del control remoto
            Ab.stop()
            print("Detener")
        if remoto == 0x5a: #BOTON 6 del control remoto
            Ab.right()
            print("Derecha")
        if remoto == 0x52: #BOTON 8 del control remoto
            Ab.backward()
            print("Retroceder")
        if remoto == 0x15: #BOTON + del control remoto
            if(PWM + 10 < 101):
                PWM = PWM + 10
                Ab.setPWMA(PWM)
                Ab.setPWMB(PWM)
                print'PWM = ', PWM
        if remoto == 0x07: #BOTON - del control remoto
            if(PWM - 10 > -1):
                PWM = PWM - 10
                Ab.setPWMA(PWM)
                Ab.setPWMB(PWM)
                print'PWM = ', PWM
    remoto = 0
    
    
# Funcion que recupera la valor del remoto    
def motor_callback2(msg):
        #print(remoto)
        global remoto
        remoto = msg.data #el mesaje en el topic '/remoto_valor' transmitido por el remoto


#Empezar el nodo y suscribir a /remoto_valor y /ultrasonido_distancia que llama motor_callback2 y motor_callback1 a cada nuevo mesaje
rospy.init_node('motor', anonymous=True)
rospy.Subscriber('/ultrasonido_distancia', Float32, motor_callback1)
rospy.Subscriber('/remoto_valor', Int32, motor_callback2)
print ''                   
print '          -----Nodo motor empeza-----'
print ''
print '   Presione Ctrl+C para detener el nodo !'
print ''
print 'Empeza el nodo "ultrasonido" y el nodo "remoto" para que funcione'
print 'Asegurate tambien de tener un obstaculo a mas de 20cm del ultrasonido'
print ''
#hace un bucle infinito hasta que se presione ctrl+c
try:
    while True:
        rospy.spin() #permete de mantener la transmision y recepcion ROS de datos 

except (KeyboardInterrupt, SystemExit):
    gpio.cleanup() #retablece el estado de los pines gpio
    sys.exit(0)
except: #para asegurarse de retablecer los pines incluso con un apago en modo de error
    gpio.cleanup() #retablece el estado de los pines gpio



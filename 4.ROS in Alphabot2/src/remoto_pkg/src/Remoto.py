#!/usr/bin/env python

#Bibliotecas
import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
from std_msgs.msg import Int32


IR = 17 #PIN (BCM) del sensor infrarrojo
PWM = 50 # Valor initial del PWM
n=0
gpio.setmode(gpio.BCM) #configura la numeracion de pines gpio en modo BCM
gpio.setwarnings(False) #desactiva los mesajes de advertancia que se muestran cuando se reconfiguran los pines gpio ya configurados
gpio.setup(IR,gpio.IN) #configura IR como una entrada


# Funcion para quitar el programa -> ctrl + c 
def signal_handler(signal, frame):
    print('Presionaste Ctrl+C !')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


#Funcion que publica el valor sobre el topic
def valor_remoto(valor):
    msg = Int32() #publicar un tipo int
    msg.data = valor
    pub.publish(msg)


#Funcion para obtener el valor generado por el control remoto
def getkey():
    if gpio.input(IR) == 0:
        count = 0
        while gpio.input(IR) == 0 and count < 200:  #9ms
            count += 1
            time.sleep(0.00006)
        if(count < 10):
            return;
        count = 0
        while gpio.input(IR) == 1 and count < 80:  #4.5ms
            count += 1
            time.sleep(0.00006)
        idx = 0
        cnt = 0
        data = [0,0,0,0]
        for i in range(0,32):
            count = 0
            while gpio.input(IR) == 0 and count < 15:    #0.56ms
                count += 1
                time.sleep(0.00006) 
            count = 0
            while gpio.input(IR) == 1 and count < 40:   #0: 0.56mx
                count += 1                               #1: 1.69ms
                time.sleep(0.00006)
            if count > 7:
                data[idx] |= 1<<cnt
            if cnt == 7:
                cnt = 0
                idx += 1
            else:
                cnt += 1
        if data[0]+data[1] == 0xFF and data[2]+data[3] == 0xFF:  #check
            return data[2]
        #else:
            #print("repeat")
            #return "repeat"


#Empezar el nodo y decir que este nodo publica en el topic /remoto_valor 
rospy.init_node('remoto', anonymous=True)
pub = rospy.Publisher('/remoto_valor',Int32, queue_size=10)
print ''                   
print '          -----Nodo remoto empeza-----'
print ''
print '   Presione Ctrl+C para detener el nodo !'
print ''


#hace un bucle infinito hasta que se presione ctrl+c
try:
    while True:
        key = getkey()            
        if(key != None): #condicion para verificar que no se transmita ningun "None" en el Topic  
            if isinstance(key, int): #condicion para verificar que no se transmita ningun tipo string en el Topic
                valor_remoto(key) #dar el valor a la funcion que despues lo enviara en el Topic
            n = 0
            if key == 0x18: #BOTON 2 del control remoto
                print("Avanzar")
            if key == 0x08: #BOTON 4 del control remoto
                print("Izquierda")
            if key == 0x1c: #BOTON 5 del control remoto
                print("Detener")
            if key == 0x5a: #BOTON 6 del control remoto
                print("Derecha")
            if key == 0x52: #BOTON 8 del control remoto
                print("Retroceder")
            if key == 0x15: #BOTON + del control remoto
                if(PWM + 10 < 101):
                    PWM = PWM + 10
                    print'PWM = ', PWM
            if key == 0x07: #BOTON - del control remoto
                if(PWM - 10 > -1):
                    PWM = PWM - 10
                    print'PWM = ', PWM
            if key == 71: #BOTON CH+ del control remoto
                print('Ampliar la imagen')
            if key == 69: #BOTON CH- del control remoto
                print('Reducir la imagen')
            if key == 68: #BOTON |<< (vol-) del control remoto
                print('Led ROJO')    
            if key == 64: #BOTON >>| (vol+) del control remoto
                print('Led AZUL')
            if key == 67: #BOTON >|| del control remoto
                print('Led VERDE')
        else:
            n += 1
            if n > 20000:
                n = 0
                
except (KeyboardInterrupt, SystemExit):
    gpio.cleanup() #retablece el estado de los pines gpio
    sys.exit(0)
except: #para asegurarse de retablecer los pines incluso con un apago en modo de error
    gpio.cleanup() #retablece el estado de los pines gpio









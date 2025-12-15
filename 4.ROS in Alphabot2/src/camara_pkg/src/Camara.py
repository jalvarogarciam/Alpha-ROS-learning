#!/usr/bin/env python

#Bibliotecas
from pyimagesearch import imutils
import numpy as np
import time
import sys
import signal
import rospy
import time
import cv2
from std_msgs.msg import Int32


camera = cv2.VideoCapture(0) #abre la camara "0"
size = 1 #es la valor con la cual dividiremos el tamano de la ventana video
size_before = 1 #es la valor size guardado
x = camera.get(cv2.CAP_PROP_FRAME_WIDTH) #recoge el tamano initial x de la ventana video
y = camera.get(cv2.CAP_PROP_FRAME_HEIGHT) #recoge el tamano initial y de la ventana video


# Funcion para quitar el programa -> ctrl + c 
def signal_handler(signal, frame):
    print 'Presionaste Ctrl+C !'
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


# Funcion que cambia el tamano de la ventana video dependiendo del boton presionado del remoto
def CambiarTamano(msg):
    remoto = msg.data #el mesaje en el topic '/remoto_valor' transmitido por el remoto
    #print(remoto)
    global size
    global size_before
    global x
    global y
    if remoto == 69: #Reducir la imagen --> BOTON CH- del control remoto
        if size < 1:
            size = size_before + 0.1
        elif size > 9:
            size -= 1
        else:
            size += 1
        x = camera.get(cv2.CAP_PROP_FRAME_WIDTH)/size #el nuevo tamano x
        y = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)/size #el nuevo tamano y
    if remoto == 71: #Ampliar la imagen --> BOTON CH+ del control remoto
        if size <= 1 and size > 0.5:
            size -= 0.1
        elif size <= 0.6:
            size += 0.1    
        else:
            size -= 1
        x = camera.get(cv2.CAP_PROP_FRAME_WIDTH)/size #el nuevo tamano x
        y = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)/size #el nuevo tamano y
    size_before = size 
    return x, y


#Empezar el nodo y suscribir a /remoto_valor que llama CambiarTamano() a cada nuevo mesaje
rospy.init_node('camara', anonymous=True)
rospy.Subscriber('/remoto_valor', Int32, CambiarTamano)     
print ''                   
print '          -----Nodo camara empeza-----'
print ''
print '   Presione Ctrl+C para detener el nodo !'
print ''


#hace un bucle infinito hasta que se presione ctrl+c
try:
    while True:
    
        (grabbed, frame) = camera.read() #captura una imagen de la camara 

        if not grabbed: #comprobar que se ha realizado la captura 
            break
        
        Rframe = cv2.resize(frame, (int(x), int(y)))  #utilizar las nuevas dimensiones x, y
        cv2.imshow("Flux de la camera", Rframe) #mostrar la imagen con las nuevas dimensiones
    
        if cv2.waitKey(1) & 0xFF == ord("q"): #para salir del bucle si se presiona la tecla q
            break
    
except (KeyboardInterrupt, SystemExit):
    camera.release() # limpia la camara
    cv2.destroyAllWindows() #cierra la ventana video
    sys.exit(0)

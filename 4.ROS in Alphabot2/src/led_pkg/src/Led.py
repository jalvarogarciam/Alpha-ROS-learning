#!/usr/bin/env python3

#Bibliotecas
import time
import sys
import signal
import rospy
import subprocess
from std_msgs.msg import Int32


# Funcion para quitar el programa -> ctrl + c 
def signal_handler(signal, frame): 
    print('Presionaste Ctrl+C !')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


#Funcion para abrir un terminal y escribir un commando que abre un codigo con la color que queremos segun el remoto
def ColorLed(msg):
    remoto = msg.data #el mesaje en el topic '/remoto_valor' transmitido por el remoto
    #print(remoto)
    #print ('Un terminal se abre')
    if remoto == 68: #ROJO --> BOTON |<< (vol-) del control remoto
        print('Led ROJO')
        command = "echo 'Led ROJO' && cd /home/pi/ros_catkin_ws/src/led_pkg/src/Color_Led && sudo python3 Led_ROJO.py" #el commando a escribir en el terminal
        subprocess.Popen(['lxterminal','--command',f'bash -c "{command}"']) #abrir el terminal y poner el commando
        
        #Estas 2 lineas abajo hace lo mismo que las 2 arriba pero sin cerrar el terminal 
        #command = "bash -c 'echo \"Lancement des LED\"; cd /home/pi/ros_catkin_ws/src/led_pkg/src; sudo python3 ws2812.py ; exec bash'"
        #subprocess.Popen(['lxterminal', '--command', command])
        
        time.sleep(3) #pongo un tiempo de 3s para ser seguro que el programa Led_ROJO.py que dura 2s se cierra antes de recibir una nueva valor del remoto
    if remoto == 64: #AZUL --> BOTON >>| (vol+) del control remoto
        print('Led AZUL')
        command = "echo 'Led AZUL' && cd /home/pi/ros_catkin_ws/src/led_pkg/src/Color_Led && sudo python3 Led_AZUL.py"
        subprocess.Popen(['lxterminal','--command',f'bash -c "{command}"'])
        time.sleep(3)
    if remoto == 67: #VERDE --> BOTON >|| del control remoto
        print('Led VERDE')
        command = "echo 'Led VERDE' && cd /home/pi/ros_catkin_ws/src/led_pkg/src/Color_Led && sudo python3 Led_VERDE.py"
        subprocess.Popen(['lxterminal','--command',f'bash -c "{command}"'])
        time.sleep(3)


#Empezar el nodo y suscribir a /remoto_valor que llama ColorLed() a cada nuevo mesaje
rospy.init_node('led', anonymous=True)
rospy.Subscriber('/remoto_valor', Int32, ColorLed)
print ('')                   
print ('          -----Nodo led empeza-----')
print ('')
print ('   Presione Ctrl+C para detener el nodo !')
print ('')


#hace un bucle infinito hasta que se presione ctrl+c
try : 
    while True :
        rospy.spin() #permete de mantener la transmision y recepcion ROS de datos 
        
except (KeyboardInterrupt, SystemExit):
    sys.exit(0)


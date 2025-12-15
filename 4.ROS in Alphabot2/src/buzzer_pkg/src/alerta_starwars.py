#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Bool

# --- CONFIGURACIÓN ---
BUZZER_PIN = 4
T = 0.25 

# Variable Global para saber si hay peligro en tiempo real
hay_obstaculo = False

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)

def tone(pwm, frequency, duration):
    if frequency == 0:
        pwm.stop()
    else:
        pwm.ChangeFrequency(frequency)
        pwm.start(50)
    time.sleep(duration)
    pwm.stop()
    time.sleep(0.05)

def tocar_marcha_imperial(pwm):
    global hay_obstaculo
    
    # --- NOTAS (Hz) ---
    a3=220; la3s=233; b3=247; c4=261; c4s=277; d4=293; d4s=311; e4=329
    f4=349; f4s=370; g4=392; g4s=415; a4=440; a4s=466; b4=493
    c5=523; c5s=554; d5=587; d5s=622; e5=659; f5=698; f5s=740; g5=784
    
    Negra = T
    Corchea = T / 2
    
    melodia = [
        # Frase 1
        (g4, Negra), (g4, Negra), (g4, Negra),
        (d4s, T*0.75), (a4s, T*0.25), (g4, Negra),
        (d4s, T*0.75), (a4s, T*0.25), (g4, Negra * 2),
        
        # Frase 2
        (d5, Negra), (d5, Negra), (d5, Negra),
        (d5s, T*0.75), (a4s, T*0.25), (f4s, Negra),
        (d4s, T*0.75), (a4s, T*0.25), (g4, Negra * 2)
    ]

    for nota, duracion in melodia:
        if rospy.is_shutdown(): break
        
        # --- LA SOLUCIÓN ESTÁ AQUÍ ---
        # Antes de tocar cada nota, preguntamos: "¿Sigue el obstáculo?"
        if not hay_obstaculo:
            pwm.stop()
            time.sleep(0.1)
            return # <--- Si no hay obstáculo, SALIMOS DE LA FUNCIÓN YA
            
        tone(pwm, nota, duracion)

def callback_obstaculo(mensaje):
    global hay_obstaculo
    # Ahora el callback es tonto: solo actualiza la "verdad"
    hay_obstaculo = mensaje.data

def loop_principal():
    rospy.init_node('alerta_buzzer_node')
    setup()
    
    # Iniciamos el PWM una sola vez
    pwm = GPIO.PWM(BUZZER_PIN, 440)
    
    # Nos suscribimos
    rospy.Subscriber("/aviso_obstaculo", Bool, callback_obstaculo)
    
    rospy.loginfo("Esperando obstaculos...")
    
    # Bucle infinito manual (en vez de rospy.spin)
    while not rospy.is_shutdown():
        # Si la bandera está arriba, tocamos
        if hay_obstaculo:
            # Esta función ahora se cortará sola si hay_obstaculo cambia avFalse
            tocar_marcha_imperial(pwm)
        else:
            # Si no hay obstáculo, descansamos un poco para no quemar CPU
            pwm.stop()
            time.sleep(0.1)

    GPIO.cleanup()

if __name__ == '__main__':
    loop_principal()

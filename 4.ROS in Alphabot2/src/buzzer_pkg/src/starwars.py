#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time

# --- CONFIGURACIÓN ---
BUZZER_PIN = 4
# Velocidad de la marcha (0.5s por negra aprox)
T = 0.5 

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)

def tone(pwm, frequency, duration):
    if rospy.is_shutdown(): return
    
    if frequency == 0:
        pwm.stop()
        time.sleep(duration)
        return

    pwm.ChangeFrequency(frequency)
    pwm.start(50) 
    time.sleep(duration)
    # Pequeña pausa entre notas para articular (staccato)
    pwm.stop()
    time.sleep(0.05)

def starwars():
    rospy.init_node('starwars_node', anonymous=True)
    
    try:
        setup()
        pwm = GPIO.PWM(BUZZER_PIN, 440) 
        rospy.loginfo("Reproduciendo: Imperial March (Star Wars)")

        # --- NOTAS (Hz) ---
        a3 = 220
        la3s = 233
        b3 = 247
        c4 = 261
        c4s = 277
        d4 = 293
        d4s = 311 # Eb
        e4 = 329
        f4 = 349
        f4s = 370
        g4 = 392
        g4s = 415
        a4 = 440
        a4s = 466 # Bb
        b4 = 493
        c5 = 523
        c5s = 554
        d5 = 587
        d5s = 622
        e5 = 659
        f5 = 698
        f5s = 740
        g5 = 784
        
        # --- DURACIONES ---
        # La marcha imperial es muy rítmica
        Negra = T
        Corchea = T / 2
        Triplete = T / 3 # Para los tresillos rápidos si los hubiera

        melodia = [
            # Frase 1: Tan, tan, tan...
            (g4, Negra), (g4, Negra), (g4, Negra),
            (d4s, T*0.75), (a4s, T*0.25), # Ta-ti
            (g4, Negra), (d4s, T*0.75), (a4s, T*0.25),
            (g4, Negra * 2), # Nota larga

            # Frase 2 (Aguda): Tin, tin, tin...
            (d5, Negra), (d5, Negra), (d5, Negra),
            (d5s, T*0.75), (a4s, T*0.25),
            (f4s, Negra), (d4s, T*0.75), (a4s, T*0.25),
            (g4, Negra * 2),

            # Frase 3 (Compleja): Sol agudo...
            (g5, Negra), (g4, T*0.75), (g4, T*0.25), 
            (g5, Negra), (f5s, T*0.75), (f5, T*0.25),
            
            (e5, T*0.25), (d5s, T*0.25), (e5, Corchea), # Tresillo rápido
            (0, Corchea), (c4s, Corchea), # Pausa breve y sigue
            (f4s, Negra), (d5s, T*0.75), (d5, T*0.25),

            (c5s, T*0.25), (c5, T*0.25), (b4, Corchea),
            (0, Corchea), (d4s, Corchea), 
            (g4s, Negra), (d4s, T*0.75), (g4s, T*0.25), # Ta-ti

            (a4s, Negra), (g4, T*0.75), (a4s, T*0.25),
            (d5, Negra * 2) # Fin primera parte
        ]

        for nota, duracion in melodia:
            if rospy.is_shutdown(): break
            tone(pwm, nota, duracion)

        rospy.loginfo("Que la fuerza te acompañe.")

    except Exception as e:
        rospy.logerr(e)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    starwars()
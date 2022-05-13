

import time
import threading
import machine
import RPi.GPIO as GPIO

servo_right_pin = 8
servo_left_pin = 0
hw_laucher_pin = 1
hw_receive_pin = 7


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


hw_t = machine.io(hw_laucher_pin)
hw_r = machine.io(hw_receive_pin)

hw_t.setinout('OUT')
hw_r.setinout('IN')

hw_t.setioout('HIGH')
while True:
                           
    hw_r.getioin()
    print(hw_r.ioin)
    if hw_r.ioin == 1:
        time.sleep(0.2)
        if hw_r.ioin == 1:
            continue

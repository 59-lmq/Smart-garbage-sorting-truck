import time
import threading
import machine
import RPi.GPIO as GPIO

import serial
from pynput import keyboard
import string
import threading


laser_right_address = '/dev/ttyUSB1'
laser_front_address = '/dev/ttyUSB0'

laser_front = machine.laser_detect(laser_front_address)
laser_right = machine.laser_detect(laser_right_address)


s = machine.Mecanum_wheel()
s.uart_init()


def on_press(key):
    try:

        if key.char == 'w':
            s.car_contr(-40,0,0)
        if key.char == 'a':
            s.car_contr(-0,0,1000)
        if key.char == 's':
            s.car_contr(40,0,0)
        if key.char == 'd':
            s.car_contr(-0,0,-1000)
        if key.char == 'q':
            s.car_contr(-0,50,0)
        if key.char == 'e':
            s.car_contr(-0,-50,-0)

            
    except AttributeError:
        print('special key {0} pressed'.format(key))


def on_release(key):
    s.car_contr(0,0,0)
    if key == keyboard.Key.esc:
        return False

def anjian():
    with keyboard.Listener(
        on_press=on_press, on_release=on_release) as listener:
        listener.join()

def main():
    for i in range(3):
        laser_front.get_distance()
        laser_right.get_distance()
    o_f_d = laser_front.distance
    o_r_d = laser_right.distance
    distance_f_data = []
    distance_r_data = []
    while True:
        laser_front.get_distance()
        laser_right.get_distance()
        f_d = laser_front.distance
        r_d = laser_right.distance
        print('x, y', r_d, f_d)
        if f_d == 0 or r_d == 0:
            s.stop()
            continue
        else:
            distance_f_data.append(f_d)
            distance_r_data.append(r_d)
            
            if len(distance_f_data) == 8:
                f_ = abs(distance_f_data[7] - distance_f_data[0])
                r_ = abs(distance_r_data[7] - distance_r_data[0])
                if f_>60:
                    s.stop()
                    s.car_contr(0, 30, 0)
                    time.sleep(2.5)
                    s.stop()
#                     r_angle()
                    continue
                if r_>60:
                    s.stop()
                    s.car_contr(0, -30, 0)
                    time.sleep(2.5)
                    s.stop()
#                     r_angle()
                    continue
                distance_f_data = []
                distance_r_data = []
def car_():
    s.car_contr(-20, 0, 0)

thread1 = threading.Thread(target=main)
thread2 = threading.Thread(target=car_)
thread1.start()
thread2.start()
from control import gpio
import RPi.GPIO as GPIO
import time
import serial
from pynput import keyboard
import string
import threading

# 初始化麦轮小车
s = gpio.Mecanum_wheel()
s.uart_init()


# 按键控制函数
'''
car_contr(a, b, c)
a: 控制前进后退的参数， 负数表示前进
b: 控制左右平移的参数， 负数表示右平移
c: 控制旋转的角度参数，
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


# 松开按键的时候小车停止运动，所有参数值为零
def on_release(key):
    s.car_contr(0,0,0)
    if key == keyboard.Key.esc:
        return False


# 按键监听函数
def anjian():
    with keyboard.Listener(
        on_press=on_press, on_release=on_release) as listener:
        listener.join()


# 键盘监听
thread = threading.Thread(target=anjian)
thread.start()


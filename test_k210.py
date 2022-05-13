import machine
import time
import RPi.GPIO as GPIO

k_address = '/dev/ttyUSB2'

k = machine.trash_detect(k_address)
hw_laucher_pin = 7
hw_receive_pin = 1


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

hw_t = machine.io(hw_laucher_pin)
hw_r = machine.io(hw_receive_pin)

hw_t.setinout('OUT')
hw_r.setinout('IN')

hw_t.setioout('HIGH')

k_data = []
s = machine.Mecanum_wheel()
s.uart_init()
b_time = time.time()
car_speed = 250
while True:
    k.receive()
    k_time = time.time()
    k_ob = k.object
    if k.pos != None:
        print(car_speed)
        k_pos = k.pos[0]
        if k_pos > 110:
            s.car_contr(0, 0, -car_speed)
            time.sleep(0.1)
        elif k_pos < 70:
            s.car_contr(0, 0, car_speed)
        else:
            car_speed = 250
            s.car_contr(0, 0, 0)
            if k_ob != None:
                s.stop()
                if k_ob == '2' or k_ob == '4':
                    ob = k.object
                    continue
                k_data.append(k_ob)
            if len(k_data) == 3:
                if k_data[0] == k_data[1] and k_data[1] == k_data[2]:
                    ob = k_data[2]
                    continue
                k_data = []
    
        car_speed -=10
    if car_speed <=100:
        car_speed = 100
    
# while True:
#     s.car_contr(-20, 0, 0)
#     time.sleep(1.3)
#     hw_r.getioin()
#     if hw_r.ioin == 1:
#         _servo_ud(up=False)
#         time.sleep(0.3)
#         s.stop()
#         r_angle()
#         break

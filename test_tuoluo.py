import machine
import threading
import time

s = machine.Mecanum_wheel()
s.uart_init()

mpu60 = machine.mpu6050()
def mu():
    mpu60.get_angle()
    

def angle_init():
    car_speed = 500
    error_rate = 1
    while True:
        
        i = mpu60.initial_z_angle
        n = mpu60.z_angle
        print(i,n)
        if i != 0:
            if n > 0 :
                if i < 0:
                    turn_angle = n - i
                    print('顺时针旋转：',  turn_angle)
                    if turn_angle > error_rate:
#                         s.car_contr(0, 0, car_speed)
                        u = 0
                if i > 0:
                    if n>i:
                        
                        turn_angle = n - i
                        print('顺时针旋转：', turn_angle)
#                         s.car_contr(0, 0, car_speed)
                        u = 0
                    else:
                        turn_angle = n - i
                        print('逆时针旋转', -turn_angle)
#                         s.car_contr(0, 0, -car_speed) 
                        u = 1
            elif n < 0:
                if i > 0:
                    turn_angle = i - n
                    print('逆时针旋转', turn_angle)
                    u = 1
#                     s.car_contr(0, 0, -car_speed)
    
                    
                if i < 0:
                    if i > n:
                        turn_angle = i - n
                        print('逆时针旋转', turn_angle)
#                         s.car_contr(0, 0, -car_speed)
                        u = 1
                    else:
                        turn_angle = n-i
                        print('顺时针旋转', turn_angle)
                        u = 0
#                         s.car_contr(0, 0, car_speed)
            if turn_angle != 0 :
                if turn_angle > error_rate:
                    if u == 0:
#                         back_(i, turn_angle, rl=False)
                        s.car_contr(0, 0, car_speed)
                    elif u == 1:
#                         back_(i, turn_angle, rl=True)
                        s.car_contr(0, 0, -car_speed)
                        
                    car_speed -= 10
                    if car_speed <= 50:
                        car_speed = 50
                else:
                    car_speed = 500
                    s.car_contr(0, 0, 0)
                    break
    return 1


t1= threading.Thread(target=mu)
t2 = threading.Thread(target=angle_init)
t1.start()
t2.start()
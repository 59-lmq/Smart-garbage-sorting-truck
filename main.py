__author__ = "Mingqian Li"
'''
此项目名称为泥头车一路向北，具体比赛是第八届光电设计竞赛的东南赛区的赛题二，智能垃圾分拣车
'''

import time
import threading
import machine
import RPi.GPIO as GPIO
from pynput import keyboard


labels = {'0': ['A4纸纸团', 0], '1': ['矿泉水瓶',0], '2': ['一次性水杯', 3],
          '4': ['7号电池', 2], '3': ['柑橘皮', 1]}


'''
    矿泉水瓶     可回收垃圾     蓝色
    7号电池      有害垃圾      红色
    一次性纸杯    其他垃圾      黑灰色
    柑橘皮       厨余垃圾      绿色
    A4纸纸团     可回收垃圾    蓝色
'''

color_dictionary = {'0': '白色', '1': '蓝色', '2': '红色', '3': '黑灰色',
                    '4': '绿色', '5': '蓝色', '6': '黑色', '7': '黄色'}

initial_pose = 1

right_pos = [103, 105]
left_pos = [239, 105]

yellow_pos = [220, 110]
black_pos = [168, 50.5]

blue_pos = [236, 28]
green_pos = [190, 28]
red_pos = [146, 28]
gray_pos = [100, 28]

garbage_area = [blue_pos, green_pos, red_pos, gray_pos]

# [a, b] a 指代到右边墙壁的距离， b 指代到前方的距离
rubbish_pos = [[171, 503], [171, 425], [173, 341],
               [172, 260], [173, 175]]

order_1 = [0, 4, 3, 2, 1]


color_RGB = {'white': [255, 255, 255], 'blue': [0, 137, 225], 'red': [235, 61, 0],
             'gray': [125, 129, 130], 'green': [0, 144, 74], 'black': [35, 25, 21],
             'yellow': [255, 240, 0]}

'''

___________________________________________
|     |蓝 绿 红 黑灰       蓝 绿 红 黑灰|    |
|_____|______________________________|____|
|     |   1                  2       |    |
|     |   12       13        14      |    |
|     |   9        10        11      |    |
|_____|___6________7_________8_______|    |
|     |   3        4         5       |    |
|     |   0        1         2       |    |
|_____|______________________________|____|
|     |   1                  2       |    |
|_____|______________________________|____|

'''

# servo_pin 是舵机的接口， hw_laucher_pin 和 hw_receive_pin 分别是红外发射口和接收口
servo_right_pin = 8
servo_left_pin = 0
hw_laucher_pin = 1
hw_receive_pin = 7


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

s = machine.Mecanum_wheel()
s.uart_init()

servo_right = machine.duoji(servo_right_pin)
servo_left = machine.duoji(servo_left_pin)

# hw_t = machine.io(hw_laucher_pin)
# hw_r = machine.io(hw_receive_pin)
# 
# hw_t.setinout('OUT')
# hw_r.setinout('IN')
# 
# hw_t.setioout('HIGH')

# 1、插前面的激光
# 2、插右边的激光
# 3、接颜色传感器
# 4、接K210

K210_address = '/dev/ttyUSB2'
# color_address = '/dev/ttyUSB2'

laser_right_address = '/dev/ttyUSB1'
laser_front_address = '/dev/ttyUSB0'


K210 = machine.trash_detect(K210_address)
# co = machine.color_detect(color_address)

laser_front = machine.laser_detect(laser_front_address)
laser_right = machine.laser_detect(laser_right_address)


mu = machine.mpu6050()


def _servo_ud(up=True):
    right_angle = 112
    left_angle = 65
    if up == False:
        right_angle = 0
        left_angle = 180
    servo_right.pwm_start(right_angle)
    servo_left.pwm_start(left_angle)


def mpu():
    mu.get_angle()
    
    
def angle_init():
    car_speed = 300
    error_rate = 0.8
    while True:
        i = mu.initial_z_angle
        n = mu.z_angle
        if i != 0:
            if n > 0:
                if i < 0:
                    turn_angle = n - i
                    u = 0
                if i > 0:
                    if n > i:
                        
                        turn_angle = n - i
                        u = 0
                    else:
                        turn_angle = n - i
                        u = 1
            elif n < 0:
                if i > 0:
                    turn_angle = i - n
                    u = 1
                if i < 0:
                    if i > n:
                        turn_angle = i - n
                        u = 1
                    else:
                        turn_angle = n-i
                        u = 0
            if turn_angle != 0:
                if turn_angle > error_rate:
                    if u == 0:
                        # print('顺时针旋转：',  turn_angle, i, n)
                        s.car_contr(0, 0, car_speed)
                        time.sleep(0.1)
                    elif u == 1:
                        # print('逆时针旋转', turn_angle, i, n)
                        s.car_contr(0, 0, -car_speed)
                        time.sleep(0.1)
                    car_speed -= 4
                    if turn_angle < 40:
                        car_speed -= 10
                    if car_speed <= 70:
                        car_speed = 70
                else:
                    s.car_contr(0, 0, 0)
                    break
    return 1


def r_angle():
    time.sleep(0.2)
    s.car_contr(0, 0, 0)
    time.sleep(0.2)
    angle_init()
    time.sleep(0.2)
    
    
def fbcl(ideal_d, origin_d, now_d, car_speed=40, fbcl=True):
    global num
    # fbcl=True，表示 fb是真的，就是直走
    #     =False, meaning that 就是评议
    error_rate = 10
    if fbcl == True:
        if now_d > ideal_d + error_rate:
            s.car_contr(-car_speed, 0, 0)
            if now_d < ((ideal_d+origin_d) / 2) and num == 0 and (ideal_d-origin_d) > 200:
                r_angle()
                num = 1
        elif now_d < ideal_d - error_rate:
            s.car_contr(car_speed, 0, 0)
            if now_d > ((ideal_d+origin_d) / 2) and num == 0 and (ideal_d-origin_d) > 200:
                r_angle()
                num = 1
        else:
            r_angle()
            return 1
    elif fbcl == False:
        if now_d > ideal_d + error_rate:
            s.car_contr(0, -car_speed, 0)
            if now_d < ((ideal_d+origin_d) / 2) and num == 0 and (ideal_d-origin_d) > 200:
                r_angle()
                num = 1
        elif now_d < ideal_d - error_rate:
            s.car_contr(0, car_speed, 0)
            if now_d > ((ideal_d+origin_d) / 2) and num == 0 and (ideal_d-origin_d) > 200:
                
                r_angle()
                num = 1
        else:
            r_angle()
            num = 0
            return 1


def zouwei(ideal_pos, parament=True, front_speed=20, pingyi_speed=20):
    # parament = True meaning that first cl, then, front
    # parament = False meaning that first front, then cl
    front_distance = ideal_pos[1]
    right_distance = ideal_pos[0]
    c_flag = 0
    f_flag = 0
    i_angle = mu.initial_z_angle
    n_angle = mu.z_angle
    f_speed = front_speed
    for i in range(3):
        laser_front.get_distance()
        laser_right.get_distance()
    o_f_d = laser_front.distance
    o_r_d = laser_right.distance
    distance_f_data = []
    distance_r_data = []
    while True:
        n_angle = mu.z_angle
        dif_angle = abs(n_angle-i_angle)
        print('dif_angle', dif_angle)
        laser_front.get_distance()
        laser_right.get_distance()
        f_d = laser_front.distance
        r_d = laser_right.distance
        if f_d == 0 or r_d == 0:
            s.stop()
            continue
        if (n_angle - i_angle) > 12:
            print('you died')
            s.stop()
            s.car_contr(0, -20, 0)
            time.sleep(1.5)
            r_angle()
            continue
        elif (i_angle - n_angle) > 12:
            print('you died')
            s.stop()
            s.car_contr(0, 20, 0)
            time.sleep(1.5)
            r_angle()
        if parament == True:
            if c_flag == 0:
                c = fbcl(right_distance, o_r_d, r_d, pingyi_speed, fbcl=False)
            if c != 1:
                continue
            else:
                c_flag = 1
                if f_flag == 0:
                    f = fbcl(front_distance, o_f_d, f_d, f_speed)
                
                if f != 1:
                    continue
                else:
                    f_flag = 1
                    break
        elif parament == False:
            if f_flag == 0:
                if f_d < o_f_d + (front_distance - o_f_d)*1/4 and (front_distance - o_f_d) > 200:
                    f_speed = 50
                else:
                    f_speed = front_speed
                f = fbcl(front_distance, o_f_d, f_d, f_speed)
            if f!=1:
                continue
            else:
                front_speed=20
                f_flag = 1
                if c_flag == 0:
                    c = fbcl(right_distance, o_r_d, r_d, pingyi_speed, fbcl=False)
                if c != 1:
                    continue
                else:
                    c_flag = 1
                    break
    return 1


def main():
    s_time = time.time()
    order = order_1
    pa = True
    trash_num = 0
    trash_ = 0
    f_speed = 20
    p_speed = 20
    k_flag = 0
    i_angle = mu.initial_z_angle
    n_angle = mu.z_angle
    num = 0
    k_time_flag = 0
    _servo_ud(up=True)
    while True:
        e_time = time.time()
        n_time = e_time - s_time
        print('now_time:', n_time)
        n_angle = mu.z_angle
        if trash_num == 5:
            e_time = time.time()
            print('using time:', e_time - s_time)
            break
        if i_angle != 0:
            for i in order:
                ideal_pos = rubbish_pos[i]
                while True:
                    if trash_num == 5:
                        e_time = time.time()
                        print('using time:', e_time - s_time)
                        break
                    laser_front.get_distance()
                    laser_right.get_distance()
                    
                    f_dis = laser_front.distance
                    r_dis = laser_right.distance
                    if f_dis == 0 or r_dis == 0:
                        s.stop()
                        continue
                    
                    zo = zouwei(ideal_pos, pa, f_speed, p_speed)
                    
                    if zo == 1 and trash_ == 0:
                        # 找到第一个垃圾之后，开启识别，然后根据识别到的结果确定垃圾位置，
                        # 识别到物体之后，直走，直到框住物体，然后走向垃圾区域
                        # 此时，小车先平移，再直走
                        # 即pa = True
                        
                        k_data = []
                        if len(k_data) == 0 and k_flag == 0:
                            b_time = time.time()
                            car_speed  = 200
                            while True:
                                k_time = time.time()
                                
                                K210.receive()
                                k_ob = K210.object
                                k_pos_ = K210.pos
                                if k_pos_ != None:
                                    k_pos = K210.pos[0]
                                    if k_pos > 120:
                                        s.car_contr(0, 0, -car_speed)
                                        time.sleep(0.05)
                                    elif k_pos < 60:
                                        s.car_contr(0, 0, car_speed)
                                        time.sleep(0.05)
                                    else:
                                        car_speed = 200
                                        s.car_contr(0, 0, 0)
                                        if k_ob != None:
                                            s.stop()
                                            if k_ob == '3' or k_ob == '4':
                                                if num == 0:
                                                    num += 1
                                                    continue
                                                ob = k_ob
                                                num = 0
                                                break
                                            k_data.append(K210.object)
                                        if len(k_data) == 3:
                                            if k_data[0] == k_data[1] and k_data[1] == k_data[2]:
                                                ob = k_data[2]
                                                break
                                            k_data = []
                                else:
                                    if k_time_flag == 0:
                                        if k_time - b_time < 2:
                                            s.car_contr(0, -10, 0)
                                        elif 2 < k_time - b_time < 6:
                                            s.car_contr(0, 10, 0)
                                        elif k_time - b_time > 6:
                                            k_time_flag = 1
                                            b_time = k_time
                                    else:
                                        if k_time - b_time < 4:
                                            s.car_contr(0, -10, 0)
                                        elif 2 < k_time - b_time < 6:
                                            s.car_contr(0, 10, 0)
                                        elif k_time - b_time > 6:
                                            b_time = k_time
                                            k_time_flag  = 0
                                        
                                    time.sleep(0.01)
                                
                                    car_speed -=10
                                if car_speed <= 100:
                                    car_speed = 100
                                
                                
                        while True:
                            s.car_contr(-20, 0, 0)
                            if ob in ['0','1','2', '3', '4']:
                                time.sleep(1.8)
                                _servo_ud(up=False)
                                time.sleep(0.3)
                                s.stop()
                                r_angle()
                                break
#                           
                        print(ob, type(ob), labels[ob], garbage_area[labels[ob][1]])
                        
                        ideal_pos = yellow_pos
                        if ob == '4' or ob == '2':
                            ideal_pos = [right_pos[0]+20,right_pos[1]+20]
                        
                        ideal_la_pos = garbage_area[labels[ob][1]]
                        f_speed = 50
                        p_speed = 50
                        trash_ = 1

                        pa = True
                        continue
                    if zo == 1 and trash_ == 1:
                        ideal_pos = ideal_la_pos
                        pa = True
                        trash_ = 2
                        angle_init()
                        f_speed = 20
                        p_speed = 20
                        continue
                    if zo == 1 and trash_ == 2:
                        # 舵机上抬
                        _servo_ud(True)
                        time.sleep(0.3)
                        f_speed = 20
                        p_speed = 20
#                         r_angle()
                        print(mu.initial_z_angle, mu.z_angle)
                        # 这里是到达垃圾区域之后的，需要让舵机上台，将垃圾推进垃圾区域，同时垃圾计数
                        # 下一步，小车应该后退，再平移，即pa=False
                        # 然后跳出这个循环，路径规划至下一个垃圾位置
                        trash_num += 1
                        trash_ = 3
                        ideal_pos = [yellow_pos[0]+30, yellow_pos[1]-20]
                        if ob == '4' or ob == '2':
                            ideal_pos = [right_pos[0], right_pos[1]-20]
                        pa = False
                        ob = None
                        k_pos_ = None
                        continue
                    if zo == 1 and trash_ == 3:
                        trash_ = 0
                        pa = False
                        ob = None
                        
                        k_pos_ = None
                        break
                        

def on_press(key):
    try:
        if key.char == 's':
            main()
        
    except AttributeError:
        print('special key {0} pressed'.format(key))


def on_release(key):
    s.car_contr(0, 0, 0)
    if key == keyboard.Key.esc:
        return False


def anjian():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
        
        
def stop():
    s.car_go(20)
    time.sleep(0.5)
    s.stop()


if __name__ == "__main__":
    t1 = threading.Thread(target=mpu)
    t2 = threading.Thread(target=anjian)
    t1.start()
    t2.start()

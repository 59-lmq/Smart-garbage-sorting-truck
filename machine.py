__author__ = "Mingqian Li"

import RPi.GPIO as GPIO
import time
import serial
import smbus
import numpy as np
import re, math
from MPU6050 import MPU6050

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

io2gpio = {0: 16, 1: 12, 2: 25,
           3: 24, 4: 22, 5: 23,
           6: 27, 7: 17, 8: 4}
old_fb = 0
old_lr = 0
old_tn = 0


class mpu6050:
    def __init__(self):
        self.i2c_bus = 1
        self.device_address = 0x68
        self.x_accel_offset = -5489
        self.y_accel_offset = -1441
        self.z_accel_offset = 1305
        self.x_gyro_offset = -2
        self.y_gyro_offset = -72
        self.z_gyro_offset = -5
        self.enable_debug_output = True
        self.z_angle = 0
        self.initial_z_angle = 0
        
        self.mpu = MPU6050(self.i2c_bus, self.device_address, self.x_accel_offset, self.y_accel_offset,
                           self.z_accel_offset, self.x_gyro_offset, self.y_gyro_offset,
                           self.z_gyro_offset, self.enable_debug_output)

        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        mpu_int_status = self.mpu.get_int_status()
#         print(hex(mpu_int_status))

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
#         print(self.packet_size)
        FIFO_count = self.mpu.get_FIFO_count()
#         print(FIFO_count)
        
    def get_angle(self):
        count = 0
        data = []
        ini_angle = []
        k = 0
        f = 0
        while count >= 0:
            if len(data)==3:
                data = []
            if len(ini_angle)==3:
                ini_angle = []
            FIFO_count = self.mpu.get_FIFO_count()
            mpu_int_status = self.mpu.get_int_status()
        
            # If overflow is detected by status or fifo count we want to reset
            if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                self.mpu.reset_FIFO()
            # Check if fifo data is ready
            elif (mpu_int_status & 0x02):
                # Wait until packet_size number of bytes are ready for reading, default
                # is 42 bytes
                while FIFO_count < self.packet_size:
                    FIFO_count = self.mpu.get_FIFO_count()
                FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
                accel = self.mpu.DMP_get_acceleration_int16(FIFO_buffer)
                quat = self.mpu.DMP_get_quaternion_int16(FIFO_buffer)
                grav = self.mpu.DMP_get_gravity(quat)
                roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
                if count % 1 == 0:
                    
                    z_angle = roll_pitch_yaw.z * 2
                    z_angle = math.floor(z_angle*10**2)/(10**2)
                    data.append(z_angle)
#                     print('yaw: ' + str(roll_pitch_yaw.z * 2))
                if len(data) == 3:
                    aver = (data[0]+data[1]+data[2])/3
                    if abs(aver-data[2]) < 0.3:
                        self.z_angle = math.floor(aver*10**2)/(10**2)
                        ini_angle.append(self.z_angle)
                        if self.initial_z_angle != 0 and f == 0:
                            print('yaw: {0}, ini:{1}'.format(self.z_angle, self.initial_z_angle))
                            f = 1
                if k == 0 and len(ini_angle) == 3:
                    if abs(ini_angle[2]-ini_angle[1])<=0.01 and abs(ini_angle[1]-ini_angle[0])<=0.01:
                        self.initial_z_angle = ini_angle[2]
                        k = 1
                    ini_angle = []
                count += 1


# io应用
class io():
    def __init__(self, io_num):
        self.gpio = io2gpio[io_num]
        self.ioin = 404
        GPIO.setmode(GPIO.BCM)
    
    def setinout(self, inorout):
        # 设置GPIO的输入或者输出
        if inorout == 'IN':
            GPIO.setup(self.gpio, GPIO.IN)
        elif inorout == 'OUT':
            GPIO.setup(self.gpio, GPIO.OUT)
    
    def setioout(self, dianping):
        # GPIO输出高or低电平
        if dianping == 'HIGH':
            GPIO.output(self.gpio, GPIO.HIGH)
        elif dianping == 'LOW':
            GPIO.output(self.gpio, GPIO.LOW)
    
    def getioin(self):
        # 获取GPIO口的输入电平
        if GPIO.input(self.gpio) == 0:
            # 低电平,返回false
            time.sleep(0.01)
            if GPIO.input(self.gpio) == 0:
                # 防抖设计
                self.ioin = 0
                # 输入为低电平
        else:
            self.ioin = 1
    
    def cleanio(self):
        # 清理io口
        GPIO.cleanup(self.gpio)


class Mecanum_wheel():
    # 麦轮驱动
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyAMA0', 115200)
        self.dec = 'none'
        self.came = 0  # 识别到等待线之后才把cam置为1
        
        GPIO.setwarnings(False)  # 关闭警告说明
        GPIO.setup(0, GPIO.IN)  # 设置引脚1（BCM编号）为输入通道1GPIO.setup(0, GPIO.IN) #设置引脚1（BCM编号）为输入通道
        GPIO.setup(1, GPIO.IN)  # 设置引脚1（BCM编号）为输入通道
    
    def uart_init(self):
        if self.ser.isOpen == False:
            self.ser.open()  # 打开串口
    
    def uart_receive(self):
        try:
            # 打开串口
            if self.ser.is_open == False:
                self.ser.open()
            while True:
                count = self.ser.inWaiting()
                if count != 0:
                    # 读取内容并显示
                    recv = self.ser.read(count)
                    print(recv)
                
                # 清空接收缓冲区
                self.ser.flushInput()
                # 必要的软件延时
                time.sleep(0.1)
        except KeyboardInterrupt:
            if self.ser != None:
                self.ser.close()
    
    def stop(self):
        self.car_contr(0, 0, 0)
    
    # 小车控制函数
    def car_contr(self, contr_fb=0, contr_lr=0, contr_tn=0):
        global old_fb, old_lr, old_tn
        if (contr_fb != old_fb) or (contr_lr != old_lr) or (contr_tn != old_tn):
            old_fb = contr_fb
            old_lr = contr_lr
            old_tn = contr_tn
            # 当速度为负的，做数据处理，得到电机反转的速度
            # fb 控制前后移动，lr控制左右平移，tn控制左右转向
            # fb = -10表示前进，lr = -10 表示向左平移，tn = -500表示左转
            if contr_fb < 0:
                contr_fb = 65536 + contr_fb
            if contr_lr < 0:
                contr_lr = 65536 + contr_lr
            if contr_tn < 0:
                contr_tn = 65536 + contr_tn
            # 格式化要发送的数据帧
            contr_law = b"%c%c%c%c%c%c%c%c" % (
                0xaa, int(contr_fb / 256), int(contr_fb % 256), int(contr_lr / 256), int(contr_lr % 256),
                int(contr_tn / 256), int(contr_tn % 256), 0xee)
            # 发送数据帧
            self.ser.write(contr_law)


class duoji():
    # 舵机驱动
    def __init__(self, pin):
        d_pin = io2gpio[pin]
        # print(pin, io2gpio[pin])
        GPIO.setup(io2gpio[pin], GPIO.OUT)
        self.pwm = GPIO.PWM(d_pin, 50)
    
    def pwm_start(self, angle, initial_angle=20):
        self.pwm.start(initial_angle)
        duty = 2.5 + (int(angle) * 10) / 180
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        self.pwm.start(initial_angle)
    
    def destroy(self):
        self.pwm.stop()
        GPIO.cleanup()


class uart_use():
    # 串口
    def __init__(self, uart_address, baudrate):
        self.uart_address = uart_address
        self.baud = baudrate
        self.response = None
        self.ser = serial.Serial(self.uart_address, self.baud)

        if self.ser.isOpen == False:
            self.ser.open()

    def uart_send(self, string):
        self.ser.write(string.encode())

    def receive(self, initial_size=False):
        size = self.ser.inWaiting()
        if initial_size == True:
            count = 9
            if size > 8:
                self.response = self.ser.read(count)
#                 print(self.response)

        else:
            if size != 0:
                self.response = self.ser.read(size)
#                 print(self.response)

        # 清空缓冲区
        self.ser.flushInput()
        time.sleep(0.1)

    def close(self):
        self.ser.close()


class laser_detect():
    def __init__(self, address):
        self.laser = uart_use(address, baudrate=115200)
        self.distance = 0
        self.strength = 0
        self.temp = 0
    
    def get_distance(self):
        self.laser.receive(initial_size=True)
        recv = self.laser.response
#         print(recv)
        if recv != None:
            if recv[0] == 0x59 and recv[1] == 0x59:
                self.distance = np.int16(recv[2] + np.int16(recv[3] << 8))
                self.strength = recv[4] + recv[5] * 256
                self.temp = (np.int16(recv[6] + np.int16(recv[7] << 8))) / 8 - 256
                # 计算芯片温度
        

class color_detect():
    def __init__(self, address):
        self.detect = uart_use(address, baudrate=9600)
        self.data = None
        self.RGB = []
        self.txt = 'AT+COLOR\r\n'
    
    def receive(self):
        self.detect.uart_send(self.txt)
        self.detect.receive()
        self.data = self.detect.response
        self.deal_data()
    
    def deal_data(self):
        try:
            if self.data != None:
                k = str(self.data, encoding='utf-8')
                k_ = re.findall(r'\d+.?\d*', k)
                self.RGB = [int(k_[0]),
                            int(k_[1]),
                            int(k_[2][:-1])]
                
        except IndexError:
            self.RGB = []


class trash_detect():
    def __init__(self, address):
        self.detect = uart_use(address, baudrate=115200)
        self.txt = '0808'
        self.data = None
        self.object = None
        self.value = None
        self.pos = None
    
    def receive(self):
        self.detect.uart_send(self.txt)
        self.detect.receive()
        self.data = self.detect.response
        self.deal_data()
    
    def deal_data(self):
        number = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
        try:
            labels = {'0': ['A4纸纸团', 0], '1': ['矿泉水瓶',0], '2': ['一次性水杯', 3],
                      '4': ['7号电池', 2], '3': ['柑橘皮', 1]}
            if self.data != None:
                k = str(self.data, encoding='utf-8')
                if k != 'None':
                    k_ = re.findall(r'\d+.?\d*', k)
                    print(k_)
                    self.object = k_[0][0]
                    self.value = k_[1]
                    if k_[3][-1] in number:
                        self.pos = [int(k_[2][:-1]), int(k_[3])]
                    else:
                        self.pos = None
#                     print(self.pos, type(self.pos[0]))
#                     self.ob = labels[self.object][1]
#                     print(self.ob, type(self.ob))
#                     if self.object == '3':
#                         print('3333')
                else:
                    self.object = None
                    self.value = None
                    self.pos = None
        except IndexError or ValueError:
            self.object = None
            self.value = None
            self.pos = None


def test_color():
    color_address = '/dev/ttyUSB2'
    co = color_detect(color_address)
    while True:
        co.receive()
        print('RGB:{0}'.format(co.RGB))


def test_laser():
    laser_right_address = '/dev/ttyUSB1'
    laser_front_address = '/dev/ttyUSB0'
    laser_right = laser_detect(laser_right_address)
    laser_front = laser_detect(laser_front_address)
    while True:
        laser_right.get_distance()
        laser_front.get_distance()
        print('x,y:', laser_right.distance, laser_front.distance)


def test_K210():
    K210_address = '/dev/ttyUSB2'
    K2 = trash_detect(K210_address)
    while True:
        K2.receive()
        print(K2.data, K2.object)


def test_mpu6050():
    mpu6 = mpu6050()
    mpu6.get_angle()


def test_all():
    K210_address = '/dev/ttyUSB3'
    color_address = '/dev/ttyUSB2'
    laser_front_address = '/dev/ttyUSB1'
    laser_next_address = '/dev/ttyUSB0'

    K2 = trash_detect(K210_address)
    co = color_detect(color_address)
    laser_front = laser_detect(laser_front_address)
    laser_next = laser_detect(laser_next_address)
    while True:
        K2.receive()
        co.receive()
        laser_front.get_distance()
        laser_next.get_distance()
        print("x :{0},\t  y:{1},\t color:{2}\t, K210:{3}".format(laser_front.distance, laser_next.distance, co.data, K2.object))


def test_servo():
    right_angle = 0
    left_angle = 180
    right_up_angle = 112
    left_up_angle = 65
    servo_left = duoji(0)
    servo_right = duoji(8)
    servo_right.pwm_start(right_angle)
    servo_left.pwm_start(left_angle)
    time.sleep(1)
    servo_right.pwm_start(right_up_angle)
    servo_left.pwm_start(left_up_angle)


if __name__ == "__main__":
    test_color()
    test_laser()
    test_all()
    test_K210()
    test_mpu6050()
    test_servo()
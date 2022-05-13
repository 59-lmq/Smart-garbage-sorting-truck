import machine

lase = machine.laser_detect('/dev/ttyUSB1')
lase1 =  machine.laser_detect('/dev/ttyUSB0')
s = machine.Mecanum_wheel()
s.uart_init()
for i in range(3):
    lase.get_distance()
    lase1.get_distance()
f_i = lase1.distance
r_i = lase.distance

while True:
    lase.get_distance()
    lase1.get_distance()
    
    f = lase1.distance
    r = lase.distance
    print('x,y', r,f)
    s.car_contr(-20, 0, 0)
    if f_i - f > 28 :
        s.car_contr(0, 0, 0)
        break
    
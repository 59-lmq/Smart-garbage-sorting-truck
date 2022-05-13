from control import gpio
import time

m = gpio.Mecanum_wheel()
m.uart_init()
car_speed = 0

m.car_contr(-20, -car_speed, 0)
time.sleep(1.3)
m.car_contr(0, 0, 0)
# time.sleep(0.2)
# m.car_contr(0, car_speed, 0)
# time.sleep(2)
# m.car_contr(0, 0, 0)

# 7.5
# 5.8
# 4.5
# 3.0
# 1.3
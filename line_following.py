from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait

hub = PrimeHub()

left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
sensor = ColorSensor(Port.E)

target = 35

Kp = 8
Kd = 3
Ki = 0

integral = 0
last_error = 0

while True:
    reflection = sensor.reflection()
    error = reflection - target

    integral += error
    derivative = error - last_error
    correction = (Kp * error) + (Kd * derivative)
    last_error = error

    base_speed = 350

    
    left_motor.run(-(base_speed + correction))
    right_motor.run(base_speed - correction)

    wait(10)

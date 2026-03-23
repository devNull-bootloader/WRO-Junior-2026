from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

hub = PrimeHub()

left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
up_motor = Motor(Port.C)
grabber_motor = Motor(Port.D)
line_sensor = ColorSensor(Port.E)
front_sensor = ColorSensor(Port.F)

drive_base = DriveBase(left_motor, right_motor, 56, 165)

def line_follow(distance):
    target = 55
    Kp = 2
    Kd = 4
    Ki = 0

    integral = 0
    last_error = 0

    # Reset distance
    drive_base.reset()

    while drive_base.distance() < distance - 12:  # Overshoot compensation
        reflection = line_sensor.reflection()
        error = reflection - target

        integral += error
        derivative = error - last_error
        correction = (Kp * error) + (Kd * derivative)

        last_error = error

        base_speed = 200   # mm/s

        # drive(speed, turn_rate)
        drive_base.drive(base_speed, correction)

        wait(10)

    drive_base.brake()

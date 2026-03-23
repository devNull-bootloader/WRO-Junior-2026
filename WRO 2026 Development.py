from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait

hub = PrimeHub()

left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
line_sensor = ColorSensor(Port.E)

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

    while drive_base.distance() < distance:
        reflection = line_sensor.reflection()
        error = reflection - target

        integral += error
        derivative = error - last_error
        correction = (Kp * error) + (Kd * derivative)

        last_error = error

        base_speed = 200   # mm/s (DriveBase uses mm/s)

        # drive(speed, turn_rate)
        drive_base.drive(base_speed, correction)

        wait(10)

    drive_base.stop()

line_follow(4000)

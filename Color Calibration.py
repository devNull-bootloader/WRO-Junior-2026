from pybricks.hubs import PrimeHub
from pybricks.pupdevices import ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait

hub = PrimeHub()
sensor = ColorSensor(Port.F)

colors_to_measure = ["black", "blue", "green", "yellow", "red"]

print("WRO Color Calibration")
print("----------------------")

for color_name in colors_to_measure:
    print("")
    print("Place the robot in front of the", color_name, "block.")
    print("Press LEFT button to record HSV...")

    # Wait for button press
    while Button.LEFT not in hub.buttons.pressed():
        wait(10)

    wait(200)

    # Read HSV
    h, s, v = sensor.hsv()

    print("Measured", color_name, "HSV:", h, s, v)

    # Wait for release
    while Button.LEFT in hub.buttons.pressed():
        wait(10)

print("")
print("Calibration complete!")
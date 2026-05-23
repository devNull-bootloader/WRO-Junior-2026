from pybricks.hubs import PrimeHub
from pybricks.pupdevices import ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait

hub = PrimeHub()
sensor = ColorSensor(Port.E)

print("Place on WHITE and press left button")

# Wait for button press
while Button.LEFT not in hub.buttons.pressed():
    wait(10)

white = sensor.reflection()
print("White:", white)

wait(1000)

print("Place on BLACK and press left button")

# Wait for release + press again
while Button.LEFT in hub.buttons.pressed():
    wait(10)
while Button.LEFT not in hub.buttons.pressed():
    wait(10)

black = sensor.reflection()
print("Black:", black)

# Calculate threshold
threshold = (white + black) / 2

print("Threshold:", threshold)

# Optional: beep to confirm
hub.speaker.beep()
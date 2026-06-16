from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

stopwatch = StopWatch()

class Robot:
    def __init__(self):
        # Hub
        self.hub = PrimeHub()

        # Motors
        self.left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B)
        self.up_motor = Motor(Port.C)
        self.grabber_motor = Motor(Port.D)

        # Sensors
        self.line_sensor = ColorSensor(Port.E)
        self.front_sensor = ColorSensor(Port.F)

        # Drive base
        self.drive_base = DriveBase(self.left_motor, self.right_motor, 56, 165)

        # Line color detection constants (hue ranges)
        self.red_range = (340, 10) # wrap-around
        self.blue_range = (220, 225)
        self.green_range = (160, 175)

        # State flags
        self.grabbing = False
        self.carrying = False

        # Probe mission
        self.probe_order = []

    # Sensors
    def detect_color(self):
        # Hue-based detection using front sensor (kept as in your code).
        h, s, v = self.front_sensor.hsv()

        # Adjusted black threshold to be more practical
        if v < 10:
            return "black"
        if h < 15 or h > 345:
            return "red"
        if 40 < h < 70:
            return "yellow"
        if 120 < h < 180:
            return "green"
        if 190 < h < 260:
            return "blue"
        return "unknown"

    def detect_color_new(self):
        h, s, v = self.front_sensor.hsv()

        # HARD FILTER: ignore weak signals (too far / ground)
        if v < 10 or s < 50:
            if 150 < h < 310 and 3 < v < 30 and 3 < s < 30:
                return "black"
            else: 
                return "unknown"

        # Color detection (based on your calibrated ranges)
        if h > 330 or h < 10:
            return "red"
        elif 45 < h < 65:
            return "yellow"
        elif 140 < h < 170:
            return "green"
        elif 210 < h < 225:
            return "blue"
        return "unknown"

    def scan_probes(self):
        detected_colors = []
        last_color = None
        stable_count = 0
        required_stable = 3 # how many consistent readings needed

        self.drive_base.reset()
        self.hub.imu.reset_heading(0)

        distance = 685 # yeah... don't try tweaking this
        speed = 150

        while self.drive_base.distance() < distance:
            # Gyro correction
            error = self.hub.imu.heading()
            correction = error * 2
            self.drive_base.drive(speed, -correction)

            # Detect color
            color = self.detect_color_new()

            if color != "unknown":
                if color == last_color:
                    stable_count += 1
                else:
                    stable_count = 1

                # Only confirm after stable readings
                if stable_count == required_stable:
                    if color not in detected_colors:
                        detected_colors.append(color)
                    else:
                        stable_count = 0

            last_color = color
            wait(10)
            
        self.drive_base.brake()
        return detected_colors

    # Movement

    def gyro_turn(self, angle, speed=300, timeout_ms=None, tuning=2):
        try:
            self.hub.imu.reset_heading(0)
        except Exception:
            pass

        target = angle
        elapsed = 0
        step = 10

        while True:
            if timeout_ms is not None and elapsed >= timeout_ms:
                break

            try:
                current = self.hub.imu.heading()
            except Exception:
                current = 0

            error = target - current

            # Stop when close enough
            if abs(error) < tuning:
                break

            # Smooth proportional turning
            turn_speed = error * 2

            # Clamp speed
            if turn_speed > speed:
                turn_speed = speed
            elif turn_speed < -speed:
                turn_speed = -speed

            self.drive_base.drive(0, turn_speed)

            # Keep grabber active
            self.update_grabber()
            wait(step)
            elapsed += step

        self.drive_base.brake()

    def gyro_straight(self, distance, speed=300, timeout_ms=None):
        # Straight drive using IMU heading correction. Polls grabber during motion.
        self.drive_base.reset()
        try:
            self.hub.imu.reset_heading(0)
        except Exception:
            pass

        direction = 1 if distance >= 0 else -1
        target_abs = abs(distance)

        elapsed = 0
        step = 10

        while abs(self.drive_base.distance()) < target_abs:
            if timeout_ms is not None and elapsed >= timeout_ms:
                break

            try:
                error = self.hub.imu.heading()
            except Exception:
                error = 0

            correction = error * (1.5 if self.carrying else 2)
            self.drive_base.drive(speed * direction, -correction)

            # Poll grabber while moving
            self.update_grabber()
            wait(step)
            elapsed += step

        self.drive_base.brake()

    def gyro_turn_acc(self, angle, speed=300, timeout_ms=None):
        try:
            self.hub.imu.reset_heading(0)
        except Exception:
            pass

        target = angle
        elapsed = 0
        step = 10

        ACCEL_ZONE = 20  # degrees to ramp up
        DECEL_ZONE = 30  # degrees to ramp down
        MIN_SPEED  = 60

        while True:
            if timeout_ms is not None and elapsed >= timeout_ms:
                break

            try:
                current = self.hub.imu.heading()
            except Exception:
                current = 0

            error = target - current

            # Stop when close enough
            if abs(error) < 2:
                break

            traveled = abs(angle) - abs(error)
            direction = 1 if error > 0 else -1

            if traveled < ACCEL_ZONE:
                turn_speed = max(MIN_SPEED, int(speed * traveled / ACCEL_ZONE))
            elif abs(error) < DECEL_ZONE:
                turn_speed = max(MIN_SPEED, int(speed * abs(error) / DECEL_ZONE))
            else:
                turn_speed = speed

            turn_speed *= direction

            # Clamp speed
            if turn_speed > speed:
                turn_speed = speed
            elif turn_speed < -speed:
                turn_speed = -speed

            self.drive_base.drive(0, turn_speed)

            # Keep grabber active
            self.update_grabber()
            wait(step)
            elapsed += step

        self.drive_base.brake()

    def gyro_straight_acc(self, distance, speed=300, timeout_ms=None):
        # Straight drive using IMU heading correction. Polls grabber during motion.
        self.drive_base.reset()
        try:
            self.hub.imu.reset_heading(0)
        except Exception:
            pass

        direction = 1 if distance >= 0 else -1
        target_abs = abs(distance)

        ACCEL_ZONE = 200  # mm to ramp up
        DECEL_ZONE = 200  # mm to ramp down
        MIN_SPEED  = 80

        elapsed = 0
        step = 10

        while abs(self.drive_base.distance()) < target_abs:
            if timeout_ms is not None and elapsed >= timeout_ms:
                break

            try:
                error = self.hub.imu.heading()
            except Exception:
                error = 0

            traveled  = abs(self.drive_base.distance())
            remaining = target_abs - traveled

            if traveled < ACCEL_ZONE:
                drive_speed = max(MIN_SPEED, int(speed * traveled / ACCEL_ZONE))
            elif remaining < DECEL_ZONE:
                drive_speed = max(MIN_SPEED, int(speed * remaining / DECEL_ZONE))
            else:
                drive_speed = speed

            correction = error * (1.5 if self.carrying else 2)
            self.drive_base.drive(drive_speed * direction, -correction)

            # Poll grabber while moving
            self.update_grabber()
            wait(step)
            elapsed += step

        self.drive_base.brake()

    def line_follow(self, distance, speed=200, target_color=None, Kp=5, timeout_ms=None):
        # Line follow PID constants (kept exactly as you requested)
        target = 28.5
        Kp = Kp
        Kd = 4
        Ki = 0
        # PD line follow using your PID constants (unchanged).
        integral = 0
        last_error = 0

        self.drive_base.reset()
        target_abs = abs(distance)

        elapsed = 0
        step = 10

        while abs(self.drive_base.distance()) < target_abs - 12: # overshoot compensation
            if timeout_ms is not None and elapsed >= timeout_ms:
                break

            reflection = self.line_sensor.reflection()
            error = reflection - target

            integral += error
            derivative = error - last_error
            correction = (Kp * error) + (Kd * derivative)

            last_error = error

            # ADAPTIVE SPEED
            if self.carrying:
                base_speed = speed
                correction *= 0.8 # smoother turns
            else:
                base_speed = speed if target_color is None else speed

            self.drive_base.drive(base_speed, correction)

            # Poll grabber while moving
            self.update_grabber()

            # color detection on the line sensor if requested
            if target_color is not None:
                h, s, v = self.line_sensor.hsv()
                print(h, s, v)

                if target_color == "red":
                    if h > self.red_range[0] or h < self.red_range[1]:
                        break
                elif target_color == "blue":
                    if self.blue_range[0] < h < self.blue_range[1] and s > 20:
                        break
                elif target_color == "green":
                    if self.green_range[0] < h < self.green_range[1]:
                        break

            wait(step)
            elapsed += step

        self.drive_base.brake()

    def drive_until_color(self, target_color, speed=300, timeout_ms=None):
        self.drive_base.reset()
        try:
            self.hub.imu.reset_heading(0)
        except Exception:
            pass

        elapsed = 0
        step = 10

        ACCEL_ZONE = 100  # mm to ramp up
        MIN_SPEED  = 80

        while True:
            # Timeout safety
            if timeout_ms is not None and elapsed >= timeout_ms:
                break

            # Gyro correction
            try:
                error = self.hub.imu.heading()
            except Exception:
                error = 0

            traveled = abs(self.drive_base.distance())
            if traveled < ACCEL_ZONE:
                drive_speed = max(MIN_SPEED, int(speed * traveled / ACCEL_ZONE))
            else:
                drive_speed = speed

            correction = error * (1.5 if self.carrying else 2)
            self.drive_base.drive(drive_speed, -correction)

            # Poll grabber
            self.update_grabber()

            # ----- COLOR DETECTION (same logic as line_follow()) -----
            h, s, v = self.line_sensor.hsv()

            if target_color == "red":
                if (h > self.red_range[0] or h < self.red_range[1]) and s > 50 and v > 10:
                    break
            elif target_color == "blue":
                if self.blue_range[0] < h < self.blue_range[1] and s > 20:
                    break
            elif target_color == "green":
                if self.green_range[0] < h < self.green_range[1]:
                    break

            wait(step)
            elapsed += step

        self.drive_base.brake()

    # Arm / Grabber
    def move_arm(self, angle, speed=400, wait=True):
        # Blocking move to target angle
        self.up_motor.run_target(speed, angle, then=Stop.HOLD, wait=wait)

    def release(self, wait=True):
        # Blocking grab: move to target and set carrying True after motion completes.
        self.grabber_motor.run_target(400, -50) # blocking
        self.carrying = True

    def grab(self):
        # Blocking release: open grabber and clear carrying flag.
        self.grabber_motor.run_target(400, 15) # blocking
        self.carrying = False

    def spread(self, angle=64, speed=400):
        # Move grabber to a spread/open angle (blocking).
        self.grabber_motor.run_target(speed, angle)

    # High level actions
    def start_spread_towers(self, speed=150):
        # Non-blocking start: spin grabber slowly while approaching towers.
        self.grabber_motor.run(speed)
        self.grabbing = True
        # Do not set carrying here; set carrying when a stall or successful grab is detected

    def start_grab_towers(self):
        # Non-blocking start: spin grabber slowly while approaching towers.
        self.grabber_motor.run(-150)
        self.grabbing = True
        # Do not set carrying here; set carrying when a stall or successful grab is detected

    def release_towers(self):
        # Stop grabber and clear flags
        self.grabber_motor.stop()
        self.grabbing = False
        self.carrying = False

    def update_grabber(self, speed=10):
        # Polling method: if we were trying to grab and the motor stalls, assume we have grabbed.
        try:
            stalled = self.grabber_motor.stalled()
        except Exception:
            stalled = False

        if self.grabbing and stalled:
            # reduce power to hold
            self.grabber_motor.run(speed)
            self.grabbing = False
            self.carrying = True

    def update_grabber_grab(self):
        # Polling method: if we were trying to grab and the motor stalls, assume we have grabbed.
        try:
            stalled = self.grabber_motor.stalled()
        except Exception:
            stalled = False

        if self.grabbing and stalled:
            # reduce power to hold
            self.grabber_motor.run(-7.5)
            self.grabbing = False
            self.carrying = True
    # -------------------------------------------------------------------------
    # Delivery helpers
    # -------------------------------------------------------------------------

    def _prepare_delivery_standard(self):
        """
        Used by most inner branches: stop towers, clamp arm, small corrective
        turn, then lower arm. Call this before the branch-specific movement.
        """
        self.release_towers()
        wait(200)
        self.release()
        wait(200)
        self.gyro_turn(-5)
        wait(200)
        self.move_arm(40)
        wait(200)

    def _prepare_delivery_close(self):
        """
        Used by the 'nearest probe' branches where the robot stays close:
        stop towers, clamp arm, lower arm immediately.
        """
        self.release_towers()
        wait(200)
        self.release()
        wait(200)
        self.move_arm(40)
        wait(200)

    def _long_route(self, straight_distance, final_distance=115):
        """
        Shared 'swing wide' route used when the target probe is far away:
        pivot on left wheel, drive forward, U-turn, final approach.
        """
        self.left_motor.run_angle(200, -600)
        wait(200)
        self.gyro_straight(straight_distance)
        wait(200)
        self.gyro_turn(-270)
        wait(200)
        self.gyro_straight(final_distance)

    # -------------------------------------------------------------------------

    def first_probe_algorithm(self):
        # Guard: we need at least indices [2] and [3] to be safe.
        if len(self.probe_order) < 4:
            print("ERROR: only", len(self.probe_order),
                  "colors detected:", self.probe_order,
                  "❌ aborting probe algorithm.")
            return

        outer = self.probe_order[3]
        inner = self.probe_order[2]

        if outer == "red":
            self.gyro_turn(157.1, speed=50)
            wait(200)
            self.drive_until_color("red")
            wait(200)
            self.gyro_straight(-2)
            wait(200)
            self.gyro_turn(33)
            wait(100)
            self.gyro_turn(-4)
            wait(200)
            
            if inner == "green":
                self.release_towers()
                wait(200)
                self.release()
                wait(200)
                self.move_arm(40)
                wait(200)
                self.gyro_turn(-11)
                wait(200)
                self.gyro_turn(9)
                wait(200)
                self.gyro_straight(-300)
                wait(200)
                self.gyro_turn(91)
                wait(200)
                self.gyro_straight(45)
                wait(200)
                self.gyro_turn(89)
                wait(200)
                self.gyro_straight(320)
                wait(200)
                self.drive_until_color("green")
            elif inner == "black":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(-48)
                wait(200)
                self.gyro_straight(60)
                wait(200)
                self.gyro_turn(40)
                wait(200)
                self.gyro_straight(78)
                wait(200)
                self.gyro_straight(-155)
                wait(200)
                self.left_motor.run_angle(200, 660)
                wait(200)
                self.gyro_straight(560)
                wait(200)
                self.drive_until_color("green")
            elif inner == "blue":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(-48)
                wait(200)
                self.gyro_straight(60)
                wait(200)
                self.gyro_turn(40)
                wait(200)
                self.gyro_straight(125)
                wait(200)
                self.gyro_turn(18)
                wait(200)
                self.gyro_straight(45)
                wait(200)
                self.gyro_turn(-15)
                wait(200)
                self.gyro_straight(-250)
                wait(200)
                self.left_motor.run_angle(200, 660)
                wait(200)
                self.gyro_straight(560)
                wait(200)
                self.drive_until_color("green")
            elif inner == "yellow":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(-48)
                wait(200)
                self.gyro_straight(70)
                wait(200)
                self.gyro_turn(55.5)
                wait(200)
                self.gyro_straight(310)
                wait(200)
                self.gyro_turn(7)
                wait(200)
                self.gyro_turn(-10)
                wait(200)
                self.gyro_straight(-355)
                wait(200)
                self.left_motor.run_angle(200, 660)
                wait(200)
                self.gyro_straight(560)
                wait(200)
                self.drive_until_color("green")

        elif outer == "green":
            self.gyro_turn(165.5, speed=50)
            wait(200)
            self.drive_until_color("red")
            wait(200)
            self.gyro_straight(-5)
            wait(200)
            self.gyro_turn(24)
            wait(200)
            
            if inner == "black":
                self._prepare_delivery_close()
                self.gyro_turn(-5)
                wait(200)
                self.gyro_straight(27)
                wait(200)
                self.gyro_turn(3)
                wait(200)
                self.gyro_straight(-70)
                wait(200)
                self.move_arm(-48)
                wait(200)
                self.left_motor.run_angle(300, -525)
                wait(200)
                self.gyro_straight(110)
                wait(200)
                self.gyro_turn(-91)
                wait(200)
                self.gyro_straight(510)
                wait(200)
                self.drive_until_color("green")
            elif inner == "blue":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(-48)
                wait(200)
                self.gyro_straight(60)
                wait(200)
                self.gyro_turn(35)
                wait(200)
                self.gyro_straight(90)
                wait(200)
                self.gyro_straight(-140)
                wait(200)
                self.gyro_turn(120)
                wait(200)
                self.gyro_straight(550)
                wait(200)
                self.drive_until_color("green")
            elif inner == "yellow":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(-48)
                wait(200)
                self.gyro_straight(60)
                wait(200)
                self.gyro_turn(40)
                wait(200)
                self.gyro_straight(150)
                wait(200)
                self.gyro_turn(15)
                wait(200)
                self.gyro_straight(35)
                wait(200)
                self.gyro_turn(-10)
                wait(200)
                self.gyro_straight(-270)
                wait(200)
                self.gyro_turn(105)
                wait(200)
                self.gyro_straight(600)
                wait(200)
                self.drive_until_color("green")
            elif inner == "red":
                self._prepare_delivery_standard()
                self.left_motor.run_angle(300, -525)
                wait(200)
                self.gyro_straight(350)
                wait(200)
                self.gyro_turn(-273)
                wait(200)
                self.gyro_straight(100)
                wait(200)
                self.gyro_straight(-70)
                wait(200)
                self.left_motor.run_angle(300, -1100)
                wait(200)
                self.gyro_straight(200)
                wait(200)
                self.left_motor.run_angle(200, 50)
                wait(200)
                self.gyro_straight(280)
                wait(200)
                self.drive_until_color("green")

        elif outer == "black":
            self.gyro_turn(176, speed=50)
            wait(200)
            self.drive_until_color("red")
            wait(200)
            self.gyro_straight(-5)
            wait(200)
            self.gyro_turn(19)
            wait(200)
            
            if inner == "blue":
                self._prepare_delivery_close()
                self.gyro_turn(-10)
                wait(200)
                self.gyro_straight(27)
                wait(200)
                self.move_arm(-48)
                wait(200)
                self.gyro_straight(-150)
                wait(200)
                self.right_motor.run_angle(300, 510)
                wait(200)
                self.gyro_straight(10)
                wait(200)
                self.right_motor.run_angle(300, 535)
                wait(200)
                self.gyro_straight(520)
                wait(200)
                self.drive_until_color("green")
            elif inner == "yellow":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(-48)
                wait(200)
                self.gyro_straight(60)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.gyro_straight(100)
                wait(200)
                self.gyro_turn(5)
                wait(200)
                self.gyro_turn(-5)
                wait(200)
                self.gyro_straight(-435)
                wait(200)
                self.left_motor.run_angle(300, 670)
                wait(200)
                self.gyro_straight(410)
                wait(200)
                self.drive_until_color("green")
            elif inner == "green":
                self._prepare_delivery_standard()
                wait(200)
                self.gyro_straight(350)
                wait(200)
                self.gyro_turn(-273)
                wait(200)
                self.gyro_straight(110)
                wait(200)
                self.move_arm(-48)
                wait(200)
                self.gyro_straight(-300)
                wait(200)
                self.gyro_turn(91)
                wait(200)
                self.gyro_straight(53)
                wait(200)
                self.gyro_turn(89)
                wait(200)
                self.gyro_straight(320)
                wait(200)
                self.drive_until_color("green")
            elif inner == "red":
                self._prepare_delivery_standard()
                self._long_route(480)
                wait(200)
                self.gyro_straight(-70)
                wait(200)
                self.left_motor.run_angle(300, -1100)
                wait(200)
                self.gyro_straight(200)
                wait(200)
                self.left_motor.run_angle(200, 50)
                wait(200)
                self.gyro_straight(280)
                wait(200)
                self.drive_until_color("green")

        elif outer == "blue":
            self.gyro_turn(187.1, speed=50)
            wait(200)
            self.drive_until_color("red")
            wait(200)
            self.gyro_straight(-5)
            wait(200)
            self.gyro_turn(12)
            wait(200)
            
            if inner == "yellow":
                self._prepare_delivery_close()
                self.gyro_turn(-10)
                wait(200)
                self.gyro_straight(27)
            elif inner == "black":
                self._prepare_delivery_standard()
                self._long_route(350)
            elif inner == "green":
                self._prepare_delivery_standard()
                self._long_route(480)
            elif inner == "red":
                self._prepare_delivery_standard()
                self._long_route(610)

        elif outer == "yellow":
            self.gyro_turn(193, speed=50)
            wait(200)
            self.drive_until_color("red")
            wait(200)
            self.gyro_straight(-5)
            wait(200)
            self.gyro_turn(7)
            wait(200)
            
            if inner == "blue":
                self._prepare_delivery_standard()
                self._long_route(300, final_distance=125)
            elif inner == "black":
                self._prepare_delivery_standard()
                self._long_route(480)
            elif inner == "green":
                self._prepare_delivery_standard()
                self._long_route(610)
            elif inner == "red":
                self._prepare_delivery_standard()
                self._long_route(740)

    # Mission run
    def ideal_run(self):
        # self.hub.imu.reset_heading(0)
        # wait(200)
        # self.gyro_straight(350)
        # wait(200)
        # self.left_motor.run_angle(300, 528)
        # wait(200)
        # self.gyro_straight(269, speed=200)
        # wait(200)
        # self.gyro_turn(-90.5, speed=50)
        # wait(200)
        # self.probe_order = self.scan_probes()
        # print(self.probe_order)
        # self.gyro_turn(50, speed=100)
        # wait(200)
        # self.gyro_straight(-240)
        # wait(200)
        # self.gyro_turn(43, speed=100)
        # wait(200)
        # self.gyro_straight(170, speed=250)
        # self.drive_until_color("green", speed=150)
        # self.gyro_straight(10)
        # self.start_spread_towers()
        # wait(2000)
        # self.update_grabber()
        # self.first_probe_algorithm()
        pass

    def blocks_task(self):
        self.up_motor.reset_angle(0)
        self.up_motor.run_target(200, -192)
        self.gyro_straight(1020, speed=300)
        wait(200)
        self.gyro_turn(-15)
        wait(200)
        self.line_follow(1000, target_color="blue", Kp=4)
        wait(200)
        self.gyro_straight(-200, speed=500)
        wait(200)
        self.gyro_turn(20)
        wait(200)
        self.gyro_straight(370, speed=400)
        wait(200)
        self.right_motor.run_angle(600, 645)
        wait(200)
        self.gyro_straight(300, speed=300)
        wait(200)
        self.gyro_straight(-350, speed=500)
        wait(200)
        self.gyro_turn(-91)
        wait(200)
        self.gyro_straight(70, speed=500)
        wait(200)
        self.gyro_turn(93)
        wait(200)
        self.gyro_straight(350, speed=400)
        print(stopwatch.time())

    def yellow_towers_task(self):
        # self.line_follow(5000, target_color="blue", Kp=5, speed=150)
        self.gyro_straight_acc(500, speed=500)
        wait(100)
        self.gyro_straight(-135, speed=150)
        self.up_motor.reset_angle(0)
        self.up_motor.run_target(300, 192, wait=False)
        wait(100)
        self.gyro_turn(91.5, speed=100)
        wait(100)
        self.gyro_straight(405, speed=300)
        wait(100)
        self.start_spread_towers()
        wait(800)
        self.update_grabber()
        wait(100)
        self.gyro_straight(-445, speed=130)
        wait(100)
        self.gyro_turn(91.5, speed=90)
        wait(100)
        self.gyro_straight_acc(400, speed=375)
        wait(100)
        self.drive_until_color("blue", speed=375)
        wait(500)
        self.gyro_straight(80, speed=200)
        wait(100)
        self.gyro_straight(-50, speed=120)
        self.release_towers()
        self.release()
        wait(200)
        self.gyro_straight(-100, speed=300)
        wait(100)
        self.gyro_turn(-91, speed=300)
        wait(100)
        self.gyro_straight(60, speed=300)
        wait(100)
        self.gyro_turn(92, speed=300)
        wait(100)
        self.spread(speed=400)
        self.gyro_straight(140, speed=300)
        wait(100)
        self.start_grab_towers()
        wait(700)
        self.update_grabber_grab()
        wait(200)
        self.gyro_straight(-100, speed=300)
        wait(100)
        self.gyro_turn(-91, speed=200)
        wait(100)
        self.up_motor.reset_angle(0)
        self.up_motor.run_target(300, -192)
        wait(100)
        self.gyro_straight(140, speed=150)
        wait(100)
        self.up_motor.reset_angle(0)
        self.up_motor.run_target(300, 90)
        wait(100)
        self.spread(speed=400)
        self.gyro_straight(-280, speed=300)
        wait(100)
        self.gyro_turn(91, speed=300)
        wait(100)
        self.gyro_straight(135, speed=300)
        wait(100)
        self.up_motor.reset_angle(0)
        self.up_motor.run_target(300, 102)
        wait(100)
        self.start_grab_towers()
        wait(700)
        self.update_grabber_grab()
        wait(100)
        self.gyro_straight(-120, speed=300)
        wait(100)
        self.gyro_turn(91)
        wait(100)
        self.up_motor.reset_angle(0)
        self.up_motor.run_target(300, -192)
        self.gyro_straight_acc(105, speed=150)
        self.up_motor.reset_angle(0)
        self.up_motor.run_target(300, 90)
        wait(100)
        self.spread(speed=400)
        self.gyro_straight(-170, speed=300)
        wait(100)
        self.gyro_turn(90, speed=100)
        wait(100)
        print(stopwatch.time())

    def run(self):
        stopwatch = StopWatch()
        stopwatch.reset()
        stopwatch.resume()
        # self.blocks_task()
        # wait(100)
        # self.gyro_straight(-215)
        # wait(100)
        # self.gyro_turn(-96)
        # wait(100)
        # self.gyro_straight(100)
        # wait(100)
        self.yellow_towers_task()
        self.release(wait=False)
        self.up_motor.reset_angle(0)
        self.up_motor.run_target(300, 102, wait=False)
        wait(200)
        self.line_follow(5000, speed=160, target_color="blue")
        wait(200)
        self.gyro_straight(60, speed=100)
        wait(100)
        self.gyro_turn(-91)
        wait(100)
        self.gyro_straight(360)
        self.start_spread_towers()
        wait(1000)
        self.update_grabber()
        self.gyro_straight(-370, speed=400)
        wait(100)
        self.gyro_turn(-92.5, speed=80, tuning=1)
        wait(100)
        self.gyro_straight_acc(100, speed=100)
        wait(100)
        self.drive_until_color("blue", speed=400)
        wait(200)
        self.gyro_turn(-26, tuning=3)
        wait(100)
        self.gyro_straight(210, speed=400)
        self.release_towers()
        self.release()
        wait(100)
        self.gyro_straight(-50, speed=400)
        wait(100)
        self.gyro_turn(20, speed=400, tuning=3)
        wait(100)
        self.spread(angle=50)
        self.gyro_straight(60, speed=400)
        self.start_grab_towers()
        wait(500)
        self.update_grabber_grab()
        wait(100)
        self.gyro_straight_acc(-20, speed=400)
        wait(100)
        self.gyro_turn(85, speed=400, tuning=4)
        wait(100)
        self.gyro_straight_acc(240, speed=400)
        self.spread()
        self.gyro_straight(-85, speed=400)
        wait(100)
        self.gyro_turn(105, tuning=4)
        wait(100)
        self.gyro_straight(175, speed=600)
        wait(100)
        self.release(wait=False)
        self.line_follow(5000, target_color="blue", speed=175)
        wait(200)
        self.gyro_straight(260, speed=350)
        wait(100)
        self.gyro_turn(-91, speed=400, tuning=3)
        wait(100)
        self.gyro_straight(360, speed=350)
        self.start_spread_towers()
        wait(750)
        self.update_grabber()
        wait(100)
        self.gyro_straight(-365, speed=350)
        wait(100)
        self.gyro_turn(-93)
        wait(100)
        self.gyro_straight_acc(300, speed=400)
        self.line_follow(5000, target_color="red")
        self.release_towers()
        self.release()
        self.gyro_straight(-70, speed=1000)
        self.gyro_turn(-30, speed=600, tuning=5)
        self.gyro_straight(90, speed=1000)
        self.gyro_turn(-165, speed=600, tuning=5)
        self.up_motor.reset_angle(0)
        self.up_motor.run_target(400, -192, wait=False)
        self.gyro_straight(700, speed=900)
        print(stopwatch.time())
        stopwatch.pause()
        self.gyro_straight(-300, speed=600)


robot = Robot()
robot.run()
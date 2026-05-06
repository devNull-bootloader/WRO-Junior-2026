from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait


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

        # Line follow PID constants (kept exactly as you requested)
        self.target = 24.5
        self.Kp = 5.5
        self.Kd = 4
        self.Ki = 0

        # Line color detection constants (hue ranges)
        self.red_range = (340, 10)    # wrap-around
        self.blue_range = (220, 224)
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
            if 230 < h < 310 and 3 < v < 30 and 3 < s < 30:
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
        required_stable = 3  # how many consistent readings needed

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

    def drive_straight(self, distance, speed=300, timeout_ms=None):
        # Drive straight using drive() loop so speed parameter is respected.
        self.drive_base.reset()
        direction = 1 if distance >= 0 else -1
        target = abs(distance)

        elapsed = 0
        step = 10  # ms per loop iteration

        while abs(self.drive_base.distance()) < target:
            # Timeout check
            if timeout_ms is not None and elapsed >= timeout_ms:
                break

            # Drive straight with heading correction from IMU if available
            try:
                heading_error = self.hub.imu.heading()
            except Exception:
                heading_error = 0

            self.drive_base.drive(speed * direction, -heading_error * 0.5)
            # Poll grabber while moving
            self.update_grabber()
            wait(step)
            elapsed += step

        self.drive_base.brake()

    def gyro_turn(self, angle, speed=150, timeout_ms=None):
        try:
            self.hub.imu.reset_heading(0)
        except Exception:
            pass

        target = angle

        # Adjust speed if carrying
        if self.carrying:
            speed = int(speed * 0.7)

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
            if abs(error) < 2:
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

        # carrying adjustment
        if self.carrying:
            speed = int(speed * 0.7)

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

    def line_follow(self, distance, target_color=None, timeout_ms=None):
        # PD line follow using your PID constants (unchanged).
        integral = 0
        last_error = 0

        self.drive_base.reset()
        target_abs = abs(distance)

        elapsed = 0
        step = 10

        while abs(self.drive_base.distance()) < target_abs - 12:  # overshoot compensation
            if timeout_ms is not None and elapsed >= timeout_ms:
                break

            reflection = self.line_sensor.reflection()
            error = reflection - self.target

            integral += error
            derivative = error - last_error
            correction = (self.Kp * error) + (self.Kd * derivative)

            last_error = error

            # ADAPTIVE SPEED
            if self.carrying:
                base_speed = 120
                correction *= 0.8   # smoother turns
            else:
                base_speed = 200 if target_color is None else 140

            self.drive_base.drive(base_speed, correction)

            # Poll grabber while moving
            self.update_grabber()

            # color detection on the line sensor if requested
            if target_color is not None:
                h, s, v = self.line_sensor.hsv()

                if target_color == "red":
                    if h > self.red_range[0] or h < self.red_range[1]:
                        break
                elif target_color == "blue":
                    if self.blue_range[0] < h < self.blue_range[1]:
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

        # Adjust speed if carrying
        if self.carrying:
            speed = int(speed * 0.7)

        while True:
            # Timeout safety
            if timeout_ms is not None and elapsed >= timeout_ms:
                break

            # Gyro correction
            try:
                error = self.hub.imu.heading()
            except Exception:
                error = 0

            correction = error * (1.5 if self.carrying else 2)
            self.drive_base.drive(speed, -correction)

            # Poll grabber
            self.update_grabber()

            # ----- COLOR DETECTION (same logic as line_follow) -----
            h, s, v = self.line_sensor.hsv()

            if target_color == "red":
                if (h > self.red_range[0] or h < self.red_range[1]) and s > 50 and v > 10:
                    break

            elif target_color == "blue":
                if self.blue_range[0] < h < self.blue_range[1]:
                    break

            elif target_color == "green":
                if self.green_range[0] < h < self.green_range[1]:
                    break

            wait(step)
            elapsed += step

        self.drive_base.brake()

    # Arm / Grabber

    def move_arm(self, angle, speed=200):
        # Blocking move to target angle
        self.up_motor.run_target(speed, angle)

    def release(self):
        # Blocking grab: move to target and set carrying True after motion completes.
        self.grabber_motor.run_target(100, -50)  # blocking
        self.carrying = True

    def grab(self):
        # Blocking release: open grabber and clear carrying flag.
        self.grabber_motor.run_target(100, 15)  # blocking
        self.carrying = False

    def spread(self, angle=0, speed=100):
        # Move grabber to a spread/open angle (blocking).
        self.grabber_motor.run_target(speed, angle)

    # High level actions

    def start_grab_towers(self):
        # Non-blocking start: spin grabber slowly while approaching towers.
        self.grabber_motor.run(100)
        self.grabbing = True
        # Do not set carrying here; set carrying when a stall or successful grab is detected

    def release_towers(self):
        # Stop grabber and clear flags
        self.grabber_motor.stop()
        self.grabbing = False
        self.carrying = False

    def update_grabber(self):
        # Polling method: if we were trying to grab and the motor stalls, assume we have grabbed.
        try:
            stalled = self.grabber_motor.stalled()
        except Exception:
            stalled = False

        if self.grabbing and stalled:
            # reduce power to hold
            self.grabber_motor.run(5)
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
        self.move_arm(-20)
        wait(200)

    def _prepare_delivery_close(self):
        """
        Used by the 'nearest probe' branches where the robot stays close:
        stop towers, clamp arm, lower arm immediately (no -5 turn).
        """
        self.release_towers()
        wait(200)
        self.release()
        wait(200)
        self.move_arm(-20)
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
                  "— aborting probe algorithm.")
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
                self.move_arm(-20)
                wait(200)
                self.gyro_turn(-11)
                wait(200)
                self.gyro_turn(6)
            elif inner == "black":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(105)
                wait(200)
                self.gyro_straight(60)
                wait(200)
                self.gyro_turn(40)
                wait(200)
                self.gyro_straight(68)
            elif inner == "blue":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(105)
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
            elif inner == "yellow":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(105)
                wait(200)
                self.gyro_straight(70)
                wait(200)
                self.gyro_turn(55.5)
                wait(200)
                self.gyro_straight(310)

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
            elif inner == "blue":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(105)
                wait(200)
                self.gyro_straight(60)
                wait(200)
                self.gyro_turn(35)
                wait(200)
                self.gyro_straight(74)
            elif inner == "yellow":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(105)
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
            elif inner == "red":
                self._prepare_delivery_standard()
                self._long_route(350)
            
        elif outer == "black":
            self.gyro_turn(176.3, speed=50)
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
            elif inner == "yellow":
                self._prepare_delivery_standard()
                self.gyro_straight(-50)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.move_arm(105)
                wait(200)
                self.gyro_straight(60)
                wait(200)
                self.gyro_turn(30)
                wait(200)
                self.gyro_straight(100)
            elif inner == "green":
                self._prepare_delivery_standard()
                self._long_route(350)
            elif inner == "red":
                self._prepare_delivery_standard()
                self._long_route(480)

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
                self._long_route(610)


    # Mission run

    def run(self):
        self.hub.imu.reset_heading(0)
        wait(200)
        self.gyro_straight(350)
        wait(200)
        self.left_motor.run_angle(300, 523)
        wait(200)
        self.gyro_straight(264, speed=200)
        wait(200)
        self.gyro_turn(-90, speed=50)
        wait(200)
        self.probe_order = self.scan_probes()
        print(self.probe_order)
        self.gyro_turn(50, speed=100)
        wait(200)
        self.gyro_straight(-240)
        wait(200)
        self.gyro_turn(43, speed=100)
        wait(200)
        self.gyro_straight(170, speed=250)
        self.drive_until_color("green", speed=150)
        self.gyro_straight(10)
        self.start_grab_towers()
        wait(2000)
        self.update_grabber()
        self.first_probe_algorithm()



robot = Robot()
robot.run()

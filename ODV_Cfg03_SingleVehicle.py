# =====================================================================================================
#  ODV(Omni-directiobal Vehicle) Configuration 03: L-shape Base Grid, with GBC loader and unloader 
# =====================================================================================================

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.tools import wait, StopWatch
from umath import pi, sin, cos, radians

# Constants
VOLTAGE_WARNING_LEVEL           :int = 7000     # Voltage warning(Yellow) threshold (mV)
VOLTAGE_CRITICAL_LEVEL          :int = 6500     # Voltage critical(Red) threshold (mV)
GEAR_RATIO                      :int = 80       # Motor rotation angle per grid pitch (deg/pitch)
MAX_MOTOR_ROT_SPEED             :int = 1400     # Max motor speed (deg/s) ~1500
HOMING_MOTOR_ROT_SPEED          :int = 200      # Homing speed (deg/s)
HOMING_DUTY                     :int = 35       # Homing motor duty (%) (adjustment required)
HOMING_OFFSET_ANGLE_X           :int = 110      # X-axis offset distance (deg) (adjustment required)
HOMING_OFFSET_ANGLE_Y           :int = 110      # Y-axis offset distance (deg) (adjustment required)

# Vehicle Control Class
class VehicleController:

    def __init__(self, x_motor, y_motor):
        self.x_motor = x_motor
        self.y_motor = y_motor
        self.watch = StopWatch()
        self.current_x = 0
        self.current_y = 0
        self.global_time = 0

    def homing(self):
        # Slowly move until the motor stalls (hits a physical stop),
        # then move forward by an offset distance and set that as the zero origin.
        hub.light.on(Color.GREEN)

        # Homing axis Y
        self.y_motor.run_until_stalled(-HOMING_MOTOR_ROT_SPEED, duty_limit=HOMING_DUTY)
        wait(200)
        self.y_motor.run_angle(MAX_MOTOR_ROT_SPEED, HOMING_OFFSET_ANGLE_Y)
        wait(200)
        self.y_motor.reset_angle(0)
        self.current_y = 0

        # Homing axis X
        self.x_motor.run_until_stalled(-HOMING_MOTOR_ROT_SPEED, duty_limit=HOMING_DUTY)
        wait(200)
        self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, HOMING_OFFSET_ANGLE_X)
        wait(200)
        self.x_motor.reset_angle(0)
        self.current_x = 0

        # Reset timer
        self.watch.reset()
        self.update_light_color()

    def move(self, target_x, target_y, speed=100):
        # target: XY target absolute position (Unit: grid tooth pitch)
        # speed: speed[%]

        # Raise error if speed is out of valid range
        if not (1 <= speed <= 100):
            raise ValueError("Speed must be between 1 and 100")

        # Convert target coordinates from grid pitch to motor angle in degrees
        target_x_deg = target_x * GEAR_RATIO
        target_y_deg = target_y * GEAR_RATIO

        # Calculate distances from current position to target
        x_dist = abs(target_x_deg - self.current_x)
        y_dist = abs(target_y_deg - self.current_y)

        # Calculate movement duration based on max distance and speed
        velo = MAX_MOTOR_ROT_SPEED * speed * 0.01
        max_dist = max(x_dist , y_dist)
        time = max_dist / velo * 1000  # [ms]

        if time > 4:
            self.global_time += time

            # Determine number of steps
            step_num = int(time * 0.25)

            # Calculate step size for each vehicle
            x_step = x_dist / step_num
            y_step = y_dist / step_num

            # Perform stepwise movement
            while step_num > 0:
                if self.current_x < target_x_deg:
                    self.current_x += x_step
                elif self.current_x > target_x_deg:
                    self.current_x -= x_step

                if self.current_y < target_y_deg:
                    self.current_y += y_step
                elif self.current_y > target_y_deg:
                    self.current_y -= y_step

                # Track the position of the controlled vehicle
                self.x_motor.track_target(self.current_x)
                self.y_motor.track_target(self.current_y)

                step_num -= 1

                # wait 4ms
                while (self.global_time - self.watch.time()) > (4 * step_num):
                    wait(1)
   
    def move_circ(self, radius, direction, rot_angle, start_angle, speed=100):
        # radius     : rotation radius [grid tooth pitch]
        # direction  : rotation direction ("CW" or "CCW")
        # rot_angle  : total rotation angle [deg]
        # start_angle: initial velocity vector direction [deg] (0:X+, 90:Y-, 180:X-, 270:Y+)
        # speed      : motor speed as a percentage [%]

        # Raise error if speed is out of valid range
        if not (1 <= speed <= 100):
            raise ValueError("Speed must be between 1 and 100")

        # Raise error if radius or rot_angle is not positive
        if radius <= 0 or rot_angle <= 0:
            raise ValueError("Both radius and rot_angle must be positive")

        # Convert direction string to numerical value
        if isinstance(direction, str):
            direction = direction.upper()
            if direction == "CW":
                direction = 1
            elif direction == "CCW":
                direction = 2
            else:
                raise ValueError("direction must be 'CW' or 'CCW'")
        else:
            direction = 1 if direction <= 1 else 2

        # Calculate initial position of the circular trajectory center
        angle_rad = radians(start_angle + 180 * (direction - 1))
        x_start = self.current_x - radius * GEAR_RATIO * sin(angle_rad)
        y_start = self.current_y - radius * GEAR_RATIO * cos(angle_rad)

        # Calculate movement duration and steps
        velo = MAX_MOTOR_ROT_SPEED * speed * 0.01
        arc_length = 2 * pi * radius * rot_angle / 360 * GEAR_RATIO
        time = arc_length / velo * 1000  # [ms]

        if time > 4:
            self.global_time += time
            step_num = int(time * 0.25)
            s_step = rot_angle / step_num
            s = 0

            # Execute circular movement
            while step_num > 0:
                current_angle = radians(s + start_angle + 180 * (direction - 1))
                self.current_x = radius * GEAR_RATIO * sin(current_angle) + x_start
                self.current_y = radius * GEAR_RATIO * cos(current_angle) + y_start

                self.x_motor.track_target(self.current_x)
                self.y_motor.track_target(self.current_y)

                s = s + s_step if direction == 1 else s - s_step
                step_num -= 1

                # wait 4ms
                while (self.global_time - self.watch.time()) > (4 * step_num):
                    wait(1)

    def get_position(self):
        return self.current_x, self.current_y

    def get_global_time(self):
        return self.global_time

    def wait_ms(self,time):
        # Used to pause the vehicle for a fixed duration
        self.global_time+=time
        self.update_light_color()
        while(self.global_time - self.watch.time()) > 0:
            wait(10)

    def update_light_color(self):
        # Change color based on voltage level
        voltage = hub.battery.voltage()
        if voltage < VOLTAGE_CRITICAL_LEVEL:
            hub.light.on(Color.RED)
        elif voltage < VOLTAGE_WARNING_LEVEL:
            hub.light.on(Color.YELLOW)
        else:
            hub.light.on(Color.BLUE)

# Initialize the hub
hub = TechnicHub()

# Initialize motors on port A,C.
x_motor = Motor(Port.A, Direction.CLOCKWISE)
y_motor = Motor(Port.C, Direction.CLOCKWISE)

# Create an instance for multi-vehicle control
vc = VehicleController(x_motor,y_motor)

# Move vehicle to the fence and reset its origin (home position)
vc.homing()

# Movement instruction description below (main program)
# Coordinate system
#      fence
#   +X ← O fence
#        ↓
#       +Y

vc.move( 10, 0) # Move to the initial position
vc.wait_ms(1000)

while True:
   
    vc.move( 30,  0)
    vc.move( 30,-20)
    vc.wait_ms(200)
    vc.move( 27,-20)
    vc.wait_ms(2500) #Unload
    vc.move( 30,-20)
    vc.wait_ms(200)
    vc.move( 30,  0)
    vc.move( 10,  0)
    vc.wait_ms(200)
    vc.move( 10, -4)
    vc.wait_ms(2500) #load
    vc.move( 10,  0)
    vc.wait_ms(200)
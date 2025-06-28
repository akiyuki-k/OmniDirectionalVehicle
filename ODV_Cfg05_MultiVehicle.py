# =====================================================================================================
#  ODV(Omni-directiobal Vehicle) Configuration 05: 4×4 Base Grid, with Four vehicles 
# =====================================================================================================

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.tools import wait, StopWatch
from umath import pi, sin, cos, radians

# === Constants specific to this vehicle ===
VEHICLE_ID                      :int = 1        # This vehicle's unique ID (1 to N) Change this ID per vehicle

# === Constants common to all vehicles ===
VEHICLE_COUNT                   :int = 4        # Total number of vehicles (N)
USE_SINGLE_SENDER_VEHICLE       :bool = True    # True: one sender and rest are receivers, False: all are receivers
SENDER_VEHICLE_ID               :int = 4        # ID (1 to N) of the vehicle used as the sender when USE_SINGLE_SENDER_VEHICLE is True
BROADCAST_CHANNEL               :int = 1        # Using BLE broadcast channel
BROADCAST_INTERVAL              :int = 500      # Synchronization signal broadcast interval (ms)
SEND_DELAY_COMPENSATION_TIME    :int = 30       # Communication delay compensation time (ms) when USE_SINGLE_SENDER_VEHICLE is True
LINEAR_ESTIMATION_START_TIME    :int = 60000    # Start time for linear estimation (ms)
TIMESTAMP_BUF_SIZE              :int = 30       # Timestamp buffer size
VOLTAGE_WARNING_LEVEL           :int = 7000     # Voltage warning(Yellow) threshold (mV)
VOLTAGE_CRITICAL_LEVEL          :int = 6500     # Voltage critical(Red) threshold (mV)
GEAR_RATIO                      :int = 80       # Motor rotation angle per grid pitch (deg/pitch)
MAX_MOTOR_ROT_SPEED             :int = 1400     # Max motor speed (deg/s) ~1500
HOMING_MOTOR_ROT_SPEED          :int = 200      # Homing speed (deg/s)
HOMING_DUTY                     :int = 35       # Homing motor duty (%) (adjustment required)
HOMING_OFFSET_ANGLE_X           :int = 110      # X-axis offset distance (deg) (adjustment required)
HOMING_OFFSET_ANGLE_Y           :int = 110      # Y-axis offset distance (deg) (adjustment required)

# Bluetooth Time Synchronization Class
class BluetoothTimeSync:

    class FIFOBuffer:
        def __init__(self, size):
            self.size = size
            self.buffer = []

        def push(self, value):
            if len(self.buffer) >= self.size:
                self.buffer.pop(0)
            self.buffer.append(value)

        def get(self):
            return list(self.buffer)

        def length(self):
            return len(self.buffer)

        def average(self):
            if not self.buffer:
                return 0
            return sum(self.buffer) / len(self.buffer)

        def maximum(self):
            if not self.buffer:
                return None
            return max(self.buffer)

        def max_index(self):
            if not self.buffer:
                return None
            max_value = max(self.buffer)
            return self.buffer.index(max_value)

        def get_at(self, index):
            if 0 <= index < len(self.buffer):
                return self.buffer[index]
            else:
                return None

    def __init__(self, hub, observe_ch, buffer_size, lin_estim_start):
        self.hub = hub
        self.observe_ch = observe_ch
        self.lin_estim_start = lin_estim_start
        self.buffer_size = buffer_size
        self.watch = StopWatch()

        self.recv_time_buf = self.FIFOBuffer(self.buffer_size)
        self.diff_time_buf = self.FIFOBuffer(self.buffer_size)

        self.mstr_time = 0
        self.send_count = 0
        self.recv_count = 0
        self.adjusted_time = 0
        self.intercept = 0.0
        self.slope = 0.0

        self.initial_time_diff_recorded = False
        self.state = "Wait"
        self.initial_recv_time = 0
        self.initial_diff_time = 0

    def find_linear_function(self, x1, y1, x2, y2):
        if x1 == x2:
            return 0.0, y1  # avoid divide by zero
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1
        return slope, intercept

    def start_message_send(self):
        self.watch.reset()
        self.state="Run"
        print("Sending start")

    def send(self):
        # Send synchronization signals at the configured interval
        if  self.watch.time()>self.send_count*BROADCAST_INTERVAL:
            data = ("Run",self.watch.time() + SEND_DELAY_COMPENSATION_TIME)
            hub.ble.broadcast(data)
            self.send_count += 1

            # Indicator changes on each transmission; color depends on voltage and alternates.
            voltage = self.hub.battery.voltage()
            if voltage < VOLTAGE_CRITICAL_LEVEL:
                base_color = Color.RED
            elif voltage < VOLTAGE_WARNING_LEVEL:
                base_color = Color.YELLOW
            else:
                base_color = Color.BLUE
            self.hub.light.on(base_color if self.send_count % 2 == 0 else Color.WHITE)


    def wait_first_message(self):
        # Check that the initial state is 'Wait'
        if self.state != "Wait":
            raise ValueError(f"Invalid state: expected 'Wait', but got '{self.state}'")
        while True:
            self.hub.light.on(Color.GREEN)
            data = self.hub.ble.observe(self.observe_ch)
            if data is not None:
                a, b = data
                if a=="Run":
                    # Ensure the received timestamp is within 10000ms (valid startup signal)
                    if b < 10000:
                        print(f"Successfully received the first message: type={a}, timestamp={b}")
                        self.watch.reset()
                        self.state = "Run"
                        break
                    else:
                        print(f"Warning: The received timestamp ({b}) is out of range. Please reset the transmitting hub.")
                else:
                    print(f"Warning: The received message is not 'Run'. Received: {a}")
                self.hub.light.on(Color.MAGENTA)
                wait(1000)
            wait(100)

    def recv(self):
        data = self.hub.ble.observe(self.observe_ch)
        if data is not None:
            a, b = data
            if a=="Run":
                self.state="Run"
                # Update buffers only if the received signal is newer than the previous one.
                if b > self.mstr_time:
                    self.recv_count += 1
                    self.mstr_time = b
                    current_time = self.watch.time()
                    diff_time = self.mstr_time - current_time

                    self.recv_time_buf.push(current_time)
                    self.diff_time_buf.push(diff_time)

                    # Indicator changes on each reception; color depends on voltage and alternates.
                    voltage = self.hub.battery.voltage()
                    if voltage < VOLTAGE_CRITICAL_LEVEL:
                        base_color = Color.RED
                    elif voltage < VOLTAGE_WARNING_LEVEL:
                        base_color = Color.YELLOW
                    else:
                        base_color = Color.BLUE
                    self.hub.light.on(base_color if self.recv_count % 2 == 0 else Color.WHITE)

            elif a=="Stop":
                self.state="Stop"
                self.hub.light.on(Color.RED)

    def update_coefficient(self):
        # Update the time correction coefficients (slope and intercept)
        # based on the buffered reception times and time differences.

        # Only proceed if at least one reception has been recorded.
        if self.recv_time_buf.length() > 0:
            # Use the index with the maximum time difference as the most reliable sample.
            max_index = self.diff_time_buf.max_index()
            latest_recv_time = self.recv_time_buf.get_at(max_index)
            latest_diff_time = self.diff_time_buf.maximum()

            # During the initial period or if the same time was received again,
            # use only the intercept (ignore slope for now).
            if self.watch.time() < self.lin_estim_start or self.initial_recv_time == latest_recv_time:
                self.intercept = latest_diff_time
                # If the buffer is full and this is the first time difference being recorded,
                # store it as the initial point for future slope calculation.
                if (self.recv_time_buf.length() == self.buffer_size and not self.initial_time_diff_recorded):
                    self.initial_recv_time = latest_recv_time
                    self.initial_diff_time = latest_diff_time
                    self.initial_time_diff_recorded = True
                    print(f"Initial time difference recorded: recv_time={self.initial_recv_time}, diff_time={self.initial_diff_time}")
            else:
                # Calculate the intercept and slope of the time correction line
                # using the initial and most recent reliable data points.
                new_slope, new_intercept = self.find_linear_function(
                    self.initial_recv_time, self.initial_diff_time,
                    latest_recv_time, latest_diff_time
                )
                # Only print and update if values have changed
                if new_slope != self.slope or new_intercept != self.intercept:
                    print(f"[{self.watch.time()}ms] slope={new_slope:.6f}, intercept={new_intercept:.2f}")
                    self.slope = new_slope
                    self.intercept = new_intercept

    def get_adjusted_time(self):
        # Return adjusted global time calculated from slope and intercept
        if self.state == "Run":
            self.adjusted_time = (1 + self.slope) * self.watch.time() + self.intercept
            return self.adjusted_time
        else:
            return 0

    def get_rawtime(self):
        return self.watch.time()

    def get_recv_count(self):
        return self.recv_count

    def get_state(self):
        return self.state

# Multi-Vehicle Control Class
class MultiVehicleController:

    def __init__(self, vehicle_count, vehicle_id, x_motor, y_motor, bt_sync:BluetoothTimeSync):
        self.vehicle_id = vehicle_id
        self.x_motor = x_motor
        self.y_motor = y_motor
        self.vehicle_count = vehicle_count
        self.current_x = [0] * (vehicle_count + 1)
        self.current_y = [0] * (vehicle_count + 1)
        self.global_time = 0

    def homing(self):
        # Slowly move until the motor stalls (hits a physical stop),
        # then move forward by an offset distance and set that as the zero origin.

        # Homing axis Y
        self.y_motor.run_until_stalled(-HOMING_MOTOR_ROT_SPEED, duty_limit=HOMING_DUTY)
        wait(200)
        self.y_motor.run_angle(MAX_MOTOR_ROT_SPEED, HOMING_OFFSET_ANGLE_Y)
        wait(200)
        self.y_motor.reset_angle(0)
        self.current_y = [0] * (self.vehicle_count + 1)

        # Homing axis X
        self.x_motor.run_until_stalled(-HOMING_MOTOR_ROT_SPEED, duty_limit=HOMING_DUTY)
        wait(200)
        self.x_motor.run_angle(MAX_MOTOR_ROT_SPEED, HOMING_OFFSET_ANGLE_X)
        wait(200)
        self.x_motor.reset_angle(0)
        self.current_x = [0] * (self.vehicle_count + 1)

    def init_20sec_sync_process(self):
        # If this vehicle is the designated sender, start broadcasting messages
        # Otherwise, wait to receive the first synchronization message from the sender
        if USE_SINGLE_SENDER_VEHICLE and VEHICLE_ID == SENDER_VEHICLE_ID:
            bt_sync.start_message_send()
        else:
            bt_sync.wait_first_message()

        # Wait 20 seconds initially for synchronization and system stabilization
        self.wait_sync_ms(20000)

    def move(self, *targets, speed=100):
        # targets: XY target absolute position (Unit: grid tooth pitch)
        # speed: speed[%]

        assert len(targets) == self.vehicle_count * 2, "The number of targets does not match the number of vehicles"

        # Raise error if speed is out of valid range
        if not (1 <= speed <= 100):
            raise ValueError("Speed must be between 1 and 100")

        # Convert target coordinates from grid pitch to motor angle in degrees
        target_x_deg = [0] + [targets[i] * GEAR_RATIO for i in range(0, len(targets), 2)]
        target_y_deg = [0] + [targets[i] * GEAR_RATIO for i in range(1, len(targets), 2)]

        x_dist = [0] * (self.vehicle_count + 1)  # X-axis distances to target
        y_dist = [0] * (self.vehicle_count + 1)  # Y-axis distances to target
        x_step = [0] * (self.vehicle_count + 1)  # X-axis step sizes
        y_step = [0] * (self.vehicle_count + 1)  # Y-axis step sizes

        # Calculate distances from current position to target
        for i in range(1, self.vehicle_count + 1):
            x_dist[i] = abs(target_x_deg[i] - self.current_x[i])
            y_dist[i] = abs(target_y_deg[i] - self.current_y[i])

        # Calculate movement duration based on max distance and speed
        velo = MAX_MOTOR_ROT_SPEED * speed * 0.01
        max_dist = max(x_dist[1:] + y_dist[1:])
        time = max_dist / velo * 1000  # [ms]

        if time > 4:
            self.global_time += time

            # Determine number of steps
            step_num = int(time * 0.25)

            # Calculate step size for each vehicle
            for i in range(1, self.vehicle_count + 1):
                x_step[i] = x_dist[i] / step_num
                y_step[i] = y_dist[i] / step_num

            # Perform stepwise movement
            while step_num > 0:
                for i in range(1, self.vehicle_count + 1):
                    if self.current_x[i] < target_x_deg[i]:
                        self.current_x[i] += x_step[i]
                    elif self.current_x[i] > target_x_deg[i]:
                        self.current_x[i] -= x_step[i]

                    if self.current_y[i] < target_y_deg[i]:
                        self.current_y[i] += y_step[i]
                    elif self.current_y[i] > target_y_deg[i]:
                        self.current_y[i] -= y_step[i]

                # Track the position of the controlled vehicle
                self.x_motor.track_target(self.current_x[self.vehicle_id])
                self.y_motor.track_target(self.current_y[self.vehicle_id])

                step_num -= 1

                # Check received data during the 4 ms wait period
                if bt_sync.get_state() == "Run":
                    if USE_SINGLE_SENDER_VEHICLE and VEHICLE_ID == SENDER_VEHICLE_ID:
                        while (self.global_time - bt_sync.get_rawtime()) > (4 * step_num):
                            bt_sync.send()
                            wait(1)
                    else:
                        while (self.global_time - bt_sync.get_adjusted_time()) > (4 * step_num):
                            bt_sync.recv()
                            if bt_sync.get_state() == "Stop":
                                self.emergency_stop_forever()
                            bt_sync.update_coefficient()
                            wait(1)
                else:
                    wait(4)
   
    def move_circ(self, radius, direction, rot_angle, *start_angles, speed=100):
        # radius     : rotation radius [grid tooth pitch]
        # direction  : rotation direction ("CW" or "CCW")
        # rot_angle   : total rotation angle [deg]
        # start_angles: initial velocity vector direction [deg] for each vehicle (0:X+, 90:Y-, 180:X-, 270:Y+)
        # speed      : motor speed as a percentage [%]

        assert len(start_angles) == self.vehicle_count, "Number of initial angles does not match vehicle count"

        # Raise error if radius or rot_angle is not positive
        if radius <= 0 or rot_angle <= 0:
            raise ValueError("Both radius and rot_angle must be positive")

        # Raise error if speed is out of valid range
        if not (1 <= speed <= 100):
            raise ValueError("Speed must be between 1 and 100")

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

        start_angle = [0] + list(start_angles)
        x_start = [0] * (self.vehicle_count + 1)
        y_start = [0] * (self.vehicle_count + 1)

        # Calculate movement duration and number of steps
        velo = MAX_MOTOR_ROT_SPEED * speed * 0.01
        max_dist = 2 * pi * radius * rot_angle / 360 * GEAR_RATIO
        time = max_dist / velo * 1000  # [ms]
        self.global_time += time
        step_num = int(time * 0.25)
        s_step = rot_angle / step_num
        s = 0

        # Compute the circular trajectory origin for each vehicle
        for i in range(1, self.vehicle_count + 1):
            angle = radians(start_angle[i] + 180 * (direction - 1))
            x_start[i] = self.current_x[i] - radius * GEAR_RATIO * sin(angle)
            y_start[i] = self.current_y[i] - radius * GEAR_RATIO * cos(angle)

        # Execute circular movement step by step
        while step_num > 0:
            for i in range(1, self.vehicle_count + 1):
                angle = radians(s + start_angle[i] + 180 * (direction - 1))
                self.current_x[i] = radius * GEAR_RATIO * sin(angle) + x_start[i]
                self.current_y[i] = radius * GEAR_RATIO * cos(angle) + y_start[i]

            self.x_motor.track_target(self.current_x[self.vehicle_id])
            self.y_motor.track_target(self.current_y[self.vehicle_id])

            s = s + s_step if direction == 1 else s - s_step
            step_num -= 1

            # Check received data during the 4 ms wait period
            if bt_sync.get_state() == "Run":
                if USE_SINGLE_SENDER_VEHICLE and VEHICLE_ID == SENDER_VEHICLE_ID:
                    while (self.global_time - bt_sync.get_rawtime()) > (4 * step_num):
                        bt_sync.send()
                        wait(10)
                else:
                    while (self.global_time - bt_sync.get_adjusted_time()) > (4 * step_num):
                        bt_sync.recv()
                        if bt_sync.get_state() == "Stop":
                            self.emergency_stop_forever()
                        bt_sync.update_coefficient()
                        wait(1)
            else:
                wait(4)

    def emergency_stop_forever(self):
        # Immediately stop both X and Y motors and enter an infinite wait loop
        self.x_motor.stop()
        self.y_motor.stop()
        while True:
            wait(1000)

    def get_position(self):
        return self.current_x[self.vehicle_id], self.current_y[self.vehicle_id]

    def get_global_time(self):
        return self.global_time

    def wait_ms(self,time):
        # Used to pause the vehicle for a fixed duration without synchronization
        self.global_time+=time
        wait(time)

    def wait_sync_ms(self,time):
        # Wait for the specified time with synchronization if active, otherwise perform simple wait
        self.global_time+=time
        if bt_sync.get_state()=="Run":
            if USE_SINGLE_SENDER_VEHICLE and VEHICLE_ID == SENDER_VEHICLE_ID:
                while (bt_sync.get_rawtime()<self.global_time):
                    bt_sync.send()
                    wait(10)
            else:
                while (bt_sync.get_adjusted_time()<self.global_time):
                    bt_sync.recv()
                    if bt_sync.get_state()=="Stop":
                        self.emergency_stop_forever()
                    bt_sync.update_coefficient()
                    wait(10)
        else:
            wait(time)

# Initialize the hub.
if USE_SINGLE_SENDER_VEHICLE and VEHICLE_ID == SENDER_VEHICLE_ID:
    hub = TechnicHub(broadcast_channel=BROADCAST_CHANNEL)
else:
    hub = TechnicHub(observe_channels=[BROADCAST_CHANNEL])

# Create an instance for communication handling
bt_sync=BluetoothTimeSync(hub,BROADCAST_CHANNEL,TIMESTAMP_BUF_SIZE,LINEAR_ESTIMATION_START_TIME)

# Initialize motors on port A,C.
x_motor = Motor(Port.A, Direction.CLOCKWISE)
y_motor = Motor(Port.C, Direction.CLOCKWISE)

# Create an instance for multi-vehicle control
mvc = MultiVehicleController(VEHICLE_COUNT,VEHICLE_ID,x_motor,y_motor,bt_sync)

# Move this vehicle to the fence and reset its origin (home position)
mvc.homing()

# Movement instruction description below (main program)
# Coordinate system
#      fence
#   +X ← O fence
#        ↓
#       +Y

# Move to the initial position (initial position must be adjusted depending on the movement plan)
# The path must avoid already placed vehicles. This can be achieved by combining multiple movement commands.
mvc.move(10, 0,  10, 0,  10, 0,   0, 0)
mvc.move(20, 0,  20, 0,  10, 0,   0, 0)
mvc.move(30, 0,  20, 0,  10, 0,   0, 0)

# Start communication and wait for 20 seconds (initialization phase)
mvc.init_20sec_sync_process()

while True:

    # Demo of figure-eight (infinity symbol) movement
    mvc.move(30,15,  20, 30,  10, 0,   0, 0)
    for _ in range(2):
        mvc.move(15.00,15.00,1.56,25.74,28.44,4.26,18.44,0.00)
        mvc.move(16.06,16.91,0.90,23.66,27.35,2.39,17.35,0.00)
        mvc.move(17.14,18.80,0.47,21.52,25.45,1.51,15.45,0.00)
        mvc.move(18.24,20.69,0.20,19.36,23.49,2.40,13.49,0.00)
        mvc.move(19.38,22.54,0.05,17.18,21.94,3.93,11.94,0.00)
        mvc.move(20.60,24.35,0.00,15.00,20.60,5.65,10.60,0.00)
        mvc.move(21.94,26.07,0.05,12.82,19.38,7.46,9.38,0.00)
        mvc.move(23.49,27.60,0.20,10.64,18.24,9.31,8.24,0.00)
        mvc.move(25.45,28.49,0.47,8.48,17.14,11.20,10.47,0.00)
        mvc.move(27.35,27.61,0.90,6.34,16.06,13.09,10.90,0.00)
        mvc.move(28.44,25.74,1.56,4.26,15.00,15.00,11.56,0.00)
        mvc.move(29.10,23.66,2.65,2.39,13.94,16.91,12.65,0.00)
        mvc.move(29.53,21.52,4.55,1.51,12.86,18.80,14.55,0.00)
        mvc.move(29.80,19.36,6.51,2.40,11.76,20.69,16.51,0.00)
        mvc.move(29.95,17.18,8.06,3.93,10.62,22.54,18.06,0.00)
        mvc.move(30.00,15.00,9.40,5.65,9.40,24.35,19.40,0.00)
        mvc.move(29.95,12.82,10.62,7.46,8.06,26.07,20.62,0.00)
        mvc.move(29.80,10.64,11.76,9.31,6.51,27.60,21.76,0.00)
        mvc.move(29.53,8.48,12.86,11.20,4.55,28.49,19.53,0.00)
        mvc.move(29.10,6.34,13.94,13.09,2.65,27.61,19.10,0.00)
        mvc.move(28.44,4.26,15.00,15.00,1.56,25.74,18.44,0.00)
        mvc.move(27.35,2.39,16.06,16.91,0.90,23.66,17.35,0.00)
        mvc.move(25.45,1.51,17.14,18.80,0.47,21.52,15.45,0.00)
        mvc.move(23.49,2.40,18.24,20.69,0.20,19.36,13.49,0.00)
        mvc.move(21.94,3.93,19.38,22.54,0.05,17.18,11.94,0.00)
        mvc.move(20.60,5.65,20.60,24.35,0.00,15.00,10.60,0.00)
        mvc.move(19.38,7.46,21.94,26.07,0.05,12.82,9.38,0.00)
        mvc.move(18.24,9.31,23.49,27.60,0.20,10.64,8.24,0.00)
        mvc.move(17.14,11.20,25.45,28.49,0.47,8.48,10.47,0.00)
        mvc.move(16.06,13.09,27.35,27.61,0.90,6.34,10.90,0.00)
        mvc.move(15.00,15.00,28.44,25.74,1.56,4.26,11.56,0.00)
        mvc.move(13.94,16.91,29.10,23.66,2.65,2.39,12.65,0.00)
        mvc.move(12.86,18.80,29.53,21.52,4.55,1.51,14.55,0.00)
        mvc.move(11.76,20.69,29.80,19.36,6.51,2.40,16.51,0.00)
        mvc.move(10.62,22.54,29.95,17.18,8.06,3.93,18.06,0.00)
        mvc.move(9.40,24.35,30.00,15.00,9.40,5.65,19.40,0.00)
        mvc.move(8.06,26.07,29.95,12.82,10.62,7.46,20.62,0.00)
        mvc.move(6.51,27.60,29.80,10.64,11.76,9.31,21.76,0.00)
        mvc.move(4.55,28.49,29.53,8.48,12.86,11.20,19.53,0.00)
        mvc.move(2.65,27.61,29.10,6.34,13.94,13.09,19.10,0.00)
        mvc.move(1.56,25.74,28.44,4.26,15.00,15.00,18.44,0.00)
        mvc.move(0.90,23.66,27.35,2.39,16.06,16.91,17.35,0.00)
        mvc.move(0.47,21.52,25.45,1.51,17.14,18.80,15.45,0.00)
        mvc.move(0.20,19.36,23.49,2.40,18.24,20.69,13.49,0.00)
        mvc.move(0.05,17.18,21.94,3.93,19.38,22.54,11.94,0.00)
        mvc.move(0.00,15.00,20.60,5.65,20.60,24.35,10.60,0.00)
        mvc.move(0.05,12.82,19.38,7.46,21.94,26.07,9.38,0.00)
        mvc.move(0.20,10.64,18.24,9.31,23.49,27.60,8.24,0.00)
        mvc.move(0.47,8.48,17.14,11.20,25.45,28.49,10.47,0.00)
        mvc.move(0.90,6.34,16.06,13.09,27.35,27.61,10.90,0.00)
        mvc.move(1.56,4.26,15.00,15.00,28.44,25.74,11.56,0.00)
        mvc.move(2.65,2.39,13.94,16.91,29.10,23.66,12.65,0.00)
        mvc.move(4.55,1.51,12.86,18.80,29.53,21.52,14.55,0.00)
        mvc.move(6.51,2.40,11.76,20.69,29.80,19.36,16.51,0.00)
        mvc.move(8.06,3.93,10.62,22.54,29.95,17.18,18.06,0.00)
        mvc.move(9.40,5.65,9.40,24.35,30.00,15.00,19.40,0.00)
        mvc.move(10.62,7.46,8.06,26.07,29.95,12.82,20.62,0.00)
        mvc.move(11.76,9.31,6.51,27.60,29.80,10.64,21.76,0.00)
        mvc.move(12.86,11.20,4.55,28.49,29.53,8.48,19.53,0.00)
        mvc.move(13.94,13.09,2.65,27.61,29.10,6.34,19.10,0.00)

    mvc.move(30,15,  20, 30,  10, 0,   0, 0)

    # Demo of looping
    mvc.move( 30, 15, 20, 15, 10, 15,  0, 15)
    mvc.wait_sync_ms(1000)
    mvc.move( 31,  5, 21, 15, 11, 15,  1, 15)
    mvc.move( -1,  5, 29, 15, 19, 15,  9, 15)
    mvc.move(  1, 15, 31,  5, 21, 15, 11, 15)
    mvc.move(  9, 15, -1,  5, 29, 15, 19, 15)
    mvc.move( 11, 15,  1, 15, 31,  5, 21, 15)
    mvc.move( 19, 15,  9, 15, -1,  5, 29, 15)
    mvc.move( 21, 15, 11, 15,  1, 15, 31,  5)
    mvc.move( 29, 15, 19, 15,  9, 15, -1,  5)
    mvc.move( 31, 15, 21, 15, 11, 15,  1, 15)
    mvc.move( 30, 15, 20, 15, 10, 15,  0, 15)
    mvc.wait_sync_ms(1000)
    mvc.move( 30, 10, 20, 20, 10, 10,  0, 20)
    mvc.move( 20, 10, 30, 20,  0, 10, 10, 20)
    mvc.move( 20, 15, 30, 15,  0, 15, 10, 15)
    mvc.wait_sync_ms(500)
    mvc.move( 20, 10, 30, 20,  0, 10, 10, 20)
    mvc.move( 10,  0, 20, 10, 10, 20, 20, 30)
    mvc.move( 15,  0, 15, 10, 15, 20, 15, 30)
    mvc.wait_sync_ms(1000)
    mvc.move( 25,  0, 25, 10, 25, 20, 25, 30)
    mvc.move(  5,  0,  5, 10,  5, 20,  5, 30)
    mvc.move( 15,  0, 15, 10, 15, 20, 15, 30)
    mvc.wait_sync_ms(500)
    mvc.move( 20,  0, 10, 10, 20, 20, 10, 30)
    mvc.move( 20, 10, 10,  0, 20, 30, 10, 20)
    mvc.move( 15, 10, 15,  0, 15, 30, 15, 20)
    mvc.wait_sync_ms(500)
    mvc.move( 20, 10, 10,  0, 20, 30, 10, 20)
    mvc.move( 20, 10, 10, 10, 20, 20, 10, 20)
    mvc.wait_sync_ms(1000)
    mvc.move( 30,  0,  0,  0, 30, 30,  0, 30)
    mvc.wait_sync_ms(500)
    mvc.move( 20, 10, 10, 10, 20, 20, 10, 20)
    mvc.wait_sync_ms(500)
    mvc.move( 30, 20, 20,  0, 10, 30,  0, 10)
    mvc.wait_sync_ms(500)
    mvc.move( 20, 10, 10, 10, 20, 20, 10, 20)
    mvc.wait_sync_ms(500)
    mvc.move( 30,  0,  0,  0, 30, 30,  0, 30)
    mvc.wait_sync_ms(500)
    mvc.move( 30,  0, 20,  0, 10, 10,  0, 10)
    mvc.move( 30,  0, 20,  0, 10,  0,  0,  0)
    mvc.wait_sync_ms(1000)

    # Demo of circular movement
    mvc.wait_sync_ms(1000)
    mvc.move(30, 5,  20, 15,  10,25,  0,15)
    mvc.move(15, 5,  25, 15,  15,25,  5,15)
    mvc.wait_sync_ms(1000)
    mvc.move_circ(10,"CW",585,180,90,0,270)
    mvc.move(20,10,  10,10,  10, 0,  20, 0)
    mvc.wait_sync_ms(1500)
    mvc.move_circ(10,"CW",270,180,180,180,180)
    mvc.move(20,30,  10,20,  10,10,  20, 0)
    mvc.move(15,30,  15,20,  15,10,  15, 0)

    # Demo of vehicles aligning in a straight line
    mvc.wait_sync_ms(1000)
    mvc.move(30,30,  20,20,  10,10,   0, 0)
    mvc.move( 0,30,  10,20,  20,10,  30, 0)
    mvc.move( 0, 0,  10,10,  20,20,  30,30)
    mvc.move(30, 0,  20,10,  10,20,   0,30)
    mvc.move(30,15,  20,15,  10,15,   0,15)
    mvc.move(30, 0,  20, 0,  10, 0,   0, 0)

    # Demo of a lap around the workspace
    mvc.wait_sync_ms(2000)
    mvc.move(30,10,  20, 0,  10, 0,   0, 0)
    mvc.move(30,20,  30,10,  20, 0,  10, 0,speed=80)
    mvc.move(20,30,  30,20,  30,10,  20, 0,speed=80)
    mvc.move(10,30,  20,30,  30,20,  30,10,speed=80)
    mvc.move( 0,20,  10,30,  20,30,  30,20,speed=80)
    mvc.move( 0,10,   0,20,  10,30,  20,30,speed=80)
    mvc.move(10, 0,   0,10,   0,20,  10,30,speed=80)
    mvc.move(20, 0,  10, 0,   0,10,   0,20,speed=80)
    mvc.move(30, 0,  20, 0,  10, 0,   0,10,speed=80)
    mvc.move(30, 0,  20, 0,  10, 0,   0, 0,speed=80)

    mvc.wait_sync_ms(1000)



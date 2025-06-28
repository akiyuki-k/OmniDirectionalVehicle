# =====================================================================================================
#  ODV(Omni-directiobal Vehicle) Configuration 01: Remote Control
# =====================================================================================================

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor, Remote
from pybricks.parameters import Button, Direction, Port
from pybricks.tools import wait, StopWatch

hub = TechnicHub()

# Set the motor power (0-100%) default=60%
power=60

# Initialize motors on port A,C.
x_motor = Motor(Port.A, Direction.CLOCKWISE)
y_motor = Motor(Port.C, Direction.CLOCKWISE)

# Connect to the remote.
my_remote = Remote()

while True:
    # Check which buttons are pressed.
    pressed = my_remote.buttons.pressed()

    # Check the y-axis button and drive y-motor.
    if Button.LEFT_PLUS in pressed:
        y_motor.dc(power)
    elif Button.LEFT_MINUS in pressed:
        y_motor.dc(-power)
    else:
        y_motor.stop()

    # Check the x-axis button and drive x-motor.
    if Button.RIGHT_PLUS in pressed:
        x_motor.dc(-power)
    elif Button.RIGHT_MINUS in pressed:
        x_motor.dc(power)
    else:
        x_motor.stop()

    wait(10)
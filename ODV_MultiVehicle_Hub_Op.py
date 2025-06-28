# =====================================================================================================
#  ODV(Omni-directiobal Vehicle) Multi Vehicle Hub Option (for Cfg04,Cfg05)
# =====================================================================================================

from pybricks.hubs import TechnicHub
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Color, Side

# Constants
BROADCAST_CHANNEL       : int = 1      # BLE broadcast channel to use
BROADCAST_INTERVAL      : int = 500    # Synchronization signal broadcast interval (ms)
STOP_INTERVAL           : int = 100    # Stop signal broadcast interval (ms)
VOLTAGE_WARNING_LEVEL   : int = 7000   # Voltage warning(Yellow) threshold (mV)
VOLTAGE_CRITICAL_LEVEL  : int = 6500   # Voltage critical(Red) threshold (mV)

# Initialize the hub and stopwatch
hub = TechnicHub(broadcast_channel=BROADCAST_CHANNEL)
watch = StopWatch()
send_count = 0

# Send synchronization signals at the configured interval
while hub.imu.up() != Side.BOTTOM:
    data = ("Run", watch.time())
    hub.ble.broadcast(data)

    # Indicator changes on each reception; color depends on voltage and alternates.
    voltage = hub.battery.voltage()
    if voltage < VOLTAGE_CRITICAL_LEVEL:
        base_color = Color.RED
    elif voltage < VOLTAGE_WARNING_LEVEL:
        base_color = Color.YELLOW
    else:
        base_color = Color.BLUE
    hub.light.on(base_color if send_count % 2 == 0 else Color.WHITE)

    send_count += 1

    # Wait until the next broadcast time
    while watch.time() < send_count * BROADCAST_INTERVAL:
        wait(10)

# Emergency stop mode: broadcast "Stop" signal repeatedly
hub.light.on(Color.RED)
while True:
    data = ("Stop", watch.time())
    hub.ble.broadcast(data)
    wait(STOP_INTERVAL)

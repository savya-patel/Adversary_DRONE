from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# Connect to the Cube Orange
print("Connecting to vehicle...")
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
print("Connected to vehicle:", vehicle.version)

# Set mode to GUIDED_NOGPS
print("Setting mode to GUIDED_NOGPS...")
vehicle.mode = VehicleMode("GUIDED_NOGPS")
while not vehicle.mode.name == 'GUIDED_NOGPS':
    print("Waiting for mode change...")
    time.sleep(1)
print("Mode set to GUIDED_NOGPS")

# Arm motors (NO PROPS!)
vehicle.armed = True
while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)
print("Vehicle armed!")

# Function to send velocity
def send_velocity(vehicle, vx, vy, vz, yaw_rate):
    """
    Sends velocity command (m/s) in body frame.
    vx: forward velocity (+x)
    vy: right velocity (+y)
    vz: downward velocity (+z)
    yaw_rate: positive = rotate clockwise (rad/s)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # relative to drone’s heading
        0b0000011111000111,  # enable velocity + yaw_rate only
        #0b0000111111000111,  # mask: use only velocity components
        0, 0, 0,             # position (ignored)
        vx, vy, vz,          # velocity in m/s
        0, 0, 0,             # acceleration (ignored)
        0, yaw_rate)         # yaw, yaw_rate (ignored)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"Sent velocity command: vx={vx:.2f} m/s, vy={vy:.2f} m/s, vz={vz:.2f} m/s, yaw_rate={yaw_rate:.2f} rad/s")
    print("Actual velocity (m/s):", vehicle.velocity)

# Send forward velocity for 10 seconds
print("Sending test velocity commands...")
for i in range(10):
    send_velocity(vehicle, 0.2, 0, 0, 0.3)  # move forward + yaw clockwise
    time.sleep(1)

# Send forward velocity for 10 seconds
print("Sending test velocity commands...")
for i in range(10):
    send_velocity(vehicle, -0.2, 0, 0, -0.3)  # move backward + yaw counter-clockwise
    time.sleep(1)

# Stop movement
send_velocity(vehicle, 0, 0, 0, 0)
print("Velocity and yaw commands sent. Stopping...")

# Disarm and close connection
vehicle.armed = False
while vehicle.armed:
    print("Waiting for disarm...")
    time.sleep(1)
print("Vehicle disarmed.")
vehicle.close()
print("Test complete.")

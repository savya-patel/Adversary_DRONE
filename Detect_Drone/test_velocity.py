from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil
import math

# ---------------------------
# Connect
# ---------------------------
print("Connecting...")
vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=False)
print(f"Connected to: {vehicle.version}")

# ---------------------------
# Parameter setup
# ---------------------------
params = {
    'ARMING_CHECK': 0,
    'EK3_SRC1_POSZ': 0,
    'EK3_SRC1_VELXY': 0,
    'EK3_SRC1_VELZ': 0,
    'AHRS_EKF_TYPE': 3,
    'GUID_OPTIONS': 8
}

print("Setting parameters for GUIDED_NOGPS...")
for p, v in params.items():
    try:
        vehicle.parameters[p] = v
        time.sleep(0.2)
    except Exception as e:
        print(f"Failed to set {p}: {e}")

print("Parameters set.\n")

# ---------------------------
# Mode + Arm
# ---------------------------
print("Switching to GUIDED_NOGPS...")
vehicle.mode = VehicleMode("GUIDED_NOGPS")
while not vehicle.mode.name == 'GUIDED_NOGPS':
    print(" Waiting for mode...")
    time.sleep(1)
print("Mode set!")

print("Arming motors...")
vehicle.armed = True
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)
print("Armed!\n")

# ---------------------------
# Function: Send attitude
# ---------------------------
def send_attitude_target(vehicle, roll=0.0, pitch=0.0, yaw=0.0, thrust=0.5):
    """Send SET_ATTITUDE_TARGET MAVLink message"""
    # quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ]

    # use time since heartbeat as ms uptime
    time_boot_ms = int((time.time() - vehicle.last_heartbeat) * 1000) & 0xFFFFFFFF

    vehicle._master.mav.set_attitude_target_send(
        time_boot_ms,
        vehicle._master.target_system,
        vehicle._master.target_component,
        0b00000100,  # ignore body rates
        q,
        0, 0, 0,
        thrust
    )

# ---------------------------
# Main loop
# ---------------------------
print("Streaming attitude + thrust...")
start = time.time()
try:
    while time.time() - start < 15:
        send_attitude_target(vehicle, roll=0.0, pitch=0.0, yaw=0.0, thrust=0.6)
        time.sleep(0.05)
    print("Command stream complete.")
except KeyboardInterrupt:
    print("Stopped manually.")
finally:
    print("Disarming...")
    vehicle.armed = False
    vehicle.close()
    print("Vehicle disarmed and disconnected.")

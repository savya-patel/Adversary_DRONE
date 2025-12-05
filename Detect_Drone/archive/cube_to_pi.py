#SAVYA

from dronekit import connect, VehicleMode
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import logging
#logging.getLogger('dronekit').setLevel(logging.CRITICAL)

import time

# --- Connect to Cube Orange ---
print("Connecting to Cube Orange...")
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
print("Connected to:", vehicle.version)

# Disable failsafes for bench testing
vehicle.parameters['FS_THR_ENABLE'] = 0    # disable radio failsafe
vehicle.parameters['FS_BATT_ENABLE'] = 0   # disable battery failsafe
vehicle.parameters['ARMING_CHECK'] = 0     # disable all arming checks
print("Failsafes disabled for bench test.")

# --- Prepare Vehicle ---
print("Setting mode to GUIDED NO GPS...")
vehicle.mode = VehicleMode("GUIDED_NOGPS")
while vehicle.mode.name != "GUIDED_NOGPS":
    print("Waiting for mode change...")
    time.sleep(1)

print("Arming motors (props OFF for safety)...")
vehicle.armed = True
for _ in range(10):
    time.sleep(1)
    vehicle_state = vehicle.system_status.state
    print(f"Arm status: {vehicle.armed}, system status: {vehicle_state}")
    if vehicle.armed:
        print("Confirmed armed.")
        break
else:
    print("Failed to arm – check Mission Planner messages.")

# --- Listen for Failsafe or Critical Warnings ---
def failsafe_listener(self, name, msg):
    if msg.get_type() == "STATUSTEXT":
        text = msg.text.lower()
        if "failsafe" in text or "critical" in text or "error" in text:
            print(f"[FAILSAFE] {msg.text}")
        elif "arm" in text or "disarm" in text:
            print(f"[STATUS] {msg.text}")
        elif "gps" in text or "compass" in text or "ekf" in text:
            print(f"[SENSOR] {msg.text}")

# Subscribe to all STATUSTEXT messages
vehicle.add_message_listener('STATUSTEXT', failsafe_listener)

# --- Command Functions ---
def send_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_attitude_target(roll=0, pitch=0, yaw_rate=0, thrust=0.5):
    q = QuaternionBase([roll, pitch, 0])
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0,
        0b00000111,
        q.q,
        0, 0, yaw_rate,
        thrust)
    vehicle.send_mavlink(msg)
    vehicle.flush()

# --- Telemetry Log ---
#logfile = open("telemetry_log.csv", "a")
#logfile.write("time,roll,pitch,yaw,vx,vy,vz,throttle,mode,armed\n")

# --- Open Loop Test ---
try:
    print("Starting open-loop command stream... (Ctrl+C to stop)\n")
    while True:
        # Send small movement commands
        send_velocity(0.2, 0, 0)
        send_attitude_target(roll=0.0, pitch=0.1, yaw_rate=0.0, thrust=0.6)

        # --- Telemetry Feedback ---
        att = vehicle.attitude
        vel = vehicle.velocity
        thr = vehicle.channels.get('3', None)
        mode = vehicle.mode.name
        armed = vehicle.armed

        # Print live status
        print(f"[MODE: {mode}] Armed: {armed}")
        print(f"  Attitude -> Roll: {att.roll:.3f}, Pitch: {att.pitch:.3f}, Yaw: {att.yaw:.3f}")
        print(f"  Velocity -> vx: {vel[0]:.3f}, vy: {vel[1]:.3f}, vz: {vel[2]:.3f}")
        print(f"  Throttle -> {thr if thr else 'N/A'}")
        print("-" * 60)

        # Log to file
        #logfile.write(f"{time.time()},{att.roll},{att.pitch},{att.yaw},{vel[0]},{vel[1]},{vel[2]},{thr},{mode},{armed}\n")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopping open-loop stream...")
finally:
    #logfile.close()
    vehicle.armed = False
    vehicle.close()
    print("Disconnected from vehicle.")


# --- Closed Test Loop ---
"""print("Sending test commands... (Ctrl+C to stop)")
try:
    for i in range(20):
        # Example: move forward slowly, pitch forward slightly, and add light throttle
        send_velocity(0.2, 0, 0)           # forward
        send_attitude_target(roll=0.0, pitch=0.1, yaw_rate=0.0, thrust=0.6)
        time.sleep(0.1)
    send_velocity(0, 0, 0)
    send_attitude_target(0, 0, 0, 0.5)
except KeyboardInterrupt:
    print("Stopping test...")
finally:
    vehicle.armed = False
    vehicle.close()
    print("Disconnected from vehicle.")"""


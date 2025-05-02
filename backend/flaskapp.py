from flask import Flask, jsonify, request
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from flask_cors import CORS
import threading
import time
import random
from math import radians, sin, cos, sqrt, atan2
app = Flask(__name__)
CORS(app)

vehicle = None  
vehicle_lock = threading.Lock()  
stop_move_to_flag = False  
detected_number_plate = None  

number_plates = ["AP12AB1234", "TS10CD5678", "MH20EF9012", "KA05GH3456", "DL03IJ7890"]  # Sample plates

def get_vehicle():
    """Connects to the vehicle if not already connected."""
    global vehicle
    with vehicle_lock:
        if vehicle is None:
            print("Connecting to vehicle...")
            try:
                vehicle = connect("tcp:127.0.0.1:5760", wait_ready=True, timeout=60)
                print("Vehicle connected successfully!")
            except Exception as e:
                print("Failed to connect: {}".format(e))
                return None
    return vehicle

@app.route('/data', methods=['GET'])
def get_data():
    vehicle = get_vehicle()
    if vehicle:
        print("Battery:", vehicle.battery)
        print("GPS:", vehicle.gps_0)
        print("System Status:", vehicle.system_status)
        print("Commands:", vehicle.commands)
        print("Channels:", vehicle.channels)

        data = {
            "Mode": vehicle.mode.name,
            "Latitude": vehicle.location.global_frame.lat,
            "Longitude": vehicle.location.global_frame.lon,
            "Yaw": vehicle.attitude.yaw,
            "Altitude": vehicle.location.global_relative_frame.alt,
            "Velocity": vehicle.velocity,
            "Pitch": vehicle.attitude.pitch,
            "Roll": vehicle.attitude.roll,
            "Speed": vehicle.groundspeed,
            "Heading": vehicle.heading,
            "Armed": vehicle.armed,
            # Extra monitoring
            "BatteryVoltage": getattr(vehicle.battery, 'voltage', None),
            "BatteryCurrent": getattr(vehicle.battery, 'current', None),
            "BatteryLevel": getattr(vehicle.battery, 'level', None),
            "IsArmable": vehicle.is_armable,
            "EKFStatus": vehicle.ekf_ok,
            "GPSFixType": getattr(vehicle.gps_0, 'fix_type', None),
            "SatellitesVisible": getattr(vehicle.gps_0, 'satellites_visible', None),
            "SystemStatus": getattr(vehicle.system_status, 'state', None),
            "RCChannels": vehicle.channels,
            "Waypoint": getattr(vehicle.commands, 'next', None),
            "TotalWaypoints": len(vehicle.commands),
            "LastHeartbeat": vehicle.last_heartbeat
        }
        return jsonify(data)
    else:
        return jsonify({"error": "Vehicle not connected"}), 500


def move_to_target(lat, lon):
    """Moves the vehicle to a target location and assigns a number plate after reaching."""
    global stop_move_to_flag, detected_number_plate

    vehicle = get_vehicle()
    if vehicle is None:
        return

    with vehicle_lock:
        if vehicle.mode.name != "GUIDED":
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(2)

    target_location = LocationGlobalRelative(lat, lon, vehicle.location.global_relative_frame.alt)
    
    with vehicle_lock:
        vehicle.simple_goto(target_location)
        print("Moving to ({}, {})...".format(lat, lon))

    detected_number_plate = None  

    while True:
        if stop_move_to_flag:
            print("Stopping move_to due to manual override or stop command.")
            return
        with vehicle_lock:
            distance = get_distance_to_target(vehicle, lat, lon)
            print(distance)
        
        if distance < 2:  
            detected_number_plate = random.choice(number_plates)
            print("Arrived at ({}, {}). Detected Number Plate: {}".format(lat, lon, detected_number_plate))
            return
        
        time.sleep(0.5)

def get_distance_to_target(vehicle, target_lat, target_lon):
    """Calculates the great-circle distance between current location and target in meters using Haversine formula."""
    current_lat = vehicle.location.global_frame.lat
    current_lon = vehicle.location.global_frame.lon

    if current_lat is None or current_lon is None:
        return float('inf')  # Avoid false detection

    # Convert latitude and longitude from degrees to radians
    lat1 = radians(current_lat)
    lon1 = radians(current_lon)
    lat2 = radians(target_lat)
    lon2 = radians(target_lon)

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    R = 6371000  # Earth radius in meters
    distance = R * c

    return distance

@app.route('/move_to', methods=['POST'])
def move_to_location():
    """Starts moving the vehicle to a given location in a separate thread."""
    global stop_move_to_flag
    stop_move_to_flag = False  

    data = request.json
    lat = data.get("latitude")
    lon = data.get("longitude")

    if lat is None or lon is None:
        return jsonify({"error": "Latitude and Longitude are required"}), 400

    move_thread = threading.Thread(target=move_to_target, args=(lat, lon))
    move_thread.daemon = True  
    move_thread.start()

    return jsonify({"message": "Moving to ({}, {})".format(lat, lon)})

@app.route('/stop', methods=['POST'])
def stop_vehicle():
    """Stops the vehicle and any auto movement immediately."""
    global stop_move_to_flag
    stop_move_to_flag = True  

    vehicle = get_vehicle()
    if vehicle is None:
        return jsonify({"error": "Vehicle not connected"}), 500

    with vehicle_lock:
        print("Stopping vehicle...")
        vehicle.mode = VehicleMode("GUIDED")
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

    return jsonify({"message": "Vehicle stopped."})

@app.route('/vehicle/move', methods=['POST'])
def move_vehicle():
    """Moves the vehicle manually in all four directions and can stop it."""
    global stop_move_to_flag
    stop_move_to_flag = True

    vehicle = get_vehicle()
    if vehicle is None:
        return jsonify({"error": "Vehicle not connected"}), 500

    data = request.json
    direction = data.get("direction")

    if direction not in ["forward", "backward", "left", "right", "stop"]:
        return jsonify({"error": "Invalid direction."}), 400

    with vehicle_lock:
        if vehicle.mode.name != "GUIDED":
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)

    velocity_x = 2 if direction == "forward" else -2 if direction == "backward" else 0
    velocity_y = 2 if direction == "right" else -2 if direction == "left" else 0
    velocity_z = 0  

    with vehicle_lock:
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111, 0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

    if direction != "stop":
        message = "Moving {} at 2 m/s".format(direction)
    else:
        message = "Vehicle stopped."

    return jsonify({"message": message})
@app.route('/')
def home():
    return "Hello from Rover"
@app.route('/arm', methods=['POST'])
def arm_vehicle():
    """Arms the vehicle if it's armable."""
    vehicle = get_vehicle()
    if vehicle is None:
        return jsonify({"error": "Vehicle not connected"}), 500

    with vehicle_lock:
        if not vehicle.is_armable:
            return jsonify({"error": "Vehicle is not armable yet (check sensors/GPS)"}), 400

        print("Arming vehicle...")
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(2)

        vehicle.armed = True

        # Wait until the vehicle is armed
        timeout = time.time() + 10  # wait max 10 seconds
        while not vehicle.armed and time.time() < timeout:
            print("Waiting for arming...")
            time.sleep(1)

        if vehicle.armed:
            return jsonify({"message": "Vehicle is now armed."})
        else:
            return jsonify({"error": "Failed to arm the vehicle."}), 500


@app.route('/disarm', methods=['POST'])
def disarm_vehicle():
    """Disarms the vehicle."""
    vehicle = get_vehicle()
    if vehicle is None:
        return jsonify({"error": "Vehicle not connected"}), 500

    with vehicle_lock:
        print("Disarming vehicle...")
        vehicle.armed = False

        # Wait until the vehicle is disarmed
        timeout = time.time() + 10  # wait max 10 seconds
        while vehicle.armed and time.time() < timeout:
            print("Waiting for disarming...")
            time.sleep(1)

        if not vehicle.armed:
            return jsonify({"message": "Vehicle is now disarmed."})
        else:
            return jsonify({"error": "Failed to disarm the vehicle."}), 500

if __name__ == "__main__":
    app.run(port=5000, debug=True, use_reloader=False)

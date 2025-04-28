from flask import Flask, jsonify, request
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from flask_cors import CORS
import threading
import time
import random

app = Flask(__name__)
CORS(app)

vehicle = None
vehicle_lock = threading.Lock()
stop_move_to_flag = False
detected_number_plate = None

number_plates = ["AP12AB1234", "TS10CD5678", "MH20EF9012", "KA05GH3456", "DL03IJ7890"]

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
                print(f"Failed to connect: {e}")
                return None
    return vehicle

@app.route('/data', methods=['GET'])
def get_data():
    """Retrieves vehicle telemetry data and detected number plate."""
    vehicle = get_vehicle()
    if vehicle:
        with vehicle_lock:
            data = {
                "Mode": vehicle.mode.name,
                "Latitude": vehicle.location.global_frame.lat,
                "Longitude": vehicle.location.global_frame.lon,
                "Yaw": vehicle.attitude.yaw,
                "Altitude": vehicle.location.global_relative_frame.alt,
                "Velocity": list(vehicle.velocity),
                "Pitch": vehicle.attitude.pitch,
                "Roll": vehicle.attitude.roll,
                "Speed": vehicle.groundspeed,
                "Heading": vehicle.heading,
                "Armed": vehicle.armed,
                "NumberPlate": detected_number_plate
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
        print(f"Moving to ({lat}, {lon})...")

    detected_number_plate = None

    while True:
        if stop_move_to_flag:
            print("Stopping move_to due to manual override or stop command.")
            return

        with vehicle_lock:
            distance = get_distance_to_target(vehicle, lat, lon)

        if distance < 2:
            detected_number_plate = random.choice(number_plates)
            print(f"Arrived at ({lat}, {lon}). Detected Number Plate: {detected_number_plate}")
            return

        time.sleep(0.5)

def get_distance_to_target(vehicle, lat, lon):
    """Calculates approximate distance to the target location."""
    current_lat = vehicle.location.global_frame.lat
    current_lon = vehicle.location.global_frame.lon
    return ((current_lat - lat) ** 2 + (current_lon - lon) ** 2) ** 0.5 * 111000

@app.route('/move_to', methods=['POST'])
def move_to_location():
    """Starts moving the vehicle to a given location in a separate thread."""
    global stop_move_to_flag
    stop_move_to_flag = False

    data = request.get_json()
    lat = data.get("latitude")
    lon = data.get("longitude")

    if lat is None or lon is None:
        return jsonify({"error": "Latitude and Longitude are required"}), 400

    move_thread = threading.Thread(target=move_to_target, args=(lat, lon))
    move_thread.setDaemon(True)  # ✅ Correct way for Python 3.6-3.10 (Flask 2.0.3 compatible)
    move_thread.start()

    return jsonify({"message": f"Moving to ({lat}, {lon})"})

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
            0b0000111111000111,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
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

    data = request.get_json()
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
            0b0000111111000111,
            0, 0, 0,
            velocity_x, velocity_y, velocity_z,
            0, 0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

    return jsonify({"message": f"Moving {direction} at 2 m/s" if direction != "stop" else "Vehicle stopped."})

if __name__ == "__main__":
    app.run(port=5000, debug=True, use_reloader=False)


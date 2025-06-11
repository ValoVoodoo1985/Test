import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import math
from pymavlink import mavutil
from geographiclib.geodesic import Geodesic
from geopy.distance import geodesic
from geopy.point import Point

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(5)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def calculate_azimuth_0_to_360(lat1, lon1, lat2, lon2):
    geod = Geodesic.WGS84  # Use the WGS84 ellipsoid model
    inverse_result = geod.Inverse(lat1, lon1, lat2, lon2)
    azimuth = inverse_result['azi1']
    if azimuth < 0:
        azimuth += 360
    return azimuth

def calculate_geodesic_distance(lat1, lon1, lat2, lon2):
    point1 = Point(latitude=lat1, longitude=lon1)
    point2 = Point(latitude=lat2, longitude=lon2)
    distance = geodesic(point1, point2).m
    return distance

def get_global_location():
    global_location = vehicle.location.global_frame
    return global_location

def get_yaw():
    azimuth = vehicle.attitude.yaw * 180 / math.pi
    if azimuth < 0:
        azimuth += 360
    return azimuth

def yaw_to_target(azimuth, distance):
    yaw = get_yaw()
    if azimuth > yaw:
        direction = 1
        angle = azimuth - yaw
    else:
        direction = -1
        angle = yaw - azimuth

    if angle < 23:
        turnspeed = 22
    else:
        turnspeed = 40

    vehicle.channels.overrides['4'] = 1500 + (direction * int(turnspeed))
    return turnspeed

def hold_altitude(alt):
    if vehicle.location.global_relative_frame.alt < alt:
        vehicle.channels.overrides['3'] = 1700
    if vehicle.location.global_relative_frame.alt > alt:
        vehicle.channels.overrides['3'] = 1400

def pitch_from_distance(distance):
    if distance < 5:
        vehicle.channels.overrides['2'] = 1480 # Pitch
    elif distance < 50:
        vehicle.channels.overrides['2'] = 1400 # Pitch
    else:
        vehicle.channels.overrides['2'] = 1000 # Pitch

starting_point = (50.450739, 30.461242)
target_point = (50.443326, 30.448078)
altitude = 100

connection_string = "tcp:127.0.0.1:5762"
vehicle = connect(connection_string, wait_ready=True)

coordinates = get_global_location()
distance = calculate_geodesic_distance(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
azimuth = calculate_azimuth_0_to_360(coordinates.lat, coordinates.lon, target_point[0], target_point[1])


arm_and_takeoff(altitude)
vehicle.mode = VehicleMode("ALT_HOLD")
time.sleep(2)


while True:
    coordinates = get_global_location()
    distance = calculate_geodesic_distance(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
    azimuth = calculate_azimuth_0_to_360(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
    yaw = get_yaw()
    angle = yaw_to_target(azimuth, distance)
    hold_altitude(altitude)
    if angle < 30:
        pitch_from_distance(distance)
    if distance < 8:
        vehicle.channels.overrides['2'] = None # Pitch
        break
    print(f'VMode: {vehicle.mode.name}, Dist: {round(distance, 1)}, Alt: {round(vehicle.location.global_relative_frame.alt, 1)}, Azim: {round(azimuth, 1)}, Yaw: {round(yaw, 1)}, Moving to the point.')
    time.sleep(0.5)

while True:
    coordinates = get_global_location()
    distance = calculate_geodesic_distance(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
    azimuth = calculate_azimuth_0_to_360(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
    yaw = get_yaw()
    hold_altitude(100)
    yaw_to_target(350, distance)
    print(f'VMode: {vehicle.mode.name}, Dist: {round(distance, 1)}, Alt: {round(vehicle.location.global_relative_frame.alt, 1)}, Azim: {round(azimuth, 1)}, Yaw: {round(yaw, 1)}, Point reached. Coords: {coordinates.lat, coordinates.lon}')
    time.sleep(0.5)

# vehicle.close()

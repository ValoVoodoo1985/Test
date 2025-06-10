
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import math
from pymavlink import mavutil
from geographiclib.geodesic import Geodesic
from geopy.distance import geodesic
from geopy.point import Point

starting_point = (50.450739, 30.461242)
target_point = (50.443326, 30.448078)
altitude = 100

connection_string = "tcp:127.0.0.1:5762"
vehicle = connect(connection_string, wait_ready=True)

# start_location = LocationGlobal(50.450739, 30.461242, 0)
# vehicle.home_location = start_location

print(vehicle.location.global_relative_frame.alt)

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


def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1 # yaw relative to direction of travel
    else:
        is_relative = 0 # absolute yaw
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,       # confirmation
        heading, # param 1, yaw in degrees
        0,       # param 2, yaw speed (deg/s)
        1,       # param 3, direction - 1 clockwise, -1 counter clockwise
        is_relative, # param 4, relative (1) or absolute (0)
        0, 0, 0) # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)

def get_global_location():
    global_location = vehicle.location.global_frame
    return global_location

def get_yaw():
    azimuth = vehicle.attitude.yaw * 180 / math.pi
    if azimuth < 0:
        azimuth += 360
    return azimuth

coordinates = get_global_location()
distance = calculate_geodesic_distance(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
azimuth = calculate_azimuth_0_to_360(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
print('Azimuth: ', azimuth)
print('Distance: ', distance)

if vehicle.armed == False:
    arm_and_takeoff(100)

condition_yaw(azimuth)
time.sleep(10)
print(f" Yaw: {vehicle.attitude.yaw * 180 / math.pi:.2f} degrees")

vehicle.mode = VehicleMode('ALT_HOLD')
time.sleep(5)
print(vehicle.mode.name)

while True:
    distance = calculate_geodesic_distance(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
    if distance < 30:
        vehicle.channels.overrides['3'] = 1050 # Throttle
        vehicle.channels.overrides['2'] = 1450 # Pitch
    elif distance < 150:
        vehicle.channels.overrides['3'] = 1200 # Throttle
        vehicle.channels.overrides['2'] = 1400 # Pitch
    else:
        vehicle.channels.overrides['3'] = 1800 # 4 Throttle
        vehicle.channels.overrides['2'] = 1000 # 4 Pitch
    time.sleep(0.5)

    coordinates = get_global_location()
    distance = calculate_geodesic_distance(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
    azimuth = calculate_azimuth_0_to_360(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
    yaw = get_yaw()
    if yaw < azimuth:
        vehicle.channels.overrides['4'] = 1522 # Yaw
    if yaw > azimuth:
        vehicle.channels.overrides['4'] = 1478 # Yaw
    print('Distance: ', distance,
          'Altitude: ', vehicle.location.global_frame.alt,
          'Target azimuth: ', azimuth,
          'Vehicle azimuth: ', yaw,
          'Vehicle mode: ', vehicle.mode.name )
    if distance < 1.5:
        vehicle.channels.overrides['2'] = None # Pitch
        vehicle.channels.overrides['3'] = None # Throttle
        vehicle.channels.overrides['4'] = None # Yaw
        break

coordinates = get_global_location()
distance = calculate_geodesic_distance(coordinates.lat, coordinates.lon, target_point[0], target_point[1])
vehicle.mode = VehicleMode("GUIDED")
time.sleep(5)
condition_yaw(350)
yaw = get_yaw()
time.sleep(15)
print('Coordinats: ', coordinates.lat, coordinates.lon,
      'Distance to target: ', distance,
      'Yaw: ', yaw,
      'Altitude: ', vehicle.location.global_frame.alt)
vehicle.close()